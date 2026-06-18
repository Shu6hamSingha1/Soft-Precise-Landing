---
name: so3-quaternion-omega
description: "Body-frame angular velocity from pose data MUST use quaternion difference, NOT np.gradient on rotation matrices — the latter breaks SO(3) and over-reports ω_z by ~2×"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

When computing body-frame angular velocity ω from a logged pose stream, NEVER use the `np.gradient(W_R_B) → R^T @ dR → skew_to_vec` pipeline. Use quaternion central-difference instead:

```python
def _body_omega_from_quats(quats, t):
    """quats: (N, 4) wxyz; returns body-frame ω in the quaternion's body frame."""
    N = len(quats); w = np.zeros((N, 3))
    for i in range(N):
        i0 = max(0, i - 1); i1 = min(N - 1, i + 1)
        if i1 == i0 or t[i1] - t[i0] < 1e-9: continue
        w0, x0, y0, z0 = quats[i0]; w1, x1, y1, z1 = quats[i1]
        # δq = conj(q0) * q1   (w-first convention)
        dq_x = w0*x1 - x0*w1 - y0*z1 + z0*y1
        dq_y = w0*y1 + x0*z1 - y0*w1 - z0*x1
        dq_z = w0*z1 - x0*y1 + y0*x1 - z0*w1
        # ω_body ≈ 2 · δq.xyz / (t[i+1] - t[i-1])  for small δq
        w[i] = 2.0 * np.array([dq_x, dq_y, dq_z]) / (t[i1] - t[i0])
    return w
```

If the quaternion is FLU-body→ENU-world (Gazebo), post-multiply by `FLU_2_FRD = DCM(x=180)` to get FRD-body ω. If the quaternion is already FRD-body→world (PX4), no conversion needed.

**Why:** Verified 2026-05-31 against the `output_calibration` recording in `PX4_Gazebo/`:
- Theory: 10° yaw amplitude × 2π × 0.5 Hz = 0.548 rad/s peak (0.388 RMS)
- PX4 IMU telemetry: 0.748 peak / 0.501 RMS (1.29× theory — physical rate-loop overshoot)
- GT via `np.gradient(W_R_B)+skew`: **3.051 peak / 0.879 RMS (2.27× theory — non-physical)**
- GT via quaternion-difference: 1.858 peak / 0.591 RMS (1.52× theory — close to IMU)

`np.gradient` on the 9 elements of W_R_B independently breaks the orthogonality constraint between samples. The resulting `R^T @ dR/dt` matrix is NOT pure skew-symmetric; the non-skew residual leaks into the extracted `[skew[2,1], skew[0,2], skew[1,0]]` and inflates the reported ω. Smoothing (even sgf(101, 2)) does not fix this — the bias is in the differentiation, not the noise. The quaternion form stays on the unit-norm manifold, so the extracted ω is mathematically clean.

**How to apply:**
- Anywhere body-frame ω is derived from logged pose for sensor calibration, IBVS interaction-matrix validation, or controller diagnosis: use the quaternion form.
- Files already patched (2026-05-31): `plotter_output_calibration.ipynb` cell 6, `plotter_input_calibration.ipynb` cell 12, `aggregate_calibration.py`, `aggregate_calibration_phased.py`, `analyze_calibration.py`, `validate_pose_transforms.py` CHECK 8, `tune_savgol.py`.
- When inheriting Python from the legacy `~/ws/scripts/soft_precise_landing/` baseline that uses the gradient form, port to quaternion-difference before trusting the output.
- Sanity test for any new pose-derivation pipeline: pure-yaw excitation at known amplitude/frequency should match `2πfA × sqrt(2)/2` RMS to within IMU overshoot (≤1.3×). >2× over-estimate → orthogonality violation.

Related: [[project-camera-calibration-status]] (sensor_cal_hw was originally tuned against the over-estimated GT and currently over-amplifies w_cal ~2.8× until re-derived).
