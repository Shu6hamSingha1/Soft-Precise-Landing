---
name: reference_gt_optical_flow
description: "Canonical, CORRECT way to compute ground-truth optical flow / loom (h_z) / virtual centroid / yaw from a recording, and the alignment to measured signals. Reusable tool: PX4_Gazebo/tools/gt_optical_flow.py (compute_gt_flow(rep_dir)). Use it instead of ad-hoc gradients — it ports plotter_output_calibration.ipynb cell 6 + plotter_landing_test.ipynb cell 24. Encodes the 4 mistakes to NEVER repeat: fabricated linspace time axis, raw-gradient on jittery GT, fractional-index alignment, and dividing by wrong/no depth."
metadata:
  node_type: memory
  type: reference
  originSessionId: 7415f420-9591-41b1-8349-bb9361a8dc82
---

**Tool:** `PX4_Gazebo/tools/gt_optical_flow.py` — `compute_gt_flow(rep_dir)` returns the GT reference `{t_g, alt, B_h_g, V_h_g, loom(=V_h_g[:,2]=vz/Z), alpha(yaw), start_time, align()}`. CLI: `python3 tools/gt_optical_flow.py <rep_dir>` prints GT loom vs measured `h_z`. **ALWAYS use this for any "check X against GT" — do NOT hand-roll a gradient.**

## The CORRECT method (ports the notebooks — verified)
1. **GT velocity:** `W_x_tu = target_pos − UAV_pos` (NED, from Gazebo ENU/FLU via `NED_FROM_ENU @ R @ FRD_2_FLU`). Then **uniform-dt interp → savgol(W≈0.225/dt, p=2) → np.gradient(·, t_g) → interp back to t_g**. Bridge jitter makes raw `np.gradient` blow up velocity/ω.
2. **GT optical flow / loom:** rotate the relative velocity into the gravity-leveled **V-frame** (`V_R_body`, same de-rotation as the controller's `_getVirtualPts` — VERIFIED correct), divide by the **true depth** `z_V = W_x_tu[:,2]` (= relative altitude): `V_h_g = (V_R_body @ B_v_tu)/(z_V+0.01)`. **Loom `h_z = V_h_g[:,2] = vz/Z`** (negative = descending; matches the controller's `h_d_z=h_rd` sign).
3. **GT angular velocity:** quaternion-DIFFERENCE (NOT `np.gradient(R)` — that breaks SO(3), over-reports ω_z ~3× — see [[feedback_so3_quaternion_omega]]).
4. **Yaw/alpha:** `to_angles()[2]` of the UAV quaternion.

## ALIGNMENT — the load-bearing part (skew-free)
GT, image, control, telemetry are on **DIFFERENT clocks** ([[feedback_imgdata_gt_clock_skew]]). The shared anchor is **`gt['Start Time']`** (absolute). `gt['Time']` is already 0-based (seconds since descent start). Measured signals (Img `Time`, Telemetry `Odometry Timestamp`) are ABSOLUTE → subtract Start Time: `t_meas = stamp − Start_Time`. Then both are seconds-since-start and `np.interp` onto `t_g` is valid. The tool's `align(t_abs, y)` does this.

## THE 4 MISTAKES TO NEVER REPEAT (made 2026-06-13, this is why the file exists)
1. **Fabricated time axis** — `np.linspace(0, t_ctrl[-1], N)` for the GT dt. WRONG: GT has its own clock; a wrong dt mis-SCALES velocity. Gave a spurious **~6 m/s descent that contradicted the 0.37 m/s touchdown** — pure artifact.
2. **Raw gradient on jittery GT** — always uniform-interp + savgol BEFORE differentiating.
3. **Fractional-index alignment** (`i/n` matched across arrays) — INVALID across the skewed clocks; use Start-Time-relative seconds.
4. **Wrong/no depth** — loom is `vz/Z`; divide by the GT depth `z_V`, in the V-frame (not body, not raw).

## What it confirmed (the finding it was built for)
On a soft-config gate rep: **the measured loom `h_z` UNDER-REPORTS the descent** — early/altitude (slow) it tracks GT well (−0.15 vs −0.13), but mid-late (alt 1.6–4 m) the GT loom grows to −1…−4.7 while measured `h_z` stays ~0 (registers only at the deck, −1.86). `|GT_loom|` mean 2.15 vs `|meas h_z|` 0.21, corr 0.74. This is the **1/Z observability** behind the descent residual `h_e_z` — see [[feedback_descent_softness]].
