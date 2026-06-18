---
name: gt-noise-uniform-dt
description: "GT velocity/ω noise is ROS bridge timestamp jitter, not Gazebo — fix is uniform-dt interp before gradient"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

In the plotter notebooks, GT velocity and angular-velocity look noisy. The source is **ROS bridge timestamp jitter** (mean dt = 35 ms, std = 3 ms, max gap = 92 ms), not Gazebo state itself. `np.gradient(position, t)` divides by jittery dt → ~3 m/s² acceleration noise on what should be a smooth velocity signal. Same story for `np.gradient(W_R_B, t)` for angular velocity.

**Why:** Gazebo state per simulator step is deterministic; the noise comes from `ros_gz_bridge` re-packaging poses into ROS messages with their arrival timestamps. The gradient amplifies that into the derived quantity.

**How to apply:**
- Interpolate GT to a uniform-dt grid before differentiating; interpolate the gradient back to original t for downstream array alignment. Implemented in both `plotter_output_calibration.ipynb` and `plotter_input_calibration.ipynb` GT prep cells.
- Use **adaptive sgf window** sized for ~2 Hz cutoff: `W = max(5, int(round(0.225 / dt_med)) | 1)`. Both notebooks compute and print this. Fixed `W=5` is too narrow at the new ~125 Hz GT rate (only ~11 Hz cutoff — barely filters); fixed `W=51` is too aggressive (boundary distortion). Bumped sgf polyorder to 3 for better signal preservation at the wider window.
- Apply sgf AFTER interpolation (not before) — sgf assumes uniform dt; applying to jittery samples is technically wrong even when it looks OK.
- Filter `W_R_B` element-wise before its gradient (same uniform-dt path). Slightly violates SO(3) per sample, but the violation is far smaller than the bridge-jitter noise it removes. For better SO(3) preservation use the quaternion-derivative method in [[so3-quaternion-omega]].

Result: `B_v_tu` 1st-diff std dropped from ~3.2 m/s² (raw gradient) to ~0.005 m/s² with uniform-dt + sgf(adaptive, 3). The 0-5s "boundary spike" in `B_w_tug` (was 2.8 rad/s) drops to ~0.02 rad/s — confirming the original spike was bridge noise, not real drone yaw rate.

Note: in input-cal the drone DOES actually yaw fast (commanded), so post-fix `B_w_ug` correctly shows ~1-2 rad/s during commanded windows. The fix preserves real signal, only removes jitter noise.
