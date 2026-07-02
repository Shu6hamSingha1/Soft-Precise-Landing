---
name: feedback_rover_yaw_cal_resolved
description: "Moving-target yaw 'calibration' work item resolved: the alpha (s[3]) feature is ALREADY calibrated (cal_s[3]=1.0, alpha tracks GT yaw r=1.00), stationary-rover yaw is verified clean (baseline e_a->0), and GT-FB computes RELATIVE yaw+rate so it is turning-target-correct by construction. The only real turning-rover gap is the controller's alpha-rate cap PLASMC_YAW_ALPHA_MAX_RATE=0.30 rad/s, which clamps a rover turning faster than ~17 deg/s."
metadata:
  node_type: memory
  type: feedback
---

**Moving-target "yaw calibration" (the [[project_moving_target_prep]] item 2) — RESOLVED 2026-07-02.**
It is NOT a scale/offset cal task; the feature is already calibrated. Findings:

1. **The yaw feature is ALREADY calibrated.** `cal_s[3]=1.0` is correct (alpha tracks
   GT yaw slope +1.00, r=1.00 per the extensive 2026-06-04 analysis in
   [[project_yaw_calibration_pending]]; the aggregator correctly keeps s[3]=1.0). No
   scale/offset redo. (`_sensor_cal_s = diag([1.0273,1.0669,1.0,1.0])`, 4th=identity.)

2. **Stationary-rover yaw is VERIFIED clean** (this session, baseline rover+platform
   GT-FB n=5): `e_a` (yaw error) converges to ~0 (ends −0.3° to −1.5°, drone aligned with
   the marker), yaw-rate cmd `u_a` max 0.17–0.22 rad/s (well below the 0.30 cap, well-behaved).
   No yaw runaway. So the yaw sensing + control is correct for the stationary rover.

3. **GT-FB is TURNING-target-correct by construction.** `gt_feedback.py` computes
   `ry = yaw(uav) − yaw(target)` (RELATIVE yaw) and `w_z = −d(ry)/dt`, so it already
   subtracts the target's rotation. For a turning rover the relative-yaw feature + rate are
   correct (the earlier "gt_feedback w_z=−alpha_dot is a stationary assumption" caveat was
   imprecise — `ry` is relative, so d(ry)/dt already carries the target rotation).

4. **The ONE real turning-rover gap = the controller alpha-rate cap.**
   `PLASMC_YAW_ALPHA_MAX_RATE=0.30 rad/s (~17°/s)` (controller.py ~345) rate-limits the alpha
   change to reject the terminal marker-fill corruption (~1 rad/s). It applies to the GT-FB
   alpha too (verified: `_alpha=self._s[-1][3]` → the rate-limit at controller.py ~1732). A
   Circular rover turns at wz≈0.48 rad/s (~27°/s) → the relative alpha transiently changes
   FASTER than 0.30 while the drone lags the turn → the cap clamps the GENUINE turn → the
   drone under-yaws → could fall behind the heading.
   - **GT-FB turning rover:** set `PLASMC_YAW_ALPHA_FILT=0` (GT-FB has NO corruption to reject
     → the filter only clamps genuine turns), OR raise `PLASMC_YAW_ALPHA_MAX_RATE≈0.8`.
   - **Perception-ON turning rover:** raise `PLASMC_YAW_ALPHA_MAX_RATE≈0.8` (passes the 0.48
     turn, still rejects the ~1 rad/s marker-fill corruption).
   - **Straight (Linear) rover:** no turn → no change needed.

5. **⚠ Do NOT swap the alpha SOURCE.** The yaw SMC is tuned to the MOMENT-alpha convention
   (period π, offset MOMENT_ALPHA0=−2.533, [4,3,2,1] weights) — swapping to the geometric
   board+x alpha catastrophically regressed the yaw loop (reverted, [[project_yaw_calibration_pending]]).
   In GT-FB the alpha is the relative yaw from gt_feedback (not the moment alpha), and s_d[3]=0
   (land aligned) → driving `e_a→0` makes the drone track the rover heading.

**Remaining = VALIDATION only:** run a TURNING-rover landing (ROVER_MOTION=1 + a Circular
trajectory) in GT-FB with the alpha-rate cap raised, confirm `e_a→0` (tracks heading, no
runaway). That converges with the moving-rover test. See [[project_moving_target_prep]],
[[feedback_moment_yaw_canonical]], docs/MOVING_TARGET_PREP.md.
