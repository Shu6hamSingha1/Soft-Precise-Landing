---
name: compass-yaw-drift
description: PX4 EKF yaw drifts 30-46° under sustained aggressive maneuvers; body-frame projection amplifies into ~100m position mismatch
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 367ac3aa-13f8-41ea-b23e-c929f5e1e50d
---

Under sustained fast attitude-rate commands (e.g. input-cal), PX4's EKF yaw drifts **30-46° from GT truth** even though world position stays accurate (~10 m off at most). The body-frame projection `B_x = inv(R_TEL) @ NED_pos` then multiplies that yaw error by the world-position magnitude — on a 130 m drift this produces ~100 m of phantom body-frame mismatch.

**Why:** The EKF integrates gyro for short-term yaw and slow-corrects via magnetometer (sub-Hz, noisy in SITL). Under aggressive maneuvers, gyro integration error accumulates faster than magnetometer can correct. Roll/pitch are unaffected because they're observable from the accelerometer (gravity is unambiguous). Validated by:
- TEL yaw vs GT yaw: 30-46° offset throughout input-cal run, present even at t=0.5s (EKF init transient) and growing during the run.
- GT yaw rate vs gyro_z: corr = +0.834 (GT faithful to IMU).
- TEL world position vs GT world position: agrees within ~10 m — only the yaw component is broken.

**How to apply:**
- **Live IBVS controller:** use [[v-yaw-source-alpha]] — the V frame becomes marker-relative, so compass-yaw drift can't poison flow or centroid. Already the recommended architecture.
- **Post-flight analysis (input-cal):** use `BODY_YAW_SOURCE=gt` env in `plotter_input_calibration.ipynb` (default now). Re-projects TEL world-position through GT R; isolates world-position error from yaw error. Body-frame disagreement drops from ~97 m to ~0.9 m (100×). Set `BODY_YAW_SOURCE=ekf` to see the raw EKF drift as a diagnostic.
- **Don't chase EKF yaw with parameter tuning** — `EKF2_MAG_TYPE=1` and friends can reduce drift but trade off magnetic interference robustness. The marker-derived approach above bypasses the problem entirely for the landing phase, which is the only phase that matters for IBVS performance.
- **Mixed-mission setups** (transit + terminal landing): use compass for transit, switch to marker-relative for terminal. Compass at takeoff is typically fine; drift accumulates with aggressive maneuvering.

The yaw plot section in `plotter_input_calibration.ipynb` (cells inserted after the Euler-angle plot) overlays TEL vs GT yaw and shows the drift trace — useful diagnostic for new runs.
