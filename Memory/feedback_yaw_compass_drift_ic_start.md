---
name: yaw-compass-drift-ic-start
description: "ROOT CAUSE of the IC2-5 'yaw runaway' (pinned 2026-06-04): NOT the alpha feature — it's the COMPASS/EKF yaw drifting ~77deg by landing start, so the IC-convergence rig (which holds+gates on EKF yaw) starts the descent with the drone PHYSICALLY yawed ~77deg. The yaw SMC can't unwind it during the fast descent -> psi_d winds to 180deg. Fix is in the TEST RIG, not the controller. Includes the yaw-usage audit."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**The IC2-5 'yaw runaway' is the COMPASS DRIFT at the landing START, not the alpha computation.**
Measured (IC1 rep1, 2026-06-04): at landing start GT yaw = +77deg, EKF yaw = +0deg -> the EKF/compass has
drifted 77deg during takeoff+IC-convergence. The IC rig holds yaw via `send_position_ned(...,INITIAL_YAW_DEG)`
(PX4 tracks EKF) and gates `yaw_err` on the EKF quaternion -> both read "0" while the drone is physically yawed
77deg. So the descent STARTS at GT yaw ~77deg; the yaw SMC (image alpha) correctly sees a real 77deg error and,
trying to unwind it during the ~6s descent, winds psi_d to +-180deg -> xy 2-16m. PRE-EXISTING: baseline reps
start at ~94deg too (it just bottoms out at ~125deg instead of diverging).

**This corrects three earlier WRONG attributions this session:** the IC2-5 yaw failure was NOT "alpha is a
non-signal", NOT the "alpha flip" ([[yaw-calibration-pending]]), and NOT fixable by any alpha redesign. THREE
alpha redesigns (geometric-unify, moment-average, moment-2pi) all drove psi_d->180 because none touched the real
cause (the bad start). The alpha was correct all along (alpha_start ~= GT_yaw_start in every rep).

**YAW-USAGE AUDIT (controller pipeline, 2026-06-04) — where yaw enters:**
- DIRECT (image/TRUE yaw, compass-free): the yaw SMC `_yawCtrl` uses `e_a = s[3]-s_d[3]` (the alpha marker-
  orientation feature) -> `u_a` body yaw rate -> integrated into `psi_d` (a VIRTUAL compass; "no external
  heading reference enters"). The control's yaw reference is 100% alpha.
- INDIRECT (EKF/compass) — ONLY via the rotation matrix:
  - SO(3) inner loop `_attCtrl`: `R = Quaternion(self._quat).to_DCM()` (EKF) in `e_R=1/2 vee(Rd^T R - R^T Rd)`.
    The loop drives R->Rd whose yaw IS psi_d; the alpha OUTER loop compensates the drift in steady state.
  - V-frame `_getVirtualPts`: EKF quat for roll/pitch LEVELING (doesn't drift); for a level drone the marker
    orientation is the physical/true view, so yaw drift never reaches alpha (why alpha tracks GT r=1.00).
  - `W_XY_DEROT=imu` (default OFF): roll/pitch only.
  So compass yaw touches the controller ONLY through the rotation matrix, where the alpha loop absorbs it. No
  compass-drift fix is needed in the control law. (Confirms the user's claim.)

**THE FIX (test rig only, `apps/landing_test.py` IC convergence, 2026-06-04, uncommitted):** servo the NED yaw
setpoint to null the TRUE (Gazebo) yaw and gate on truth, so the drone starts the descent physically aligned:
`yaw_cmd_deg += 0.5*deg(wrap(GT_yaw - INITIAL_YAW))` (the NED cmd and Gazebo ENU yaw rotate oppositely, so +err
reduces it), gate `|GT_yaw - INITIAL_YAW| <= IC_YAW_TOL`. Truth is allowed for test setup
([[scale-free-depth-free]]). roll/pitch stay on the EKF (don't drift). NEEDS validation; if IC convergence times
out, the servo sign is flipped. This also UN-CONFOUNDS the alpha comparison (every prior alpha A/B started at a
different random 77-94deg yaw).

**Alpha redesign status (img_data.py, uncommitted):** 2pi moment yaw — per-marker `_marker_principal_angle`
(2nd-moment axis disambiguated by the weighted-centroid displacement -> full 2pi, no pi-fold), averaged across
markers, offset RECALIBRATED to `_moment_alpha_0 = -2.533` (the legacy -0.9379 was single-marker-era -> board
equilibrium sat at ~88deg). Perception-validated on output-cal: tracks GT yaw slope +1.0 / r 0.98, 0 folds,
equilibrium ~0. It is the prerequisite for V_YAW_SOURCE=alpha. But it is NOT what fixes IC2-5 (the IC-start is) —
keep/commit it only if the IC-fixed landing validation passes; otherwise the working baseline is git 0008ba1.
See [[moment-yaw-canonical]], [[compass-yaw-drift]].
