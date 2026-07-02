---
name: feedback_compass_free_validation
description: "COMPASS_FREE_VALIDATE=1 (2026-07-02): at controller engage, EKF2_MAG_TYPE=5 is set via MAVSDK (readback-verified) so PX4's EKF2 stops fusing the magnetometer, and BODY_YAW_SOURCE=alpha is enforced — the validated descent carries ZERO compass influence, internal or external (mag publishes for logs only). Scaffolding (takeoff+IC rig) still uses the compass per user policy. Ablation 3/3 CLEAN (stationary 0.021/0.011 m + moving Linear 0.050 m on the moving platform)."
metadata:
  node_type: memory
  type: feedback
---

**USER POLICY (2026-07-02): compass OK for the pre-descent scaffolding (takeoff + IC
rig); during compass-free VALIDATION the descent must carry no magnetometer influence,
internal (PX4) or external (PLASMC), except logging.**

**Compass-usage map (verified in code):**
- EXTERNAL controller during descent: already compass-free with the default
  `BODY_YAW_SOURCE=alpha` — the SO(3) measured yaw is alpha-derived (−0.949 map), the
  virtual compass `psi_d` LAZY-INITS from that same alpha-derived `yaw_c`
  (controller.py:2172 — NOT the EKF yaw), the V-frame is gravity-leveled (yaw-free),
  and only EKF roll/pitch (gyro+accel) are consumed. Compass enters only via the legacy
  `BODY_YAW_SOURCE=compass` path and the no-feature fallback.
- INTERNAL PX4 during descent: our body-rate+thrust setpoints run through PX4's
  gyro-only rate loop (no mag), BUT EKF2 fuses the magnetometer by default → touches
  the FC quaternion, failsafes/land-detector, velocity estimation.
- PRE-DESCENT (action.takeoff + PositionNedYaw IC rig): PX4 position control on the
  mag-aided EKF — allowed per policy.

**⭐ BAKED DEFAULT-ON (2026-07-02, user):** `COMPASS_FREE_VALIDATE` default flipped 0→1 in
landing_test.py — ALL test configs now validate compass-free (every descent cuts EKF2 mag
fusion at engage). Regression with the new default (no env): CLEAN 0.020 m on the platform,
param readback verified. Set `COMPASS_FREE_VALIDATE=0` for a legacy mag-fused descent.
⚠ Runs BEFORE this bake (speed sweep, turning arms, velff, batch) ran mag-fused internally.

**IMPLEMENTATION `COMPASS_FREE_VALIDATE=1`:**
- `landing_test.py` (at engage, after IC convergence, before warmup): sets
  `EKF2_MAG_TYPE=5` (None) via the new `FC.set_px4_param_int` (MAVSDK param plugin,
  READBACK-VERIFIED → hard-fail if ≠5), and hard-fails if `BODY_YAW_SOURCE=compass`.
  EKF2 accepts the param live in SITL; its yaw gyro-propagates for the short descent.
  Mag keeps publishing → logs only.

**ABLATION RESULT (GT-FB, rover+platform): 3/3 CLEAN with mag fusion verifiably OFF:**
| rep | scenario | lat @min-alt | e_a end |
|-----|----------|--------------|---------|
| 1 | stationary | 0.021 m | −1.3° |
| 2 | stationary | 0.011 m | −1.2° |
| 3 | moving Linear 0.47 m/s (rover drove 5.2 m) | **0.050 m ON the moving platform** | +37.5°* |
*moving rep ran YAW_ALPHA_FILT=0; rover travel heading ~45° — bounded partial alignment,
landing unaffected. → the compass-free claim is now DEMONSTRATED end-to-end, not just
architectural: strong manuscript ablation ("no magnetometer influence anywhere in the
validated landing"). Data test_data/Rover_CompassFree/; harness
test_data/Rover_AB_harness/cfree_reps.sh. See [[feedback_compass_free_yaw_sign]],
[[feedback_rover_yaw_cal_resolved]].
