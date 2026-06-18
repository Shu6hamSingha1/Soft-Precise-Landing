---
name: compass-free-yaw-sign
description: "The compass-free SO(3) (BODY_YAW_SOURCE=alpha) yaw_alpha->R replacement REQUIRES a NEGATIVE alpha->yaw slope (BODY_YAW_ALPHA_K=-0.949), because the euler[2] it replaces is NED yaw which is ANTI-correlated with alpha (alpha≈+0.949·GT_yaw_ENU). The +0.949 sign caused psi_d windup -> ±135° yaw ring-out -> TARGET_LOST. Flipping to -0.949 + a psi_d-rate clamp eliminated the yaw divergence: first non-divergent compass-free landing (2026-06-04)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**The fix that made the compass-free yaw loop stable (2026-06-04).** Two coupled bugs in the
`BODY_YAW_SOURCE=alpha` path (controller.py `_attCtrl`, which rebuilds R with EKF roll/pitch +
alpha yaw so the SO(3) error is compass-free):

1. **SIGN (the dominant bug):** `yaw_alpha = K·s[3]+b` must use **K = -0.949**, NOT +0.949. The
   value was verified against **GT (ENU) yaw** (alpha ≈ +0.949·GT_yaw_ENU, r=0.98), but the compass
   yaw it REPLACES is `euler[2]` from the FC quat = **NED**, which is ANTI-correlated with ENU
   (the -0.99 ENU/NED relation). The working compass loop had `d(alpha)/d(euler[2]) < 0` (yaw_c
   moved WITH psi_d as alpha fell). The +0.949 sign gave `d(alpha)/d(yaw_alpha) > 0` -> yaw_alpha
   and psi_d moved OPPOSITE as alpha fell -> e_R grew -> windup. Offset `b` is irrelevant (cancels
   in e_R = psi_d - yaw_alpha and only sets an arbitrary inertial-yaw reference).
2. **psi_d windup vs the rate clamp:** psi_d integrates u_a (reaches ~2.3 rad/s) but the body-rate
   is clamped at W_U_MAX=1.0 -> psi_d races ahead -> e_R saturates. Added a psi_d-advance rate limit
   `PLASMC_YAW_PSID_RATE=0.7` (× W_U_MAX) in `_yawCtrl` so the virtual compass can't outrun the
   inner loop. (Secondary; the sign was the main fix.)

**Result (n=1, retry-wrapped, IC1):** alpha range [-216,132]°→[-18,20]°; psi_d windup 128°→±6°;
w_u_z saturation 73%→2%; outcome TARGET_LOST(25m)→FAIL(xy 1.57m). **The yaw no longer diverges** —
drone lands on the marker. First non-divergent compass-free landing.

**Diagnostic signature of yaw windup (for future):** w_u_z saturated >30% of frames + psi_d winding
past ±90° while |e_a| stays bounded + alpha sign-flips. Check the yaw_alpha sign FIRST.

**Still open (not yet soft-precise):** (a) residual yaw limit-cycle ±18° (tighten yaw-SMC gain /
psi_d clamp); (b) lateral s_e_n is stable in-flight (0.06-0.32) but spikes at the touchdown frame
(2.6) — likely terminal artifact, watch the cal cross-contamination [[wxy-unobservable-imu-fusion-deferred]];
(c) xy 1.57m -> needs precision tuning. Needs n>=5 to confirm consistency. See [[yaw-compass-drift-ic-start]],
[[moment-yaw-canonical]]. DES_ALPHA=0 constant = board square at touchdown (user requirement); the
pre-takeoff board alpha is ~0° (board square at spawn).
