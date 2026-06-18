---
name: moment-yaw-canonical
description: "alpha (s[3]) MUST stay on the MOMENT-based SOURCE (geometric-source swap regressed catastrophically, 2026-06-04). BUT the controller's e_a WRAP was corrected 2026-06-07: π-fold→full 2π wrap. Our moment alpha IS a disambiguated 2π DIRECTION (_marker_principal_angle, [4,3,2,1] weights + 1st-moment centroid, 'clean 360°'); the old factor-of-2 π-fold to [-π/2,π/2] re-created a 90° saddle → yaw LIMIT CYCLE. 2π wrap puts the discontinuity at ±180° (unreachable) → no limit cycle. This keeps the moment SOURCE (NOT the reverted geometric swap)."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**User directive (2026-06-04): "We will stick to moment yaw because the control law has been designed for that."**

> **⚠️ CORRECTION (2026-06-07) — the controller's e_a WRAP was π-period, but the alpha is genuinely 2π.**
> The yaw error wrap at `controller.py:703` was `e_a = arctan2(sin(2·e_raw), cos(2·e_raw))/2` (fold to
> [-π/2,π/2]) on the stale belief "alpha is π-period (2nd-moment is 180°-symmetric)". That's true for the
> *plain 2nd-moment AXIS* but FALSE for our actual alpha: `_marker_principal_angle` DISAMBIGUATES it to a
> full **2π DIRECTION** ([4,3,2,1] weights + 1st-moment centroid displacement, "clean 360°"). The π-fold
> re-collapsed that → re-created the **90° saddle** (two equilibria 0°/180°, unstable at ±π/2) → noise
> flipped e_a across ±π/2 → yaw-rate command flip → **LIMIT CYCLE** (the very chatter `PLASMC_TAU_UA` was
> added to damp). **FIX: full 2π wrap `e_a = arctan2(sin(e_raw), cos(e_raw))` → [-π,π]; only discontinuity
> at ±180° (a landing never gets there) → limit cycle STRUCTURALLY GONE.** This keeps the moment SOURCE —
> it is NOT the reverted geometric-source swap (that changed the alpha definition + sign/offset/2π and fed
> +180° runaway; this only changes the error wrap). Validate IC1 + IC2-5; watch the 1st-moment disambiguation
> near touchdown (centroid displacement shrinks as the marker shrinks). Below "Period pi" line is now stale.

**The rule:** alpha = `s[3]` (img_data.py `_getImgFeatures`) is computed as the **2nd-moment principal axis** of
the weighted marker corners: `alpha = 0.5*arctan2(2*mu11, mu20-mu02) - alpha_0`, with per-corner weights
`[4,3,2,1]` (TL,TR,BR,BL) and `alpha_0 = -0.9379` (compass mode; 0 in V_YAW_SOURCE=alpha mode). Period **pi**,
not 2pi. These specifics (period, weights, -0.9379 offset, sign) are **LOAD-BEARING** — the yaw SMC was tuned to
them. **Keep this definition. Do NOT swap to the geometric "marker +x edge direction" form.**

**Why:** the geometric alpha (full 2pi, offset 0 — `edge = (TR-TL)+(BR-BL); alpha = arctan2(edge_y,edge_x)`) was
introduced 2026-06-04 to remove the touchdown "flips" and looked clean OFFLINE (0 flips, tracks GT yaw r=1.00).
But the IC1+IC4 landing validation was catastrophic: desired-yaw EA_d[2] maxed to ~180 deg on EVERY rep
(pre-fix 112-126), xy 2.2-10.4 m (pre-fix 0.74-2.45), even centered IC1 0.4->2.8 m. The 2pi/sign/offset mismatch
fed the SMC positive feedback -> runaway to the +-180 deg equilibrium -> lateral coupling via the PX4 mixer.
Reverted in commit 0008ba1. Full detail + numbers in [[yaw-calibration-pending]].

**How to apply:**
- The single-marker-fallback "flips" (alpha jumps at marker dropout, the board-homography path uses a 2pi
  geometric angle while the moment fallback is pi-periodic) are a KNOWN cosmetic artifact the controller
  TOLERATES — do not "fix" them by changing the alpha definition.
- If the flips ever genuinely need fixing, the ONLY safe path is to **re-derive/re-tune the yaw SMC for the new
  alpha convention first**, then IC1 + IC2-5 validate before merge — not a drop-in definition swap.
- Same class of mistake as [[feedback_image_center_bug]]: a "geometrically correct" perception change that broke
  the runtime and was reverted. IC-validate any control-path perception change before trusting an offline check.
- Analysis findings about alpha are independent of this and still hold: alpha tracks GT yaw r=1.00,
  sensor_cal_s[3]=0.994 (~1.0), wx/wy unobservable (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]]), alpha drift-free under EKF drift.
