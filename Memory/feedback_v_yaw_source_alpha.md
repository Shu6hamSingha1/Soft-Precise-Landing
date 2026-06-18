---
name: v-yaw-source-alpha
description: "REMOVED/INCORRECT (2026-06-04): V_YAW_SOURCE=alpha (marker-align the V frame) is the WRONG knob — rotating V by -alpha forces the yaw feature s[3]->0 by construction, zeroing the yaw-SMC error -> OPEN-LOOP yaw -> board orientation uncontrolled at touchdown. The whole V_YAW_SOURCE knob was deleted from img_data.py. Compass-freeness belongs on the CONTROL side (BODY_YAW_SOURCE=alpha, now default), not the perception frame."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**SUPERSEDED — `V_YAW_SOURCE=alpha` was REMOVED from `img_data.py` on 2026-06-04 (user
decision); it is INCORRECT and not practically possible.** The original entry below claimed it
was "the right architecture for IBVS landing." That was wrong.

**Why it's wrong (verified in code 2026-06-04):** `V_YAW_SOURCE=alpha` computed alpha from the
current ArUco corners, then rotated EVERY V point (aruco, flow, board) by `-alpha` so the marker
lies along V.x. Then `_getImgFeatures` re-measured alpha on the rotated points with `alpha_0=0` —
so by construction **`s[3] (alpha) ≈ 0 always`**. The yaw SMC error is `e_a = s[3]-s_d[3] ≈ 0` →
the yaw loop is **open**: the controller can't drive the drone to face the marker, so the board
orientation at touchdown (the user's "board square at 0°" requirement) is **uncontrolled**. It is
also self-referential (alpha, an OUTPUT, used to define the frame alpha is measured in) and folds
the relative-yaw-rate (wz) out of the optic flow.

**The premise was also wrong.** "Compass drift propagates into the flow with compass-V" is false:
`_getVirtualPts` builds V from `g = R.T@[0,0,1]` (gravity = roll/pitch, yaw-invariant) + the
body-intrinsic camera-y axis. **No yaw NUMBER enters the feature measurement** — V is
gravity-leveled and body-RELATIVE, so it is already compass-DRIFT-free (alpha tracks GT yaw
r=1.00). Yaw is the alpha **OUTPUT**, not an input.

**The correct architecture:** keep the perception as-is (compass-V, which preserves the honest
yaw feature s[3]); achieve compass-freeness on the **CONTROL** side via `BODY_YAW_SOURCE=alpha`
(now the default, [[compass-free-yaw-sign]]) — the SO(3) attitude loop uses the drift-free s[3]
instead of the drifting euler[2], while the perception keeps measuring s[3] truthfully. No
sensor-cal redo is needed (the cal stays in its compass-V frame). The "V_YAW_SOURCE=alpha cal
redo" follow-up is DROPPED. See [[compass-yaw-drift]], [[moment-yaw-canonical]].
