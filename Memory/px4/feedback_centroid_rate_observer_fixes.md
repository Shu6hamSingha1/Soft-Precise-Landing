---
name: feedback_centroid_rate_observer_fixes
description: "Four validated 2026-07-04 fixes to the default-on centroid-rate observer (frame-pair, lstsq consolidation, w_z sign, KF q) — off-center velocity now tracks GT ~0.85-0.94"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

Four corrections to the default-on centroid-rate observer (`img_data.py`), committed 2026-07-04
(3cc7b0b + ede3058), each validated by **sim-time-aligned perception-vs-GT off-center comparison**
(IC2 perception x6; `tools/perc_diag_aligned.py` methodology — align by `Start Time + GT Time` vs
`Img Time`, perception-driven only).

1. **FRAME-PAIR FIX.** The observer leveled frame-0 corners (`aruco_pts_0`) with the frame-1
   attitude (`quats[1]`), contradicting the code's own convention (`V_aruco_norm`/`V_flow_norm`
   at img_data ~1413-1414 pair `_0 ↔ quats[0]`). The ~1-frame attitude mismatch left a residual
   tilt ∝ angular rate (worst banking) = a source of the off-center yaw leak. Fixed to
   `quats[0]`/`angvels[0]`; guard null-checks `quats[0]`. Result: low-v `h_x` 0.41→0.73.

2. **LSTSQ CONSOLIDATION** (`_obs_active`): when the observer is active its centroid-rate defines
   `h_xy`, moment loom defines `h_z`, gyro defines `w_z` → the σ_min corner lstsq is redundant
   (only `w_z` survived it). Skip the degenerate 6-DOF solve. Keeps flow alive where the lstsq
   collapsed (|GTh| 0.5-0.8: dead 0.00 → 0.53).

3. **W_Z SIGN FIX** (the biggest). `_fill_A`'s ω_z is the interaction-matrix z-rate =
   **−body_yaw_rate** (validated 06-25, corr −0.91 w/ body yaw — see [[feedback_gtfb_wz_sign_bug]]),
   but `_vframe_w` returns `_wv[2] = +body_yaw_rate`. So ω_z = −_wv[2]. The observer used +_wv[2]
   → sign-flipped yaw coupling: **harmless on h_x** (small `y0·w_z` when drifting in x) but
   **ANTI-correlated h_y** (large `x0·w_z`). Fix `_oz = −_wv[2]` in both couplings + the output w_z.
   Result: off-center `h_y` (|GTh_y| 0.2-0.4) ratio 0.18→0.89, corr −0.18→**+0.54**; the −0.82
   anti-corr at 0.4-0.8 eliminated. NOTE: the lstsq h_y was NEVER affected (it never uses the gyro);
   the bug reached the controller only via the observer's `V_v[0:2]` override.

4. **KF q 1e-4→1e-3** (`CENTROID_RATE_KF_Q`, ede3058). The CV-Kalman on the centroid over-smoothed
   (q/r=0.1 was the extreme outlier vs every other KF at q/r≥50) → low-velocity attenuation.
   Offline q-sweep + live A/B: `h_x` 0.66→0.90 (mid), 0.81→0.94 (low), no landing regression. The
   old low q was tuned for the PRE-FIX noisy observer; the corrected observer affords more gain.

**Why:** these are the velocity signal the whole landing depends on; off-center it was
dead/anti-correlated/0.3-0.6, now ~0.85-0.94.
**How to apply:** the observer is sound now — do NOT re-tune these blind. Residual limits are
ORTHOGONAL: (a) low-v KF attenuation floor, (b) the terminal marker-overflow death
([[feedback_terminal_overflow_deck_flyaway]]). KF audit: the observer was the ONLY over-smoothed
KF; V_ds q=1 is low for a different (severity) reason [[feedback_vds_kf_q_severity_bandaid]].
