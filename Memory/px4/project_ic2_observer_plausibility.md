---
name: project_ic2_observer_plausibility
description: "2026-07-24: third raw-decode plausibility gate added (commit c97202d), for the centroid-rate observer's aruco_pts_0 (independent of the two gates added earlier the same session, 9075a97/9f0c490) -- traced IC2_rep3's theta explosion (2->3080) + alternating h_x (-161/+130 frame-to-frame) during a real attitude tumble. Also: why IC2 still can't hit SOFT+PRECISE even after all three fixes -- two separate PRE-EXISTING issues (terminal close-range marker-loss forcing a blind open-loop touchdown; baseline lateral imprecision on non-loss attempts), neither caused by or fixed by this session's perception work."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-24T12:16:38.562Z
---

**Context.** Same session as [[project_ic1_ds_guard_gap_reset]] and
[[project_ic2_ic5_20260723_investigation]]. After fixing the raw-decode plausibility gap on
`aruco_pts_1` (the LK-correspondence path), a fresh IC2_rep3 trace (ICValidation/20260723-191943)
still showed `theta` exploding 2->3080 and `h_x` alternating roughly -161/+130 frame-to-frame
during a ~1.3s total marker-loss window, even though `MARKER_EXTENT_PX`/`_feature_pts` correctly
FROZE during that same window (confirming the LK-gated path itself was behaving correctly).

**Root cause: a THIRD, independent raw-decode consumer with zero validation.** The
centroid-rate observer (`PLASMC_SINGLE_MARKER`+`PLASMC_CENTROID_RATE`, BOTH DEFAULT-ON since
2026-07-03 -- a stale docstring comment nearby still said "default-off", fixed) reads
`aruco_pts_0` -- a raw per-frame ArUco decode with explicitly NO LK-correspondence requirement
(its whole design purpose is to survive `nfc=0`). It recomputes `_x0,_y0` (V-frame centroid) and
`_Mo` (corner-spread/scale) FROM SCRATCH every single frame directly off whatever `aruco_pts_0`
holds -- no comparison to any previous state, no plausibility check. During the 1.3s blackout, a
real attitude tumble (confirmed via Quat, ~90 deg of real rotation) meant the tumbling camera
swept across background clutter, producing spurious/false ArUco decodes at different locations
each frame; each fed straight into `_obs_vel_kf` (the observer's own small velocity KF), which
then output wildly different, alternating velocity estimates -- directly explaining both the
theta explosion and the h_x alternation.

**The h_x,h_y formula itself was independently verified CORRECT** (re-derived from first
principles: `Ṗ = -v - w×P` rigid-body kinematics -> standard interaction matrix -> cross-checked
term-by-term against this codebase's own `_fill_A`, including its documented/empirically-validated
w_z sign flip (`_oz = -_wv[2]`, corr -0.91 with body yaw) and the loom's sign (`_loom_dec =
-0.5 d(lnM)/dt` independently derives to exactly `_fill_A`'s h_z convention, no extra flip needed).
No double-counting, no formula bug -- purely garbage-in from the ungated `aruco_pts_0`.

**Fix (commit c97202d):** gate `aruco_pts_0` through `_planarMapPredictionPlausible` (the same
check used for the two earlier gates) before the observer trusts it. `DECODE_PLAUS_DBG=1` prints
rejections from all three gates now (aruco_pts_1's LK path, aruco_pts_0's observer path, and the
original map override/rescue path).

**Validated:** n=5 IC2 (ICValidation/20260724-164522) -- 5/5 landed, zero fly-aways (max xy
2.72m). n=3 IC1/IC3/IC4 regression clean, IC3 and IC4 each landed SOFT+PRECISE.

**Follow-up: WHY IC2 still can't reliably hit SOFT+PRECISE even with all three fixes.** Traced
`IC2_rep2`'s recorded (3rd retry-attempt) result: classified `TARGET_LOST [UNKNOWN]` despite
excellent final numbers (xy_err=0.071m, rel_vel=0.053m/s) -- per `apps/landing_test.py`'s rule,
ANY mid-flight target-lost event disqualifies the landing regardless of touchdown quality.
GT altitude confirms: marker lost continuously from z=1.402m to z=0.23m (t=41.35-42.69s) -- the
ENTIRE final ~1.2m of descent happened blind (open-loop `thrust=0.65` fallback), landing safely
more by luck than control. `MARKER_EXTENT_PX` grew smoothly right up to the loss (no discontinuous
jump) -- this is NOT the spurious-decode bug class fixed above, it's the pre-existing, already-
catalogued "Marker-detection breakdown at z<0.3m" failure mode (tuning-skill mode #2), just
triggering higher (z~1.4m) and outlasting the KLT-fallback's bridging window + the 1.0s grace
timer. SEPARATELY, the same rep's first two retry attempts (before the one that got recorded)
never got close to SP even with NO marker loss at all: attempt1 FAIL (xy=1.157m, vel=1.4m/s),
attempt2 SOFT-only (xy=2.613m, vel=0.162m/s) -- matching the long-documented IC2 lateral-gain/
kappa-ratchet family (out of scope, extensive prior dead-end history, see `feedback_kappa_clamp_bandaid`,
`feedback_lateral_kappa_runaway`). **Conclusion: today's perception fixes did their job (fly-aways
0/5 across two separate IC2 n=5 validations) but were never what was blocking IC2's SP rate --
that's two separate, pre-existing, already-catalogued issues (terminal marker-loss timing;
baseline lateral control precision) outside this investigation's scope.**
