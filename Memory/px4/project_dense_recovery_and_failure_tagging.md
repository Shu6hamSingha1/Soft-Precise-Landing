---
name: project_dense_recovery_and_failure_tagging
description: "Dense-homography-recovery (RANSAC, unified staleness gate) implemented for centroid fallback -- results MIXED, do NOT bake as default. New auto-tagging tool (getFailureCause -> DRIFT_OFF/OVERFLOW/UNKNOWN) added to TARGET_LOST classification."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

**Dense-homography-recovery** (img_data.py `_dense_recover_*`, `PLASMC_DENSE_RECOVER`, default OFF):
when the strict all-4-primary-corner gate fails, recovers the full quad via a RANSAC homography fit
from the marker's dense CANONICAL point layout (~180 pts, deterministic scaled-quad, NOT GFT -- GFT
regressed for single-marker mode, only ~4-8 resolvable corners starve the point count vs 180 scaled-
quad points, point-count beats per-point quality here) to their currently-tracked positions, then maps
the canonical 4 corners through it. No depth/scale needed (pure 2D homography, planarity-only
assumption) -- consistent with the scale-free/depth-free hard constraint.

**UNIFIED STALENESS GATE (2026-07-07)**: originally the 4-corner KLT fallback (hard-capped
`MARKER_KLT_MAX_STEPS=20`) and dense-recovery (originally UNCAPPED) were disjoint, uncoordinated
LK-chain trackers -- structurally wrong (same drift-accumulation risk, arbitrarily different trust
bounds). Fixed: (1) short corner-fallback successes (`_lk_step_count <= DENSE_SOFT_ANCHOR_MAX_STEPS`,
default 2) now ALSO soft-re-anchor the dense canonical state; (2) dense-recovery gets a principled cap
(`DENSE_RECOVER_MAX_FRAMES=60` frames-since-anchor, `DENSE_RECOVER_MIN_INLIER_FRAC=0.5` RANSAC
inlier ratio) instead of running unbounded.

**VALIDATION STATUS: MIXED, NOT clean -- do not bake as default yet.** Original densedbg batch
(unbounded): recovery ran 30+ CONTINUOUS seconds in 2 reps (rep3: 2.69m miss; rep5: fly-away). Root
cause of the long span: NOT dense-recovery's own fault -- it was chasing an ALREADY-DIVERGED
trajectory (see feedback_terminal_overflow_deck_flyaway CORRECTION below). Unified-gate retest
(densedbg2, combined with h-extrap baked mid-batch -- CONFOUNDED): rep5 produced a 23.93m fly-away,
the worst single outlier this session (now separately traced to `_savgol_predict`, see
[[feedback_savgol_predict_suspect_flyaway]], NOT dense-recovery itself). Net: dense-recovery's
mechanism is sound in principle (works well over short windows, e.g. rep4 in densedbg: 6s/112 firings,
GT precise 0.070m) but has NOT been cleanly validated in isolation with the unified gate. Needs a
dedicated A/B before considering default-on.

**CORRECTION to the standing "terminal deck-overflow" root-cause claim
([[feedback_terminal_overflow_deck_flyaway]]):** that memory is NOT universal. Verified case
(densedbg rep5, "Tue Jul 7 22-36-17 2026"): the `Ncorn->0` trigger happened at GT altitude ~2.5m
(NOT near-deck) with lateral error ~2.5m (bearing ~45 deg) -- this is DRIFT-OFF (marker left the FoV
edge, off-center), not OVERFLOW (marker spanning/too-close). Always verify altitude at the Ncorn->0
transition before attributing a failure to "terminal overflow" -- both mechanisms are real and
DISTINCT, and which one applies must be checked case-by-case, not assumed.

**NEW TOOLING (2026-07-07): auto failure-cause tagging.** `img_data.py`: `self._last_overflow`
(companion to existing `_last_drifted_off`), both now logged per-frame ("Drift Off"/"Overflow" fields
in Img_Data.npy) and exposed live via `getFailureCause()` -> "DRIFT_OFF"/"OVERFLOW"/"UNKNOWN".
`apps/landing_test.py`: captured at the TARGET_LOST trigger point, folded into the classification tag
(`TARGET_LOST [DRIFT_OFF]` etc.) and the `SOFT_PRECISE` dict (`failure_cause` key). Use this to
short-circuit future root-cause digging -- check the printed tag before manually tracing altitude.

**Also confirmed this session (unrelated to any code change here): a recurring SIGSEGV (rc=139) SITL
crash** appeared across 3+ UNRELATED configs (clamp_off test, hextrap rep5, audrift rep1) -- a
pre-existing Gazebo/PX4 infra flake, not caused by any session code change. Don't attribute a crashed
(no-recording) rep to whatever feature was being tested; check the log tail for `rc=139` first.
