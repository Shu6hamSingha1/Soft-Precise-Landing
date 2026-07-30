---
name: project_single_marker_default_mismatch_20260728
description: "2026-07-28: chasing an elevated TARGET_LOST/UNKNOWN rate found PLASMC_SINGLE_MARKER had silently defaulted ON for over a month (since 2026-06-24), overriding the nested board's intended bigger-marker-priority handoff with reactive lock mode. Reverting exposed a real, previously-dead-code shape bug (all_pts_1 reshape) that crashed every rep -- fixed both. Commit bf64c08."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-28T11:18:01.231Z
---

**Chase: why is `TARGET_LOST` failure_cause=UNKNOWN so dominant?** Started from the
2026-07-27/28 DESCENT_ANOMALY/dense-recovery work showing an elevated `TARGET_LOST` rate (60%)
with `UNKNOWN` cause dominant (11/15). Traced to `img_data.py`'s `self._single_marker`
(`PLASMC_SINGLE_MARKER`) default.

**Finding 1: accidental month-long default flip.** `self._single_marker` has defaulted to
`"1"` since commit `758dcb2a` (2026-06-24 23:21), a working-tree commit from an unrelated
GT-feedback control-isolation session. That default-on was for a DIFFERENT, never-committed
single-marker world+cal experiment, which its own inline comment says FAILED the IC2-5 gate
(2/12 sub, 6/12 fly) and instructs "Set =0 for the board." Nothing in the standard launch path
(`run_aruco_landing.sh`/`run_ic_validation.sh`) ever sets `PLASMC_SINGLE_MARKER=0` -- so for
over a month, every standard IC1-5 gate/A-B against the nested board (including the go-around,
DESCENT_ANOMALY, and dense-recovery work from 2026-07-26/27/28) silently ran **reactive
single-marker LOCK** (re-lock only once the current marker fully vanishes) instead of the
board's own documented "(default)" behavior: **proactive bigger-marker-priority handoff**
(switch to the small marker as soon as it's better-conditioned, even before the big one is
gone). The CBF small-marker preference, `HANDOVER_LATCHED` terminal commit, and marker-switch
KF-reset machinery are all designed around the proactive handoff.

**Finding 2: the handoff path was silently broken.** Reverting the default to `"0"` (board
mode) crashed literally every rep of the first IC1-5 gate re-run within ~1s of arming:
`Flow Streamer Thread: could not broadcast input array from shape (4,1,2) into shape (4,2)`.
Root cause: `img_data.py`'s decode-correspondence fallback (`all_pts_1[sl] = (...).reshape(-1, 1, 2)`)
assumed `all_pts_1` (from `cv2.calcOpticalFlowPyrLK`) is `(N,1,2)`, but it's actually `(N,2)`
here since `all_pts_0` was built via `np.vstack` of `(4,2)`-shaped marker corners, never
reshaped to `(N,1,2)` before the LK call. This code path is only reachable when
`FLOW_DECODE_CORR=1` (default-on since 2026-07-04) AND `marker_ids is not None` (board mode)
-- both conditions require `_single_marker=False`, so this bug has been DEAD CODE, unexercised,
for the same month the wrong default was live. Fixed the reshape target to `(-1, 2)`.

**Validated (n=25 IC1-5 gate, post-fix):** zero crashes (was 25/25 crashed before the reshape
fix), `TARGET_LOST` rate 60%->44%, IC2/IC3/IC4 showing markedly better precision (IC2 got 2
precise/3 soft incl. two near-perfect 0.03-0.08m landings). Both fixes committed together,
commit `bf64c08`.

**New residual finding (NOT fixed, flagged as follow-up): board mode has no failure-cause
classification.** `getFailureCause()` (`DRIFT_OFF`/`OVERFLOW`/`UNKNOWN`) reads
`self._last_drifted_off`/`self._last_overflow`, which are ONLY computed inside
`if self._single_marker and ...` (the "SINGLE-MARKER visibility-by-MARGIN" block,
`img_data.py` ~line 2295). With `_single_marker` now correctly `False`, this block never runs,
so `_last_drifted_off`/`_last_overflow` stay `False` forever and `getFailureCause()`
**unconditionally returns UNKNOWN** in board mode. Confirmed in the post-fix n=25 gate: all 11
`TARGET_LOST` reps show `failure_cause=UNKNOWN`, zero `DRIFT_OFF`/`OVERFLOW` -- not because the
mechanism improved, but because the classifier itself is inapplicable to board mode as written.
A board-mode-native version of this margin check (per-marker FoV-edge/span check, not gated on
`_single_marker`) would need to be written from scratch -- not simply un-gating the existing
block, since it references `aruco_pts_0` as if there's one active marker's corners, whereas
board mode has multiple markers via `marker_corners_0`/`marker_ids`.

**Broader implication:** every recent bake/A-B this session (go-around removal, DESCENT_ANOMALY
detector, dense-homography-recovery bake-on) was validated under the WRONG (single-marker-lock)
default. None of those conclusions are necessarily wrong -- the mechanisms they addressed
(marker loss handling, smooth-descent classification, partial-occlusion recovery) are largely
orthogonal to primary-marker-selection strategy -- but their absolute numbers were generated
under a config that's no longer the running default, so don't cite those bundles' precise rates
as current-baseline without re-checking. Worth a fresh dense-recovery/DESCENT_ANOMALY sanity
pass under the corrected `_single_marker=0` default if their numbers become load-bearing again.
