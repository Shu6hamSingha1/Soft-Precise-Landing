---
name: project_ic2_ic5_20260723_investigation
description: "2026-07-23 in-depth IC2/IC5 fly-away investigation: IC2's catastrophic fly-aways (41.7m/8.1m) traced to a spurious-but-cleanly-tracked wrong ArUco decode after a 1-frame marker-loss gap, fixed by extending the existing map-path plausibility gate to raw decode too (n=5 IC2 clean after fix); IC5's short flights are its own initial-condition geometry (known short-runway canary), not a perception bug -- left untouched."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-23T15:17:36.311Z
---

**Context.** Same session as [[project_ic1_ds_guard_gap_reset]] (the `_s_prev` marker-loss
gap-reset fix, commit 9075a97). After that fix, a follow-up IC1-5 gate (n=3 IC2-5) surfaced
two more IC2 fly-aways (41.7m, 8.1m) and one IC5 non-landing -- this memory covers tracing
and resolving those.

**IC2_rep3 (8.1m, pre-existing, NOT investigated further):** onset t=0.00s, GT lateral
2.88m == IC2's own initial 2m,2m ENU offset. `diagnose_failure_cause.py` calls this
CONTROL-led (kappa_y ratchets to 19+, `KAPPA_MAX_XY` is uncapped at 1e6 by design --
`controller.py::_kappa_max = pa("KAPPA_MAX", 1e6, 1e6, 3.0)`, only Z is capped). This is
the long-documented lateral kappa-ratchet family ([[feedback_kappa_clamp_bandaid]],
[[feedback_lateral_kappa_runaway]]) with an extensive dead-end history (capping kappa_xy
tested before and found to trade one failure for another) -- NOT re-attempted this session,
out of scope.

**IC2_rep2 (41.7m, THE NEW FINDING, FIXED):** traced end-to-end via Control_Data.npy +
Img_Data.npy (same absolute-clock alignment between the two files, confirmed empirically --
no offset needed within one flight's data). At t~44.4s: a single-frame marker-loss gap
(`N Flow Corners`==0 for exactly ONE frame) was immediately followed by a DIFFERENT,
spurious 4-corner ArUco detection -- a ~9px marker near the top-left of the frame vs the
true ~150px marker mid-frame -- that then decoded and LK-tracked CLEANLY (status==1 all 4
corners) for several subsequent frames, drifting smoothly (44->165px over ~0.2s, i.e.
self-consistent frame-to-frame). Because each frame was smooth relative to the PREVIOUS
(wrong) frame, the existing `ds` outlier-hold guard (`FLOW_DS_MAX=0.15`, single-frame delta
check, `_s_prev`/`_s_hold`) only rejected the FIRST bad frame and then admitted the rest --
its blind spot is structurally different from (though related to) the gap-reset bug: it
can't distinguish "smoothly-tracked-but-wrong" from "smoothly-tracked-and-right". Fed
straight into `s`/`sigma`, kappa_y ratcheted 0.04->3.8+ within ~0.5s, `a_u` detonated
(peak 1129).

**Root cause:** raw ArUco decode success (`FEATURE_DATA_IS_LOGGED=True`, all 4 LK corners
tracked) was trusted UNCONDITIONALLY -- zero sanity check against the last known-good
position/size. The exact same class of check (`_planarMapPredictionPlausible`: FoV-position
bound + size-ratio bound vs `_last_real_extent_px`) already existed and was validated for
the PlanarFeatureMap override/rescue paths ([[feedback_planar_map_plausibility_gate]]) but
was never extended to the raw-decode path itself -- an oversight, not a design choice (raw
decode was implicitly assumed more trustworthy than the map, which is usually true but not
when ArUco false-positives on the wrong pattern).

**Fix (commit 9f0c490, img_data.py ~line 2384):** gate the raw decode through
`_planarMapPredictionPlausible(aruco_pts_1, quats[1])`, checked BEFORE `_last_real_extent_px`
is overwritten (so it compares against the pre-event known-good state, not the bad frame's
own size). An implausible raw decode is treated as a decode miss this frame -- falls through
to the existing rescue/hold machinery via `FEATURE_DATA_IS_LOGGED` staying False, same
"reject, never clip" discipline as the map path. `DECODE_PLAUS_DBG=1` env var added for
future debugging.

**Validation:** n=5 IC2 post-fix: 5/5 landed, ZERO fly-aways (mean xy 0.388m, max 0.635m --
vs pre-fix mean 16.6m/max 41.7m with 2/3 catastrophic). n=3 IC1/IC3/IC4 regression-checked
clean (no fly-aways, comparable to prior baselines).

**IC5 (untouched, characterized as pre-existing/separate):** both n=3 reps that landed
(rep1 0.99m, rep2 1.11m) and the flake (rep3, SITL "Image resolution not received yet"
before controller engagement -- unrelated infra flake) show onset at t~0 with GT lateral
2.6-2.83m -- i.e. IC5's OWN initial condition (ENU 2,2,3 -> ~2.83m lateral offset at only
3m altitude) leaves minimal runway before `TARGET_LOST` at 3-3.4s flight time. Matches the
pre-existing "IC5 = known short-runway canary" characterization already in
`px4/MEMORY.md`'s failure-mode table (#1 in the historical taxonomy). This is a
control-authority/timing constraint, not a perception bug -- explicitly NOT chased this
session.

**Methodology note that paid off:** the ds outlier-hold's single-frame delta check and the
new raw-decode plausibility check are COMPLEMENTARY, not redundant -- the former catches a
single discontinuous glitch that self-corrects immediately, the latter catches a WRONG value
that is internally smooth/self-consistent across multiple frames (which the former
structurally cannot, no matter how it's tuned, since it only ever compares against the
immediately-preceding raw sample). Any future "smoothly-tracked-but-wrong" symptom should
check the plausibility-gate coverage first, not retune the delta threshold.
