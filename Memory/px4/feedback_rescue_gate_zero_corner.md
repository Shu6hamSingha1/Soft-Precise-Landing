---
name: feedback_rescue_gate_zero_corner
description: "2026-07-25: PlanarFeatureMap rescue gate was structurally blocked during a total zero-corner marker dropout because RESCUE_GATE_MARKER_AWARE=1 (default) gates on self.confidence, which the _rigid_fail_streak persistence decay collapses to a hard 0.0 after just 3 consecutive zero-corner frames, even while map_confidence (marker-independent) stayed healthy. Fixed (commit 2a5c21f) by adding PlanarFeatureMap.primary_zero_corners, letting the rescue gate fall back to map_confidence specifically in the genuinely-zero-corner case while a deforming-but-present marker (1-4 corners) still requires the marker-aware confidence."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-25T10:05:18.999Z
---

**The tension (three iterations on the same gate, now resolved):**
1. Original: rescue gated on `map_confidence` (marker-independent) — robust to occlusion, but
   BLIND to a marker whose corners are present but the shape is deforming (foreshortening at a
   tilt-grazing terminal) — confident-wrong extrapolation, fixed 2026-07-19 by switching to
   `self.confidence` (marker-aware, folds in `marker_rigid_ok`).
2. That fix (`RESCUE_GATE_MARKER_AWARE=1`, default-on) correctly caught the foreshortening case,
   but `marker_rigid_ok` is forced `False` on ANY zero-corner frame too (a totally different,
   occlusion-class scenario), and its `_rigid_fail_streak` persistence decay (added 2026-07-17,
   `PLANAR_MAP_RIGID_FAIL_STREAK_MAX=3`) collapses `confidence` to a hard, PERMANENT 0.0 after just
   3 consecutive zero-corner frames (~50-100ms) — for the ENTIRE remaining outage, however long.
   Traced live (IC1_rep3/IC3_rep2/IC4_rep2, ICValidation/20260724-172603): a genuine >1s total
   decode/tracking dropout (attitude flat, no tumble, extent smoothly growing beforehand — not a
   deforming marker) pinned `confidence` at exactly 0.0 the whole time while `map_confidence`
   stayed healthy (0.54-0.58, above the 0.5 floor) throughout — the rescue's 5-frame streak
   requirement never got a chance to build, `RESCUE_ACTIVE` never fired, and the outage outlasted
   `MARKER_LOSS_GRACE` (1.0s) -> `TARGET_LOST` (tagged `UNKNOWN` by `getFailureCause()`, since
   `_last_drifted_off`/`_last_overflow` also require a raw decode to update and hadn't fired either)
   despite the map's underlying scene/homography track being fine the whole time.
3. **Fix (commit 2a5c21f):** `PlanarFeatureMap.primary_zero_corners` — set precisely when the
   PRIMARY slot has 0-of-4 corners survive KLT THIS frame (planar_map.py's `update()`, the exact
   branch already distinguishing 0/1-3/4 corners for `rigid_ok`/`shape_change` handling). The
   rescue gate (img_data.py ~2156) now uses `map_confidence` specifically when
   `primary_zero_corners` is True; a marker with 1-4 corners present (real shape data, possibly
   deforming) still gates on the marker-aware `self.confidence`, preserving the 07-19 fix for its
   actual target. The OVERRIDE gate (replacing an already-successful decode) is untouched — stays
   strictly marker-aware always, as before (a different, correctly-conservative consumer).

**Validated:** n=5 IC1/IC3/IC4 + n=5 IC2/IC5 (same session), no new TARGET_LOST/fly-away pattern
attributable to the fix. IC2: zero fly-aways, mean xy 0.29m (vs the session's earlier 16.6m/2
catastrophic fly-aways before this session's three garbage-in fixes), 1 SOFT+PRECISE. One IC3 rep
(27.4m) investigated in depth via the newly-added `primary_zero_corners` shadow-log field's sibling
evidence (`err_px` escalating smoothly to 13-16px while `fresh_decode` stayed True = corners were
NOT zero) and traced to a genuine shape-deformation event (the exact case #1's fix targets) plus
the already-documented IC3 offset/CBF-cone-clamp family (`w_u` pinned from t=0.03s) — unrelated to
this change; `primary_zero_corners` is now logged going forward for direct verification if a
similar case recurs.

**Same family as this session's other three fixes** ([[project_ic1_ds_guard_gap_reset]],
[[project_ic2_ic5_20260723_investigation]], [[project_ic2_observer_plausibility]]) but a distinct
bug class: those were "garbage got THROUGH a gate that should have rejected it"; this is "good
data got BLOCKED by a gate that couldn't tell why confidence was low." Both are instances of a
recurring meta-lesson for this project: a single scalar confidence/gate threshold conflates
multiple distinct failure causes that need different responses — decomposing WHY a signal is bad
(zero information vs. wrong information) is usually the actual fix, not raising or lowering the
threshold.
