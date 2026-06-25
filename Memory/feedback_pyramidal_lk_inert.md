---
name: feedback_pyramidal_lk_inert
description: "⭐ NEGATIVE RESULT (2026-06-19): pyramidal LK levels are INERT — raising maxLevel 3->4->5 (and winSize 21->51) changes the corner optical flow by ~0 in every GT-speed bin (n=2 fly-away recordings). When LK tracks, raw lateral flow is PROPORTIONAL to GT (ratio ~1.0-1.3) up to 3 m/s. The lateral-wall 's_dot under-report ~0.5x' is an AVAILABILITY artifact (only ~46-63% of frames yield a flow; controller holds stale flow on dropouts), NOT raw-LK dynamic-range saturation. So pyramidal LK is NOT the lever; the real lever is flow AVAILABILITY / decode-track robustness."
metadata:
  node_type: feedback
  type: feedback
---

> ⛔ SUPERSEDED/CORRECTED 2026-06-26: The pyramidal-LK-inert negative result stands, but 'real lever = flow AVAILABILITY / decode-track robustness' is obsolete — the real lever was control (combined surface + lateral velocity damping). The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/inner-loop-velocity limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on and gives 10/10 bounded landings. The residual is a terminal SOFT velocity kick (≈38ms lag), not a precision wall. See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

**Pyramidal LK is NOT the perception lever for the lateral wall.** Tested directly
(2026-06-19) with a new GT-dynamic-range harness `tools/tune_lk_dynamic_range.py`
on two IC2 fly-away recordings (`IMG_RECORD_RAW=1`, peak GT lateral speed ~3 m/s,
the high-velocity overshoot regime). The harness faithfully ports the live corner
pipeline (ArUco decode -> 4 corners + goodFeaturesToTrack -> `calcOpticalFlowPyrLK`
-> `_getVirtualPts` V-frame -> `_fill_A` -> lstsq -> raw `V_v[:2]`) and bins the raw
lateral-flow / GT ratio by GT speed.

**Findings (both reps, robust):**
- **`maxLevel` 3/4/5 = IDENTICAL to 2 decimals in EVERY speed bin.** `winSize`
  21->31->41->51 barely moves the ratio. If pyramid depth/search-window did
  anything for dynamic range, it would show *somewhere*; it shows nothing.
- **When LK tracks, raw flow is PROPORTIONAL to GT:** ratio ~0.98-1.01 in v[1-2 m/s),
  ~1.1-1.5 in v[2-4 m/s). NOT the ~0.5x under-report. (The v[0-1) bin shows 5-8x =
  divide-by-near-zero GT noise, ignore.)
- **The deficit is AVAILABILITY, not magnitude:** only ~46-63% of frames yield a
  valid flow. Track-rate collapses at CLOSE RANGE (90% @ alt 5 m -> 22-41% @ alt
  ~1.7 m) — the known close-range corner/decode breakdown
  ([[feedback_descent_perception_ceiling]], [[feedback_terminal_descent_loom_overreport]],
  [[feedback_marker_detection_stale]]), not a high-velocity LK saturation at altitude.
  The >4 m/s frames are simply UNtracked (marker out of FoV = geometry).

**Reconciles the combined-barrier 0.5x:** the controller's logged `s_dot`/`h` averages
over tracked AND dropped frames; on a dropout it holds the last (stale, smaller) flow,
so the time-averaged magnitude reads ~0.5x in a fast-changing regime. That is an
availability artifact, not raw-LK pyramid saturation. See [[feedback_combined_surface_divergence]].

**Falsifies** the `PERCEPTION_FLOW_FINDINGS.md` "one evidence-supported lever =
pyramidal LK levels 2->3" claim *for levels >= 3* (3 was already the default; going
higher is inert). **Real lever = flow AVAILABILITY / decode-track robustness**
(KLT corner-track persistence through decode-fail, relax the strict 4-corner primary
gate, or marker/board design) — consistent with [[feedback_lateral_overshoot_root]]'s
"inner-loop velocity observability" but the fix is keeping the marker DECODED, not a
fancier flow estimator. Don't spend SITL on `FLOW_LK_LVL`/`FLOW_LK_WIN` sweeps.

Data: `test_data/PyrLK_record/` (rep dirs) + `test_data/Test_Videos/Fri Jun 19 01-3{5,8}-*_raw`.
