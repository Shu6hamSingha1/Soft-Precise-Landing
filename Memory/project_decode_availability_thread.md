---
name: project_decode_availability_thread
description: "⭐ ACTIVE THREAD (opened 2026-06-19): the binding perception limit for the lateral wall is FLOW AVAILABILITY, and the availability dropout is 100% ArUco DECODE-FAILURE (corner-track 0% loss, lstsq 0% reject) — 53/36% of frames overall, 88/61% at close range (<2.5m). Lever = keep producing flow through decode-fail via KLT corner-track on the surviving corners (NOT pyramid depth, NOT decode-param tuning which is a known dead-end). Existing _max_lk_steps fallback is the under-powered seed."
metadata:
  node_type: project
  type: project
---

**Thread goal:** lift corner-flow AVAILABILITY (currently ~46–64% of frames, ~12–39%
at close range) so the controller stops holding stale flow across dropouts — the
availability artifact that reads as the ṡ "0.5× under-report" behind the lateral wall
([[feedback_pyramidal_lk_inert]], [[feedback_combined_surface_divergence]]).

**Evidenced root cause (2026-06-19, `tools/tune_lk_dynamic_range.py` dropout-cause
breakdown on 2 IC2 fly-aways):** the dropout is **100% ArUco decode-failure** —
`corner_lost` 0%, `lstsq_rej` 0%. decode-fail = 53%/36% overall → **88%/61% at
alt<2.5m**. So corners track fine and the lstsq is fine; the marker just won't DECODE
at close range (it fills/over-fills the FoV). Matches [[feedback_marker_detection_stale]]
(4/4 corners in-FoV, ArUco can't decode) and the close-range breakdown in
[[feedback_descent_perception_ceiling]] / [[feedback_terminal_descent_loom_overreport]].

**Lever ranking (do in this order):**
1. **KLT corner-track persistence through decode-fail (PRIMARY).** When ArUco fails but
   the last-good corners are in-FoV, keep computing flow by LK-tracking the stored
   corners forward — instead of declaring FEATURE_IS_STALE. The existing `_max_lk_steps`
   / `MARKER_KLT_MAX_STEPS=20` fallback ([[feedback_klt_marker_fallback]]) is the seed
   but under-powered: capped horizon, strict all-4-corner gate, resets on each decode.
   Extend the horizon / relax the gate to per-corner. ⚠️ MUST clip PHANTOM corners only
   (the off-screen virtual-centroid κ-runaway, [[feedback_lateral_kappa_runaway]]):
   track GENUINE in-FoV corners, never extrapolate off-screen.
2. **Multi-marker board / inner-cluster** so a sub-marker stays decodable when the
   primary over-fills the FoV ([[project_landing_target_design]] inner-cluster 0.675m best).
3. **ArUco decode-param tuning = KNOWN DEAD-END** ([[feedback_descent_perception_ceiling]]:
   maxMarkerPerimeterRate / adaptiveThresh / deblur did NOT improve decode). Deprioritize.

**Methodology:** extend `tune_lk_dynamic_range.py` to report AVAILABILITY (track-rate +
ratio) under each lever OFFLINE on the existing recordings (`test_data/PyrLK_record/` +
`Test_Videos/Fri Jun 19 01-3{5,8}-*_raw`) BEFORE SITL — close the offline loop first.
Then IC2 n≥5 SITL on whatever lifts offline availability + holds ratio ~1.
Do NOT sweep `FLOW_LK_LVL`/`FLOW_LK_WIN` (inert). Default-off env gate; n=1 is noise.
