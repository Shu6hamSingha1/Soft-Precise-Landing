---
name: project-overall-arc-and-phase-history
description: The whole-project arc — MATLAB sim, PX4 perception tuning (largely cal/gain-bug confounded), GT-feedback as a permanent ablation tool, marker evolution, cross-marker, now rover + a parallel hardware track
metadata:
  type: project
---

Orientation for "how did we get here". Validated against git history + memory on 2026-09-03,
correcting a natural but slightly-off four-step summary.

## The arc

1. **MATLAB Phase 1** — PLASMC numerical sim + the 5-controller comparison + the IEEE TAES
   manuscript. Done. This is what the paper is actually built on; the PX4 work is Phase 2.
2. **PX4 SITL, perception-ON, ArUco** (~May–June 2026) — "tune the controller on perception data".
   ⚠ **A large part of this was chasing artifacts, not durable tuning.**
   [[feedback_historical_cal_confound]] marks ~2000 pre-June reps as run at a **2-13× broken
   calibration** (legacy sensor-cal ~10× off on optical flow, the likely cause of the earliest
   `a_u` blow-ups). The era-defining "lateral wall" was later traced to a **gain-parity bug +
   missing velocity damping + the `Z_REG=0.01` GT-FB harness artifact** — not a perception limit.
   Treat pre-June conclusions as suspect unless independently re-confirmed.
3. **GT-feedback** (2026-06-23, [[project_gt_feedback_control_tuning]]) — feed V-frame GT `s`/`h`
   to the controller to "decouple the CONTROL problem from the PERCEPTION problem… once the
   control law converges on GT feedback, RE-INTRODUCE real perception."
   ⚠ **NOT a phase that ended.** `PLASMC_GT_FEEDBACK=1` is a permanent instrument still in daily
   use: the 2026-08-28 cross-marker cal was derived under it, and `record_robustness_set.sh`
   flies under it deliberately so the detector under test cannot gate the flight.
4. **Marker evolution** — began the SAME week as GT-FB and was driven first by CONTROL failures,
   only later by detector robustness:
   - nested ArUco board → **single large marker** (2026-06-23): the primary marker SWITCHED as
     concentric markers overflowed the FoV → corner-spread jump → loom spike → terminal vertical
     launch ([[feedback_single_marker_rank_deficiency]]);
   - textured variants finetex/reftex (2026-08-02): ring-LK survival was texture-starved (~15.7%)
     ([[feedback_textured_marker_falsified]]);
   - **cross-marker, decode-free** (2026-08-01, `916aa53e`) — current;
   - hi-res speckle (2026-08-09): texture refined SUBTLER, so it would not itself become spurious
     Hough content ([[feedback_cross_marker_texture_history]] — texture was never removed).
5. **Now: perception robustness + the MOVING rover.** Stationary cross-marker landing WORKS IC1-5
   (2026-08-31, the `CROSS_ALPHA_0` fix); moving `rover_cross` does NOT (0/5), diagnosed
   2026-09-01/02 as oblique-view detector collapse + terminal overfill
   ([[project_20260901_rover_cross_perception_diagnosis]]). Detector must key on
   CONTRAST-to-background, not darkness ([[feedback_cross_detector_contrast_not_darkness]]).
6. **Parallel hardware/Pi track since 2026-07-10** — real flights, own calibration, own
   `hardware_landing.py`. NOT a sub-step of SITL; a second platform, ~4.2 GB of Jul–Aug flight
   data ([[project_repo_git_size_and_hardware_data]]).

## The framing that actually fits

It is less a clean four-step sequence than **an oscillation between "is this control or
perception?"** — GT-feedback is the instrument added to answer that question, and it keeps being
used because the question keeps recurring. Most recent instance, the 2026-09-01 rover diagnosis:
*"NOT a code regression — the failure is what the shared code is FED."*

**Why this matters:** a phase summary that reads as "control done → perception now" invites
treating pre-June tuning results as settled and GT-FB as history. Both are wrong, and both
mislead when diagnosing. Also do not describe ArUco as a phase that ended — it is
comparison-only by directive ([[feedback_aruco_perception_scope]],
[[project_marker_roadmap_gt_ablation]]), with a stale 320×240 cal the user has accepted
([[project_two_output_cals_aruco_vs_cross]]).
