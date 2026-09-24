# PerceptionEvalSet — eval set for the locked-design cross detector (2026-09-24)

Built BEFORE/alongside the stroke-detector rewrite (`src/cross_stroke_detector.py`,
`CROSS_DETECTOR=stroke`), per the user requirement in
`Memory/px4/feedback_cross_detector_robustness_requirement.md`: robust to lighting, marker/background
colour, texture and perception noise; score ACCURACY / POISON + flight outcome, never detect-rate
alone.

    ~/ws/scripts/env2025/bin/python3 tools/validate_detector_gt.py --set test_data/PerceptionEvalSet --variant stroke
    #   --variant current  (shipped defaults)   --ref reg  (old regularized GT reference)

Bulk data gitignored (~1.1 GB incl. `_reps/`, the original landing reps the tags were copied
from); this manifest is tracked.

## How every tag was recorded

- One GT-FB descent (`PLASMC_GT_FEEDBACK=1`): the detector under test never gates the flight, so
  every scene gets a full descent to touchdown and comparable trajectories.
- `IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0` — clean frames (DetectorFrameset is overlay-contaminated).
- **Exact frame↔log pairing.** `CrossMarkerNode` writes `frames/frames.tsv` (saved frame → the
  capture stamp `process_frame()` used = `Img_Data['Stamp']`). `tools/build_evalset_tag.py`
  pairs a rep with its raw dir by a CONTIGUOUS in-order stamp run (true pairing 0.76–1.00;
  best unrelated run ≤0.068). Stamp SET membership is not safe — sim time restarts at 0 each
  launch, so unrelated runs of one campaign share 59–70% of stamps.
- `meta.json` per tag: `world`, `marker_dz` (0.0 flat worlds, 0.5 rover worlds), `case`, sources.
- Scored against the TRUE bearing `x/z` (`compute_gt_flow(...)['V_s_true']`), not the GT-FB feed
  `x/(z+0.2)` which books a correct close-range bearing as a 30–90% error.

Recorders: `scripts/record_perception_evalset.sh` (static + rob_* tags);
`scripts/run_sperc_gtfb_ab.sh ARMS=gt CASES="Sinusoidal Circular" EXTRA_ENV="IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0"`
then `tools/build_evalset_tag.py` (moving tags).

## Tags

| tag | world | IC / case | frames | purpose |
|---|---|---|---|---|
| flat_IC1 / IC3 / IC4 | cross_marker | IC1, IC3, IC4 | 342 / 374 / 433 | clean reference, approach-geometry variety, terminal overfill |
| clutter_IC2 | cross_marker_clutter | IC2 | 250 | dark box distractor |
| rover_static_IC2 / IC4 | rover_cross | IC2, IC4 (static) | 326 / 468 | raised platform: side face + shadow (the corner-lock contaminant) |
| rover_sin_r1–r5 | rover_cross | Sinusoidal, IC2 | 354–390 | MOVING target on the raised platform |
| rover_circ_r1–r5 | rover_cross_deck | Circular + deck, IC2 | 416–477 | moving + deck — **⚠ GT unusable for accuracy, see below** |
| rob_base / dim / bright / lowsun / darkbg | cross_marker / cm_* | IC2 | 306–401 | lighting (re-records of RobustnessFrameset, now exactly paired) |
| rob_inv | cm_inv | IC2 | 387 | POLARITY FLIP (light cross on dark plate) |
| rob_col | cm_col | IC2 | 330 | CHROMATIC iso-V (red cross / green plate, both V=150) |

## ⚠ Known issues

- **`rover_circ_*`: FIXED 2026-09-24 — marker height, not target identity.** An earlier note here
  claimed the deck-world GT target was the rover; WRONG. `POSE_IDX_TARGET=1` in `rover_cross_deck`
  IS `deck_platform` (recorded target z 0.26–0.29 m with deck roll/pitch), but its marker sits
  +0.201 m above that origin while the rover launcher defaulted `PLASMC_GT_MARKER_DZ=0.5` (right for
  `rover_cross`). meta.json now says 0.201: stroke err 0.006 / poison 0% (was 24–32% poison).
  `scripts/run_rover_landing.sh` now defaults 0.201 for `WORLD=rover_cross_deck`. Every GT-FB
  Linear/Circular flight before this fix steered at (and scored touchdown against) a GT marker
  0.3 m too high.
- **`test_data/RobustnessFrameset/inv` is mis-paired** (tail-offset assumption fails: f150 shows
  ~2 m, paired GT alt 4.9 m) — every `inv` number from that set is invalid. Use `rob_inv`.
