# HANDOFF — cross-marker partial visibility: off-frame junction is not handled (2026-09-26)

Written on Windows for a Claude Code session on Ubuntu. Read `docs/PLASMC_TUNING_GUIDE.md` first, then this.
Follow CLAUDE.md: NED; **no marker-size details, scale-free thresholds only** (user directive); gain/threshold
values are not portable between plants; do not launch Gazebo unless the user asks (HEADLESS=1 only).

## Requirement (user, 2026-09-25/26)
The cross marker exists so the target does NOT have to be fully in view. **Any two lines partially visible must be
enough** to produce s (and alpha where possible), even when the junction (centre) is OUTSIDE the frame.
Keeping the target in view is the controller's job; perception must not depend on the junction being in frame.

## Finding (Gazebo, GT-scored; `tools/score_partial_visibility.py`)
74 of 372 GT reps had consistent geometry (the rest mix worlds / marker heights). alt > 0.3 m:

| true centre | frames | detected | centre err (detected) | s err px-equiv |
|---|---|---|---|---|
| in frame | 23003 | 95% | 2.9 px median, 12.7 p90 | 1.9 |
| out 0-20 px | 114 | 42% | 170 px | 174 |
| out 20-40 px | 79 | 49% | 176 px | 197 |
| out 40-80 px | 92 | 50% | 203 px | 224 |
| out >80 px | 209 | 31% | 347 px | 418 |

In-frame (arms clipped by the edge) works. Off-frame: the detector reports `ok` on 31-50% of frames but the centre is
wrong by far MORE than the distance outside the frame, so it locks onto something else. The off-frame sample is small
(~500 frames; the controller mostly keeps the marker in view): re-measure after the fix and add off-frame cases
to the eval set. The projection ignores camera tilt (fine for these small tilts). Numbers were computed on Windows;
re-run the script on Ubuntu to confirm before changing code.

## Root cause (code, verify line numbers, they drift)
1. `src/cross_stroke_detector.py` `side_stats()` (~L409-445): `avail(sgn)` is the length from J to the frame edge on each
   side. For J outside the frame one side has avail ~ 0 (< `vmin = max(2*width, 8)`), so it returns `bal=None`
   ("unverifiable"). In the pair loop (~L465): `if ba is None and bb is None: continue`. So **an off-frame junction (and
   one within ~8 px of the edge) is always discarded.** The candidate window `-0.5W <= J <= 1.5W` (~L461) shows off-frame was
   intended, but the balance logic vetoes it. Whatever passes off-frame is a spurious pair.
2. `src/cross_marker_detector.py` (legacy, `CROSS_DETECTOR=legacy`) has an off-frame extrapolation path
   (`in_fov=False`, `MAX_EXTRAPOLATION_DIAGS=3.0`, ~L1650-1815). The default detector is `stroke` (~L2114).
3. `src/cross_marker_perception.py` **never reads `det.in_fov`** (grep: no hits). An extrapolated centre would be fed into
   s / the KFs like an in-frame one, with no confidence handling. (`_center_fresh` only records whether this frame confirmed a centre.)
4. Controller / CBF (`src/visibility_projection.py`, `controller.py` uses `FEATURE_IS_VISIBLE`) assumes a measured in-frame
   centre. Check what it should do with an off-frame-but-valid centre (the tau*d lead term / safe set).

## Proposed change (env knob; default ON only after the gate passes)
A. Stroke detector: accept J outside the frame when >=2 lines with different angles (existing PAIR angle limits) each
   have verified support on the VISIBLE side (span >= a scale-free minimum, e.g. a fraction of the frame min-dimension or
   a multiple of stroke width), consistent width ratio and polarity, and J within an extrapolation cap (frame-diagonal
   multiples, like legacy 3.0). Do not weaken the in-frame X-junction/balance checks. Do NOT let two border lines meeting at a
   frame corner pass (the reason `vmin` exists). Return `in_fov=False`.
B. Perception: consume `in_fov`. Use the off-frame centre for s with inflated measurement noise in the centroid KF
   (`_stepCentroidKf`); do not use it where a real in-frame point is required; alpha only if a stub/heading is verifiable;
   flow (h, w) falls back to the existing bbox/bg-flow path when the junction is off-frame.
C. Controller/visibility: confirm `visibility_projection` handles a centre reported outside the frame sanely (it is
   the input the CBF pulls back in).
D. Keep it portable to hardware: `Hardware/scripts/cross_marker_detector.py` is a copy; sync per
   `Hardware/docs/CROSS_MARKER_PORT_PLAN.md`. Thresholds must be scale-free (the Pi camera is fx~513, 320x240).

## Validation (in this order)
1. `python3 tools/score_partial_visibility.py test_data 0.5` before and after. Target: off-frame rows detected with
   error close to the in-frame error plus a small extrapolation error, and no rise in wrong-place detections in the in-frame row.
2. `tools/validate_detector_gt.py --set test_data/PerceptionEvalSet --variant stroke` (accuracy / POISON) must not regress.
   Add tags with the cross partly out of frame if none exist.
3. IC gate `scripts/run_ic_validation.sh` (IC2-5, mandatory pre-merge) plus a moving-target case. Run headless only when the user asks.
4. Update `docs/PERCEPTION_FLOW_FINDINGS.md` and the CBF docs; save a memory note.

## Related hardware findings (2026-09-25, Windows; scripts in Hardware/scripts, may be uncommitted)
Real-world scoring of both cross detectors against an image-only oracle (>=2 strokes visible, partial view allowed):
stroke detector is precise (0.7 px median) but misses 60% of visible frames (weak above 2.5 m and below 1.2 m);
legacy reports a wrong place on 55%. Scripts: `hw_perception_quality.py`, `perception_hw_common.py`, `cross_oracle.py`,
`oracle_scan.py`, `score_cross_perception.py`. The user plans a mocap recording of the cross marker (GT for s, alpha, h, w)
and an output calibration; real-world detector tuning follows that. Hardware h, w, alpha are NOT computed on the Pi yet
(`Img_Data` is all `coast` on 09-24/09-25).
