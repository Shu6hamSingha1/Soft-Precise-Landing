# RobustnessFrameset — lighting / polarity / colour / background eval set

Recorded 2026-09-03. Purpose: make the user's stated acceptance bar **measurable** —
*"robust to different lighting conditions, different colour combinations of the marker and
textured background, and other perception noise"*. `DetectorFrameset` cannot test it: all 6
of its sets are ONE lighting condition, ONE polarity (dark cross on light plate), ONE
texture, so tuning against it re-fits to a single scene.

    ~/ws/scripts/env2025/bin/python3 tools/validate_detector_gt.py \
        --set test_data/RobustnessFrameset --variant baseline

Bulk data gitignored (263 MB); this manifest is tracked. Regenerate the scenes with
`tools/make_robustness_eval_assets.py`, re-record with
`test_data/Rover_AB_harness/record_robustness_set.sh`.

## How it was recorded (two deliberate choices)

1. **`PLASMC_GT_FEEDBACK=1`.** The detector under test must NOT gate the flight — otherwise
   the variants where it is blind never take off and yield no frames (exactly what happened
   to the `cross_marker_clutter` IC2 attempt). GT-FB gives every variant the SAME trajectory,
   so offline detector scores are comparable ACROSS scenes.
2. **`CROSS_RING_OVERLAY_DBG=0`.** Verified 0.00 % drawn pixels. `DetectorFrameset` is
   overlay-contaminated (~3-5 % of pixels; 15-21 % of the adaptive-gate mask) — see
   [[feedback_detector_offline_replay_gotchas]].

All runs IC2 (2,2,5) on the flat `cross_marker` geometry; only the scene variable changes.

## Variants

| tag | world | what changes | frames |
|-----|-------|--------------|--------|
| base | `cross_marker` | unmodified reference | 375 |
| dim | `cm_dim` | sun intensity 1.0 -> **0.30** | 344 |
| bright | `cm_bright` | sun intensity 1.0 -> **2.50** | 374 |
| lowsun | `cm_lowsun` | sun direction -> `0.70 0.30 -0.20` (grazing, long hard shadows) | 391 |
| inv | `cm_inv` | **POLARITY FLIP** — light cross (V 235) on dark plate (V 25) | 395 |
| col | `cm_col` | **CHROMATIC, ISO-V** — red cross / green plate, both **V=150** | 550 |
| darkbg | `cm_darkbg` | ground 0.8 -> **0.18** (global darkening) | 314 |

`cast_shadows` is untouched everywhere (per [[feedback_reject_disable_cast_shadows]]).

### Why `inv` and `col` are the load-bearing cases

Measured on the textures alone, before any flight — what `inRange(hsv,[0,0,0],[180,255,100])`
retains:

| variant | plate V | cross V | legacy gate keeps |
|---|---|---|---|
| base | 193.7 | 10.0 | 5.2 % (the cross) ✅ |
| **inv** | 25.0 | 235.0 | **94.8 % — keeps the PLATE, misses the cross** |
| **col** | 150.0 | 150.0 | **0.0 % — mask entirely empty** |

## BASELINE (current detector @ `a53a5f63`, span rescue ON) — CORRECTED 2026-09-03

⛔ **The first baseline published here was CONTAMINATED and has been replaced.**
`validate_detector_gt.py` scored frames recorded AFTER the GT log ends against
`np.interp`-CLAMPED (frozen touchdown) values. 16-42 % of frames per variant fall outside
that window. Fixed by a hard GT-window guard in the scorer (`CROSS_GT_WINDOW_STRICT`,
default on) — it now DROPS those frames and prints how many. Corrected figures:

| variant | detOK | centroid err (med) | within-0.15 | frames dropped |
|---|---|---|---|---|
| base | 100.0 % | 0.014 (1.8 px) | **97 %** | 20 % |
| dim | 100.0 % | 0.017 (2.3 px) | 81 % | 24 % |
| bright | 95.0 % | 0.013 (1.8 px) | 96 % | 19 % |
| lowsun | 100.0 % | 0.014 (1.9 px) | 97 % | 19 % |
| darkbg | 99.6 % | 0.013 (1.8 px) | **94 %** | 20 % |
| **col** | **62.4 %** | 0.013 (1.8 px) | 83 % | 16 % |
| **inv** | **99.1 %** | **0.814 = 109.9 px** | **0 %** | 42 % |

(For the record, the contaminated version said base 78 %, darkbg 76 %, inv 23 % — clean
scenes were DEFLATED and `inv` was FLATTERED. Both directions wrong.)

**⭐ LIGHTING IS NOT THE WEAKNESS.** 8x sun-intensity range, grazing sun, dark ground: all
95-100 % detOK at 81-97 % within-0.15. That half of the requirement is already met.

**⭐ `inv` IS THE HEADLINE FAILURE, and it is SILENT.** 99.1 % detOK — the detector reports a
detection on essentially every frame — and **0 % of them are usable**, 109.9 px off, at every
altitude band. `inRange(V<=100)` keeps 94.8 % of pixels (the PLATE, not the cross), so the mask
becomes the plate with the cross as HOLES; Canny/Hough still find arm-like edges, so nothing
downstream objects. **A detector that fails loudly is recoverable; this one lies.** Any
health metric based on detect-rate would score this scene as perfect.

**`col` fails by RANGE**, 62.4 % detOK, but is ACCURATE when it fires (83 % within-0.15) — the
V gate keeps 0.0 % of pixels by construction, so detections come from the ROI/shape fallback.

## Using it as the gate for front-end work

Target = polarity-agnostic, chroma-aware segmentation
([[feedback_cross_detector_contrast_not_darkness]]). Pass criteria:
- **`inv` within-0.15 must rise from 0 %** (hard pass/fail — the silent-lying case).
- **`col` detOK must rise from 62.4 %** while keeping within-0.15 >= 83 % (graded).
- **base/dim/bright/lowsun/darkbg must NOT regress** (must-not-regress-clean).
- Score on **within-0.15 AND flight outcome**, never detOK alone.

## Caveats
- **n=1 run per variant**, all IC2, one trajectory. Direction-of-effect only.
- Scenes are SYNTHETIC (textures generated by the tool); `col` is an adversarial
  construction, not a photograph of a real marker.
- Bulk is gitignored, so this set is NOT reproducible from git alone — regenerate assets +
  re-record if lost.
