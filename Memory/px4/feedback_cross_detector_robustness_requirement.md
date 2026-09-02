---
name: feedback_cross_detector_robustness_requirement
description: "USER REQUIREMENT (2026-09-02): the perception pipeline must be robust to different LIGHTING conditions, different marker/background COLOUR combinations, TEXTURED backgrounds and general perception noise. This rules out patching gates one at a time -- the pipeline currently hard-codes two absolute assumptions (inRange(V<=100); the mask-centroid junction proxy) that both fail the requirement, and the eval set cannot even MEASURE it."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**Stated by the user, 2026-09-02:**

> "I need a robust perception pipeline which should work for different lighting conditions,
> different colour combinations of the marker and textured background and it should be robust
> to other perception related noise."

This is the ACCEPTANCE BAR for the cross-marker front end. It is stricter than "make
`rover_cross` land", and it retires the patch-a-gate-at-a-time approach.

## Why the current pipeline cannot meet it

Two hard-coded ABSOLUTE assumptions, both the same mistake -- *the marker is the only dark
thing*:

1. **`cv2.inRange(hsv, [0,0,0], [180,255,CROSS_COLOR_GATE_V_MAX=100])`** -- "the marker is dark
   in absolute terms." Breaks on: dim light (background falls below 100), bright light (marker
   rises above it), light-on-dark or coloured markers, and any dark object nearby.
   ([[feedback_cross_detector_contrast_not_darkness]] -- the locked design principle.)
2. **The `centroid_mismatch` gate** -- validates the fitted junction against the MASK PIXEL
   CENTROID, i.e. "the marker is the only thing in the mask." Breaks whenever foreign structure
   merges into the same connected component.
   ([[project_20260901_rover_cross_perception_diagnosis]])

Every fix attempted so far patches assumption 2 in one scene: the `ee858086` fill band, the
compact-blob strip, `CROSS_CENTROID_SPAN_RESCUE` + its fill ceiling. None touches lighting or
colour, and the last one made clutter flight WORSE
([[project_20260902_spanrescue_sitl_gate]]).

## What the requirement implies (the locked design, now load-bearing)

- **Contrast-based, POLARITY-AGNOSTIC segmentation** -- gradient magnitude / local deviation,
  relative not absolute. Identical response to dark-on-light and light-on-dark; indifferent to
  scene brightness. Drop the absolute V gate entirely.
- **Perpendicular intensity-profile STROKE validation** -- a marker stroke is a symmetric
  ridge/valley of ~constant width; a platform edge or shadow boundary is a STEP. This is the
  discriminant that rejects today's contaminant BY SHAPE rather than by darkness, which is what
  survives lighting and colour changes.
- **Geometry-first confirm** -- two profile-validated strokes, ~perpendicular, meeting in-frame.
  This also REMOVES the need for the mask-centroid proxy, so assumption 2 stops existing rather
  than being patched again.

## ⛔ Two constraints on how to validate it

1. **Score on ACCURACY and FLIGHT OUTCOME, never detect-rate alone.** Proven the hard way:
   clutter detOK 5-7 %->22-73 % while flight got WORSE (median xy ~1.9->~5.6 m, two 12 m
   fly-aways). `within-0.15` was the offline signal that predicted it; the headline rate was not.
2. **The eval set is the BOTTLENECK -- robustness is currently UNMEASURABLE.**
   `test_data/DetectorFrameset` has 6 sets but ALL are one lighting condition, one polarity
   (dark cross on light plate), one texture. Tuning against it = tuning to one scene again.
   The requirement needs sets spanning: sun angle + intensity, INVERTED polarity, a coloured
   marker, textured/cluttered backgrounds, and sensor noise -- recorded with
   **`CROSS_RING_OVERLAY_DBG=0`** so the frames are not overlay-contaminated
   ([[feedback_detector_offline_replay_gotchas]]).

**Build the eval set BEFORE the front end.** Otherwise there is no way to tell a robustness gain
from a scene-specific fit.

## ✅ EVAL SET BUILT + BASELINED 2026-09-03 — and it RESHAPES the target

`test_data/RobustnessFrameset/` (7 variants, 2743 frames, IC2, flat `cross_marker` geometry;
bulk gitignored, `MANIFEST.md` tracked). Assets: `tools/make_robustness_eval_assets.py`;
recording: `test_data/Rover_AB_harness/record_robustness_set.sh`. Recorded under
**`PLASMC_GT_FEEDBACK=1`** (the detector under test must NOT gate the flight, else the blind
variants never take off — and every variant gets the SAME trajectory so scores compare across
scenes) and **`CROSS_RING_OVERLAY_DBG=0`** (verified 0.00 % drawn pixels).

**Baseline, current detector @ `a53a5f63`:**

| variant | detOK | err med | within-0.15 | |
|---|---|---|---|---|
| base | 100.0 % | 0.019 | 78 % | reference |
| dim (sun 0.30) | 100.0 % | 0.017 | 81 % | robust |
| bright (sun 2.50) | 96.0 % | 0.013 | 96 % | robust |
| lowsun (grazing) | 100.0 % | 0.014 | 97 % | robust |
| darkbg (0.18) | 99.7 % | 0.017 | 76 % | robust |
| **col** (chromatic, iso-V 150/150) | **66.5 %** | 0.013 | 83 % | 95 % @4-6 m -> **5 % @0.7-1.3 m** |
| **inv** (polarity flip) | 82.3 % | **0.399 = 53.9 px** | **23 %** | **CONFIDENTLY WRONG** |

### ⭐ LIGHTING IS NOT THE WEAKNESS — this was NOT predicted

An 8x sun-intensity range (0.30-2.50), a grazing sun and a dark background all hold 96-100 %
detOK with accuracy EQUAL TO OR BETTER THAN base. The "different lighting conditions" half of
the requirement is, on this evidence, **already met**. Do not spend effort there without new
evidence. (I expected `inRange(V<=100)` to break under dim light; it does not — Gazebo's
auto-exposed render keeps plate/cross V separation roughly intact across intensity.)

### ⭐ COLOUR IS THE WEAKNESS — in two DIFFERENT ways

Measured on the textures alone, before any flight, what the legacy gate retains:
base 5.2 % (the cross) / **inv 94.8 % (the PLATE)** / **col 0.0 % (empty)**.

1. **`inv` (polarity) is the DANGEROUS one — high detOK, wrong answers.** 82.3 % detOK but
   53.9 px centroid error and only 23 % usable. The detector is NOT failing; it is
   confidently returning WRONG centroids, because the gate keeps the whole plate and Hough
   fits PLATE EDGES instead of cross strokes. **A detector that reports garbage confidently is
   worse than one that reports nothing** — the same lesson
   [[project_20260902_spanrescue_sitl_gate]] reached from the opposite direction.
2. **`col` degrades with RANGE**, 95 % -> 5 % as the marker fills the frame: the V gate keeps
   0 % by construction, so every detection comes from the ROI/shape fallback, which loses the
   marker up close.

### PASS CRITERIA for the front-end work (use these, not detOK)

- **`inv` within-0.15 must rise from 23 %** — hard pass/fail, the confidently-wrong case.
- **`col` 0.7-1.3 m band must rise from 5 %** — graded.
- **base/dim/bright/lowsun/darkbg must NOT regress.**
- Score on **within-0.15 AND flight outcome**, never detOK alone.

⚠ n=1 per variant, single IC/trajectory, SYNTHETIC textures (`col` is an adversarial
construction, not a photo of a real marker). Direction-of-effect only. Bulk is gitignored so
the set is NOT reproducible from git alone — regenerate + re-record if lost.
