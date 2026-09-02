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
