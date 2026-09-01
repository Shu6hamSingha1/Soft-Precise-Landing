---
name: feedback_cross_detector_contrast_not_darkness
description: "The cross-marker detector's segmentation MUST key on CONTRAST-to-background, not on the marker being dark. The current fixed inRange(V<=100) gate is fragile to any dark object near the marker (PROVEN in a controlled empty-world test: one dark box -> IC2 detOK 100%->10%, flight can't start) and collapses if the scene darkens globally. Design principle + the robustness plan + the GT-scored harness for it."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 165e87c8-3181-4ae1-9945-1405e9e2021d
  modified: 2026-09-01T19:52:27.835Z
---

**Locked 2026-09-01 (user).** From the [[project_20260901_rover_cross_perception_diagnosis]]
session.

## The weakness

`cross_marker_detector.py::_detect_core` segments the marker with a **fixed absolute-
brightness gate**: `cv2.inRange(hsv, [0,0,0], [180,255,CROSS_COLOR_GATE_V_MAX=100])` — "keep
V ≤ 100, the strokes are dark, everything else is light." This is brittle two ways:

- **Any dark object near the marker** (below ~40% grey: shadow, dark-painted structure,
  vehicle part, wet tarmac, ground marking) passes the gate. If it also has straight edges
  (most man-made things), those edges land in the binary mask alongside the cross strokes;
  Hough → angle-cluster → line-pair selection can lock onto the wrong pair. The geometry
  gates (`centroid_mismatch`, `near_parallel`, 90°-apart) catch a lot but not when a real
  arm + a spurious edge form a plausible cross.
- **Global darkening** (dim light, or the plate itself rendering below V100) — the whole
  frame passes, no structure at all → `hough_lt2_lines` / `color_gate_empty`. (This is the
  two-instance camera-warm-up frozen-grey case on `rover_cross`.)

## Controlled proof (no rover needed)

Empty `cross_marker` world + ONE dark visual-only box (0.6×0.6×0.3 m, V≈31, no collision)
on the marker plane at (1.2, 1.2), well clear of the (0,0) landing point. **Same legacy
detector that lands IC1-5 cleanly on the truly-empty world.** Result: **IC2 detOK 100% →
10 %; the flight never starts** (`TARGET_IS_VISIBLE` never fires, 6 retries time out).
Test world kept as `~/PX4-Autopilot/Tools/simulation/gz/worlds/cross_marker_clutter.sdf`
(own world name) for developing the fix.

## The right invariant (user)

> "Relying on black colour is wrong. We can't always guarantee black cross-marker. But
> we can guarantee cross-marker with contrast colour to the background."

So the front end must be:

1. **Contrast-based, not level-based, and polarity-agnostic.** Drop `V ≤ 100`. Use a
   quantity that says "this pixel deviates from smooth local background" regardless of
   sign/absolute value: gradient magnitude (Sobel/Scharr) with an adaptive/relative
   threshold, or `|CLAHE-normalized − local mean|`, or a local-std map. Fires the same on
   light-cross-on-dark-plate as dark-on-light, and doesn't care if the scene is dim.
2. **Perpendicular intensity-profile stroke validation** — the discriminator that rejects
   the dark box. Sample a short profile across each candidate segment at several points:
   a marker stroke = a single-signed *ridge or valley* (bright-dark-bright OR
   dark-bright-dark) of ~constant width, symmetric; an object boundary / shadow edge = a
   *step* (monotonic, one side stays dark) → fails symmetry; texture/marking = shallow or
   width-inconsistent → fails depth/consistency. Polarity-agnostic (it's profile SHAPE).
3. **Geometry as the primary confirm, not a post-filter.** Two profile-validated strokes,
   ~perpendicular, meeting at one in-frame point (+ optional stub) → the cross, scored by
   fit quality. That's the accept/reject, not "whatever two lines Hough returned."

## What was already tried (do not just repeat)

- `CROSS_ADAPT_GATE` (committed `8ed004ad`, DEFAULT OFF): CLAHE + `adaptiveThreshold` +
  Otsu fallback + structureless reject. On `rover_cross` IC2 offline it lifts detOK
  49 %→83 % (descent bands 0 %→72-100 %) — the local-contrast idea WORKS for detection —
  **but only 73 % of the extra detections are within 0.15 norm of GT** (~40 px median
  centroid disagreement vs legacy): local contrast alone over-admits texture/other-object
  edges and the fitter picks wrong line pairs. Needs (2)+(3) on top before it's a net win.
- The 2026-08-27 `validate_bgflow_corr.py` bake-off (8 strategies) picked CLAHE+FB for the
  FLOW-point path — the DETECTOR gate was explicitly left as future work
  ([[project_20260827_framerate_and_h_texture_investigation]]).

## The harness (built this session)

`tools/validate_detector_gt.py` — GT-scored, offline, per recorded frame: run a candidate
detector, compare leveled `det.center` to GT V-frame bearing
(`gt_optical_flow.compute_gt_flow → V_s_g`), report detOK% + per-altitude-band, centroid
err median-px + **within-0.15 hit-rate**, top fail reasons. Variants = env-dict entries in
`VARIANTS`. Add `gradient` / `stroke_profile` code variants behind env flags in the
detector + one line each here. Score across {flat clean, `cross_marker_clutter`,
`rover_cross`}: **must-not-regress-clean, fix-the-rest**, then n≥5 SITL.

## Also worth doing

Change the SIM marker to a **contrasting non-black colour** (saturated hue, or light-on-
dark) so development can't silently lean on darkness again.
