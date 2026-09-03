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

## Refinement 2026-09-02 (user, measured): the FIRST rover_cross break is NOT the gate

The V<=100 gate is fragile (proof above stands), but on `rover_cross` the cross is
**in the mask on every frame** -- it's lost one stage later, at
`_isolate_marker_by_shape` (`cross_marker_detector.py`). That selector scored
`-area` -> kept the LARGEST aspect-passing connected component. Measured over 128
acquisition frames (alt 4-6 m):

| component            | area     | bbox aspect | fill = area/(bw*bh) |
|----------------------|----------|-------------|---------------------|
| cross marker         | 246-416  | 1.02-1.19   | **0.070-0.083**     |
| platform shadow bar  | 818-1224 | 2.57-2.71   | **0.35-0.375**      |

The shadow bar is diagonal, so its axis-aligned bbox is square-ish (aspect 2.6 <
`ISOLATE_MAX_ASPECT`=3.2); it then wins on area and the cross is discarded ->
downstream sees one bar -> `lt2_angle_clusters`. Wrong on **100/128 (78 %)** = the
observed 78 % `lt2_angle_clusters` rate; **0/123 on flat-clean**. The two fill
populations are cleanly bimodal, 4x gap, no overlap.

**FIX (committed `ee858086`):** `_isolate_marker_by_shape` restricts the pick to
fill in `[CROSS_ISOLATE_FILL_LO=0.02, CROSS_ISOLATE_FILL_HI=0.25]` (a thin X vs a
solid slice), max-area within the band, falls back to the old behaviour if none
qualify (marker-fills-frame). `CROSS_ISOLATE_FILL_HI=1.0` reverts.

**Eval-set result (`validate_detector_gt.py --set`, `baseline`):** flat IC1/IC2
unchanged (100 % detOK -- NO regression); **rover_IC4 detOK 78->88 %**
(`lt2_angle_clusters` 102->33); **rover_IC2 45->59 %**. **clutter IC1 unchanged
(63 %)** -- a DIFFERENT mechanism there: the solid box (fill ~1, already dropped
by `_reject_blobby_components`) plus its cast shadow plus the cross MERGE into one
connected component after the Hough dilate -> `_isolate_marker_by_shape` keeps the
merged blob -> mixed cross/box line fit -> `centroid_mismatch`. Fix = tighter
`CROSS_HOUGH_MASK_DILATE_PX` or split merged components. STILL OPEN.

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

## Curated eval set (2026-09-02)

`test_data/DetectorFrameset/` (gitignored data + tracked `MANIFEST.md`) -- 6 IMG_RECORD
paired sets: flat IC1/IC2 (100% baseline detOK, MUST-NOT-REGRESS), cross_marker_clutter
IC1/IC2 (one dark box near marker -> baseline 63%), rover_cross IC2/IC4. Run:
`validate_detector_gt.py --set test_data/DetectorFrameset --all`.

**Baseline vs `adapt` (2026-09-02):** adapt lifts detOK where baseline fails (clutter
63->94%, rover_IC2 45->86%), detection-neutral where baseline works (flat, rover_IC4),
BUT drops within-0.15 centroid hit-rate ~15-20 pts in EVERY scenario incl. clean flat
(80->66%). Tuning C/block doesn't fix it. => the next variant (gradient / stroke_profile)
must keep the detection recovery AND restore accuracy.

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

## ⚠ Before scoring ANY contrast variant on the frameset (2026-09-02)

The recorded frames carry a **debug overlay** (`CROSS_RING_OVERLAY_DBG`, default ON):
~3-5 % of pixels are drawn flow points. The LIVE detection path is unaffected (the
overlay is applied to the recording copy only) but **offline replay is contaminated**,
and it hits contrast-based gates ~20x harder than the legacy dark gate --
**15.5-20.9 % of the `CROSS_ADAPT_GATE` mask is overlay halo, vs 0.8-3.7 % of the legacy
mask**. Any gradient/stroke-profile front end will key on it harder still.

Measured, so it is not a reason to dismiss the adapt result: with the overlay inpainted
away, adapt's within-0.15 STILL sits ~7-8 pts below baseline wherever baseline works
(flat_IC1 87.6 vs 94.8, flat_IC2 65.1 vs 72.9, rover_IC4 84.8 vs 92.5). **The accuracy
deficit is real and is the line-pair fitter, exactly as this file's design section says.**
Overlay removal does shift absolutes ~3-8 pts, so read the numbers above with a ~+/-5 pt band.

**Re-record the eval set with `CROSS_RING_OVERLAY_DBG=0` before the front-end work.**
Full detail + two more offline-scoring gotchas (post-touchdown ground frames inflating
detOK; clamped-GT tails on hand-paired dirs): [[feedback_detector_offline_replay_gotchas]].

## ⭐ The dominant failure is a GATE, not the segmentation (2026-09-02)

On `rover_cross` the front end is not mainly failing to FIND the cross -- it is finding it and
then rejecting it. `centroid_mismatch` is **79 % of all failures** across the 9 static-offset
perception runs, which detect at **0.0 % from 5 m down to 1 m** (live `Detection Status`).
Disabling only that gate: rover_IC2 28.5 -> 95.9 %, clutter_IC2 50 -> 97.5 %, flat unchanged,
with the median centroid error flat-to-better. The gate validates the fitted intersection
against the MASK PIXEL CENTROID, which merged platform structure corrupts.

So the stroke-profile / contrast work in this file is still right for segmentation, but the
cheapest available win is fixing what the intersection is validated AGAINST. **LANDED
`f49f567f` as `CROSS_CENTROID_SPAN_RESCUE` (DEFAULT OFF, pending a SITL gate):** legacy check
first, and only on failure ask whether the intersection lies ON both fitted arms' inlier spans
-- rover_IC2 28.5->94.8 %, clutter_IC2 50->87.7 %, flat bit-identical. ⛔ The obvious-looking
alternative (validate against the arms' inlier CENTROID) was tried and is WORSE
(rover 28.5->11.0 %): unequal inlier counts put that mean off the junction. Full data:
[[project_20260901_rover_cross_perception_diagnosis]].
