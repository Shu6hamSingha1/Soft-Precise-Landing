---
name: project_20260824_cross_marker_montage_overlay_robustness
description: "2026-08-24: rebuilt all 6 test_data/Test_Videos/montage_*.mp4 with cross-marker (new rover_cross world+model), built tools/overlay_image_features.py (s/h/alpha HUD overlay), diagnosed a drone-shadow-near-touchdown perception failure via the overlay, and fixed a real dead-code bug in cross_marker_detector.py's _robust_fit_line that let contamination (shadow, and generally any non-marker dark content) corrupt alpha/s."
metadata:
  node_type: memory
  type: project
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-24T11:24:04.251Z
---

## What was built (all committed except PX4-Autopilot-side files, which are outside this repo)

1. **`rover_cross` world/model** (outside this repo, `~/PX4-Autopilot/Tools/simulation/gz/`):
   `models/rover_cross/` (ported from `rover_aruco`, marker swapped to `cross_marker` texture at
   validated 3.0x3.0m size) + `worlds/rover_cross.sdf` (copy of `rover.sdf`, renamed world). Also
   added a chase-cam sensor to `cross_marker.sdf` (for the stationary/flat-world case).
2. **`run_rover_landing.sh`**: `WORLD`/`ROVER_MODEL` now overridable (were hardcoded rover/rover_aruco).
3. **`run_aruco_landing.sh`**: added `CHASE_CAM` bridging (ported from run_rover_landing.sh).
4. **`test_data/Rover_AB_harness/montage_landing_cross.sh`**: cross-marker montage harness
   (`WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1`), ON-PLATFORM
   altitude band widened to 0.35-1.15m (cross-marker's loom-inversion touchdown detector fires
   ~0.5m higher than the flat ArUco case, stacked on the platform's own 0.5m height).
5. **All 6 `test_data/Test_Videos/montage_*.mp4`** (static/linear/circular/sinusoidal/eightshape/
   lissajous) regenerated with cross-marker + GT-feedback, all landed ON-PLATFORM or SOFT+PRECISE.
   Old ArUco versions preserved in `test_data/Obsolete_ArUco_Montages/`.
6. **`tools/overlay_image_features.py`** (new): overlays s/h/alpha (and optionally w) onto a
   recorded down-cam video, reading `Img_Data.npy`'s cross-marker schema (`Center Px`, `s_V`,
   `alpha(t)`, `h_V`). `--split` groups into `(s,alpha)` [position+orientation] and `(h)` [flow]
   videos. HUD uses PIL/DejaVuSans (not cv2.putText, which can't render `α`). Alpha is drawn via
   an approximate inverse of the V-frame/offset transform (`_alpha_to_pixel_dir`, see [[feedback_cross_marker_alpha_not_pixel_frame]]),
   and a held/stale alpha (stub not re-detected that frame) is drawn gray + labeled `(held)`.

## Key diagnosis: drone's own cast shadow corrupts s/alpha near touchdown

Root-caused by directly inspecting raw recorded frames (not just logs): the world's `sunUTC`
directional light casts the drone's own shadow onto the marker plate in the final ~10 frames
before touchdown; it grows to overlap the stub arm specifically. `cross_marker_detector.py`'s
Hough-line + color-mask pipeline can't distinguish shadow edges from real marker lines once they
overlap the SAME connected component (the existing `_reject_blobby_components`/
`_isolate_marker_by_shape` ghost-defenses are component-level and explicitly documented as unable
to split a merged blob). Symptom: `alpha` did a ~183 deg disambiguation flip then froze (held)
for the last 2 frames, while `Detection Status` stayed `"ok"` throughout (falsely).

## Fix applied (backed up as `*.bak_before_robustpoints_20260824`)

`cross_marker_detector.py::_robust_fit_line` had a **dead-code bug**: it computed Huber outlier
weights each iteration then immediately discarded them (reset `pts` to the full unfiltered input
every pass) — so the "iterative robust fit" never actually excluded contaminated points, just
refit unchanged data 3x. Fixed to genuinely prune (keep points within 2.5x median residual,
refit on the shrinking inlier set, floor at 30% survival to avoid over-pruning) and now returns
an inlier mask. That mask is used to prune BOTH `line_points_i`/`line_points_j` (hardens center/s)
AND the stub cluster's points before they're stored as `stub_points` (hardens alpha, since
`cross_marker_perception.py`'s `_unweighted_principal_angle` consumes these directly).

**Validated** offline (same raw frames, before/after) and via a fresh live SITL landing
(`linear`, try 2, ON-PLATFORM lat=0.157m): the ~183 deg flip no longer occurs; the previously
falsely-"ok" shadow-corrupted final frame now correctly reports `Detection Status="miss"` instead
of a silently-biased center. Does NOT fix detection *through* heavy shadow occlusion (that's a
genuine occlusion, not a bug) — converts a silent wrong answer into an honest miss instead.

## Open items / natural next steps
- World-side fix not applied: disabling `cast_shadows` on the drone model, or repositioning
  `sunUTC`, would prevent the shadow from falling on the marker at all. Not done this session.
- Whether this shadow issue matters for **perception-mode** (non-GT-feedback) flights specifically
  is unconfirmed — all validation this session used `PLASMC_GT_FEEDBACK=1` (control bypassed
  perception; only logging was checked).
- Cross-marker's own background texture history (see [[feedback_cross_marker_texture_history]])
  turned out to be a live tangent, not directly related to the shadow fix, but touches the same
  "reject non-marker content" theme.
