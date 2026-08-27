---
name: reference_finalized_montage_video_layout
description: "The finalized 2-PiP landing montage video layout (chase + s/alpha overlay + h overlay + 3D/plots), the exact tool invocation to reproduce it, and what its panels mean."
metadata: 
  node_type: memory
  type: reference
  originSessionId: a486c0ea-32ca-4384-97c7-b0136fa1c290
  modified: 2026-08-26T19:18:16.060Z
---

The finalized montage layout (reference example:
`test_data/Test_Videos/montage_final_lowangle_touchdowncrop_cross.mp4`, reproduced
2026-08-27 for a fresh rover-static-IC1 run as
`test_data/Test_Videos/montage_final_rover_static_ic1_touchdowncrop.mp4`) is TWO
stacked PiPs on the left (not one), plus the existing right-side 3D/plot panel:

- **top-left, labeled "onboard"**: `tools/overlay_image_features.py --channels s,alpha`
  overlay — orange+cyan lines are the two detected cross-arm lines
  (`_robust_fit_line`/`_line_intersection` in `cross_marker_detector.py`), green
  crosshair + "s" label is the centroid, magenta/gray arrow is the alpha
  orientation reading (gray+dashed = held/stale), yellow reference arrow is the
  alpha=0 direction, white arc+label is the alpha angle itself. Text box top-left
  of the panel shows live `s = (...)` and `alpha = ... deg`.
- **bottom-left, labeled "onboard2"**: `tools/overlay_image_features.py --channels h`
  overlay — small yellow arrows are the per-point tracked flow field, red arrow is
  the aggregate lateral h_xy, the colored ring at centroid is h_z/loom (red=
  expanding/positive, cyan=contracting/negative, ring radius+thickness scale with
  |h_z|). Text box shows live `h = (hx, hy, hz)`.
- **right side, unchanged from the single-PiP layout**: "UAV & target (3D)" trajectory
  plot (top) + `|rel. position|` and `|rel. velocity|` time-series (bottom two),
  alpha-blended over the chase view per `--plot-corner`/`--plot-alpha`.
- **center**: the world chase-cam view, optionally digitally cropped tighter as
  touchdown approaches via `--chase-crop-touchdown` (ramps from `--chase-crop`,
  default 1.0/no-crop, down to the touchdown value over the last
  `--chase-crop-ramp-s` seconds before touchdown, then holds through the
  `--tail-s` post-touchdown freeze).

## Exact reproduction recipe

Requires a run recorded with `CHASE_CAM=1 IMG_RECORD=1` (chase video + raw onboard
down-cam video both saved), then THREE tool invocations:

```bash
# 1. Build the two overlay videos from the raw recorded down-cam video + run dir
tools/overlay_image_features.py --video "<drone.mp4>" --run "<run_dir>/" \
    --out overlay_s_alpha.mp4 --channels s,alpha
tools/overlay_image_features.py --video "<drone.mp4>" --run "<run_dir>/" \
    --out overlay_h.mp4 --channels h

# 2. Composite into the final montage (pip-corner=tl/pip-corner2=bl and the
#    onboard/onboard2 labels are ALREADY the argparse defaults -- don't need to
#    pass them explicitly)
tools/make_landing_montage.py --chase "<chase.mp4>" \
    --drone overlay_s_alpha.mp4 --drone2 overlay_h.mp4 \
    --run "<run_dir>/" --out montage_final_<label>.mp4 \
    --tail-s 1.0 --chase-crop-touchdown 0.5 --chase-crop-ramp-s 3.0
```

See [[project_20260824_cross_marker_montage_overlay_robustness]] and
[[project_20260825_overlay_detection_artifact_logging]] for the overlay tool's own
development history.

**CAVEAT (2026-08-27, found while reproducing this layout):** `--chase-crop-touchdown
0.5` did NOT produce a visually obvious zoom-in on a `rover_cross`-world run — the
chase view looked similarly scaled at touchdown as at descent-start. Likely because
that world's chase-cam SDF already has a wide native FOV, so a 0.5 crop reads as
subtle rather than dramatic. The exact crop value used for the original reference
video (`montage_final_lowangle_touchdowncrop_cross.mp4`) was never recorded anywhere
(no checked-in script builds it) and could not be recovered — 0.5 was a best-effort
guess. If a stronger zoom effect is wanted, try something tighter (e.g. 0.2-0.3) and
re-check visually; don't assume 0.5 is the "right" value from this precedent.

**Also note (same investigation thread):** `tools/overlay_image_features.py`'s
`have_logged_pts` path (drawing the EXACT live flow correspondences instead of
re-detecting/re-tracking from the possibly lossy recorded video) requires
`cross_marker_perception.py::getLogData()` to expose `"Line Points I"`/`"Line Points
J"`/`"Stub Points"`/`"Flow Points Prev Px"`/`"Flow Points Curr Px"` -- as of
2026-08-27 these keys are NOT present in the live `getLogData()` dict, so every
current overlay (including both examples above) is built via the offline
re-detection/re-tracking fallback, not the flight's own real correspondences. Check
whether this has been added before trusting an overlay's drawn points as "what the
live flight actually saw."
