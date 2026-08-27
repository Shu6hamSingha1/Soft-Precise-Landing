---
name: project_20260827_framerate_and_h_texture_investigation
description: "2026-08-27: root-caused cross-marker's low process_frame() rate (needed >=30Hz), dropped camera to 320x240 (fx=fy=270->135, matching MATLAB's native scale) to fix it (n=3 confirms ~14.7-22.6Hz -> ~37.9Hz), then found h (optical flow) never actually uses the textured background at all -- a design gap against the ORIGINAL intent (s from marker geometry, h from surface-texture flow, so the same algorithm generalizes to a cross drawn on any real-world textured surface). A first fix attempt (precise rotated 'plate mask' via cv2.minAreaRect) was rejected -- wrong geometric basis (X diagonal != plate edges) AND wrong problem (no plate/ground distinction exists in real deployment). Correct direction: extent-based ROI around the marker, not a precise plate quad, mirroring img_data.py's ArUco ring-flow design intent."
metadata: 
  node_type: memory
  type: project
  originSessionId: a486c0ea-32ca-4384-97c7-b0136fa1c290
  modified: 2026-08-27T13:21:07.589Z
---

## Frame-rate root-cause (need >=30Hz, was 14.7-23.8Hz)

n=3 reps each, IC5, GT-feedback, `cross_marker` world, `WORLD=cross_marker MARKER_TYPE=cross`:

| texture | mean process_frame() rate |
|---|---|
| old 1024px | 22.6 Hz (22.0/22.1/23.8) |
| hires 3072px | 14.7 Hz (12.9/18.4/12.9) |

Ruled OUT as the cause: Python-side algorithmic cost from background noise. Traced the
exact code path -- `cv2.Canny`+`cv2.HoughLinesP` run on the ALREADY color-thresholded/
shape-isolated BINARY mask (background zeroed out before this point), and
`cv2.goodFeaturesToTrack` is restricted via its `mask=` param to a 2-4px dilated band
around the same isolated line mask. Background texture structurally cannot reach either
step -- ruled out per-frame CPU cost as texture-dependent.

Most likely real cause: GPU-side rendering/texture-bandwidth cost on this host's modest
integrated GPU (Intel UHD 630, confirmed via `/dev/dri/renderD128`, no discrete GPU) --
a 9x larger texture asset (3072² vs 1024²) costs more to sample/mip during rendering,
independent of what the Python pipeline later does with the fixed 640x480 output. NOT
fully confirmed via profiling (recommended but skipped per user direction to go straight
to the resolution fix) -- if revisited, instrument `detect()`/`process_frame()` with
per-stage timing before assuming this is settled.

Also notable: even the FASTEST condition (22.6Hz, old texture) was well below the
documented ~62Hz raw camera feed baseline (`CLAUDE.md`, measured on the plain `aruco`
world) -- meaning EITHER the cross-marker's full PBR metal/roughness material is
meaningfully more expensive to render than ArUco's simpler material, or the Python
detection pipeline itself has an inherent ~20-25Hz throughput ceiling. Not disambiguated.

## Fix: dropped camera 640x480 -> 320x240 (fx=fy=270 -> 135)

**This is a RESTORE, not a new resolution** -- MATLAB's own `Constants.m` already uses
f=135 at 320x240 (img_data.py's own superseded comment said as much); PX4/Gazebo had
deliberately DOUBLED to 640x480 for pixel-margin headroom. Files changed (all backed up
before editing):
- `~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` (OUTSIDE this repo,
  not git-tracked) -- width/height 640/480 -> 320/240, same `horizontal_fov=1.74`.
  Backup: `model.sdf.bak_before_320x240_20260827`.
- `img_data.py`: `fx=fy=270` -> `135`.
- `controller.py`: `_rho_fov_0`/`_rho_fov_inf` halved (210/290/80 -> 105/145/40) -- these
  were themselves exactly 2x MATLAB's native values by design (file's own comment).
- `cross_marker_perception.py`: `GFT_MIN_DIST` (6->3), `MASK_DILATE_PX` (4->2),
  `FLOW_BOUNDARY_MARGIN_PX` (20->10), `RING_CENTER_MIN_R_PX` (20->10) -- proportional
  scale, explicitly flagged in-code as NOT independently re-validated at this resolution.

**Result (n=3, IC5, GT-feedback): 37.6/38.3/37.8 Hz, mean ~37.9Hz** -- target met, clean,
tight spread.

**But a real, measured cost**: detect-ok rate dropped (77-92% at 640x480-lowres ->
63-65% at 320x240), and `lt2_angle_clusters`/`hough_lt2_lines`/`insufficient_fit_points`
all rose substantially -- the predicted stroke-width-margin risk (halving resolution
halves the marker's stroke width in pixels, the SAME axis that forced the original
1.0m->3.0m marker resize) materialized empirically, not just theoretically.

**BLOCKING before this is usable for real (non-GT-feedback) landing validation:**
1. `sensor_cal_hw`/`sensor_cal_s` in BOTH `img_data.py` and `cross_marker_perception.py`
   were derived from 640x480/fx=270 recordings -- NOT recalibrated for 320x240. GT-
   feedback masked this in all testing so far (control never touches perception). See
   the `io-calibration` skill before trusting any perception-mode flow number at this
   resolution.
2. Detection-reliability degradation above is unaddressed -- no detector-threshold
   retuning attempted yet.

ArUco-side pixel-domain constants in `img_data.py` were NOT touched (out of scope per
the standing `feedback_aruco_perception_scope` rule -- ArUco is comparison-only) -- they
are now stale for this resolution if anyone runs ArUco.

## h (optical flow) does NOT use the textured background -- confirmed structurally

Traced exactly: `cross_marker_detector.py::detect()` builds `mask = cv2.inRange(hsv,
lower, upper)` (isolates ONLY the near-black cross/stub pixels), then morphology/ROI/
shape-isolation, all still binary and background-free. `cross_marker_perception.py`'s
`_dilate_mask()` grows it by a few px. EVERY point-sampling call --
`_sample_flow_points_ring()` (default) and its `_sample_flow_points_unconstrained()`
fallback -- calls `cv2.goodFeaturesToTrack(gray, mask=<that dilated line-only mask>)`.
The Canny+Hough line-detection step also runs on the isolated BINARY mask, not the raw
grayscale. Background speckle is structurally excluded from every step. Confirmed this
is texture-independent (reproduced identically swapping in the old 1024px texture,
see [[reference_finalized_montage_video_layout]]'s sibling investigation) -- the fine
speckle background built by `make_cross_marker_hirestex.py` has had NO consumer since
it was created; it has never actually fed any point-sampling code.

**⭐ Design-intent correction from the user (2026-08-27), important for all future
cross-marker perception work:** `s` (centroid+orientation) is meant to come from the
printed cross geometry (decode-free, robust, precise). `h` (optical flow) is meant to
come from optical flow across the TEXTURED SURFACE itself, not the printed lines --
this is WHY a textured background exists at all instead of a blank one: it stands in
for whatever natural real-world texture (concrete, grass, dirt, a rug) a real deployed
cross-marker would be drawn on, so the SAME algorithm generalizes to the field without
retuning. The current line-only restriction is a GAP against this intent, not a
neutral design choice -- see `_sample_flow_points_ring`'s own docstring, which
explicitly contrasts itself with `img_data.py`'s ArUco ring-flow design ("ArUco...
sampling FIXED ring stations over the whole (generally-textured) background. The
cross-marker's candidate region is instead restricted to the thin dilated-mask band...
since fixed station positions would mostly land on UNTEXTURED background") -- that
restriction was a workaround for the background being BLANK at the time it was
written, not a permanent design decision. Now that real texture exists, ArUco's
"sample broadly around the marker" approach is the right model to follow for the
cross-marker too.

## First fix attempt REJECTED: precise rotated "plate mask" (cv2.minAreaRect)

Tried: `cv2.minAreaRect()` on `det.isolated_mask` (the cross-line pixels), scaled up by
`WHOLE_PLATE_SCALE` (1.3x), to estimate the whole plate's extent for GFT sampling.
Function `cmd.plate_mask_from_detection()` added to `cross_marker_detector.py`, gated
by `CROSS_WHOLE_PLATE_FLOW`/`CROSS_WHOLE_PLATE_SCALE` env vars (NOT wired into the live
solve). Visual check (per user's own "verify visually before trusting it" call) on real
recorded frames found it BROKEN, and inconsistently so:
- One frame: plate-mask rotated to match the X's own diagonal, badly misaligned --
  spills off two corners onto the ground while missing real plate area on the other
  two sides.
- Another frame: coincidentally well-aligned.
- Root cause: the physical plate is a SQUARE; the arms are drawn along its DIAGONALS,
  not parallel to its edges. `minAreaRect` fits whatever rectangle tightly bounds the
  X shape itself, which only coincidentally matches the plate's true orientation
  sometimes (depends on which arm-tip pixels dominate that frame's fit) -- not a
  tunable-scale problem, the geometric basis is wrong.
- Reproduced IDENTICALLY with the old 1024px texture (see above) -- confirms this is a
  pure line-geometry bug, unrelated to which texture file is used.
- ALSO conceptually wrong per the design-intent correction above: there is no "plate
  vs ground" distinction in a real deployment (a cross painted on concrete has no
  separate bounded "plate" at all) -- fitting a precise plate quad solves a sim-only
  problem with no real-world equivalent.

**Correct direction (in progress, not yet implemented as of this memory):** extent-
based ROI around the marker's own detected size in the CURRENT frame -- not a precisely
oriented plate quad, not a fixed pixel margin. Mirrors ArUco's own ring-flow design
(scale-free, sized to the target's own apparent extent). Apply LK/GFT within that
extent-sized region, feeding the SAME existing ring-band/sector + interaction-matrix
solve machinery already built -- no changes needed to `_fill_A`/the lstsq solve itself,
it's already generic to point positions/velocities regardless of source region.

## Uncommitted at various points, check current state before assuming

`cross_marker_detector.py`'s `plate_mask_from_detection()`/`WHOLE_PLATE_FLOW`/
`WHOLE_PLATE_SCALE` are dead code (unused by the live solve) if the extent-based
rewrite replaces rather than builds on them -- check whether they were removed or
superseded before reusing.
