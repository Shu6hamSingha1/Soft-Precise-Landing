---
name: project_20260827_framerate_and_h_texture_investigation
description: "2026-08-27 + 2026-08-28 follow-on. (1) camera 640x480->320x240 (fx 270->135). (2) h optical flow never used the textured background -- fixed with extent-ROI + line-exclusion + multiscale GFT through the real _solve_jacobian, GT-validated r=0.66-0.78 on resolvable-grain texture, weak on sub-resolution grain (a resolvability wall). (3) 2026-08-28: the '37.9Hz' figure below is an OUTLIER -- real process_frame() rate was ~17.5Hz; root-caused to two marker-size-QUADRATIC hot spots (detect()'s Canny/Hough, multiscale_good_features' GFT-over-full-extent + O(n^2) dedup), NOT GPU/PBR/Python-ceiling; fixed with bbox-crop + working-resolution caps => 46.5Hz live. (4) hybrid bg-flow built (CROSS_BG_FLOW_HYBRID, default OFF): CLAHE+fwd-back-LK on det.ok path, resid-gated bbox-only dense-DIS fallback on miss frames. Reusable GT-corr harness: tools/validate_bgflow_corr.py. See the 2026-08-28 FOLLOW-ON section at the bottom."
metadata: 
  node_type: memory
  type: project
  originSessionId: a486c0ea-32ca-4384-97c7-b0136fa1c290
  modified: 2026-08-27T17:31:53.955Z
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

## ⭐⭐ SUPERSEDES THE ABOVE "in progress" note -- extent-based approach built AND
VALIDATED against real GT correlation (2026-08-27, same day, later in the session)

`plate_mask_from_detection()` was fully REPLACED (not just supplemented) by
`extent_mask_from_detection()` in `cross_marker_detector.py`: axis-aligned box
centered on `det.mask_bbox`, scaled by `WHOLE_PLATE_SCALE` (1.3x) -- NOT rotated to
the cross's fitted line angle. Visually re-verified against real frames from BOTH
textures (including the exact frame that showed the worst 45-degree plate-mask
misalignment): consistently close-fitting, only minor corner spillover, no more
systematic edge-missing. `WHOLE_PLATE_FLOW`/`WHOLE_PLATE_SCALE` env vars carried over
from the rejected attempt, same meaning.

**Second empirical finding, also from real frames (not just theory): widening the
mask alone does NOT make GFT prefer background texture over line edges.**
`cv2.goodFeaturesToTrack` ranks by Shi-Tomasi corner strength; the printed line edges
(~180-value jump) are a far stronger signal than background speckle (std~10)
wherever both are eligible in the same mask, so corners kept clustering on the lines
even inside the wider extent box. Fix: `background_mask_from_detection()` = extent
mask MINUS a dilated (`LINE_EXCLUDE_DILATE_PX=3`) copy of `det.isolated_mask` --
physically excludes line-adjacent pixels from candidacy, which ALSO makes GFT's
internal quality threshold auto-adapt to whatever the background actually offers
(no longer judged against an unbeatable line gradient in the same search space).

**Third finding: single-scale GFT has a texture-grain blind spot.** A/B on old
(1024px, coarse grain) vs current-live (3072px, fine grain) texture, same 320x240
camera: old texture's coarser blobs registered as real GFT corners; hires texture's
finer grain mostly didn't (too fine for GFT's window to resolve after downsampling
to the delivered camera resolution -- NOT a texture-strength difference, both have
the same measured background std~10; it's a SPATIAL SCALE problem). Fix:
`multiscale_good_features()` -- runs GFT at MULTISCALE_LEVELS (default 1x/2x/4x
downsample) within the background mask, pools + dedups results (finer-scale points
win ties, since they carry less position uncertainty). Full-video quantitative
check (not cherry-picked frames): mean distance-from-nearest-line-pixel ~29px for
BOTH textures (confirms genuine background sampling, not just occasional luck);
point AVAILABILITY differs sharply (old: mean 125/frame, min 22; hires: mean 52,
min 4) -- texture quality affects how much there is to find, not where points land
once the line-exclusion is in place.

## ⭐⭐⭐ GT-CORRELATION VALIDATION (2026-08-27) -- the approach genuinely works,
with a real, well-quantified limiting factor

**First correlation attempt was NEGATIVE (~0 on all axes) and WRONG -- root cause was
a time-sync bug, not a flaw in the flow approach.** Compared `Img_Data`'s per-frame
raw flow proxy against `Control_Data.npy`'s `h(t)` (exact GT under
`PLASMC_GT_FEEDBACK=1`) using a fractional-elapsed-time approximation, because
`Img_Data['Time']` and `Control_Data['t']` looked like mismatched clocks (different
apparent "durations" measured earlier in this same investigation thread, which led
to an unnecessary two-anchor arm-to-end RESCALE technique built for the touchdown-
detect false-positive work -- THAT rescale was solving a problem that doesn't
actually exist, see correction below).

**Root cause, found by reading `notebooks/plotter_output_calibration.ipynb` (per
user instruction to check the established methodology before re-guessing):**
`Ground_Truth.npy` has a `Start Time` field; the validated alignment is
`img_t_rel = Img_Data['Time'] - gt['Start Time']`. `Control_Data['t']` is ALSO on
this same absolute clock -- confirmed empirically, `Control_Data['t'][0] ==
gt['Start Time']` EXACTLY. **There is no clock-RATE mismatch between these logs at
all** -- the large apparent "duration" differences found earlier in this session
(e.g. Img_Data spanning 31.19s vs Control_Data's 13.64s for the same run) are
entirely explained by `Img_Data` logging starting ~17.7s earlier (thread creation,
includes pre-engage frames) while `Control_Data`/`Ground_Truth` only start at
`CONTROLLER_READY` -- their END points line up to within 0.12s once both are
offset by `gt['Start Time']`. **The earlier two-anchor rescale approach (used
ad-hoc in conversation for the touchdown-detect investigation, never written to a
persistent memory file) was solving a problem that doesn't exist -- a plain
subtraction is sufficient and correct.** [[feedback_imgdata_gt_clock_skew]] (shared
memory, 2026-06-04) independently found the same kind of offset earlier and advised
avoiding Img_Data-vs-GT comparison entirely (measuring "Δorigin≈29.8s" without
naming/using `Start Time`) -- that avoidance advice is now SUPERSEDED for any
recording that has `Ground_Truth['Start Time']`: a direct, validated correction
exists and should be used instead of avoiding the comparison. See that file's own
2026-08-27 update.

**Second alignment gotcha, found empirically (assert failure caught it immediately):**
the IMG_RECORD video does NOT contain every `Img_Data.npy` frame -- video recording
only starts at `CONTROLLER_READY` (same gate as the pre-engage-frames issue above),
so **video frame index `i` == `Img_Data` index `(n_pre_engage + i)`**, where
`n_pre_engage = (img_t_rel < 0).sum()`. Confirmed exactly: 995 total Img_Data
entries, 743 with `t<0`, 252 with `t>=0` == 252 video frames, exact match.

**With BOTH fixes applied, real correlation appears --** first with a simplified
raw-pixel radial/lateral proxy (no de-rotation): old-texture corr(raw_radial, GT
h_z)=-0.630, corr(raw_lat_x, GT h_x)=+0.659, corr(raw_lat_y, GT h_y)=-0.632 (hires
texture: same sign pattern, weaker, 0.24-0.30 -- consistent with the point-
availability gap above, not a different relationship). The alternating-sign pattern
was suspected to be a raw-pixel-vs-V-frame axis-convention artifact, not a real
physical relationship.

**Confirmed by re-running through the REAL `_solve_jacobian`** (proper V-frame
reprojection via `_getVirtualPts` + interaction-matrix lstsq, i.e. the actual
production de-rotation, not a proxy) on the SAME background-derived point
correspondences: old-texture corr(h_x)=+0.664, corr(h_y)=+0.685, corr(h_z)=+0.764 --
all positive now (confirms the sign flip WAS an axis-convention artifact of the raw
proxy), strong, clean correlation with GT on all 3 axes. **Hires texture stayed
weak/negative (-0.20, -0.20, +0.12) even with proper de-rotation** -- this pins the
bottleneck precisely on point-tracking quality (garbage-in-garbage-out: de-rotation
can't recover signal from unreliable input correspondences), not on any remaining
math/rotation-handling defect in the approach.

**Bottom line: the extent-ROI + line-exclusion + multiscale approach, fed through
the real production interaction-matrix solve, produces a genuinely valid, GT-
correlated `h` signal -- PROVIDED the surface texture's grain is coarse enough to
survive the camera's delivered resolution.** That's a real, generalizable
capability with a precisely quantified limiting factor (grain-vs-resolution
resolvability), not an open question.

**Wired into the LIVE `h` solve, BAKED ON by default**, same session:
`cross_marker_perception.py`'s `CROSS_BG_FLOW` (default `"1"`), a new
`_compute_hw_bgflow()` method (deliberately separate from the mature persistent-
tracking `_compute_hw()`, so it stays faithful to exactly the method that was GT-
validated -- fresh multiscale GFT every pair, no cross-frame point-pool tracking),
falling back to the existing line-mask `_compute_hw()` whenever the background path
can't produce a result that frame (no background mask, <5 tracked points, etc.) --
can't make behavior worse than the pre-2026-08-27 baseline, only better or a no-op.
Verified live both ways (flag on/off) with real SITL reps: no exceptions, default-
off path unchanged.

**Baked to default WITHOUT the n>=5 IC1-5 gate, per explicit user direction: that
gate (`feedback_ic_validation`) applies to control/gain tuning, not perception
fixes** -- a perception fix with real correctness evidence (the GT-correlation
check above, not just an IC-landing-outcome check) can default without it. Worth
recording as a precedent/clarification for `feedback_ic_validation` itself if a
similar perception-vs-control gating question comes up again.

## Uncommitted at various points, check current state before assuming

`cross_marker_detector.py`'s `extent_mask_from_detection()`, `background_mask_from_detection()`,
`multiscale_good_features()` (all validated per above) -- check whether they were
superseded before reusing.

## ⭐⭐⭐ LIVE TEXTURE FILE IDENTITY, EXPLICIT (added 2026-08-27, after a different
session got confused because this wasn't stated plainly enough above)

**As of this session's end, the LIVE `~/PX4-Autopilot/Tools/simulation/gz/models/
cross_marker/cross_marker.png` is the ORIGINAL 1024px texture (742,542 bytes,
mean~193.7/std~10.1 background) -- i.e. what this memory calls "old 1024px"
throughout, NOT `cross_marker_hires.png` (3072px, 6,689,220 bytes).** Verified by
MD5: the live file is byte-identical to `cross_marker.png.bak_before_hirestex_
20260809`. This is EXACTLY the file the "old 1024px" GT-correlation A/B in this
memory was run against (r=0.66-0.76, the STRONG case) -- **we are finalizing on
the texture that already validated well, not the weak hires case.**

**Known trap for anyone re-deriving this: `cross_marker.png` and
`cross_marker.png.bak_before_hirestex_20260809` share an identical mtime
(2026-08-09 14:09) -- this is a `cp -p` artifact (every restore in this session
used `cp -p`, which preserves the SOURCE's timestamp), NOT evidence that
`cross_marker.png` is a downscaled copy of the hires asset.** The hires backups
(`cross_marker.png.bak_before_lowres_test_20260827` etc., 6,689,220 bytes) share
their OWN distinct shared timestamp (2026-08-10 14:15, the original hires-
generation date) for the same `cp -p` reason -- two genuinely different assets,
each internally consistent in mtime, not one derived from the other via a fresh
resize.

**Why the live state ended up on the old texture: an unintentional leftover, not
a deliberate final choice.** Mid-session this texture was swapped back and forth
several times for A/B testing (plate-mask geometry check, LK-in-extent-box
texture-resolvability check, GT-correlation check) -- the LAST swap in the
session's texture-comparison thread landed on old/1024px and was never swapped
back to hires afterward. **This means the live sim currently does NOT have the
near-touchdown blur mitigation that `cross_marker_hires.png` was originally built
for** (see the earlier "texture-resolution crossover" investigation, predating
this memory file) -- predicted crossover for old-1024px at the CURRENT fx=135 is
~0.396m, still slightly above the ~0.34m touchdown camera-to-marker distance, so
near-touchdown blur is a real, still-open, unresolved cost of leaving it here.
**This is an open decision for whoever picks this up, not something already
settled:** keep old texture (validated background-flow correlation, real near-
touchdown blur) vs. restore hires (no validated background-flow correlation yet
at that grain, better near-touchdown sharpness). Don't assume either answer;
ask the user if it matters for the immediate task, or re-run the GT-correlation
check against whichever file is actually live if picking this up cold.

## NEXT SESSION: explicit next task (user, 2026-08-27 session end)

**Improve extent-ROI + line-exclusion + multiscale GFT + `_solve_jacobian` for a
LARGER VARIETY of textures**, not just the two grain sizes tested so far (old
1024px-native / hires 3072px-native, both synthetic speckle at the same base
statistics). This session only established the approach works when texture grain
survives the camera's delivered resolution and is weak when it doesn't -- it has
NOT been tested against textures with different STATISTICAL character (not just
grain size): e.g. structured/repetitive patterns (aliasing risk for LK
correspondence -- the ArUco ring-flow's own multi-scale/self-similarity concerns
apply here too), low-contrast or non-Gaussian real-world-like textures (concrete,
grass, gravel), or textures with directional bias (wood grain, tiling). Starting
points: `multiscale_good_features`'s `MULTISCALE_LEVELS` (currently a fixed
1,2,4 list -- may need to be wider or content-adaptive), `LINE_EXCLUDE_DILATE_PX`
(currently a fixed 3px margin, untested against thicker/thinner drawn lines),
and `WHOLE_PLATE_SCALE` (1.3x, untested against markers where the true plate-to-
line-extent ratio differs from this session's specific marker design). The
GT-correlation methodology from this session (proper time-sync via
`Ground_Truth['Start Time']`, real `_solve_jacobian`, not a raw-pixel proxy) is
the validated way to check any of this -- reuse it, don't re-derive from scratch.

---

## 2026-08-28 FOLLOW-ON (session continuation) -- perf root-cause CORRECTED, hybrid bg-flow built, reusable harness

### ⭐ The "37.9 Hz" frame-rate figure above is MISLEADING -- and the "PBR-material-cost vs ~20-25Hz-Python-ceiling, not disambiguated" question is now ANSWERED

Fresh headless cross_marker GT-FB reps (default IC, `CROSS_BG_FLOW=1` live default)
measured `process_frame()` at **17.5 Hz**, not ~38. The 2026-08-27 `20-33-35` rep
that this file cites as the r=0.66-0.76 validation rep was ITSELF only **18.6 Hz**
(median frame dt 20ms but p90=172ms -- hitching). The "37.6/38.3/37.8 Hz n=3 IC5"
figure is an outlier / different conditions; ~18 Hz is this world's normal on this
host.

**Root cause (profiled, offline, on real recorded frames -- NOT GPU/render/PBR, NOT
a Python throughput ceiling):** two marker-pixel-size-QUADRATIC hot spots that spike
near touchdown when the marker fills the frame:
- `cross_marker_detector.detect()` / `_detect_core`: `cv2.Canny` + `cv2.HoughLinesP`
  + per-segment Python angle loop + `_filter_segments_by_corner_join` -- **mean 23ms,
  p90 89ms, max 195ms**. (An earlier 2.3ms micro-benchmark was misleading -- it hit
  the warm tracked-crop fast path on small-marker mid-descent frames only.)
- `cross_marker_detector.multiscale_good_features()` (wired live via `_compute_hw_bgflow`
  since the 2026-08-27 `CROSS_BG_FLOW` bake -- which was "verified no exceptions" but
  NEVER rate-checked): `goodFeaturesToTrack(qualityLevel=0.01)` x3 scales over the
  whole extent-box mask + an **O(n^2)** pairwise dedup -- **mean 37ms/frame** near the
  deck (thousands of weak candidates found + sorted + dedup'd).
Raw Gazebo down-cam publish rate measured directly (`gz topic -e`): **59.7 Hz** --
the camera/render side was never the limit. Whole Python perception pipeline pure
compute (detect+GFT+CLAHE+DIS) is **~7ms => ~140Hz capable**.

### FIX (both default-ON -- pure speedups, no GT-correlation regression on 3 reps, detect-ok slightly UP 75->78 / 97->99 / 108->108)

`cross_marker_detector.py`:
- `multiscale_good_features()`: crop to mask bbox first -> integer-downscale so long
  side <= `CROSS_GFT_WORK_MAX_PX` (200) -> multiscale loop on THAT -> O(n) grid-cell
  dedup (`_grid_dedup`, replaces the O(n^2) pairwise) -> map points back (origin+scale).
  `CROSS_GFT_QUALITY` default 0.01 -> **0.03**. Docstring's stale "NOT YET WIRED"
  removed (it IS wired). **`_compute_hw_bgflow` 37ms -> 2.6ms**, now marker-size-
  independent.
- `_detect_core_capped()` + `_scale_detection()`: the **tracked-crop fast path only**
  (acquisition full-frame path left EXACT) downscales the crop by an integer factor
  when its long side > `CROSS_DETECT_WORK_MAX_PX` (200), runs `_detect_core`, scales
  center/bbox/line_points/stub_points back and NEAREST-resizes `isolated_mask` back to
  crop size before the existing `_shift_detection`. Line geometry is scale-invariant.
  **`detect()` 23ms -> 6.5ms** (p90 89->22, max 195->40).

**Live result: `process_frame()` 17.5 Hz -> 46.5 Hz** (headless, CROSS_BG_FLOW_HYBRID=1,
detect-ok 100%, landing SOFT+PRECISE xy=0.004m). >=30Hz requirement MET with margin.
Offline `process_frame` is ~95-125 Hz capable; p90 28ms, max 45ms.

### HYBRID BG-FLOW (`CROSS_BG_FLOW_HYBRID`, default OFF pending full validation)

Aimed at texture-SCALE robustness (real surfaces = any grain; can't dictate the
texture). Built + measured via `tools/validate_bgflow_corr.py` (the 2026-08-27 ad-hoc
GT-correlation check turned into a reusable tool -- time-sync via
`Ground_Truth['Start Time']` plain subtraction, video index i <-> Img_Data (n_pre+i),
real `_solve_jacobian`; 6 strategies + ok/notok correlation split).

1. **`det.ok` path** (`_compute_hw_bgflow`, gated by `CROSS_BGF_CLAHE`/`CROSS_BGF_FB`):
   CLAHE-normalise the ROI before GFT (removes absolute-contrast dependence --
   concrete/gravel/low-light all normalise), + forward-backward LK consistency
   rejection (`CROSS_BGF_FB_THRESH_PX` 0.7). On the validated old-1024px rep:
   **h_z corr 0.28 -> 0.71-0.78**, point-availability floor 15 -> 44/frame. No-op /
   near-zero-noise on texture too fine to resolve (hires 3072px stays ~0 -- the
   resolvability wall, unchanged).
2. **`det.ok == False` path** (`_compute_hw_bgflow_fallback`, new): run bg-flow anyway
   from `det.mask_bbox` alone via
   `cross_marker_detector.background_mask_bboxonly_from_detection()` (extent box minus
   a THRESHOLD-derived near-black dilated line mask -- no dependence on
   `det.isolated_mask`) + dense `cv2.DISOpticalFlow` PRESET_FAST, grid-sampled.
   Injected into the hw KF as a real measurement in `process_frame`'s `not det.ok`
   branch (the lone `_kf_update_hw(None,t)`); **`self._ok`/s/alpha/visibility stay
   untouched -- flow-only**. GATED on `_solve_jacobian`'s `rel_resid <=
   CROSS_BGF_RESID_GATE` (0.45): on resolvable texture recovers the near-touchdown
   overflow/loom phase the `det.ok` subset misses ENTIRELY (one rep h_z 0.08 -> 0.70);
   on sub-resolution grain the gate keeps it to near-zero noise instead of a confident
   WRONG-SIGN h_z (measured: gate 0.6 -> -0.02, gate 0.4 -> +0.10, ungated -> -0.18).
   ~70% of otherwise-discarded frames become usable.

Best-so-far config per harness: `det.ok` -> CLAHE+FB sparse (or dense DIS, marginally
better but ~2700 pts/frame), `notok` -> resid-gated bbox dense. `dense_dis` on `det.ok`
frames gave old1024 +0.81/+0.66/+0.73.

### Uncommitted-state note SUPERSEDED
`extent_mask_from_detection` / `background_mask_from_detection` / `multiscale_good_features`
are all COMMITTED and live-wired now. New this session: `background_mask_bboxonly_from_detection`,
`_detect_core_capped`, `_scale_detection`, `_grid_dedup` (detector);
`_compute_hw_bgflow_fallback`, `_bgf_clahe_pair`, `_bgf_lk_fb`, the `CROSS_BG_FLOW_HYBRID`
flag family (perception); `tools/validate_bgflow_corr.py`.

### STILL OPEN / NEXT
- Full on/off harness comparison on a FRESH hybrid rep (recorded rep launched at
  session end) -> then decide whether to default `CROSS_BG_FLOW_HYBRID` ON. Perception
  fix with GT-correlation evidence can default without the n>=5 IC gate (precedent
  already set for `CROSS_BG_FLOW`).
- The `MULTISCALE_LEVELS` / `LINE_EXCLUDE_DILATE_PX` / `WHOLE_PLATE_SCALE` content-
  adaptivity from the original "NEXT SESSION" note above is still untouched -- the
  hybrid work targeted contrast + correspondence-rejection + coverage, not scale-
  adaptivity per se.
- Old-vs-hires texture decision (near-touchdown blur mitigation) still open -- the
  perf + hybrid work helps EITHER texture, doesn't force the choice.

---

## 2026-08-28 (cont.) -- "why is h correlation low" dig: it mostly ISN'T + an angular-filter dead-end

Committed: f6e102d (infra), following bf7af54 (perf fix).

### The h correlation was never really low -- two things stacked

1. **Offline-harness artifact.** `tools/validate_bgflow_corr.py` re-solves `Img_Data`'s
   logged flow points through `_solve_jacobian`, but `Img_Data` logged NO IMU body-rate,
   so the re-solve fell back to the rank-deficient full-6-unknown lstsq (Wy aliases Tx)
   instead of the LIVE gyro-derotated 4-unknown solve. This alone understated h_z by
   ~0.2-0.3 and added noise on every axis. **FIX (f6e102d): `getLogData()` now emits
   "IMU AngVel" / "FPS" / "Stamp"** (all already accumulated per frame, just weren't
   exposed). Re-run offline analysis with these -> pass `prev_angvel`/`curr_angvel` to
   `_solve_jacobian`.

2. **Correlate the LIVE `Img_Data['h_V']` against `Control_Data['h(t)']` directly** (no
   re-solve) and the real picture appears: **while the marker is a normal size, h_x / h_y
   / h_z all correlate +0.976 / +0.979 / +0.993 with GT** (ic2, `MARKER_EXTENT_PX < 200`).
   The whole-flight Pearson is dragged down ENTIRELY by the terminal phase.

### Terminal-phase collapse (the real, narrow issue)

Once the marker overflows the FoV near touchdown (`MARKER_EXTENT_PX >= ~200` at 320x240):
- `h_x` std goes 0.019 (early) -> 0.320 (terminal), a 17x jump, on a CENTERED rep where
  true h_xy ~= 0.
- The noise does NOT correlate with tilt or yaw-rate (`corr(|h_x|, tilt) = -0.18`) ->
  **not a derotation error**; derotation is working.
- It DOES scale with marker size (`corr(|h_x|, extent_px) = +0.46`) and with corner
  starvation (`corr(|h_x|, 1/N_corners) = +0.72`).
- Mechanism: the background-flow ROI (extent box minus lines) is reduced to thin slivers
  at the FRAME EDGES, whose rays have small `z_v` -> perspective-divide amplification, and
  there are few of them -> little averaging.
- `h_z` is edge/rotation-robust and SURVIVES: ic2 terminal h_z corr stays +0.86-0.95.

### DEAD END: centered angular window (`CROSS_FLOW_ANG_MAX`)

Idea: drop flow points with `|x_norm|` or `|y_norm|` > 0.7 so the FoV-edge slivers are
never sampled. **Built, GT-FB-validated, REVERTED same day.** It made the terminal regime
WORSE: ic2 `extent>=200` h_x/h_y corr +0.23/+0.13 -> **-0.06/-0.05**, h_x std 0.28 ->
**3.15 (10x)**. Root cause: once the marker fills the FoV the background is ONLY at the
edges, so the window drops nearly every point and the solve runs on 4-5 near-collinear
survivors -> unbounded Tx/Ty variance (worse than the bounded edge-point bias it removed).
`FLOW_ANG_MAX` left in the code default-INERT (99) as an experiment knob only; do not
re-enable without a different mechanism.

### Grazing-ray guard (`CROSS_Z_V_MIN_FLOW`) -- KEPT, minimal

`_solve_jacobian` now drops flow points with `z_v <= 0` (behind/at the virtual camera ->
sign-flipped perspective-divide garbage) before the lstsq, via `_getVirtualPts(return_zv=
True)`. Default `0.0` = that and nothing more (can't starve the solve; count-floor
fallback keeps all points if <MIN_FLOW_POINTS_SOLVE survive). The aggressive `0.4` value
(also drop amplified-but-not-flipped near-grazing rays) is UNTESTED in isolation -- it was
only ever run bundled with the reverted angular window.

### The real terminal fix (NOT done) + why it waits

Extent-gated h_x/h_y CONFIDENCE derate: when `MARKER_EXTENT_PX` is large there is
genuinely no valid background to measure lateral flow from, so inflate the h_x/h_y KF
measurement noise (the drone is centered by then, h_d->0, so it barely needs them).
Deferred because (a) it is control-path -> needs the `feedback_ic_validation` n>=5 gate,
and (b) it is MOOT until the 320x240 sensor-cal is redone -- real perception can't be
validated at all right now (`_sensor_cal_hw`/`_sensor_cal_s` still from 640x480/fx=270).

### IC1-5 real-perception sweep (hybrid OFF, perf-fix code) -- NOT a regression

Centered IC: converged (SOFT+PRECISE on the 3rd retry). All off-center ICs (2-5): FAIL /
TARGET_LOST at xy 2-4 m -- **matches `project_20260824_crossmarker_offcenter_convergence_
wall` exactly** (pre-existing kappa-leakage/funnel off-center wall), now also confounded by
the stale 320x240 cal. Sweep B (hybrid ON) was skipped -- it would hit the same wall and
tells us nothing until recal.

### NEXT (ordered)
1. 320x240 sensor recalibration (io-calibration skill) -- the blocker for ALL
   real-perception work.
2. THEN: extent-gated h_x/h_y confidence derate + real-perception IC validation.
3. Re-open the `CROSS_BG_FLOW_HYBRID` default-on decision (still default OFF) once (1)+(2)
   give a real-perception baseline; the GT-FB correlation evidence for it already stands.
4. `Z_V_MIN_FLOW=0.4` in isolation: quick offline A/B with the now-logged IMU AngVel.

---

## 2026-08-28 (cont.) -- 320x240 sensor RECAL: excitation maneuver unusable -> landing-data diagonal cal

Committed: 149cd5b.

### The phased-excitation recal FAILED (all R^2 negative) -- and it is NOT a derive bug

Ran 5 `apps/record_cross_marker_calibration.py` phased runs at 320x240 (defaults:
CROSS_BG_FLOW=1, perf-fix, z_v guard) -> `tools/derive_cross_marker_cal.py`:
  per-axis R^2 (mean): Hx=-0.79  Hy=-0.59  Hz=-143.9  Wz=-3.3  -- ALL NEGATIVE.

Root-caused (per the io-calibration skill's own rule: check the RAW signal before
blaming the fit):
- **Raw h_z is FINE at 320x240**: +0.72-0.73 vs GT, stable across all 5 runs.
- **The x/y excitation barely moves the drone.** During the x-phase the UAV's
  actual x-position travel is **6 cm** (UAV-pose log, run 01-06-12) for a
  commanded 0.35 m sinusoid -- PX4's horizontal position loop tracks only ~15% of
  a 0.5 Hz lateral setpoint. Achieved GT h_x/h_y std ~0.007 (vs the cross-marker's
  ~0.03 raw h_x/h_y attitude-jitter noise floor DURING that chase-an-untrackable-
  sinusoid maneuver). The Hx/Hy rows then fit from ~4x noise and the joint 6x6
  lstsq blows up EVERY row.
- **`compute_gt_signals` is CORRECT** (user asked to double-check): Vz depth
  median 5.11 m (not 30, not 1), 0 NaN, the 5% duplicate-`gt['Time']` samples are
  correctly filtered by `valid` before `np.gradient`, and the double Savitzky-
  Golay (win 101 then 51) is NOT the cause -- windows 101/51 vs 11/5 vs raw
  gradient all give the same GT h_x std once dup-timestamps are filtered (my
  scratch reimpl gave 6x more only because I skipped that filter -> divide-by-~0).

### Amplitude probe -- hit a wall

CALIB_AMP_XY / CALIB_FREQ_XY sweep (made both env-overridable in the recorder):

| FREQ_XY | AMP_XY | phase_s | GT h_x std |
|---|---|---|---|
| 0.5  | 0.35 | 8  | 0.008 (default) |
| 0.15 | 0.80 | 8  | 0.028 |
| 0.35 | 1.60 | 12 | 0.041 |
| 0.5  | 1.20 | 14 | 0.021 |

Even 1.6 m amplitude -> ~0.24 m travel -> GT h_x std 0.041, still noise-floor.
Longer phase made NO difference. To match the z-phase's GT std 0.22 you'd need
~8 m lateral amplitude. **PX4 will not track lateral position sinusoids in SITL.**
(`record_output_calibration.py` uses the IDENTICAL `send_position_ned` sinusoid --
and is the ArUco `IMG_PROCESSOR`-only pipeline, can't record the cross-marker
regardless. rollexc/pitchexc there are also position oscillations, not attitude
setpoints.)

### FIX: near-diagonal cal from GT-FB LANDING recordings

An off-center GT-FB landing produces real sustained lateral flow (GT h_x/h_y std
~0.05-0.08, 6-10x the maneuver). At normal marker size (MARKER_EXTENT_PX < 200),
raw h_V vs GT h is a clean per-axis line through ~origin, NO cross-coupling.

`tools/derive_cross_marker_landing_cal.py` (new): pooled MAD-trimmed slope
through 0 per axis, leave-one-out R^2. Recorded 4 GT-FB landings to
`calibration_data/landing_cal_cross/` (IC5 killed twice by a concurrent
`run_kfretune_ic2_ab.sh` SSH session grabbing SITL -> only IC2/IC3/IC4 usable).

  s_hx=0.730 (r .977)  s_hy=0.693 (r .900)  s_hz=0.955 (r .990)
  s_wz=0.592 (GT w_z std 0.092 -- real, not the 0.52 fallback)
  s_sx=0.957  s_sy=0.943
  LOO R^2:  h_z +0.97..+0.99 on all 3 (SOLID)
            h_x +0.79 / +0.90 / +0.24   h_y +0.91 / +0.69 / -0.56
            -> IC4 hold-out weak: its raw h_y is ~1.6x inflated vs the other two
               (scale mismatch, NOT a sign flip -- pooled h_y r is +0.90). IC4 is
               the highest-start (7 m) / weakest-lateral rep, possibly concurrent-
               SITL contaminated.

**Pasted into `CrossMarkerPerception.__init__` as a PURE DIAGONAL 6x6** (Wx/Wy
rows 0 per the level-target convention; no cross terms -- the stale 6x6 had
Hz<-Hx -0.22 and Wz<-Hy +1.96, artifacts of the aliased excitation fit).
`_sensor_cal_s = diag(0.957, 0.943, 1, 1)`. Prior 640x480 cal backed up:
`cross_marker_perception.py.bak_before_320cal_20260828`.

### ⚠ STILL PROVISIONAL / NEXT
- **Hx/Hy rows are provisional.** Re-derive with 2-3 MORE clean off-center
  landings (no concurrent SITL) + validate on a held-out landing. Hz row +
  `_sensor_cal_s` are trustworthy now.
- Only THEN is real-perception (GT-FB off) cross-marker IC validation meaningful
  -- and even then the off-center kappa-leakage control wall
  (`project_20260824_crossmarker_offcenter_convergence_wall`) is a separate
  blocker.
- The terminal-phase h_x/h_y collapse fix (extent-gated confidence derate) is
  still not done -- do it after this cal is firmed up.

### RECAL FINALIZED (5-run fit) -- commit 534b78c

Recorded 4 more GT-FB off-center landings + re-derived on 7, then on 5.

**Adopted (LIVE): 5-run IC2(x3)/IC3(x2) fit** --
  s_hx=0.796 (r .989)  s_hy=0.782 (r .976)  s_hz=0.950 (r .995)
  s_wz=0.590 (GT w_z std .097)   s_sx=0.959  s_sy=0.947
  leave-one-out R^2 on ALL 5: h_x +0.83..+0.94  h_y +0.81..+0.94  h_z +0.97..+0.99
  -- clean, no anomalies. `_sensor_cal_hw` is pure diagonal (no cross terms).

**IC4 anomaly RESOLVED = real altitude effect, NOT contamination.** IC4 (ENU 2,2,7,
~7m start) recorded TWICE; both reproduce the weak hold-out (h_x +0.31/+0.48, h_y
-0.25/+0.21) + degraded detection (~84% ok, hough_lt2_lines -- stroke width thins
at 7m). The h-block scale is altitude-dependent. IC4 runs moved to
`calibration_data/landing_cal_cross_highalt_excluded/`. **This cal is trustworthy
for a nominal ~5m descent; degraded h_x/h_y above ~6m.** A height-scheduled cal
M(altitude) is a separate future effort (cf. io-calibration skill's multisine mode).

7-run pooled fit (for reference, incl. both IC4): s_hx=0.745 s_hy=0.717 s_hz=0.945
-- IC4's inclusion drags r_hy 0.976->0.930 and adds the 2 bad hold-outs. Not used.

**Cal status now: usable for GT-FB-OFF cross-marker landing from ~5m.** Remaining
before trusting a real-perception IC sweep: (a) a fresh held-out validation landing
(the LOO already covers this statistically), (b) the separate off-center kappa
control wall (project_20260824_crossmarker_offcenter_convergence_wall) still blocks
off-center real-perception convergence regardless of cal quality.

### HELD-OUT VALIDATION + 6-run re-fit -- commit d56c0be (FINAL for this pass)

Fresh GT-FB landing at ENU (2,-2,5) -- a quadrant NOT in the IC2/IC3 training set.
Applied the LIVE cal to its raw h_V vs GT:
  h_x R^2 +0.768 (r +0.965)   h_y R^2 +0.767 (r +0.971)   h_z R^2 +0.945 (r +0.972)
=> PASS. h_z scale exact. h_x/h_y: relationship generalizes (r ~0.97) but
calibrated output is ~1.15-1.35x hot on that quadrant -- a mild per-quadrant scale
residual (the R^2/r^2 gap), not a structural miss. Consistent with the codebase's
known image-x-hotter-than-y asymmetry + training being all +y-quadrant.

Folded that rep in -> **6-run LIVE cal (d56c0be)**:
  _sensor_cal_hw = diag(0.7868, 0.7937, 0.9513, 0, 0, 0.5869)
  _sensor_cal_s  = diag(0.9574, 0.9503, 1, 1)
  Barely moved from the 5-run fit -- the fit is stable. LOO R^2: h_z +0.95..+0.99
  all 6; h_x/h_y +0.85..+0.94 on the 5 IC2/IC3 runs, +0.77 held-out quadrant.

**CAL DONE for the ~5 m descent regime.** Remaining known gaps (not cal-fixable
here): (1) high-altitude >6 m -- see the IC4 note above; (2) the terminal-phase
h_x/h_y collapse (extent-gated confidence derate, still not implemented);
(3) off-center kappa control wall blocks real-perception off-center convergence
regardless of cal.

### Terminal-phase h_x/h_y confidence derate -- IMPLEMENTED (commit 680f4a1, default OFF)

`CROSS_HXY_EXTENT_DERATE` (default "0"). In `_kf_update_hw`, on a real measurement:
- inflate the h_x/h_y KF R by a factor ramping 1x -> `CROSS_HXY_DERATE_MAX` (300)
  linearly over `[CROSS_HXY_DERATE_EXTENT_START, _FULL]` px (200..300);
- above `CROSS_HXY_DERATE_RATE_FRAC` (0.5): also bleed the h_x/h_y RATE state x0.5/
  step (R-inflation alone is undone by the KF's constant-velocity coasting of a
  std~1 terminal measurement) and hard-clamp the h_x/h_y VALUE to
  +-`CROSS_HXY_DERATE_CLAMP` (0.20).
- h_z / w channels untouched. INERT under GT-FB (h substituted).
- `_kf_step` already broadcasts a (6,) r -- no change there. New "HxHy Derate
  Mult" logged in Img_Data.

Offline A/B on 3 landing_cal_cross reps: terminal (ext>=200) h_x/h_y std
0.80/1.35 -> 0.076/0.037 (IC2), 0.36/1.02 -> 0.075/0.077 (IC3c_heldout, corr
+0.30 -> +0.72). small-marker regime + h_z bit-identical. It tames terminal
garbage to the healthy small-marker magnitude -- not an accuracy fix (impossible
with no background there), a "stop injecting noise into control" fix.

⚠ DEFAULT OFF -- control-path -> needs the `feedback_ic_validation` n>=5 IC1-5
sweep (GT-FB-off, once the 320x240 cal + the off-center kappa wall allow a
meaningful real-perception sweep) before baking on.

### Terminal h_x/h_y: REDESIGNED -- centroid-rate, bg-flow-health gated (commit 47333a0, DEFAULT ON)

The 680f4a1 extent-gated derate ("suppress h_x/h_y to ~0 near touchdown") was
replaced -- it assumed a centered STATIONARY touchdown and would hurt a moving
target (true terminal h_x/h_y != 0 there; h_d != 0). New approach COMPUTES a
reliable h_x/h_y instead of suppressing:

  h_xy_centroid = d(s_V)/dt + h_z_est * s_V     (image-Jacobian identity, de-loom)

- `_scen_kf_*`: 2-state KF on the V-frame centroid s_V, fed `_center_px` (fresh OR
  BRIDGED -- survives the terminal detection flicker) every frame via
  `_stepCentroidKf()`, called in all 3 process_frame paths before `_kf_update_hw`.
  q=2.0/r=0.02 (offline-tuned). `_getVirtualPts(log_zv=False)` so the extra
  projection doesn't skew `_z_v_log`.
- **Scale-free**: s_V normalized-image, d/dt = 1/s, h_z = moment-loom ratio. NO
  depth/altitude/metric (user-audited). h_z*s makes it correct off-center ->
  MOTION-AGNOSTIC.
- **GATE = background-flow HEALTH, not MARKER_EXTENT_PX** (user directive: keep the
  switch a pure signal-quality decision, no proximity proxy):
    frac = max( ramp(rel_resid, 0.50..0.90), ramp(16 - n_pts, ..6) )
  ⚠ `N Flow Corners` stays ~160 even at touchdown -- the terminal failure is point
  QUALITY (grazing rays on FoV-edge slivers), NOT starvation -- so `rel_resid` is
  the load-bearing term. `_bgflow_health` (rel_resid, n_pts) set by
  `_compute_hw`/`_compute_hw_bgflow`/`_compute_hw_bgflow_fallback`, reset (inf,0)
  each frame. New Img_Data logs: "HxHy Centroid Blend" (frac), "BgFlow Health".
- Blend + mild R bump (x8) + value clamp (+-0.20 backstop for a divergent rep).
- DEFAULT ON (`CROSS_HXY_TERMINAL_CENTROID=1`). Perception change, not control-law
  -> no n>=5 IC gate (per user 2026-08-28: that gate is for CONTROL changes only).
  Safe: no-op if rel_resid stays low near touchdown. INERT under GT-FB.

Offline A/B (extent-gated precursor, same centroid-rate mechanism), 5 landing
reps, terminal (ext>=200) h_x/h_y corr: ~0 -> +0.55..+0.96 on 4/5; +0.64/+0.75
on the 5th (a divergent rep, clamp backstops). Unit-tested: healthy solve ->
frac 0 passthrough; rel_resid 0.95 -> frac 1, garbage bg-flow rejected/clamped.

**PENDING (next free SITL):** fresh GT-FB rep -> confirm `BgFlow Health` rel_resid
actually rises in the terminal phase (early terminal-resolve diag showed
median 0.79 / p90 0.96 but that was the no-angvel worse solve); tune
`CROSS_HTC_RESID_LO/HI` if the live gyro-derotated solve's terminal rel_resid
sits lower.

### LIVE-VALIDATED (fresh GT-FB IC2 rep, new code) -- gate works, no retune needed

Landing SOFT+PRECISE (xy 0.018), 44.8 Hz, detect-ok 100%, no exceptions.
  rel_resid  SMALL(ext<200): med 0.19 p90 0.38   BIG(ext>=200): med 0.95 p90 0.99 max 0.997
  blend frac SMALL 0.02 (6% of frames)            BIG 0.80 (90% of frames)  max 1.00
  n_pts ~160 in BOTH regimes -- confirms point QUALITY (grazing edge slivers), not
    starvation, is the terminal failure; rel_resid is the load-bearing gate term.
  h_V vs GT corr: SMALL h_x/h_y 0.99/0.99 (untouched) | BIG h_x/h_y 0.60/0.81
    (pre-fix: ~0 / negative), std tamed 0.8/1.3 -> 0.056/0.055. h_z BIG 0.55 (not derated).
The CROSS_HTC_RESID_LO/HI = 0.50/0.90 window sits cleanly between the 0.19 healthy
and 0.95 terminal medians -- NO retune. Live terminal rel_resid is HIGHER than the
earlier no-angvel offline estimate (0.79/0.96), so the gate is if anything more
decisive on the real gyro-derotated solve. **Terminal h_x/h_y fix: DONE + shipped
default-on.**

### IC1-5 GT-FB regression sweep with the terminal h_x/h_y fix -- 5/5 SP, CLEAN

| IC | class | xy_err | rel_vel | rate | detect-ok |
|----|-------|--------|---------|------|-----------|
| IC1 | SOFT+PRECISE | 0.002 | 0.014 | 46.1 Hz | 100% |
| IC2 | SOFT+PRECISE | 0.017 | 0.031 | 45.1 Hz | 100% |
| IC3 | SOFT+PRECISE | 0.018 | 0.010 | 44.6 Hz | 81% |
| IC4 | SOFT+PRECISE | 0.016 | 0.012 | 45.6 Hz | 84% |
| IC5 | SOFT+PRECISE | 0.017 | 0.012 | 44.2 Hz | 70% |

All the session's changes (terminal h_x/h_y centroid-rate + `_scen_kf` + `_bgflow_health`
gate + `_getVirtualPts(log_zv=)`, 320x240 diagonal cal, `process_frame` perf fix)
coexist with NO GT-FB landing-quality regression across IC1-5. detect-ok 70-84% on
IC3-5 is the pre-existing 320x240 stroke-width degradation, ridden through fine.
(Terminal blend is INERT under GT-FB -> this is a no-regression gate, not a benefit
test; benefit was shown live on the perception signal, prior rep.)

### CROSS_BG_FLOW_HYBRID defaulted ON (commit aa8d0ee) + IC1-5 GT-FB re-sweep -- 5/5 SP

Scale-free/depth-free re-audit of ALL session runtime changes: CLEAN. Every quantity
is a normalized image coord (s_V), a rate of one (1/s: h_x/h_y/h_z, scen_rate), a
dimensionless ratio (rel_resid, cal scale factors, blend frac), a pixel count/factor
(n_pts, WORK_MAX_PX), or a body rate (rad/s). NO Z/altitude/metric/marker-real-size
in the control path. MARKER_EXTENT_PX was removed from the terminal-h_x/h_y gate (now
rel_resid + n_pts only) -- grep-confirmed zero refs in _kf_update_hw / _scen_kf / _htc_*.
GT/depth confined to gt_feedback.py (scaffold) + cal-derivation tools (both allowed).

IC1-5 GT-FB sweep, hybrid ON + terminal-centroid ON + 320x240 cal + perf fix:
| IC | class | xy_err | rel_vel | rate | detect-ok |
|----|-------|--------|---------|------|-----------|
| IC1 | SP | 0.002 | 0.016 | 44.1 | 100% |
| IC2 | SP | 0.017 | 0.038 | 43.5 | 100% |
| IC3 | SP | 0.027 | 0.024 | 42.2 | 84% |
| IC4 | SP | 0.020 | 0.031 | 44.1 | 86% |
| IC5 | SP | 0.017 | 0.073 | 41.2 | 89% |
5/5 SOFT+PRECISE, all >=41 Hz, no exceptions. detect-ok on IC3-5 slightly BETTER
than hybrid-OFF (84/86/89 vs 81/84/70). Small xy/rel_vel wiggles are GT-FB n=1
noise; hybrid is INERT under GT-FB -> no-regression pass. Both perception flags
(CROSS_BG_FLOW_HYBRID, CROSS_HXY_TERMINAL_CENTROID) now DEFAULT ON.
