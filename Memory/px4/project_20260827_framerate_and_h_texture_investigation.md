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
