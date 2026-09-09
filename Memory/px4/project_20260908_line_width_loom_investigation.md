---
name: project_20260908_line_width_loom_investigation
description: "⛔ THREAD CLOSED 2026-09-09. Line-width / extent / size-derived loom RATE for h_z: dead end, fully chased. Static line-width VALUE is good (0.89-1.00 vs GT alt) but its DERIVATIVE never tracks loom usably (best ~0.3 in the .5-2m band, unrecoverable <0.35m). Extent-fused 'Scale Loom Rate' tracks loom 0.84-0.98 mid-band but 3 h_z-fusion forms ALL regressed the IC gate. Everything stays SHADOW-MODE; CROSS_SCALE_RATE_FUSE default-OFF is permanent. Geometry-width (CROSS_WIDTH_GEOM=1) + tools/overlay_width_loom_rate.py landed."
metadata: 
  node_type: memory
  type: project
  originSessionId: 12257c7c-a2c9-46f1-a6c7-d09063093486
  modified: 2026-09-09T11:48:27.156Z
---

## ⛔⛔ THREAD CLOSED — 2026-09-09 (read this, skip the 700-line chronology below unless digging)

**Goal was:** replace / backstop the corrupted terminal-overfill `h_z` (LK image-Jacobian Tz
spikes to +5.6 when tracked points lose spread) with a loom estimate derived from the marker's
apparent SIZE (line-width, then extent). **Outcome: dead end, exhaustively verified.**

**Final state of each piece:**
| piece | verdict |
|---|---|
| Static line-width VALUE (mask-scan → now geometry-width) | **Good** (0.89-1.00 corr w/ GT altitude). `width_loom_from_detection` DEFAULTS to the geometry method (`CROSS_WIDTH_GEOM=1`, `bf812f1f`): `2√3·std(transverse residual of arm inliers)`, gated on arm-perpendicularity + junction-in-frame, `None` on gate-fail. Bounded terminally (12-35 px vs mask-scan's 14-201). SHADOW-ONLY, consumed by nothing. |
| Pure line-width RATE (`-d/dt ln width`, `"Width Loom Rate"`) | **Dead.** ~0.22 corr w/ GT loom in the .5-2m band (per-rep -0.09..+0.68), noise elsewhere. The VALUE fits position at R²>0.95 but its residual is a low-freq drift `ε` whose derivative `ε̇` (std 0.7-1.3) swamps the loom signal (std 0.1-0.5). Not lag, not binarisation, not tuning, not direction-wobble — all ruled out. |
| Extent-fused RATE (`0.3·ln width + 0.7·ln extent`, `"Scale Loom Rate"`) | **Tracks loom** 0.84-0.98 in the .5-2m band (extent is the workhorse; width adds ~nothing), live-parity confirmed. **But cannot be fed into `h_z`:** naive / +overfill-gate / +affine-debias+band+loose-r — **all 3 fusion forms REGRESSED the IC1-5 gate** (hard landings, then blown lateral xy). `h_z` couples into the middle-loop SMC c-term + sliding surface, and pinv `h_z` is already good enough that any perturbation only costs accuracy. `CROSS_SCALE_RATE_FUSE` default-OFF is **PERMANENT**. |
| Terminal window (<~0.35 m) | **Unrecoverable for ANY size-derived loom.** ~70-80% of the logged `<0.5m` band is post-touchdown (parked drone, GT loom≈0). The genuine last ~0.3 m has a real SIGN INVERSION (width shrinks during the fastest drop) because the marker fragments / exits the FOV and the arm fits latch onto background — `corr(width, inlier_count)=+0.65` there. V-frame leveling: no effect. Extent saturates (318 px = frame). Moment-loom dies here too, same reason. |

**The line-width loom's only usable altitude range is ~2 m → ~0.35 m.** Below that the touchdown
detector (`_touchdownDetectV2`: n_corners / extent / flow-freeze — **none use `h_z`**) already
owns the regime; every landing in every gate latched via `[overfill]`/`[flow-freeze]`.

**Do not re-open** without a fundamentally different observable (not apparent size, not its
derivative) OR a fundamentally different consumer (not a KF measurement of the live `h_z`).

**Artifacts kept (all shadow / diagnostic, nothing control-facing):**
- `cross_marker_perception.py`: `"Width Loom Px/Rate"`, `"Scale Loom Rate"`, `"Scale Fuse Z"`
  logs; `_wgeom_arm` + geometry `width_loom_from_detection` (default); `_wloom_*` mask-scan
  (`CROSS_WIDTH_GEOM=0` fallback); `_scale_rate_*` KF; the whole `_scale_fuse_*` fusion machinery
  behind `CROSS_SCALE_RATE_FUSE` (default 0, marked DEAD-END in-file).
- `tools/overlay_width_loom_rate.py` — width/extent/scale loom-RATE overlay on raw IMG_RECORD
  frames vs GT loom (companion to `overlay_image_features.py`).
- `tools/gt_optical_flow.py` — the 3-bug fix (stale Z_REG, missing mount offset, stale alt gate)
  from this thread is a real correctness fix that OUTLIVES it; keep.
- Backups: `Obsolete/src/cross_marker_perception_pre_{scalerate,geomwidth}_20260909.py`.

Related: [[feedback_cross_detector_contrast_not_darkness]] (the terminal-overfill blocker this
was trying to help), [[reference_gt_optical_flow]], [[project_20260831_perception_mode_landing]].

---

## Context / goal
Continuation of the terminal-overfill investigation ([[feedback_cross_detector_contrast_not_darkness]] /
PLASMC_TUNING_GUIDE's "#1 OPEN blocker"). Root failure (re-confirmed on the actual failed rep
`test_data/ICValidation/20260831-144626/IC1_rep1`): at overfill, the LK image-Jacobian's
position-weighted Tz/Wz columns (`_fill_A`, `cross_marker_perception.py:414`) go ill-conditioned
when tracked points lose spread relative to the image center -- `h_z` spiked from +1.1 to +5.6 in
~0.15s while GT loom was only +0.6, triggering a slam-up-thrust ascent that then destroyed
detection entirely (the ascent is the CAUSE of the later detection collapse, not a separate failure).

**`origin_ratio`** (`M0/||centroid||^2`, already computed in `_solve_jacobian`'s moment-loom gate,
`cross_marker_perception.py:1599-1601`) is a validated, LEADING indicator of this: measured 5-23
during a healthy plateau, collapsing through 1.0 about 0.3s before the h_z spike, bottoming at
0.15-0.2 during it. **The bug: when this gate fails, the code falls back to the plain pinv Tz,
which is corrupted by the exact same degenerate geometry** -- the "safety net" catches nothing
because both estimators share the same weakness (position-weighted columns need point spread;
neither the moment method nor the pinv method has it when the gate fires). Fixing this (veto BOTH
estimators on gate failure, hold-last-good instead) is the clear, validated, not-yet-implemented
next action -- see the very end of this file, "STATUS AT HANDOFF".

## Line-width alternative: full investigation, bugs found, and the fix

Explored replacing/supplementing the loom (h_z) estimate with a line-width-based signal (arm
stroke thickness change over time), on the theory that width is a purely local, image-center-independent
measurement, immune to the position-spread degeneracy above. Long iterative investigation (many
false starts, all preserved below because each ruled out a real hypothesis):

### Ruled out (in order tested)
1. **Azimuthal clustering re image-CENTER** -- user correction: `_getVirtualPts` normalizes
   relative to `self.center` (image/optical center), not marker center; my first framing
   conflated the two. Corrected: the real requirement for the joint lstsq is POINT SPREAD
   (position variance across sampled points), not azimuthal diversity per se -- a tightly bunched
   cluster aliases with Tx/Ty regardless of WHERE in frame it sits.
2. **Axis-jitter / i-vs-j label instability** across frames when propagating a tracked line
   direction -- confirmed real (flip_deg jumps of 45-90 deg near touchdown, matching arm-stub /
   arm-arm angles) but proven NOT the dominant cause: removing all cross-frame propagation
   (pure per-frame recompute) made low-altitude correlation WORSE, not better.
3. **Stub-inclusion inconsistency** (2-line vs 3-line frames diluting a plain average) -- ruled
   out the same way: restricting to arms-only, fixed-composition-2 did not fix the sign flips.
4. **Downscale/rescale quantization** (`_detect_core_capped`/`DETECT_WORK_MAX_PX=200`, used by
   the tracked-crop fast path -- and unconditionally when `_ADAPT_GATE=1` -- specifically active
   when the marker is large near touchdown) -- directly A/B tested (bypassed the cap entirely vs
   default): correlation with GT altitude was essentially unchanged (rep1 0.27=0.27, rep2
   0.34->0.30, rep3 0.37->0.38). Real mechanism, negligible practical effect here.

### Confirmed real bugs in the existing width machinery
5. **Self-referential pruning collapse (`line_points_i/j`).** `_robust_fit_line`
   (`cross_marker_detector.py:821`) prunes to inliers using `outlier_thresh * scale` where
   `scale = median(residual)` OF WHATEVER CURRENTLY SURVIVES -- a self-calibrating threshold with
   no physical unit. `CrossMarkerDetection.line_points_i/j` (what every prior width computation in
   this codebase used, incl. `_confirm_cross_geometry`'s width-ratio sanity check) IS this pruned
   inlier set (`cross_marker_detector.py:1604-1614`'s own comment says so explicitly). Measured on
   the real failed rep: width collapses to near-zero on some frames with no corresponding real-size
   dip -- confirmed via w_proj dumps showing near-collinear, pixel-quantization-level scatter, not
   real stroke geometry.
6. **Coarse-band contamination (`line_points_i_raw/j_raw`, the naive fix for #5).** Going back to
   `_cluster_points_from_mask`'s pre-pruning coarse band selection re-admits a SECOND real
   structure at close range: measured directly on `rep5/f00364.png` -- bimodal w_proj with a clean
   gap (cluster A ~40 pts spanning -15.67..-6.01, cluster B ~120 pts spanning +8.10..+9.56), total
   spread ~25.2px matching the PREDICTED `_band` diameter (`0.05*max(bw,bh,1)` clipped [5,35],
   here 2*12.8=25.6px) almost exactly. Root cause: `_band` is sized as a fixed fraction of the
   marker's OWN bbox, untethered from the actual current stroke width -- selectivity degrades
   exactly as the marker (and hence bbox) grows, i.e. exactly at close range where width-loom
   matters most. Also implicated: `_rep_line()`'s reference (Hough-cluster-mean angle + Hough
   segment-endpoint centroid, `cross_marker_detector.py:1561-1576`) is a rough proxy, not a fit --
   plausible but not independently isolated as a compounding cause of the band's asymmetric
   admission of contamination.
7. **MAD-formula distributional bias (`_arm_geometry`, `:633`).** `width = 2.0*1.4826*MAD` --
   the 1.4826 constant is the MAD->std conversion for a GAUSSIAN distribution; a stroke's
   cross-section is closer to uniform/rectangular. Verified numerically (simulated uniform[-5,5],
   true width 10): formula returns 7.44, a consistent ~26% underestimate. Doesn't affect
   correlation-based tests (constant multiplicative factor) but would corrupt any future
   absolute-width calibration.
8. **Adaptive-band middle ground** (EMA-tracked last-known-width, banding tied to
   `clip(mult*last_width, floor, old_band)` instead of a fixed bbox fraction) -- tested as a
   patch: more CONSISTENT than either #5 or #6 alone (never the worst of the three in any rep;
   best in rep4 at 0.67), but still not uniformly good (0.33-0.67 range) and adds real complexity
   (bootstrap rule, EMA state).

### The actual fix: direct mask-thickness scan (much simpler, and it works)
User's design push ("is this overcomplicated? make it simpler") was correct. All of #5/#6/#8 are
compensating machinery for reusing a POINT-CLOUD-STATISTICS pipeline (built for contamination-robust
DIRECTION/JUNCTION fitting) to answer a fundamentally different, local, single-purpose question:
"how thick is the mask at this spot." Replaced entirely:
- Use the fitted line's already-reliable direction (from the pruned inlier set -- pruning doesn't
  meaningfully corrupt DIRECTION, only transverse spread, confirmed via aspect-ratio checks all
  session).
- At 5 real detected points (NOT idealized straight-line interpolations -- see bug below) spread
  across the middle 70% of the arm's along-track extent, walk a sub-pixel (ds=0.4px, bilinear-
  sampled) perpendicular ray through `det.isolated_mask` in both directions, counting the
  continuous on-mask run.
- Median across stations that (a) are independently confirmed on-mask before scanning and (b)
  meet a quorum (>=3 of 5 valid), else hold last-good.

**Station-placement bug found and fixed en route:** first attempt interpolated station positions
along the idealized straight PCA line (`c + s*v`) -- landed completely off-mask on some frames
DESPITE abundant real points existing (measured: all 5 stations reading exactly 0.0 with
n_pts=177 real detected points available same frame) because the real point cloud is not
perfectly straight (perspective/lens/pruning-asymmetry). Fix: use actual detected points as
station origins (guaranteed on/near real mask material), not synthetic interpolated ones.

### Validation results (GT-altitude correlation, `corr(ln(width), -ln(alt))`)
`GtfbMulti_col` (color-variant world, GT-feedback, proxy data -- 5 reps): old(pruned) 0.27-0.54,
raw(coarse band) 0.23-0.58, adaptive-band 0.27-0.67, **mask-scan v2 (fixed): 0.55-0.75, zero
collapses in every rep** -- first method that's uniformly decent everywhere; every other method
swung wildly rep-to-rep.

**Fresh real-perception IC1 capture** (`test_data/OverfillCapture_IC1`, 2026-09-08, plain
`cross_marker` world, real perception mode not GT-FB, 3 reps, `IMG_RECORD=1` raw frames --
raw PNGs land in `test_data/Test_Videos/<timestamp>_raw/`, NOT under the copied `Landing_Test`
dir, pair by capture-order/mtime): all 3 reps reached genuine deep overfill (alt down to
0.6-6cm) without reproducing the exact catastrophic ascent from the reference failed rep (all
landed close, FAIL/PRECISE-only on precision thresholds, not a fly-away) --
**mask-scan width vs GT altitude: 0.89, 0.99, 0.92** (old pruned-inlier method on the SAME
frames: 0.54, 0.94, 0.71). Decisively the strongest result of the whole investigation, on the
actual target world under real perception -- not proof the catastrophic event specifically is
fixed (that exact event didn't recur to test against), but strong evidence the method is sound
in the regime it occurred in.

### IC2-5 confirmation (2026-09-08, same day, `test_data/OverfillCapture_IC2to5`)
Same recipe (real perception, `IMG_RECORD=1`, plain `cross_marker` world) run once each on IC2-5.
All 4 reached deep overfill (min_alt 0.02-0.15m). Mask-scan wins in EVERY rep tested across all
5 ICs (7 reps total), often by a wide margin:
IC2 old=0.92/new=0.96, IC3 old=0.97/new=1.00, IC4 old=0.58/new=0.90, IC5 old=0.68/new=0.94
(IC1 x3: 0.54/0.89, 0.94/0.99, 0.71/0.92). Full generalization confirmed, not an IC1-only or
lucky-rep effect. **Known gap, not yet checked:** this validation (like all mask-scan work so
far) used RAW camera-plane points/mask, no `_getVirtualPts` leveling -- unlike every other
metric quantity in this codebase (`h_V`, `s_V`, `alpha`), which are leveled specifically because
`alpha`'s own history showed un-leveled geometry aliases tilt-foreshortening as signal (sign-flip
r=+0.31 low-tilt vs r=-0.87 high-tilt). Width has not been checked for the same failure mode --
the strong correlations above are on real flights with real (if modest, mostly-hover) tilt, which
is reassuring but not a substitute for the same explicit tilt-stratified check `alpha` got.

### Key file:line references for implementation
- `_robust_fit_line`: `cross_marker_detector.py:821`
- `line_points_i/j` (pruned) vs `line_points_i_raw/j_raw` (coarse, added this session):
  `cross_marker_detector.py:767-796` (dataclass), `:1597-1599` (pts_i_raw/pts_j_raw saved before
  pruning), `:1863-1865` (returned on success), `_shift_detection`/`_scale_detection`
  (`:1869-1899`, `:2109-2129`) also updated to thread the new fields through crop/scale transforms
  -- both had a silent-drop-to-default bug for any NEW dataclass field before this fix.
- `_cluster_points_from_mask` / `_band` / `_rep_line`: `cross_marker_detector.py:1553-1595`
- `_arm_geometry` (MAD width): `cross_marker_detector.py:617-634`
- `det.isolated_mask`: already returned, used directly by the mask-scan method (no new detector
  plumbing needed beyond the `_raw` fields, which the mask-scan method doesn't even strictly need
  -- it only needs `line_points_i/j` for direction+station origins, and `isolated_mask` for the
  scan itself).

## STATUS: ALL PLANNED FIXES LANDED (2026-09-08, same session, continued)

### 1. Width-loom shadow signal -- LANDED + VERIFIED in `cross_marker_perception.py`
- New module-level block (right after `_fill_A`): `_wloom_bilinear`, `_wloom_scan_thickness`,
  `_wloom_width_at_points`, `width_loom_from_detection(det)` -- the validated mask-scan method
  (real detected points as scan origins, sub-pixel bilinear perpendicular walk, median+quorum
  >=3/5). Constants `_WLOOM_DS=0.4, _WLOOM_MAX_STEPS=60, _WLOOM_THRESH=127.0,
  _WLOOM_N_STATIONS=5, _WLOOM_MIN_QUORUM=3`.
- Wired into `_log_frame_data()`: `self._width_loom_log` / `self._last_width_loom` (hold-last-
  good on a None reading), exposed via `getLogData()` as `"Width Loom Px"`. SHADOW MODE ONLY --
  not consumed by any control path. Wrapped in try/except so it can never break real logging.
- **Verified correct**: drove the actual production `process_frame()` entry point (not the
  offline replay script) across `OverfillCapture_IC1/rep1_data` -- live `Width Loom Px` gives
  corr=0.886 with GT altitude, matching the offline validation's 0.89 for the same rep almost
  exactly, zero NaN gaps.

### 2. `origin_ratio` double-gate fix -- LANDED + VERIFIED, RESTRUCTURED to close a coverage gap
- `__init__`: new `self._tz_unreliable_this_solve` (reset each `_solve_jacobian` call) +
  `self.CROSS_TZ_VETO_R_MULT` (env `CROSS_TZ_VETO_R_MULT`, default 1e6).
- `_solve_jacobian`: origin_ratio computation RESTRUCTURED -- previously nested behind
  moment-loom's OWN point-count floor (`moment_min_pts=6`), so the veto could never even run
  when tracked points fell between `MIN_FLOW_POINTS_SOLVE=4` (the solve's absolute floor) and 6 --
  exactly the regime the reference failed rep collapsed through right after its h_z spike
  (ext->76, n_corners 165->27->15->0). Now: origin_ratio computed at the solve's own
  `MIN_FLOW_POINTS_SOLVE` floor, independent of `moment_min_pts` (which still gates ONLY whether
  the moment ESTIMATE specifically gets trusted -- its own separate concern, smaller n = noisier
  mean). Three-way outcome: `origin_ratio is None` (too few points even for the general check) ->
  hold pinv unmodified, unchanged pre-existing behavior; `origin_ratio < threshold` -> veto (NEW:
  sets `_tz_unreliable_this_solve`, doesn't silently trust pinv); `origin_ratio >= threshold and
  len(prev_n) >= moment_min_pts` -> trust moment-loom override (unchanged).
- `_kf_update_hw`: consumes the flag right after the existing loom-R-schedule block (same lever,
  multiply r[2], independent/multiplicative so both can fire the same frame) -- inflates r[2] by
  `CROSS_TZ_VETO_R_MULT` (predict-only on the Tz channel specifically via the KF's own math, not
  a hand-rolled skip), then resets the flag. Doesn't touch h_x/h_y/w_x/w_y/w_z's KF channels.
- **Verified on real data** (instrumented replay, `OverfillCapture_IC1/rep1_data` +
  `rep3_data`): 361 solve calls / 14 vetoes fired (rep1), 391 calls / 100 vetoes (rep3, rougher
  conditions). `h_z` stayed finite and bounded in both (rep1 [-0.50,0.56], rep3 [-2.11,0.38]) --
  no NaN/Inf, no runaway blow-up like the original failed rep's +5.6 spike. Vetoes are reaching
  real low-point-count frames now that were unreachable before the restructuring.

### 3. Tilt-leveling check on width -- DONE, RESOLVED AS A FALSE ALARM (closes the flagged gap)
Initial coarse check (20 deg split) found no difference raw-vs-leveled (0.92-0.94 either way) --
but that was too coarse: alpha's own historical sign-flip happened between 1.7-3.7 deg, a much
finer threshold. Re-split at 3.7 deg on `rep3_data` (the highest-tilt rep, up to 18 deg observed)
found what LOOKED like a real confound: corr_lo(<3.7deg,n=61)=0.99 vs corr_hi(>=3.7deg,n=11)=
**-0.24** -- a sign flip, matching alpha's pattern superficially. Built the natural fix (level
each scan's measured run via `_getVirtualPts` on its two endpoints, not the whole mask) and
retested: **leveling did NOT fix it** (-0.12 leveled vs -0.24 raw, same sign, same rough
magnitude). Investigated why: the entire high-tilt (>=3.7deg) subset is the LAST 0.9s of the
flight (t=10.30-11.18s), a 0.108m altitude span (0.172->0.064m) at actual ground contact --
width bounces noisily (28.2, 28.8, 29.8, 31.8, 28.6, 30.4, 17.4, 23.4, 26.0, 21.4, 41.6, no clean
trend) from genuine touchdown/contact noise, not systematic foreshortening. **Conclusion: this
is the SAME narrow-altitude-range statistical artifact already diagnosed earlier in this session
(the "rep4 0.85 correlation was a trend-inflation fluke" finding), not a real tilt/foreshortening
bug** -- confirmed precisely BECAUSE leveling (which only fixes foreshortening) had no effect.
Unlike `alpha`'s genuine bug, width does not need `_getVirtualPts` leveling based on everything
tested this session. The endpoint-leveling code from this check was NOT ported into
`width_loom_from_detection` (no evidence it's needed); flag as revisit-if-a-future-check-on-a-
higher-tilt-mid-descent-window (not just terminal-touchdown-contact) finds something real.

### ⛔ REGRESSION found + mitigated same day (2026-09-08, reported by 2 independent sessions)
The `origin_ratio` double-gate fix above CRASHED every off-center IC (IC2/IC3/IC5). Root
cause: `origin_ratio = M0/||c0||^2` where `c0` = tracked points' mean normalized position =
the marker's real angular offset from the image principal point. Off-center approaches have
large `c0` FROM T=0 by construction (nothing to do with point-spread health), so
`origin_ratio` reads persistently low the ENTIRE descent, not just during a real collapse.
The restructuring above turned this into a hard veto (r[2] *= 1e6, KF predict-only) --
correct for the brief centered-IC1 terminal transient it was tuned on, but catastrophic
when the SAME low-but-STABLE value persists for a whole off-center approach: h_z froze at
init value the whole descent (measured: IC5 h_z std 0.034 vs 0.13 pre-fix over a 3.0->0.14m
descent) -> unbraked open-loop descent -> crash (0.65-19m miss, up to 7.2 m/s impact, 3/3
off-center ICs, 12/12 correlation across both reporting sessions).

**Attempted fix (relative-drop + streak gating):** veto only on a RELATIVE fall from a slow
EMA baseline (matching the actual failure SHAPE, not the raw absolute value) + require the
drop to persist N consecutive frames (noise-debounce). Implemented
(`_origin_ratio_ema`/`CROSS_ORIGIN_RATIO_DROP_THRESH`/`_EMA_TAU`/`_DROP_STREAK`). Directly
verified via synthetic test: off-center-stable(0.3) -> 0 vetoes; reference-collapse-shape
(healthy plateau -> 1.8->0.15 over 15 frames -> sustained) -> 0/100 healthy, 15/15 collapse,
50/50 sustained-after. BUT on REAL centered-IC1 data, a plain 3-frame streak still
over-fired (78-160 vetoes vs the original fix's 14-100, `origin_ratio` is genuinely noisy
frame-to-frame even when healthy -- observed 0.86/1.47/4.17/1.08 swings on a real successful
landing earlier this session) and raising the streak to 8 to quiet that noise let a real,
SHORTER-than-8-frame collapse on rep3 through UNVETOED (h_z reached 9.32, worse than any
prior state). **This is a genuine, unresolved noise-vs-sensitivity tension that centered-
only data cannot resolve** -- needs real off-center flight data (which this session doesn't
have locally) to tune properly, not more blind iteration against centered reps.

**Decision taken under time pressure (another session blocked):** `CROSS_TZ_VETO_R_MULT`
DEFAULT CHANGED 1e6 -> **1.0 (no-op)**. This makes the veto mechanism inert by default --
restores the exact pre-c3a46d1a data flow for Tz (pinv value passes through the KF
unmodified when origin_ratio fails, same as before this whole fix existed). The improved
relative-drop/streak DETECTION logic is kept in the code (computed, logged via the
`_tz_unreliable_this_solve` flag path) but doesn't DO anything by default until
`CROSS_TZ_VETO_R_MULT` is explicitly raised again. ⚠ **DO NOT re-enable
(`CROSS_TZ_VETO_R_MULT>1`) without a fresh SITL gate covering BOTH a real off-center IC
(IC2/IC3/IC5) AND a rep that reproduces something like the original centered collapse** --
neither alone is sufficient, per the tension found above.

### ⛔ Relative-drop/streak fix DECISIVELY FAILED on real off-center data -- do not re-attempt
Peer session re-gated IC1-5 n=5 on the fixed (veto-off) base: 20/25 land clean (IC1/2/3/4),
confirming the r_mult=1.0 revert works. Copied their off-center bundle
(`test_data/ICValidation/20260908-182815/{IC2,IC3,IC4}_rep{1..5}`) and reconstructed
`origin_ratio` frame-by-frame from `Flow Points Prev/Curr Px` + `Img_Params.txt` (same
formula as the live code) to properly test the relative-drop/streak design against REAL
(not synthetic) off-center data for the first time.

**Confirms the regression mechanism quantitatively:** median origin_ratio is persistently
0.40-0.91 across all 15 successful off-center reps (IC2 ~0.40-0.47, IC3 ~0.53-0.81, IC4
~0.59-0.91), with 51-56% of ALL frames below the old absolute threshold of 1.0 -- during
CLEAN, VALIDATED landings. The absolute-threshold bug would have vetoed roughly half of
every off-center flight.

**Decisively kills the relative-drop/streak fix, independent of tuning:** replayed the
EXACT veto logic (EMA + consecutive-frame streak) against these real reps across a
sweep -- drop_thresh in {0.15, 0.2, 0.3}, streak in {3, 8, 15} -- and false-veto rate
never drops below ~24% of all frames, peaking near 36-45% at looser settings. Tried
smoothing `origin_ratio` with a median-filter window (5/15/30 frames, up to ~0.6s) before
the drop check: rate barely moves (26.1%->24.4%). **This proves the false-triggering
isn't single-frame noise the streak/smoothing was designed to reject -- `origin_ratio`
itself genuinely swings by large relative factors on sub-second timescales throughout
ordinary off-center flight.** A "relative collapse from recent baseline" is not a rare,
distinctive event for this signal off-center; it's routine. No parameter combination
found in ~2 hours of testing (synthetic + 2 real datasets) makes this safe.

**Separately, checked IC5's actual terminal blowup** (`h_z` spike to -7.22 at t~23s,
IC5_rep1) as a candidate "reference-like collapse" for calibrating sensitivity: found
`origin_ratio` does NOT show a clean decaying-drop signature there the way the original
centered-IC1 case did (1.8->0.15) -- it's noisy and non-monotonic (0.06->0.16->0.51->NaN)
right through the spike, with the ratio actually HIGHER right at the peak than
immediately before it. This is consistent with the peer session's own framing that IC5
is a SEPARATE, not-yet-understood terminal-loom blocker, not something origin_ratio-based
vetoing was ever going to catch -- don't force-fit IC5 data into future origin_ratio work.

**Verdict: `CROSS_TZ_VETO_R_MULT=1.0` (no-op) should be treated as the PERMANENT state for
this mechanism, not a temporary holding pattern pending better tuning.** The underlying
quantity (`M0/||c0||^2`) structurally conflates "off-center in frame" with "conditioning
collapse" and no amount of gating on top of it separates them reliably. If Tz reliability
needs a real fix beyond this, the width-loom direction (structurally origin/position-
independent, already validated 0.89-1.00 corr with GT altitude) is the more promising
path -- once its own open item (a usable rate signal, see below) is solved -- not further
work on origin_ratio.

### ⛔⛔ FULL MECHANISTIC PROOF: origin_ratio is unsuitable for cross-marker's SENSOR
### ARCHITECTURE, not just mistunable (2026-09-08, same day, user-directed follow-up)
The user's hypothesis, investigated and CONFIRMED with real-data evidence: `origin_ratio`
(and by extension the whole point-position-statistics Tz family -- pinv AND moment-loom)
is fundamentally unsuited to cross-marker because, unlike ArUco, cross-marker's tracked
flow points have NO fixed physical identity across frames. This is a stronger, more
general, PROVABLE conclusion than "the veto is mistuned" -- it explains WHY every tuning
attempt (relative-drop, EMA, streak, smoothing) failed, and it rules out ever fixing this
with a smarter gate, not just the ones tried.

**1. Architectural root cause, in the code's own design comment.**
`RESAMPLE_PERIOD_S=1.0` (`cross_marker_perception.py:130-152`) forces the tracked LK/GFT
corner pool to periodically re-diversify -- a deliberate 2026-08-07 fix for a DIFFERENT
bug (a one-time-draw corner set freezing Hz/Wz bias for a whole flight, per that comment's
own trace). The explicit trade-off, never previously connected to THIS failure: cross-
marker's point set has no persistent identity the way ArUco's 4 decoded corners do (same
physical corners, same identity, every single frame, by construction of the ArUco decode
algorithm). Cross-marker substitutes a periodically-refreshed, content-dependent sample of
whatever background/arm texture happens to be trackable.

**2. Measured directly on real data: point-set churn is CONTINUOUS, not periodic.**
`N Flow Corners` in a real off-center rep (`IC2_rep1`) swings every single frame --
70,70,54,38,29,42,54,37,35,41... -- not just at 1s resample boundaries. This is LK/GFT
tracking naturally losing/gaining corners frame to frame as background texture and mask
boundaries shift, not an occasional event tied to the explicit resample trigger.

**3. Quantified causal link: churn magnitude predicts origin_ratio volatility, monotonically,
in BOTH centered and off-center real flight data** (`|delta point-count|` vs
`|delta ln(origin_ratio)|`, bucketed, across 15 off-center reps + 5 centered IC1 reps from
`test_data/ICValidation/20260908-182815`):

| `|delta point-count|` bucket | IC1 (centered), median `|delta ln ratio|` | off-center (IC2/3/4), median |
|---|---|---|
| 0 | 0.000 | 0.000 |
| 1-2 | 0.450 | 0.095 |
| 3-5 | 0.454 | 0.099 |
| 6-10 | 0.504 | 0.123 |
| 11+ | 0.569 | 0.370 |

More churn -> more ratio volatility, cleanly, in every dataset tested. This is architecture-
level noise, present regardless of where the marker sits in frame.

**4. The decisive point: the churn-driven relative volatility is actually WORSE for the
centered case (0.45-0.57) than off-center (0.10-0.37) -- centered flights only "worked"
under the old absolute-threshold veto because their BASELINE origin_ratio (median 58.6-70.8
across the 5 IC1 reps) sits so far above the 1.0 threshold that even this larger noise band
almost never crosses it. Off-center's baseline (median 0.40-0.91, driven structurally by
`||c0||^2` being large from the marker's real, legitimate angular offset in frame) sits AT
the threshold, so the SAME architectural noise crosses it constantly (51-56% of frames,
per the earlier finding).** Origin_ratio was never actually a clean signal for cross-marker
-- centered flights were numerically lucky (huge baseline margin absorbing real noise), not
evidence the metric itself was sound.

**Conclusion, provable rather than just empirically-failed:** no threshold on this metric
-- absolute, relative-drop, smoothed, streaked, or any future variant -- can separate "real
Tz conditioning collapse" from "routine point-set churn at a structurally-low baseline",
because both land in the same numeric territory of the metric's own value space for this
sensor architecture. This is a direct, provable consequence of cross-marker's flow points
lacking fixed physical identity (unlike ArUco), not a tuning-effort limitation. **Reject
origin_ratio-based Tz gating for cross-marker permanently and on principle, not just
empirically** -- and treat this as a caution against ANY future point-position-statistics-
based conditioning metric for cross-marker's flow-point source (the same churn mechanism
would corrupt a differently-shaped gate just as thoroughly). This is also the strongest
argument yet for the width-loom direction: it measures a physical mask property at fresh
detected locations, with no dependency on point identity persisting across frames at all --
structurally immune to this exact failure mode.

### ✅ Width-loom RATE signal: negative result REVERSED via 2 bug fixes (2026-09-09)
User directed fixing the rate signal properly (not baking width into control until it's
control-ready) after the 64-combo q/r sweep came back negative. Root-caused instead of
re-sweeping blind, using OverfillCapture_IC1/rep1_data's raw frames:

1. **`_WLOOM_MAX_STEPS=60` capped measurable half-thickness at 24px (~48.4px total).**
   Confirmed directly: from frame ~311 onward in rep1 (well before touchdown), BOTH
   arms' width PINNED EXACTLY at 48.4 for the whole remaining terminal descent -- zero
   real signal exactly where accurate loom matters most. Raised to 300 (~120px half,
   ~240px total -- frame-edge clipping is now the real ceiling, not this constant).
2. **`width_loom_from_detection`'s "median of [w_i, w_j]" degenerates to a plain AVERAGE
   for exactly 2 values** -- not robust to one contaminated arm. Confirmed: a single
   bad-arm spike (itself partly a symptom of bug 1, but not exclusively) dragged the
   reported width up even with the other arm clean. Fix: `_WLOOM_ARM_AGREE_RATIO=2.0`
   -- if the two arms disagree by more than 2x, return None (hold last-good) rather
   than average a good value with a bad one.

**Result:** static width-vs-altitude correlation on rep1 improved further (0.89->0.976),
cap-pinning eliminated (only 3% of frames near the old cap now, with real variation
instead of a flat pin, max width 52 not a repeated 48.4). Re-swept q/r on the FIXED
signal across the same 4 reps (rep1-3 + IC4) and found the entire previously-explored
low-q/high-r region was chasing noise from the buggy input, not smoothing a clean one --
a much MORE RESPONSIVE KF flips the result:

| q | r | rep1 | rep2 | rep3 | IC4 | mean |
|---|---|---|---|---|---|---|
| (old default) 5.0 | 0.05 | -0.38 | -0.69 | -0.39 | 0.24 | -0.30 |
| 10.0 | 0.005 | 0.15 | 0.18 | 0.49 | 0.69 | **0.38** |
| 30.0 | 0.02 | 0.15 | 0.16 | 0.49 | 0.70 | 0.37 |

**All 4 reps positive for the first time**, a flat plateau across q=10-100 (with r scaled
to keep q/r roughly constant) giving similar results -- suggests the KF's steady-state
gain (q/r ratio), not either absolute value, is what matters. Defaults updated:
`CROSS_WLOOM_KF_Q` 5.0->10.0, `CROSS_WLOOM_KF_R` 0.05->0.005.

**Honest status: real, qualitative progress (negative -> consistently positive), not yet
control-ready.** 0.15-0.70 is far short of the static width's own 0.89-1.00, and this was
only tested on 4 reps. Before considering this for control: (a) validate on more reps
including off-center ICs (only IC4 tested off-center so far), (b) check whether the
weaker reps (0.15-0.18) have their own specific, findable bug the way the strong ones did
after the first two fixes, rather than assuming this is the signal's natural ceiling.

### ⚠⚠ CRITICAL: `tools/gt_optical_flow.py` had 3 real bugs — every terminal-window
### correlation in this file (both hypothesis tests + the retracted "breakthrough") was
### scored against a corrupted reference. FIXED 2026-09-08.

User flagged (from prior-project-history instinct, not a re-derivation): `V_h_g = V_v/(zB+0.01)`
in `tools/gt_optical_flow.py` is the OLD formula; `gt_feedback.py` (the maintained sibling doing
the identical job for the online GT-FEEDBACK path) baked `Z_REG` 0.01->0.2 back on 2026-06-30/07-02,
but this file — the canonical **offline** GT-scoring tool this entire session used as ground truth
— was never touched (`git log --follow` shows exactly 2 commits ever, neither about Z_REG).
Comparing the two files side by side surfaced two MORE bugs beyond the one the user named:

1. **`Z_REG` 0.01 -> 0.2** (the one the user flagged). Confirmed via `PLASMC_GT_Z_REG` default in
   `gt_feedback.py:191`.
2. **Missing camera/marker mount-offset correction (the dominant one).** `gt_feedback.py` computes
   `W_x_tu = marker_ned - cam_ned` (camera +0.15 m off base_link via `_CAM_OFF_FLU`; marker offset
   0/0.5 via `_MARKER_OFF_FLU` for flat/rover targets). `gt_optical_flow.py` used the RAW
   `target_origin - uav_base_link` vector with **zero** offset correction. Measured on rep1 IC1's
   last logged sample: raw zB=+0.014 m vs camera-corrected +0.162 m — an **11x relative error at
   the exact touchdown instant**. This alone explains most of the "near-zero-depth blowup" in the
   terminal window that both rejected hypotheses (point-count churn, tilt-leveling) were built to
   explain, and that the self-corrected "0.85 breakthrough" was implicitly fit against.
3. **Unclamped signed depth + a stale `abs(zB)>=0.1` gate.** `gt_feedback.py` clamps
   `zB=max(W_x_tu[2],0.0)` and explicitly has NO altitude gate ("1/(z+Z_REG) stays bounded to the
   deck" once Z_REG=0.2). `gt_optical_flow.py` kept the old unclamped signed zB AND the
   `abs(zB)>=0.1` gate (a leftover from the 0.01-regularizer era) — `nan`-blanking `loom`/`B_h_g`/
   `V_h_g` below 0.1 m altitude, i.e. a hard reference gap in exactly the terminal window under
   dispute.

**Net effect after all 3 fixes** (rep1 IC1, near-ground band): the old reference showed a sharp
loom ramp to -0.96..-1.10 approaching 0.10-0.13 m alt then `nan` below it. The corrected reference
is smooth and bounded through the whole descent (no nan anywhere), peaks around -0.6 near
alt~0.18m, and never reads the fake near-zero depths the old version reported (true min
camera-marker depth this rep ~0.16 m, not ~0.01-0.05 m).

**Implication: every terminal-window conclusion earlier in this file (point-count-churn hypothesis,
tilt-leveling hypothesis, the retracted small-sample "breakthrough") was validated against a
reference that stacked all 3 of these artifacts, concentrated in the exact window under dispute.**
The real sensor-vs-GT discrepancy in that window may be smaller than characterized, or may not
match the shape previously assumed. **Any correlation number computed before 2026-09-08 against
`gt_optical_flow.py` in the sub-0.3m altitude band should be treated as unreliable** until
re-validated against the fixed tool. Re-validation was queued next (rebuild width_loom offline
from `Line Points I/J Raw` per-frame, since masks weren't logged in `OverfillCapture_*`) but not
yet completed as of this entry.

### Re-validation against the fixed GT reference — REGRESSES, not confirms, the earlier "0.38" win

Reconstructed real (not approximated) `isolated_mask`+`line_points_i/j` by re-running the ACTUAL
production `cross_marker_detector.detect()` on the raw `IMG_RECORD` PNG frames (all 7 reps have
them, paired to their `*_data` dir by mtime order: rep1-3 -> 13:38/39/40, IC2-5 -> 13:49-53 on
2026-09-08) — no rotation needed, `gz_subscriber.py` already rotates 90° CW upstream of the
detector, matching the stored frames. Ran the SAME `width_loom_from_detection` + `_kf_step`
(Q=10.0, R=0.005) machinery live code uses, then correlated the resulting rate against the fully
FIXED `gt_optical_flow.py` (all 3 bugs above). Script: replay_wloom.py (scratchpad, not yet
committed to tools/).

| rep | corr (whole descent) | corr (alt<0.5m, terminal) |
|---|---|---|
| IC1 rep1 | 0.31 | 0.04 |
| IC1 rep2 | 0.25 | 0.10 |
| IC1 rep3 | 0.19 | -0.02 |
| IC2 | 0.22 | -0.03 |
| IC3 | 0.29 | 0.13 |
| IC4 | 0.10 | -0.10 |
| IC5 | 0.31 | 0.29 |
| **mean** | **0.24** | **0.06** |

**This is WORSE than the previously reported 0.38 mean, not better.** The earlier "breakthrough"
was computed against `gt_optical_flow.py` while it still had all 3 bugs (stale Z_REG, missing
mount offset, unclamped-depth altitude gate) — i.e. against a reference whose terminal-window
shape was itself a sharp artificial ramp-then-nan. The width-loom-rate KF output apparently
correlated with THAT ARTIFACT'S SHAPE, not with real physical loom. Once the artifact is removed,
there is no reliable terminal-window signal left (-0.10 to +0.29, essentially noise).

**Honest conclusion: the width-loom-rate signal, as currently built, does NOT solve the
terminal-overfill loom problem.** The whole-descent correlation (0.10-0.31, consistently positive
across all 7 reps) says the signal carries SOME real information about loom in general -- just not
specifically where it's needed (the terminal window). This re-validation should be treated as
closing this particular thread's "is width-rate control-ready" question with a NO for now, not as
a bug to chase further without a new idea for what's actually missing. The STATIC width measure
(0.89-1.00 corr, no derivative) remains solid and unaffected by any of this -- only the RATE/
derivative signal is in question.

### Remaining genuinely open item (not started)
Turning width into a CONTROL-READY RATE signal (`d(ln width)/dt`, Tz-like) -- decided to use a
KF-based derivative (not Savitzky-Golay, which adds ~0.5s lag at the smoothing needed for
acceptable noise). Not yet designed or implemented. The natural approach: reuse `_kf_step`
(already in this file, 2-state value+rate per channel) on `ln(width_loom_from_detection(det))`
as a new, separate KF channel -- same machinery `_hw_kf_x`/`_hw_kf_P` already uses, just a 7th
(or standalone) channel. Validate the resulting rate against GT loom the same way every other
candidate was validated this session (corr with `gt_optical_flow.py`'s `loom` field) before
considering it for anything beyond shadow-mode logging.

### ⛔ PURE line-width RATE (`"Width Loom Rate"` = `-d/dt ln(width)`, `_wloom_kf` Q=10/R=0.005) —
### CONFIRMED NON-STARTER (2026-09-09, 18 real reps: the 2 Multi_IC gate bundles + 3 sanity reps)
`width_perf.py` — corr with GT loom, per-rep mean / min / max over the 18 reps:

| band | Width Loom Rate | (Scale Loom Rate, same reps) |
|---|---|---|
| whole descent | **0.31** / 0.15 / 0.45 | 0.49 / 0.33 / 0.58 |
| >2m | **0.23** / 0.11 / 0.34 | 0.40 |
| **.5-2m (the only usable band)** | **0.22** / −0.09 / 0.68 | **0.77** / 0.38 / 0.97 |
| <0.5m | 0.32 / −0.84 / 0.88 (pure noise, wild swing) | 0.05 (noise) |

- In the `.5-2m` band the affine-fit SLOPE of Width Loom Rate vs GT loom is **near zero on most
  reps** (a = 0.004, 0.003, 0.02, 0.036, 0.061, 0.072, 0.116, ...) — the signal carries almost
  no loom information there; the fit is all intercept (b ≈ −0.33, i.e. "predict the mean descent
  rate"). Half the reps are flat-zero corr; a handful reach 0.4-0.68 — inconsistent, not usable.
- The `<0.5m` "0.32 mean" is an artefact of huge per-rep spread (−0.84 to +0.88), not signal.
- Matches the earlier finding (width-only `a=1` blend → mid-band 0.14). **The extent term is what
  makes `"Scale Loom Rate"` work; the pure line-width rate does not track loom well enough for
  anything.** The static line-width VALUE (0.89-1.00 corr with GT altitude) remains fine — it's
  specifically the DERIVATIVE that fails, because `ln(width)`'s slow state-dependent drift
  (residual autocorr 0.5-0.9) dominates its own time-derivative.

**Whole line-width-loom-RATE thread is now closed NEGATIVE.** Static width value: good, unused.
Pure width rate: doesn't track loom (~0.22 in-band, often 0). Extent-fused rate (`"Scale Loom
Rate"`): tracks loom (0.77 in-band) but can't be fed into h_z (see the dead-end verdict below).

### ✅ RATE signal made usable via EXTENT FUSION (2026-09-09, next session) — SHADOW-MODE landed

> ⚠ **NAMING CLARIFICATION: `"Scale Loom Rate"` is NOT a line-width signal — it is ~70%
> `MARKER_EXTENT_PX`.** Line-width was the *starting hypothesis*; when tested alone (the separate
> `"Width Loom Rate"` log channel = pure `-d/dt ln(width)`) it was WEAK — ~0.14 corr in the
> mid-descent band. `MARKER_EXTENT_PX` (marker bbox size, a different observable) alone carried
> ~0.73. So `scale_z = 0.3·ln(width) + 0.7·ln(MARKER_EXTENT_PX)` — extent-DOMINATED; the 0.3
> width term is kept only for a small worst-rep robustness gain. Don't describe `"Scale Loom
> Rate"` as "loom rate from line-width" — that's `"Width Loom Rate"`, which didn't pan out.

Followed up the "needs a new idea" verdict. Re-diagnosed on the 7 OverfillCapture reps (real
detector replay, FIXED `gt_optical_flow.py`), scripts in scratchpad `diag_wloom_{rate,drift,gray,v3,v4}.py`:
- **Terminal loss is NOT lag** — GT-loom-shift sweep (τ=0..0.2s) shows corr *decreasing* with τ, flat.
  The <0.5m signal is genuinely gone (frame saturates, both scale observables pin), not delayed.
- **NOT mask binarisation** — a grayscale half-max sub-pixel scan vs the binary-mask walk was a
  wash (static R² and residual autocorr unchanged).
- **NOT regressable** — static `ln(width)` fit residual has autocorr 0.5-0.9 (slow drift) and is
  NOT explained by tilt / MARKER_EXTENT_PX / centroid-offset / n-corners individually. It's
  residual point-set-churn × perspective in the `line_points` scan (same no-fixed-point-identity
  architecture fact as origin_ratio).
- **Const-acceleration (3-state) KF: worse** (0.02-0.12) — 3rd state amplifies noise. Rejected.
- **THE FIX: fuse `MARKER_EXTENT_PX` (a second, smoother 1/z observable from the same detector)
  with width in log-space BEFORE a *gentle* CV-KF derivative.**
  `scale_z = 0.3*ln(width) + 0.7*ln(extent_px)` (both FRESH & >0, else predict-only coast);
  `rate = -d/dt scale_z` via `_kf_step`, **Q=1.5 R=0.03** (gentler than the width KF's Q=10/R=0.005
  — extent is smoother, wants less aggressive filtering). The constant width↔extent unit ratio
  drops out of the derivative, so NO online median-matching needed — `ln(extent_px)` used raw.
  a=0.3 (not extent-only a=0) purely for worst-rep robustness (min >2m 0.77 vs 0.68).
- **Validation (corr with GT loom, 7 reps, real-detector replay):**
  **>2m band mean 0.90 / worst-rep 0.77 ; 0.5-2m band mean 0.80 / worst-rep 0.68 ; <0.5m ~0
  (mean -0.01), fails SAFE toward 0** (unlike the pinv h_z which spikes to +5.6 / logged h_V[:,2]
  to 25 in exactly this window). Per-rep >2m all ≥0.77, 0.5-2m all ≥0.68. This is a usable loom
  estimate for the 7m→0.5m portion of the descent — the majority of it.
- **LANDED shadow-mode** in `cross_marker_perception.py` (backup:
  `Obsolete/src/cross_marker_perception_pre_scalerate_20260909.py`): new `_scale_rate_*` KF in
  `__init__` (env `CROSS_SCALE_RATE_A` / `_KF_Q` / `_KF_R`), stepped in `process_frame` right after
  the width-loom-rate block, exposed via `getLogData()` as **`"Scale Loom Rate"`**. +67 lines,
  purely additive, consumed by NO control path. SITL not running when edited (checked).
- **STILL SHADOW-MODE.** Wiring into `h_z` (inverse-variance blend with pinv, or spike-veto on
  pinv in the 0.5-2m band) is a SEPARATE, SITL-GATED step — not done.

**Off-center de-risk (2026-09-09, no SITL) — the EXTENT half validated on 25 real off-center
descents.** `20260908-182815` IC1-5 rep1-5 have no raw frames (no detector replay → no per-frame
width), but `MARKER_EXTENT_PX`+`Time`+`Ground_Truth` are logged, so ran the a=0 (extent-only)
limit of the KF (`diag_scalerate_offcenter.py`) against fixed `gt_optical_flow`:
- **IC1-IC4 (20/20 reps): strong + consistent** — **0.5-2m band corr 0.85-0.98**, >2m 0.40-0.52,
  `|rate|` bounded ≤0.82 every rep. The 0.5-2m danger band (where pinv h_z spikes) is *excellent*
  on real off-center flight — better than the static-start OverfillCapture reps. The prior worry
  that off-center geometry (large centroid offset) would break it is DISCONFIRMED.
- **IC5 (5 reps): breaks down** — rep2/3/4/5 go negative in 0.5-2m (-0.05..-0.49), `|rate|max`
  1.06-1.70 (vs ≤0.82 elsewhere), boundedness weakens. Consistent with the standing memory note
  that **IC5 is a separate, not-yet-understood terminal-loom blocker** (its h_z spike to -7.22 has
  no clean signature). Do not force-fit IC5 here either. If this ever feeds control: add an
  explicit output clamp (`|scale rate| ≤ ~1.0`) and an IC5-style hold/reject.
**✅ LIVE-PATH PARITY + FRESH OFF-CENTER SITL (2026-09-09, HEADLESS, user-authorised).** Ran 2
fresh `cross_marker` HEADLESS landings with `IMG_RECORD=1`, IC2 (`INITIAL_DRONE_ENU=2,2,5`) and
IC3 (`-2,2,5`) — real perception, not GT-FB. Checked no concurrent SITL first. Both SUCCESS
attempt 1. Data: `test_data/Landing_Test/Wed Sep  9 01-53-17 2026` (IC2) + `…01-56-11 2026` (IC3);
raw frames `test_data/Test_Videos/Wed Sep  9 01-52-59 2026_raw` / `…01-55-54 2026_raw` (375 frames
each = terminal portion only, IMG_RECORD doesn't capture the whole descent). Verify script:
scratchpad `verify_live.py`.
- **`"Scale Loom Rate"` IS in `getLogData()` output** — 1273 (IC2) / 1202 (IC3) finite rows,
  bounded range [-0.46, 0.72] (IC2) / [-0.98, 1.15] (IC3).
- **Live vs offline-replay PARITY: corr 0.965 (IC2) / 0.952 (IC3), RMSE 0.045 / 0.050** — the live
  `process_frame` path reproduces the offline detector-replay signal. Implementation is faithful.
- **Live `"Scale Loom Rate"` vs GT loom:** 0.5-2m band **0.91 (IC2) / 0.84 (IC3)** — matches the
  offline off-center prediction (0.85-0.98). Beats live `"Width Loom Rate"` (0.33 / 0.16) decisively.
- **>2m band weak LIVE** (0.46 / 0.36) vs offline-replay (0.92 / 0.94): the offline replay only
  sees the last 375 frames; the live full-descent >2m band includes the settle/engage transient +
  KF warmup. Not a concern — >2m has margin and pinv works there.
- **<0.5m: 0.35 (IC2) / -0.44 (IC3)** — terminal still unreliable/sign-unstable per rep, but
  bounded (|rate| ≤ 1.15). Confirmed yet again: no method works terminally.

### SESSION VERDICT (2026-09-09)
Width-loom RATE is now a **usable loom estimate for ~5m→0.5m** via extent fusion, validated
offline (7 static + 20 off-center reps) AND live (2 fresh off-center SITL, parity confirmed):
**0.5-2m band corr 0.84-0.98, >2m 0.77-0.94 (offline) / ~0.4 live-full-descent, <0.5m dead but
bounded/fails-safe.** Shadow-mode `"Scale Loom Rate"` is LANDED + committed (`4a9213b0`).
**NOT wired into control.** Remaining before promotion to h_z:
- SITL-gated wire-in design: inverse-variance blend with pinv h_z, or a spike-veto on pinv when
  |pinv h_z − scale_rate| large in the 0.5-2m band. Behind a default-OFF env flag.
- Output clamp `|scale rate| ≤ ~1.0` + an IC5-style hold (IC5 breaks the signal — separate blocker).
- The wire-in itself needs the full IC2-5 n=5 SITL gate (it's a control-path change, not a
  perception fix).

### ⚠ WIRE-IN LANDED DEFAULT-ON (2026-09-09, `c843a7a1`) — user directed, classified as a
### PERCEPTION change (controller.py untouched); NOT flight-tested yet
User's call: feeding `Scale Loom Rate` into `h_z` is a perception-ESTIMATOR change (the loom is a
perception output; `controller.py` still just reads `getOptFlowAngVel()[2]`), so the IC2-5 gate's
control-change requirement doesn't strictly bind. Implemented + committed default-ON over my
stated reservation that there is zero closed-loop evidence.
- **Mechanism:** second sequential scalar KF correction on `_hw_kf_x[2]` (loom channel) at the end
  of `_kf_update_hw`, measurement = `scale_rate / _sensor_cal_hw[2,2]` (0.9513 — scale_rate was
  validated on the CALIBRATED loom scale, `_hw_kf_x` is RAW). Corrects VALUE only (`H=[1,0]`);
  rate state left to the main KF. Applied in BOTH the measurement and coast branches.
- **Guards:** `_scale_fuse_on` (env `CROSS_SCALE_RATE_FUSE`, default 1) · hw-KF initialised · hw-KF
  NOT frozen (don't mutate the shared frozen array) · scale KF initialised AND
  `_scale_rate_measured_this_frame` (both width+extent fresh — skip on a coast so a collapsing
  terminal / detect-miss estimate never drags h_z) · `|scale_rate| <= CROSS_SCALE_FUSE_CLAMP`
  (1.0 — rejects IC5-style excursions; IC5 breaks the signal, still a separate blocker).
- **1-frame lag:** the scale KF steps in `_log_frame_data` which runs AFTER `_kf_update_hw`, so
  the fusion consumes the previous frame's `scale_rate` (~26 ms @ 38 Hz). Deliberate, acceptable.
- **Env:** `CROSS_SCALE_RATE_FUSE=0` fully restores shadow-only (verified bit-identical `_hw` in a
  unit smoke test). `CROSS_SCALE_FUSE_R` (0.05 — tighter than the primary loom `FLOW_KF_R=0.1` so
  it can pull a spiking h_z down over ~2-3 frames). New log key `"Scale Fuse Z"` = the applied
  pseudo-measurement (raw scale) or NaN when the fusion didn't fire.
- **Smoke test only:** `_kf_update_hw` with no scale KF → bit-identical `_hw`, NaN log; with a
  primary h_z spike to 5.0 + a valid `scale_rate=-0.4` → fused h_z pulled to ~0.94 in one frame
  (converges toward -0.42 over a few); out-of-clamp `scale_rate=-5` → skipped, NaN logged.
- **1-frame lag:** the scale KF steps in `_log_frame_data` which runs AFTER `_kf_update_hw`, so
  the fusion consumes the previous frame's `scale_rate` (~26 ms @ 38 Hz).

### ⛔⛔ WIRE-IN GATED AND REVERTED — DEFAULT-OFF again (2026-09-09, `3137d4cd`)
Sanity A/B (IC1+IC2 fuse-ON vs IC2 fuse-OFF, n=1, on `c843a7a1`): all PRECISE, no fly-away, and
the fusion visibly CAPPED a pinv h_z positive excursion (1.71→1.00). But fuse-ON xy was 3-4×
worse than fuse-OFF (0.06-0.08 vs 0.019) with rel_vel 0.4-0.56 vs 0.02-0.41 — an early warning
I under-weighted. Also found the coast-guard doesn't stop the fusion firing in deep overfill
(width stays valid, scale-rate just decays to ~0), so added `CROSS_SCALE_FUSE_MAX_EXT=310`
(`cd6dc57f`) — skip fusion near frame saturation.

**IC1-5 n=3 gate on `cd6dc57f` (`test_data/Multi_IC/20260909-022319`): HARD REGRESSION.**
13 reps scored: **3 PRECISE-only / 1 SOFT+PRECISE / 8 FAIL / 1 NOT_LANDED**, xy up to 0.41 m,
**rel_vel up to 1.44 m/s**, one false-touchdown 2.55 m up (flow-freeze). Baseline: a clean
concurrent **fuse-OFF** gate on the SAME `visibility_projection` control code minutes earlier
(`test_data/ICValidation/20260909-021038`, n=1) had IC2/3/4 all **SOFT+PRECISE, xy ~0.017 m,
rel_vel ~0.02** — i.e. the control stack was in excellent shape and the fusion broke it.

**Answer to "how does the loom rate perform in TOUCHDOWN DETECTION": the touchdown DETECTOR is
not implicated.** `_touchdownDetectV2` (the default for cross-marker perception) reads
n_flow_corners / MARKER_EXTENT_PX / background-flow-freeze — **never h_z**. Every gate rep +
every sanity rep latched via `[overfill]` or `[flow-freeze]`. The legacy `_touchdownDetect`
loom-spike path DOES use `h_z` but is only active for ArUco/GT-FB (V2 `return`s first for
cross-marker), and it needs `h_z>0` sustained + extent-flattened — and `CROSS_SCALE_FUSE_MAX_EXT`
disables the fusion in exactly that near-saturation regime anyway. So the fusion has ~zero
effect on the touchdown *decision*. **The regression is in the DESCENT-RATE control that h_z
feeds** (loom-error → middle-loop SMC): `scale_rate` correlates 0.84-0.98 with GT loom but is
NOT unbiased; fused at `r=0.05` (tight) it biased the loom-setpoint tracking → fast/erratic
arrival → the detector correctly fires `[overfill]` on contact, just too late/too fast.

**Kept:** all machinery, the shadow logs (`"Scale Loom Rate"`, `"Scale Fuse Z"`), every env flag.
`CROSS_SCALE_RATE_FUSE=1` re-enables. **Before any retry:** (a) characterise `scale_rate`'s BIAS
vs GT loom (not just correlation) and de-bias it; (b) much looser `r` (≥ the primary `FLOW_KF_R`
= 0.1, probably 0.3-0.5) so it's a gentle sanity nudge, not a co-equal sensor; (c) consider
limiting it to only VETO a pinv spike (|pinv h_z − scale_rate| large) rather than continuously
correcting; then (d) a full IC1-5 n=5 gate. The shadow signal + its 0.84-0.98 mid-descent
correlation are unaffected and still the best loom-value candidate — it's the naive KF-fusion
wiring that failed, not the signal.

### METHODOLOGY NOTE
Landing a control-feeding change **default-ON** without a gate (even when classified "perception")
cost a wasted 13-rep gate and risked confusing a concurrent session's own control work. The
sanity A/B's 3-4× xy degradation was already the reject signal per
[[feedback_reject_on_single_failure]] — should have flipped to default-OFF THEN, gated, and only
promoted on a pass.

### ⛔⛔⛔ DE-BIAS RETRY ALSO FAILED — scale-rate → h_z fusion is a CONFIRMED DEAD-END (2026-09-09)
User: "de-bias scale_rate vs GT loom and retry with looser r". Did exactly that:
- **Bias characterised** (`debias.py`, 13 gate reps + 2 clean live reps, `GT_loom = a·sr + b`
  per band): NOT a constant gain. `.5-2m` band `a≈0.41, b≈−0.22` (sr over-reads the loom
  slope ~2.4× + a consistent −0.22 offset); `>2m` band `a≈0.10` (sr ~10× hot — engage transient
  / KF warmup, sr large+noisy while true loom ≈0). This is *why* r=0.05 fusion hard-landed:
  continuously dragging h_z far too negative from altitude.
- **Retry (`246e8260`, behind the still-OFF flag):** (1) band-limit to `150 ≤ MARKER_EXTENT_PX
  ≤ 310` (the only regime with signal), (2) affine de-bias `sr_deb = 0.41·sr − 0.22`
  (`CROSS_SCALE_FUSE_GAIN`/`_OFF`), (3) `r` 0.05 → 0.3. IC5 raw-`|sr|`≤clamp reject kept.
- **IC1-5 ×3 gate (`gate_debias.log`): REJECTED on IC1 (centered!)** — rep1 FAIL xy 0.146,
  rep2 FAIL xy 0.329. `rel_vel` DID recover (0.24–0.38 vs the 1.44 before), so the de-bias
  fixed the *hard-landing* symptom — but it exposed/introduced an **xy-accuracy** regression on
  the easiest IC. Killed on the reject-on-single-failure rule (2 FAIL).
- **Mechanism:** `h_z` couples into the middle-loop SMC c-term `−(h·e3)h` and the sliding
  surface (lateral gain scheduling), so a perturbed `h_z` moves the lateral solution too. The
  pinv `h_z` is already good enough (fuse-OFF: xy ~0.017, rel_vel ~0.02) that there is nothing
  to gain and only accuracy to lose.

**VERDICT: do NOT feed `scale_rate` into `h_z` via a KF measurement, in any biasing/weighting.
Three forms tried (naive / +overfill-gate / +de-bias+band+loose-r), all regress.** Marked in
`cross_marker_perception.py` (`ff242d97`). The shadow `"Scale Loom Rate"` stays as a diagnostic;
its 0.84–0.98 in-band correlation with GT loom is real. If it's ever revisited it needs a
*fundamentally different consumer* — not a continuous correction of the live loom estimate.
`CROSS_SCALE_RATE_FUSE` default-OFF is now PERMANENT, not a holding pattern.

### Overlay tool + geometry-width replacement (2026-09-09)
`tools/overlay_width_loom_rate.py` — visualises the width/extent/scale loom-RATE pipeline on the
raw IMG_RECORD frames (arm PCA lines, the 5 scan stations, per-station perpendicular mask-scan
segments, per-arm widths + agreement ratio) with a bottom panel of W/E/S rates vs GT loom.
Companion to `overlay_image_features.py`. Analysing its output on IC1 rep1 + IC4 gave 3 stacked
failure regimes: (1) alt>3m — width is 1-2 px, `d/dt ln(width)` is a *staircase* from sub-pixel
quantisation; (2) alt .5-2m — works, but width still 7 px ≈ quantises to ~7% so E/S (extent) is
the smooth signal; (3) alt<0.5m — **the isolated_mask is a SOLID amorphous blob** (`hole_frac
0.00`, not "holes" — marker + junction + landing-gear intrusions merged), so the mask-scan's
perpendicular "on-mask run" is the *blob cross-section*: overruns to 60-240 px (frame-size) vs a
true ~24 px stroke, arms disagree ~2x, KF rate spikes WRONG-SIGNED. Extent also dead here
(saturates at 318 = frame diagonal).

**GEOMETRY WIDTH landed as the default (`bf812f1f`, `CROSS_WIDTH_GEOM=1`):** stroke width =
`2*sqrt(3)*std(transverse residual of the arm INLIER cloud)`, mean of both arms, gated on
`>=8 pts/arm` + `|angI-angJ|≈90°±22°` (rejects a background/stub "arm") + `junction inside frame
±12px`. Gate fail → `None` (hold-last-good) = honest refusal. The inliers sit on the stroke
centreline so a bigger blob / internal holes don't move them. 5-rep replay: **mask-scan width
<0.5m ranged 14-201 px → geometry 12-35 px, BOUNDED**; valid% 48-73% (rest hold-last-good).
`.5-2m` rate corr with GT loom actually slightly better (~0.25 vs ~0.09) but still not usable;
`<0.5m` still noise, just bounded. **This is a ROBUSTNESS fix to the shadow signal only — it
removes the wrong-signed terminal spikes, does NOT rescue the rate, and the fusion stays a
dead-end.** `CROSS_WIDTH_GEOM=0` restores the mask-scan (kept; used by the overlay tool). Backup
`Obsolete/src/cross_marker_perception_pre_geomwidth_20260909.py`.

### ⛔ TERMINAL WINDOW (<0.5m) IS UNRECOVERABLE — chased it to ground (2026-09-09)
User: "we need this approach near the landing surface, remove the noise using the video". Did a
full pass (`window_fit.py`, `level_test.py`, terminal mask dumps). The `<0.5m` "noise" is NOT
filterable — it decomposes as:
1. **~70-80% of the band is POST-TOUCHDOWN.** GT altitude flattens at ~0.15m and stays there for
   3-4 s (drone parked on the deck) while the clip keeps recording. GT loom ≈ 0, width bounces
   ±3px = pure measurement noise. Scoring `<0.5m` was mostly scoring parked frames. Not a
   loom-estimation problem — the touchdown detector owns that regime and doesn't use loom.
2. **The genuine last ~0.3m of descent has a real, consistent SIGN INVERSION** — `corr(-d/dt
   ln(geomW), GT loom)` = **−0.17 to −0.80** across reps (descent-only, post-TD clipped), getting
   MORE negative with a longer window. i.e. the measured width SHRINKS during the fastest part of
   the terminal drop. Mechanism (terminal mask dumps, IC5 f315): the marker fills the frame then
   **FRAGMENTS / EXITS the FOV** — for off-center approaches (IC5) the camera is mostly looking at
   the GROUND next to the marker, and `line_points_j_raw` latches onto background texture; the
   mask breaks into disconnected blobs; `corr(geomW, inlier_count)` = **+0.65** terminally (width
   tracks detector health, not depth). Motion blur from the fast drop + attitude transient
   compound it.
3. **V-FRAME LEVELING TESTED, no effect** (`level_test.py`: raw term −0.77 → leveled −0.80;
   −0.56 → −0.56; −0.60 → −0.62). The `corr(geomW, tilt)` seen earlier is tilt/altitude
   collinearity in the terminal transient, not causal foreshortening. Closes the "just level it
   like alpha" idea for the terminal window.
4. **Windowed LSQ line-fit of ln(w) or 1/w tested** (`window_fit.py`): `.5-2m` improves to
   **~0.31-0.34** (vs KF ~0.25) at a 0.8s window — a real, small gain for the WORKING band — but
   terminal only gets WORSE with any window (sees more of the inverted trend).

**CONCLUSION: the line-width loom's useful range is ~2m down to ~0.35m. There is no reliable
stroke geometry to measure below that** — the inputs (clean stroke edges) aren't extractable
when the marker is a frame-filling / fragmenting / partly-out-of-FOV blob under motion blur.
The geometry-width change already makes it fail BOUNDED + REFUSING there, which is the correct
behavior for a signal with no information left. Do not re-attempt terminal-window loom from
apparent size — extent, moment-loom, and now geometry-width all die in the same last 0.3m for
the same reason. If a better `.5-2m` shadow rate is ever wanted: geom width + 0.8s trailing
LSQ line-fit of ln(w) (~0.31), not the current KF.

### ✅ WHAT ACTUALLY LANDED FOR THE ORIGINAL PROBLEM — loom-channel innovation gate (`b6a0998d`)
The whole thread's goal was to backstop the intermittent terminal pinv-`h_z` spike (degenerate
point geometry near overfill: +1.1→+5.6 on `ICValidation/20260831-144626/IC1_rep1`). Size-derived
loom failed at that. What DID address it: the hw-KF had **no innovation test on the loom channel**
(the yaw-KF `PLASMC_YAW_KF_GATE` and the VDS lateral-rate KF `PLASMC_VDS_KF_GATE` both have one;
loom didn't). Added in `_kf_update_hw`:
- **Innovation gate** (`CROSS_LOOM_INNOV_GATE`, **default OFF**): down-weights (`r[2] *=
  CROSS_LOOM_GATE_R_MULT=1000`) a loom measurement only when BOTH (a) NIS `= y²/S >
  CROSS_LOOM_NIS_GATE=25` (`y` = residual beyond the KF's own predicted loom-accel trend) AND
  (b) `|y|/dt > CROSS_LOOM_SLEW_MAX=12` /s (plausibility bound — clean flight tops ~5-8 /s
  dt-normalised, GT `|dloom/dt|` p95 ~0.2, the spike ran 20-260). BOTH conditions ⇒ a genuine
  large-but-smooth terminal loom accel (KF rate state tracks it → low NIS) still passes.
  Debounced: `CROSS_LOOM_GATE_MAX_STREAK=6` consecutive trips then accept (don't freeze the
  channel forever — the failure mode of the abandoned origin-ratio veto). New log `"Loom Gate"`.
- **Hard abs backstop** (`CROSS_LOOM_ABS_MAX`, **default 20 = same as the moment-loom `sol[2]`
  clip, DEFAULT-ON, independent of the gate**): clamps the loom VALUE + zeros a runaway rate.
  Catches a NaN / grazing-ray perspective-divide blowup the NIS test can miss (huge `r`
  inflation → huge `S` → tiny NIS).

**Offline validation** (`gate_replay.py`, 6 OverfillCapture reps, real `process_frame`, OFF vs ON):
clear wins at altitude — rep1 `>2m` corr **−0.07 → +0.83**, `h_z` range [−3.4, 2.2] → [−0.9, 0.8];
IC2 killed a −10.9 spike; IC4 `>2m` +0.29 → +0.48 and `<0.5m` −0.30 → **+0.75**. The abs clamp
caught a **+1601** blowup on rep3 (→ +19). **Cost:** `<0.5m` corr goes more negative on
rep1/rep3/IC5 (−0.16→−0.52, −0.26→−0.54, +0.03→−0.54) — but that band is already noise / ~70-80%
post-touchdown, and the touchdown detector (not loom) owns it there. Clean-flight non-spike frames
are bit-identical. Backup `Obsolete/src/cross_marker_perception_pre_loomgate_20260909.py`.

**SITL gate: `CROSS_LOOM_INNOV_GATE=1`, n=3 IC2-5** (`test_data/ICValidation/20260909-162140`,
HEADLESS, gate-ON only vs the peer's fresh QP-gate baseline on the same HEAD — 20/20 land, xy
pooled 0.06-0.07). **Result: CLEAN WASH — no regression.** 12/12 landed, **0 TL, 0 fly-away**
(the primary risk — gate causes a fly-away — did NOT materialise). xy: IC2 .027/.068/.109,
IC3 .021/.027/.063, IC4 .109/**.796**/.085, IC5 .032/.018/.044 → 9/12 ≤0.1, ~0.06 mean dropping
the one bad rep. **IC4_rep2 (xy .796, flight only 9.7s, ended at alt 3.68m) is NOT the gate**:
only 2 gate trips that rep (all >1m), h_z clean and tracking GT (−0.37 vs GT −0.31 at the last
sample), no spike — it's a premature flight termination on a historically flaky IC4 slot (the
scale-rate gate ALSO failed IC4_rep2). Where the gate WAS active (IC4 rep1: 13 trips; rep3: 3;
IC5/IC2/IC3), `h_z` tracked GT loom well — `.5-2m` corr 0.88-0.95, `>2m` 0.31-0.47, range bounded
[−1.2,+1.7], no coasting blowups.
**VERDICT: `CROSS_LOOM_INNOV_GATE=1` is SAFE — clean wash on the stationary IC2-5 gate.** No
terminal spike reproduced in these 12 reps so no measured *benefit* here, but the offline
evidence (rep1 corr −0.07→+0.83, IC4 <0.5m −0.30→+0.75, +1601 blowup caught) stands. It's a
reasonable default-ON candidate as an always-on backstop (+ the abs clamp already default-on);
equally fine to leave default-OFF and enable situationally. n=5 wouldn't change this — the call
is "do you want an always-on safety net that is a no-op in the common case", not empirical.

**BAKED DEFAULT-ON 2026-09-09 (`7e9843ae`, user).** `CROSS_LOOM_INNOV_GATE` default `"0"`→`"1"`;
`CROSS_LOOM_INNOV_GATE=0` restores the pre-gate behaviour. Abs clamp `CROSS_LOOM_ABS_MAX=20`
was already default-on. Full session bake audit (all in `cross_marker_perception.py`):
- **ON:** geometry width (`CROSS_WIDTH_GEOM=1`), loom abs clamp (`=20`), loom innovation gate
  (`CROSS_LOOM_INNOV_GATE=1`).
- **OFF by verdict:** scale-rate→h_z fusion (`CROSS_SCALE_RATE_FUSE=0`, confirmed dead-end).
- **Shadow logs only:** `"Scale Loom Rate"`, `"Width Loom Rate"`, `"Scale Fuse Z"`, `"Loom Gate"`.

### Loom gate on MOVING ROVER — offline check (2026-09-09, unvalidated closed-loop)
The rover pipeline shares `CrossMarkerPerception` / `_kf_update_hw` (no rover conditionals), so
the baked gate is LIVE in rover runs. Checked offline:
- **Static-rover reps** (peer's `RoverCBFSweep/20260909-163929`, current HEAD): `"Loom Gate"` key
  present, 8-9 trips/descent, `h_z` bounded [−0.7,+2.8], `|dh_z/dt|` p95 ~1.2 << `SLEW_MAX=12`. Fine
  (static rover ≈ stationary marker for loom).
- **Old moving-rover cross_marker rep** (`Cross_Marker_Montage_Rover/Aug 24`, PRE the 08-27 camera
  change): the loom channel is **~15× noisier** than stationary — `|dh_z/dt|` p90=10, p95=21,
  max=135 — while GT `|dloom/dt|` never exceeds 0.8 /s. So on moving rover the gate would fire
  ~9% of descent frames, but every trip is suppressing genuine noise, not real signal;
  `SLEW_MAX=12` sits well above all legitimate loom change. Debounce (6 frames) caps any coast at
  ~0.16 s.
- **Verdict: no evidence of harm on rover; the gate is arguably MORE useful there.** NOT tested on
  the current camera/cal for moving rover, and NOT tested closed-loop (rover landing is
  perception-blocked upstream — `project_20260901_rover_cross_perception_diagnosis` — so the gate
  can't be evaluated in a rover landing regardless).
- **✅ 7-PROFILE MOVING-ROVER ANALYSIS (2026-09-09, `RoverCBFSweep/20260909-163929`, 27/28 reps,
  current HEAD, gate live):** the gate behaves sanely on Static / Linear / Circular / EightShape
  / Sinusoidal / Lissajous / CircularYaw.
  - **Trip rate 0.7-1.8%** of descent frames (worst single rep 2.5%, Lissajous/lead) — NOT
    over-firing; far below the ~9% estimated from old-camera data (current cam/cal has a cleaner
    loom channel). Post-gate `|dh_z/dt|` p95 = 1.3-2.7 /s, well under `SLEW_MAX=12` — no
    false-tripping on normal loom variation.
  - **Abs clamp (±20) NEVER fired** on any of the 27 reps; `h_z` stayed in [−2.4, +3.4] on 6 of 7
    trajectories.
  - **6-frame debounce ceiling IS reached on most rover trajectories** (unlike stationary) →
    ~0.16 s coast then re-accept. Expected given the noisier moving-target loom channel; benign.
  - **One soft spot: Lissajous** — a +9.34 (raw) `h_z` excursion partially slipped through against
    a 445 /s measured jump (streak only 4, KF prediction chasing it). Still 2× under the abs
    clamp, terminal-overfill class. Would want a tighter gate / lower abs clamp only if
    moving-rover loom accuracy ever matters — it doesn't (perception-blocked).
  - **Verdict: no evidence the gate harms rover; consistent with keeping it default-ON.**
