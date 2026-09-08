---
name: project_20260908_line_width_loom_investigation
description: "Deep investigation into terminal-overfill h_z corruption and a line-width-based replacement for the loom estimate -- root causes found, two real bugs in the existing width machinery fixed, a simpler direct mask-scan method designed and validated at 0.89-0.99 corr with GT on fresh real-perception IC1 data."
metadata: 
  node_type: memory
  type: project
  originSessionId: 12257c7c-a2c9-46f1-a6c7-d09063093486
  modified: 2026-09-08T12:55:49.931Z
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

### Remaining genuinely open item (not started)
Turning width into a CONTROL-READY RATE signal (`d(ln width)/dt`, Tz-like) -- decided to use a
KF-based derivative (not Savitzky-Golay, which adds ~0.5s lag at the smoothing needed for
acceptable noise). Not yet designed or implemented. The natural approach: reuse `_kf_step`
(already in this file, 2-state value+rate per channel) on `ln(width_loom_from_detection(det))`
as a new, separate KF channel -- same machinery `_hw_kf_x`/`_hw_kf_P` already uses, just a 7th
(or standalone) channel. Validate the resulting rate against GT loom the same way every other
candidate was validated this session (corr with `gt_optical_flow.py`'s `loom` field) before
considering it for anything beyond shadow-mode logging.
