---
name: project_cross_marker_pipeline_20260801
description: "Cross+stub fiducial pipeline built 2026-08-01 as a decode-free ArUco alternative — status, architecture, and what's next"
metadata: 
  node_type: memory
  type: project
  originSessionId: dbc47060-f292-4fca-8a3d-75a6066d2b5c
  modified: 2026-08-07T12:27:48.610Z
---

Built and committed (916aa53, pushed to main) a standalone perception pipeline for a
decode-free cross+stub marker, as an alternative to ArUco. Motivated by ArUco's
decode-coupled corner-dropout failure mode and the handover/PlanarFeatureMap
complexity needed to extend its decodable range across altitude.

**Why:** yaw is not currently needed (design confirmed early in the build), so the
marker doesn't need ArUco's ID/orientation-from-decode — center recovery via line
geometry is enough, plus an optional one-sided stub for heading if ever needed.

**Architecture (separate from ArUco, not routed through img_data.py's IMG_PROCESSOR):**
- `src/cross_marker_detector.py` — rotation-invariant relative-angle line clustering +
  robust line intersection for center. Handles partial occlusion (per-line robust fit
  degrades gracefully), off-frame extrapolation fallback (intersection can be computed
  even if it lands outside the visible frame — validated separately from a
  support-gated cluster-pairing fix so perspective skew under tilt doesn't let a
  spurious ~90°-apart noise pair beat the real, non-exactly-90° arms). Mask-centroid
  consistency check rejects biased in-frame fits (split into in-frame vs off-frame
  branches with different criteria).
- `src/cross_marker_perception.py` — computes the manuscript's actual image features
  (not the project's ArUco-shaped naming): `s` = homogeneous centroid `[xc,yc,1]`,
  `alpha` = unweighted 2nd-moment orientation over real detected pixels (no ArUco-style
  synthetic corner weighting — the stub's asymmetry is real), `h,w` = optical flow via
  the same image-Jacobian pseudo-inverse as ArUco, fed Shi-Tomasi points sampled across
  the whole color/shape-gated marker plate (not just 4 corners) — this fixes ArUco's
  documented point-starvation and Jacobian ill-conditioning problems with real data
  instead of synthetic scaled-quad points. Point-count-robust h/w tracking added after
  diagnosing GFT starvation at range (mask dilation + two-threshold hysteresis).
- `src/controller.py` — routes to `CrossMarkerNode` instead of `IMG_PROCESSOR` under
  `MARKER_TYPE=cross`. Single-point visibility CBF wired in (reuses `cbf_visibility.py`
  unmodified — its Phase-1 barrier was already centroid-only by design, corner-extent
  only mattered for Phase-2 fallback, which degenerates cleanly to zero for a
  single-point input).
- `src/img_data.py` — reverted to ArUco-only (an earlier synthetic-4-corner-packaging
  approach into IMG_PROCESSOR was tried first, then superseded by the separate pipeline
  once the "don't need PlanarFeatureMap/handover" design decision was made explicit).
- Gazebo asset: `cross_marker` model (3m plane, speckle-textured — texture chosen for
  KLT/Shi-Tomasi feature density) + `cross_marker.sdf` world + dedicated launcher
  `scripts/run_cross_marker_altitude_test.sh` (copied from `run_aruco_landing.sh`
  rather than parameterizing it, to keep the real hardware-adjacent launcher
  untouched). `scripts/validate_image_feed.sh` parameterized with `WORLD`/
  `VALIDATE_SCRIPT` env vars for reuse.
- Test apps: `apps/cross_marker_hover_sanity.py` (exercises the real perception
  pipeline hover-only, no PLASMC loop), `apps/cross_marker_altitude_test.py`,
  `apps/cross_marker_tilt_test.py` (oscillatory attitude commands to test detection
  under real tilt — found the drone drifts the marker out of frame fast with no
  position hold, so tilt validation is time-limited per attempt).

**Validated:** offline (synthetic rotation/occlusion sweep + saved live frames) and
live SITL (ground level, 5m, 7m hover — center detection solid at all three; h/w and
alpha noisier at range but degrade gracefully, not garbage). Partial live tilt
validation (~17° real pitch, short windows before drift) — coherent, no evidence of
the pairing bug recurring, but not a long sustained trial.

**Known gaps / NOT done:**
- `CrossMarkerNode` only implements the interface subset `controller.py`'s hover path
  touches (`center`, `focal`, `close`, `is_alive`, `join`, `CONTROLLER_READY`,
  `FEATURE_IS_VISIBLE`, `_feature_pts`, `getImgFeatureParam`, `getOptFlowAngVel`,
  `getLogData`, `getParams`, `RECORD`, `_ring_loom_source`, `get_center_px`). Terminal-
  kick, ring-loom-fusion, and other descent-specific `IMG_PROCESSOR` attributes are
  NOT implemented — a real closed-loop descent attempt will likely hit
  `AttributeError`s there. This is an explicit, known scope boundary, not silent.
  Fix needed before a real landing attempt.
- Residual sanity-check gap: symmetric-occlusion cases can bias both the mask centroid
  and the line-fit intersection in the same direction (~24px error slipped through in
  the characterization sweep) — the centroid-consistency check can't catch a bias it
  shares with the reference it's checked against.
- Ship-heave test (rocking the marker plate via Gazebo's `set_pose` service, to
  isolate perspective-skew testing from drone-drift confound) was proposed but not
  built — user chose to accept partial tilt data instead.
- `s` (position) is solid and load-bearing; `alpha` explicitly tolerates interruption
  (hold-last-good, no observability-metric gate — user confirmed yaw control handles
  this fine); `h,w` degrades to zero-output under point starvation rather than
  garbage — none of these three needs re-litigating design-wise, only further testing/
  tuning if problems surface.

**Output calibration DONE (2026-08-02).** Built `apps/record_cross_marker_calibration.py`
(phased-excitation recorder mirroring the ArUco one, sourced from `CrossMarkerNode`'s own
raw h,w/s) + `tools/derive_cross_marker_cal.py`. First attempt came back with unstable
centroid scale / low R^2 — root-caused to two compounding issues, both fixed:

1. **Marker plate was rendering as a mirror.** `cross_marker/model.sdf`'s PBR material had
   `<metal><albedo_map>...` with no explicit metalness/roughness, so Gazebo Harmonic
   defaulted it to a reflective surface — it was mirroring the drone flying overhead onto
   its own face. Fixed with explicit `metalness=0`/`roughness=1` (external PX4-Autopilot
   asset, backed up as `model.sdf.bak_20260801_mirrorfix`).
2. **Pre-existing Gazebo camera-render ghost, NOT marker-specific.** Confirmed via
   `apps/diag_raw_image_dump.py` that BOTH the aruco and cross_marker worlds render a
   mirrored duplicate of the drone's own body in the outer ~20% top/bottom frame margin,
   on every frame, even sitting disarmed on the ground. ArUco's small centered tag never
   reached it (a latent, previously-unnoticed rendering bug); the cross marker's large
   (3m) plate legitimately does during the calibration excitation.
   Layered defense in `cross_marker_detector.py` (tried several things, see the file's own
   inline history comments on `_restrict_to_center_roi`/`_reject_blobby_components` before
   trusting a value quoted here):
   - Symmetric `roi_frac=0.5` crop: fixed the ghost but cost real x/y translation range
     (2/5 calibration runs lost the marker out of the crop).
   - Reverting to loose 0.65 (relying on `_isolate_marker_by_shape`'s squareness gate
     alone): inconsistent — when the marker is small/distant its bbox scale is close
     enough to the ghost's for `MORPH_CLOSE` to bridge them into one merged blob that
     evades the squareness filter.
   - Anisotropic crop (`roi_frac_x=1.0`, `roi_frac_y=0.55` — the ghost is specifically a
     vertical-margin artifact post-rotation, not horizontal): better, but the 3m plate can
     still reach the crop boundary during translation and touch the ghost there.
   - Added `_reject_blobby_components` (2026-08-02): position-independent, shape-based —
     measured off a real ghost-overlap frame that the ghost's rotor-hub/housing fragments
     are small near-square near-FILLED blobs (extent = area/bbox_area ~0.65-0.74) vs the
     marker's thin sparse cross strokes (~0.15-0.25); zeroes out high-extent components
     before the ROI crop runs. Loosened `roi_frac_y` back to 0.65 alongside this (shape
     filter is now the primary defense). Does NOT fix genuine pixel-level touching (a thin
     ghost sliver crossing a marker arm merges into a still-thin blob no shape metric can
     retroactively split) — that residual failure mode still exists but is much rarer.
   Combined effect across the investigation: detection ok-rate progressed 33% (broken
   material) -> 59% (material fixed) -> 100% (anisotropic crop, best case) but with real
   bimodal run-to-run variance (some runs 99%+, others 20-80%) -> with the shape filter,
   the FLOOR improved substantially (worst observed run 69.5% vs earlier 22.9%) even
   though perfect 99%+ consistency wasn't recovered. Some residual SITL/render
   stochasticity looks irreducible without deeper Gazebo-engine investigation.
   `tools/derive_cross_marker_cal.py` gates out any run below `CROSS_CAL_MIN_OKRATE`
   (default 0.95, overridable — the 2026-08-02 derive used 0.85 to admit the
   shape-filter-era runs which cluster around 92% rather than 99%+) using the per-frame
   `Diag Log` (`(t, ok, fail_reason, bbox_area)`, from
   `CrossMarkerNode.get_diag_log()`/`CROSS_DIAG_SAVE_DIR` frame dumps) so a degraded run
   can't silently corrupt the fit.

**Derived cal PASTED into `CrossMarkerPerception.__init__`** (2026-08-02, replacing the
identity placeholder): from 4 runs (a 5th gated out at 69.5%). R^2: Hx=0.40 Hy=0.49
Hz=0.34 Wz=0.36 (Wx/Wy forced 0, same level-target convention as the ArUco board cal).
Centroid scale (`sx=0.32, sy=0.29`) still has ~2.4x inter-run spread across the 4 runs —
a usable starting point, meaningfully more robust than earlier attempts (inter-run STD on
the load-bearing Hx/Hy block dropped from ~0.1-0.17 to ~0.008-0.05), but NOT yet as tight
as the ArUco cal typically is (R^2 0.8+). Re-derive with more (gated) runs before trusting
this for precision landing.

**SUPERSEDED 2026-08-03 — two real bugs root-caused in the raw h,w/s computation
itself, both fixed, cal re-derived.** The 2026-08-02 cal above validated at
near-zero/negative R^2 on independent multisine data (the io-calibration skill's
train/validate discipline caught it). Root cause was NOT the calibration
methodology — it was the raw signal:
1. **Missing V-frame gravity-leveling.** `cross_marker_perception.py` computed
   h,w,s from raw un-leveled camera pixels with ZERO attitude compensation,
   while GT (`compute_gt_signals`) is in the tilt-compensated V-frame
   `img_data.py`'s `_getVirtualPts` always projects through. This module never
   got that port. See [[feedback_missing_vframe_leveling_port]].
2. **dt/staleness mismatch on detection-dropout recovery.** dt came from the
   outer polling clock (advances every call) while the LK "previous frame"
   state only advances on successful detections — after any dropout (every run
   had some), the next good frame divided a multi-frame-accumulated
   displacement by a one-frame dt, spiking all six solved parameters. See
   [[feedback_dt_staleness_after_detection_dropout]].
Fixing (1) alone showed no clear improvement (Hz even regressed) — bug (2) was
masking (1)'s real benefit. Fixing both together: raw Hx/Hy-vs-GT correlation
went from ~0.01-0.10 (every test before) to a consistent 0.74-0.87 across
independent flights.

**RE-DERIVED cal PASTED 2026-08-03** (supersedes 2026-08-02): from 5 clean runs
(all passed the 95% ok-rate gate). R^2: Hx=0.70 Hy=0.71 Hz=0.42 Wz=0.52.
Centroid `sx=1.06, sy=1.11`, inter-run spread now only ~7-10% (was ~2.4x).
Compared to ArUco's own 13-run board cal (R^2 Hx=0.75 Hy=0.75 Hz=0.79 Wz=0.71,
per `img_data.py`'s provenance comments): Hx/Hy are now within 0.05 of ArUco;
Hz/Wz remain the honest gap. Cross-marker's centroid inter-run stability
(~7-10%) is actually TIGHTER than ArUco's own live cal (`h_x` 1.091+-0.266 =
~24%, `h_y` 1.063+-0.198 = ~19%).

**RESOLVED 2026-08-03: Wx/Wy genuinely tested, confirmed near-unrecoverable —
zeroing is correct, not a gap.** Ported `rollexc`/`pitchexc` phases (opt-in via
`CALIB_PHASES`, same small-amplitude/high-frequency-oscillation design as
ArUco's own recorder — `tilt_angle ~ A*omega^2` but `tilt_RATE ~ A*omega^3`, so
high omega buys tilt-rate without pushing the marker out of frame) into
`record_cross_marker_calibration.py`. Flew a dedicated
`CALIB_PHASES=yaw,x,y,z,rollexc,pitchexc,yawagg` flight
(`calibration_data/diag_rprexc/`) and checked RAW (pre-cal) Wx/Wy against
un-zeroed GT roll/pitch rate, deduped to genuine new-frame updates only (not
held outer-loop duplicates): rollexc phase, 417 real frames, corr(raw Wx, GT
wx) = **-0.15**; pitchexc phase, 428 real frames, corr(raw Wy, GT wy) =
**-0.09**. Both near-zero AND wrong-signed; raw amplitude ~4x smaller than GT
(std 0.033-0.034 vs 0.11-0.13). This is now a genuine dedicated-excitation
result, not an artifact of untargeted x/y-phase incidental tilt (the earlier
-0.06..+0.28 range from x/y-phase data) — the cross-marker's line-geometry
flow solve does not recover roll/pitch rate with usable SNR. Forcing Wx/Wy=0
is the correct modeling choice for this marker, same as ArUco's own
level-target convention, not an open gap.

**ROOT CAUSE identified 2026-08-03 (radial-spread / Jacobian-leverage
investigation) — why Hz/Wz lag ArUco AND why Wx/Wy specifically die.** Added
a temporary diagnostic, `Radial Diag Log` (`CrossMarkerPerception._solve_
jacobian`/`get_radial_diag_log`, wired into `record_cross_marker_calibration.py`;
backup of the pre-instrumented file at
`src/cross_marker_perception.py.bak_before_radialdiag_20260803`), logging
max/mean `|x|,|y|` of the normalized points actually fed into `_fill_A` per
solve. `_fill_A`'s Hz/Wz columns (h3, col 2 and w3, col 5) are LINEAR in
(x,y); its Wx/Wy columns (w1/w2, cols 3/4) are QUADRATIC (`x*y`, `1+x^2`,
`1+y^2`) — quadratic terms need much larger radial spread for equivalent
leverage/conditioning than linear ones.

Measured spread at the then-live 2.7m/roi_frac_y=0.65 config: mean|x|~0.26,
mean|y|~0.25, max~0.44 in BOTH axes, against frame normalized half-extents
x=1.19/y=0.89 — i.e. tracked points only reach ~35-50% of the available
radial range, on every phase (not just rollexc/pitchexc). Two suspected
causes: (a) `_restrict_to_center_roi(roi_frac_y=0.65)` in
`cross_marker_detector.py`, added 2026-08-02 as ghost-artifact defense, caps
y-reach outright; (b) the marker's own footprint at 2.7m doesn't reach the
frame edges in x even though `roi_frac_x=1.0` is unrestricted.

**Both levers tested and found insufficient, in combination and alone** (see
[[feedback_cross_marker_radial_spread_ceiling]] for the full data table and
the reasoning against retrying either):
- Made `roi_frac_y` env-overridable (`CROSS_ROI_FRAC_Y`, default unchanged at
  0.65 — no behavior change unless explicitly set) in `cross_marker_detector.py`.
  Loosening to 0.90 and 1.00 confirmed `_reject_blobby_components` (2026-08-02)
  is now the real ghost defense — ok-rate held 99.7-99.9% even with the ROI
  crop fully removed — but Wx/Wy correlation only moved to a noisy, sign-
  flipping +0.1..+0.26 (pitchexc best case), never solid, and x-spread barely
  moved at any ROI setting (footprint-limited, not ROI-limited, confirming (b)).
- Lowering `CALIB_TAKEOFF_HEIGHT` from 2.7m: 2.4m held ok-rate 100% and grew
  x-spread modestly (max|x| 0.44->0.55) but did NOT move Wx/Wy correlation
  (-0.17/+0.18, same order as baseline). 2.0m broke detection outright —
  ok-rate collapsed to 25% (`lt2_angle_clusters`/`hough_lt2_lines` dominate;
  marker overflows frame / line-fit geometry breaks down when too close) and
  the few surviving raw values were garbage (std 1.98 vs GT std 0.13,
  10-40x the healthy-config noise floor). The combined 2.4m+ROI-0.90 config
  didn't stack the individual gains either (+0.11/-0.01).

**Conclusion: the Wx/Wy ceiling is the marker's PHYSICAL FOOTPRINT, not a
recorder/detector parameter.** Raw signal std stays 3-5x below GT std in
every tested configuration (0.02-0.09 vs 0.09-0.13) — achievable point
spread never lifts the angular-rate-induced flow meaningfully above LK
tracking noise, and pushing spread via altitude runs into a detection-
failure wall (2.0m) before the flow signal improves enough to matter. The
only remaining lever is a bigger physical marker plate (model-geometry
change in `~/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/model.sdf`,
currently 3.0m at 2.7m calibration altitude) so tracked points sit farther
from center while the whole shape still fits in frame — not attempted this
session. Useful side-finding: z-phase raw Hz vs GT correlation is strong
(r=0.93-0.96) across every tested config — the depth signal itself is NOT
noisy; the lstsq R^2=0.42 gap from the 2026-08-03 cal derive is from
cross-column conditioning/coupling in the joint 6-DOF solve, not from Hz
being intrinsically weak. `Radial Diag Log` instrumentation and the
`CROSS_ROI_FRAC_Y` env override were left in place (both opt-in / no default
behavior change) for any future retry.

**Still not validated:** the re-derived cal hasn't been re-checked against
independent multisine/landing data (only the phased-excitation training data
itself, same gap as before).

**Also found and fixed along the way (2026-08-02/03), all still live:**
- `resolution[::-1]` transpose bug in `self.center` (swapped cx/cy) — see
  [[feedback_duplicated_math_diff_check]].
- Jittered `perf_counter()`-at-callback instead of the frame's own capture
  stamp for dt.
- `derive_cross_marker_cal.py`'s ok-rate gate was counting the pre-arm settling
  period (never part of the recorded flight data) — a run measured at "86% ok"
  was actually 100% over the samples that matter.
- Per-sample time sync (via `Flow Diag Log`'s real per-frame timestamps)
  replacing naive same-index alignment between raw and GT in the derive tool.
- `_reject_blobby_components` now never rejects the single largest connected
  component, so a textured background's stray noise merging onto the cross's
  own rounded line-caps can't push the whole marker's extent over threshold.
- Adopted Hardware/'s `derive_pi_cal_clean_axis.py` strict-purity-gate pattern
  for sx/sy (didn't fix our specific inter-run spread — that turned out to be
  the h,w bugs above, not axis contamination — but is a real, kept hardening).
- `CrossMarkerNode` now sources its quaternion from `Image_Node.getQuaternions()`
  (synced inside the same image callback that captures the frame) instead of a
  separately-polled `FC.getQuat()` — first V-frame attempt used the latter and
  regressed Hx instead of fixing it, even though the measured timing lag alone
  (<=28ms) was too small to explain the regression; the real issue was an
  unrelated editing mistake (a deleted `process_frame()` call) discovered
  during that debug, not the quat source itself — but the synced source is the
  more correct pattern regardless and was kept.

Two retexture attempts (grid, then a user-supplied reference-image-derived
irregular texture) were tried to fix a separate hypothesis (LK correspondence
ambiguity on the marker's coarse ~46px-period speckle) and both failed for
reasons unrelated to that hypothesis (a real conflict between
`_reject_blobby_components`'s ghost-rejection and any added texture noise near
the cross, plus at least one genuine environmental SITL flake) — reverted,
original texture is live. Not revisited after the h,w bugs above turned out to
explain the Hx/Hy weakness on their own.

**ROLLUP 2026-08-06/07 — camera-geometry re-cal, peripheral-bias Hx/Hy fix
DEPLOYED, Wz collinearity mechanism found, z-amplitude lever tested and
ruled out for Hz.** Continues directly from the 2026-08-03 state above.

*Camera/geometry cluster (08-04/05, cause of a transient Hz/Wz dip, not yet
in a dedicated memory):* several same-day changes all touching the raw h/w/s
signal landed together — camera-mount yaw+90° (moved the landing-leg render
ghost from top/bottom into left/right margins), camera Z offset .20->.18/.15,
`cross_marker.png` line width halved, color-gate threshold 20->100 (thinner
line's anti-aliasing needed it), a tracking-based ROI fast path ported from
Hardware's ArUco `ARUCO_ROI_MARGIN_PX` pattern, and — most consequential —
an axis-sign-flip fix in `_getVirtualPts` for the new camera-mount yaw
(`[-y,x]` initially, empirically WRONG — verified via alpha reading ~90° off
and closed-loop `s_e_n` diverging instead of converging; corrected to
`[y,-x]`, see that function's own comment for the full derivation). Re-cal
after this cluster (5 clean runs, 95%+ ok-rate): R^2 Hx=0.55 Hy=0.63 Hz=0.22
Wz=0.57 — Hx/Hy/Hz all DROPPED vs the 2026-08-03 baseline (0.70/0.71/0.42),
Wz roughly flat. This drop, NOT the later peripheral-bias change, is what
took Hz from 0.42->0.22 — isolated below.

*Purity-gate false alarm (08-06, same day, reverted):* briefly suspected
z-phase GT contamination (dominant yaw-rate instead of vz) as a distinct Hz
cause; traced to a bug in an ad-hoc diagnostic script (naive `gt['Phase']`
`[:n]` truncation desyncing from `compute_gt_signals`' own duplicate-
timestamp filter), not real data. All 6 recordings' z/yaw/yawagg phases are
genuinely clean on correct alignment; the purity gate built on the false
premise made Wz worse and was reverted. See
[[feedback_cross_marker_radial_spread_ceiling]]'s 2026-08-06 entry for the
full trace.

*Peripheral-corner-bias fix (08-06) — DEPLOYED 2026-08-07.*
`_sample_flow_points` (`cross_marker_perception.py`) changed to exclude a
central disk (`CROSS_FLOW_CENTER_EXCLUDE_FRAC=0.35`x the mask's own
half-extent) from the `goodFeaturesToTrack` mask plus a frame-boundary
exclusion (`CROSS_FLOW_BOUNDARY_MARGIN_PX=20`), biasing GFT candidates
toward the arm/stub tips instead of letting the cross's central intersection
dominate every frame's point pool (falls back to the unbiased mask if too
few peripheral corners survive). Isolated effect (same 08-05 camera config,
only the point-sampling changed), confirmed via a live re-derive: **Hx
0.55->0.73, Hy 0.63->0.79 (real win, now BEATS the pre-08-05 baseline
0.70/0.71) — Hz stayed EXACTLY 0.22, Wz roughly flat 0.57->0.53 (a wash, not
a regression).** Radial spread genuinely increased as measured (`Radial
Diag Log` p90 0.113->0.184). Decision: no real tradeoff once isolated
cleanly — kept the code and pasted the matrix into
`CrossMarkerPerception.__init__` (R^2 Hx=0.73 Hy=0.79 Hz=0.22 Wz=0.53,
cal_s sx=1.0225 sy=0.9727). This is the LIVE deployed cal as of 2026-08-07.

**Wz mechanism directly confirmed (08-06/07):** raw correlation matrix of
the 6 `Opt Flow Ang Vel` columns has two near-zero eigenvalues (0.0015,
0.0021) — raw `h1`(Ty)/`w0`(Wx) correlate r=0.98-1.00 in EVERY phase alike
(x/y/z/yaw/yawagg/settle), a structural degeneracy in `_fill_A` itself:
the Wx column `-(1+y^2)` stays ~constant (~-1) whenever `|y|` is small,
numerically indistinguishable from Ty's constant `+1` column at this
marker's achievable radial spread. Explains Wz's huge/cancelling
coefficients on columns h1/w0 (+9.4/-9.3 in the deployed matrix, up to
+13.4/-13.4 in other derivations) directly, and explains why Wz did NOT
respond to the peripheral-bias fix the way Hx/Hy did (needs `y` spread
large enough to break `1+y^2~=const`, which peripheral bias's achieved
spread doesn't reach).

**Z-excitation-amplitude tested and RULED OUT as Hz's bottleneck
(2026-08-07).** `CALIB_AMP_Z=1.2` (position-sine amplitude) had been stuck
since the recorder's creation, inherited from `record_output_calibration.py`
where a stale memory had already flagged it as aggressive (~2.2g thrust,
clip-saturates flow_x/y/w_x/y) and recommended ~0.9 without ever applying
it. Single-run test at 2.2: achieved altitude swing 0.52m->0.80m, z-phase
raw-h2/noise-floor SNR 0.16-0.60->1.98 (real ~4x gain; PX4 position tracking
attenuates the commanded sine ~2.3-2.7x at 0.5Hz, so the amplitude parameter
overstates achieved velocity a lot). Full n=4 batch (6 recorded, 1 empty
armable-timeout flake, 1 below the 95% ok-rate gate) re-derived: Hz
0.22->0.25, Wz 0.53->0.56 (both within noise) while Hx 0.73->0.68 and Hy
0.79->0.75 both DROPPED — reproducing the exact ArUco-era tradeoff. A ~4x
SNR gain moving R^2 by only +0.03 rules out amplitude/noise-floor as the
binding constraint for Hz. NOT deployed — live cal stays the AMP_Z=1.2/
peripheral-bias one above. `calibration_data/output_cross_amp22/` kept on
disk as negative-result evidence, not wired into any script.

**Perspective-divide-noise suspect RULED OUT; Hz's real cause found to be a
GENUINE RAW-SIGNAL REGRESSION, not an intrinsic ceiling (2026-08-07).**
`Z_V Log` (already-instrumented min `z_v` per `_getVirtualPts` call) shows
grazing rays never occur during calibration flights — min `z_v` stays
>=0.888 across all 5 current recordings, far above the 0.5 blowup
threshold — so this idea is dead, not just untested. Chasing it prompted
re-checking the 2026-08-03 memory's claim that "raw Hz-vs-GT correlation is
strong, r=0.93-0.96" (used at the time to argue the R^2=0.42 gap was a
joint-fit-conditioning artifact, not raw-signal weakness). That claim is
now FALSE on current data and was simply stale: z-phase raw `h2`-vs-GT
correlation measured 0.07-0.67 (mean ~0.5) on the 5 current recordings, vs
0.90-0.96 on the archived pre-08-05 runs
(`calibration_data/output_cross_stale_pre20260805/`) — direct, controlled
before/after, same maneuver. **Hz's raw signal genuinely broke somewhere
in the 08-04/05 camera/geometry change cluster** (camera-mount yaw+90deg,
camera Z offset .20->.18/.15, line width halved, color-gate threshold
20->100, tracking ROI, axis-sign-flip) — a bug to find and fix, unlike
Wz's structural collinearity ceiling above.

Checked the intermediate pre-peripheral-bias/post-camera-changes batch
(`calibration_data/output_cross_pre_peripheralbias_20260806/`, 6 runs) to
rule out peripheral-bias itself as the cause: already degraded there too
(r=0.21-0.88, high run-to-run VARIANCE) — confirms the regression predates
peripheral-bias and is somewhere in the earlier camera/geometry cluster.

**Leading hypothesis (code-derived, NOT yet flight-tested):** the
2026-08-05 color-gate loosening `V<20->V<100`
(`cross_marker_detector.py`'s own comment traces exactly why: the marker's
line width was independently halved the same day, and at 10m range the
thinner anti-aliased stroke's pixels only reach V~60-140 against a
V~140-160 background, so V<20 left too few pixels for Hough line detection
to find the marker at all) now admits far more anti-aliased EDGE pixels
into the GFT mask. This noise is POSITION-correlated (worse near the thin
stroke's boundary / at range), which should disproportionately corrupt
`_fill_A`'s position-WEIGHTED columns — Hz's `[-x,-y]` and Wz's `[-y,x]` —
while leaving Hx/Hy's position-INDEPENDENT `[1,0]`/`[0,1]` columns
comparatively immune (per-point noise averages out across many points for
a constant column, doesn't for a position-weighted one). This also
explains two things that were otherwise puzzling: why the peripheral-bias
fix didn't move Hz at all (it biases point selection TOWARD the arm/stub
tips — exactly where this noise should be worst) and the high run-to-run
variance in the intermediate batch (consistent with a marginal/brittle
threshold, not a deterministic bug). Next concrete test, not yet run:
tighten the color gate back toward V<20-60 while relying on
`_reject_blobby_components`'s shape gate alone for ghost defense (the same
2026-08-05 comment already confirms that holds without the loose color
gate's help) — check whether Hz's raw correlation recovers.

**Net status 2026-08-07:** Hx/Hy/centroid solid and near ArUco parity
(0.73/0.79 vs ArUco 0.75/0.75). Wx/Wy(forced 0)/Wz(0.53) are the marker-
footprint/radial-spread ceiling (only untried lever: bigger physical
plate). Hz(0.22) is NOT that same ceiling — it's a real regression with a
concrete, testable hypothesis (color-gate anti-aliasing noise) awaiting a
flight test. Still not validated against independent multisine/landing
data (same longstanding gap). Cross-marker code changes through 2026-08-06
(`cross_marker_perception.py`, `cross_marker_detector.py`,
`cross_marker_altitude_test.py`, 3 new tool scripts) were COMMITTED AND
PUSHED 2026-08-07 (`19cfc49`/`75ca05a` after a rebase onto unrelated
Hardware/Pi commits) — but the color-gate hypothesis above and any further
Hz work are NOT yet committed as of this writing; check `git status`. See
[[feedback_cross_marker_radial_spread_ceiling]] for the detailed data
tables underlying all of the above.

**Color-gate hypothesis (above) TESTED 2026-08-07, mostly negative.**
Flew 2 runs each at `V<60` and `V<20` (new `CROSS_COLOR_GATE_V_MAX` env
override, default unchanged): neither showed a clean, consistent
improvement over the current `V<100`'s already-noisy range (V<60:
0.005/0.449; V<20: 0.557/0.745, but at the cost of unstable detection
ok-rate 90%/58%, below the 95% cal-quality gate). Reverted to `V<100` in
the working tree — no color-gate change deployed. See
[[feedback_cross_marker_radial_spread_ceiling]] for the full table.

**Camera-Z-offset bisection ALSO inconclusive (2026-08-07) — and revealed
this whole approach has a problem.** Tested `Z=.20` (2 runs, using the
pre-existing `model.sdf.bak_before_campos_20260805_010737` backup which
has Z=.20 with the 08-04 yaw+90 fix already applied — isolates the Z
change cleanly) vs the live `Z=.15`: 0.791, 0.017 — the SAME ~0-0.8
run-to-run spread seen at every other setting tested (color gate included).
This spread is bigger than the effect any single tested parameter could
plausibly produce at n=2, so none of this session's single-knob bisection
(color gate, Z-offset) can be trusted as confirming OR ruling out its
target — a real answer needs n>=5 per setting (the project's own sweep
floor) or finding the variance's own root cause first. Camera restored to
the live `Z=.15` immediately after the test — **user clarified `Z=.15` was
deliberately chosen to keep the landing legs out of the downward camera's
FoV, not an arbitrary/reversible parameter** — reverting it is not an
adoptable fix regardless of what Hz correlation shows, unless the legs'
physical footprint is separately addressed. See
[[feedback_cross_marker_radial_spread_ceiling]] for the full trace.

**"Landing-leg ghost" investigated for widenability — CORRECTS the
2026-08-02 entry above: it is NOT a mirrored render duplicate, it's the
drone's real legs (2026-08-07).** User asked whether the drone's landing-
leg gap could be widened in the SDF to allow a larger camera Z without
leg interference. Investigating this surfaced that the entry above
("Pre-existing Gazebo camera-render ghost... mirrored duplicate of the
drone's own body... rotor-hub/housing fragments") was a MISDIAGNOSIS.
Dumped raw on-ground disarmed frames (`apps/diag_raw_image_dump.py`,
`calibration_data/diag_raw_ghost_check/`) and cropped the artifact regions
closely: the pre-rotation raw frame shows two distinct leg-strut-shaped
blobs at the TOP and BOTTOM margins only (left/right margins are plain
background floor) — each blob's shape (two diagonal struts converging
into a horizontal skid bar) matches the SDF's own leg COLLISION geometry
almost exactly (`x500_base/model.sdf` `base_link_collision_1..4`: diagonal
struts at y=+-0.098 converging to horizontal skid bars at y=+-0.132, length
0.25m). These are two DIFFERENT real leg assemblies (front vs back
pairs), not one leg duplicated — no second copy of the whole drone body
anywhere in frame. **This was never a rendering bug** — it's the drone's
own real legs entering the wide-FOV downward camera's periphery, almost
certainly conflated at the time with the SEPARATE, actually-confirmed
marker-reflectivity bug from the same investigation session (a real
mirroring bug, but of the MARKER's material, not the camera render).
All of `cross_marker_detector.py`'s "ghost defense" code
(`_restrict_to_center_roi`, `_reject_blobby_components`, `ROI_FRAC_X/Y`)
still functions correctly regardless of this relabeling — it was built and
validated against real captured frames showing this real artifact, the
naming/mental-model was just wrong, not the defense itself.

Also checked: **no SDF-only lever exists to widen the leg gap.**
`x500_base/meshes/` has exactly one combined visual mesh
(`NXP-HGD-CF.dae`) for the ENTIRE airframe — body, arms, motors, AND legs
all baked into one `<visual>` element with no separate leg link/pose to
reposition. The 4 leg COLLISION boxes are independent physics-only proxies
(invisible to the camera) — moving them would not change what's rendered
at all. Widening the visual leg gap genuinely requires editing the `.dae`
mesh asset itself (3D-modeling tooling, e.g. Blender) — there is no
config/pose-only path. Per user instruction, did NOT attempt any mesh
edit. This lever (bigger camera Z via wider legs) stays open only if the
user does the mesh edit externally; not pursued further this session.
User decision: only pursue the full physical scale-up (real mass/inertia/
rotor-arm/thrust scaling, not a cosmetic mesh-only trick) if this whole
Hz/output-cal thread reaches a dead end that specifically needs more
camera height — not a default next step.

**VARIANCE SOURCE FOUND AND FIXED, Hz DOUBLED, DEPLOYED (2026-08-07,
same session).** Pure data analysis (no new flights needed to find the
cause) of the already-collected `Flow Diag Log`s showed `_sample_flow_points`
(the GFT corner search feeding the h,w Jacobian solve) was only ever
called on cold-start or when the tracked point pool dropped below
`RESAMPLE_TRIGGER=10` — between those events the SAME point set just
tracks forward via LK indefinitely. 4/5 pre-fix calibration runs showed a
perfectly CONSTANT `n_kept` for their entire ~500-frame z-phase (zero
resamples) — whichever corner subset got picked once, near takeoff, just
persisted for the rest of the flight. Because Hz/Wz's raw columns in
`_fill_A` are position-WEIGHTED (`[-x,-y]`/`[-y,x]`) while Hx/Hy's are
position-INDEPENDENT constants, this one-time draw's exact (x,y) locations
disproportionately determined the WHOLE flight's Hz/Wz quality while
leaving Hx/Hy comparatively robust — explaining the large, previously-
unexplained run-to-run variance that made every single-parameter
bisection this session (color gate, camera Z-offset) inconclusive.

**Fix implemented:** `RESAMPLE_PERIOD_S=1.0` (env `CROSS_RESAMPLE_PERIOD_S`)
forces the same top-up-refresh logic periodically, independent of pool
size, so the tracked set gets re-diversified regularly instead of frozen
from one early draw. **Validated at n=5** (flake rate was unusually high
this session — several runs auto-skipped below the 95% ok-rate gate along
the way): z-phase raw correlation mean 0.4->0.69, std 0.24->0.067 (a real
level gain AND ~3.5x tighter spread) on a 4-run quick check; full n=5
re-derive gave **R^2 Hx=0.69 Hy=0.76 Hz=0.48 Wz=0.50** vs the deployed
peripheral-bias cal's Hx=0.73 Hy=0.79 Hz=0.22 Wz=0.53 — **Hz more than
DOUBLED**, everything else within normal noise (0.03-0.06 shifts vs this
session's typical 0.1-0.2+ swings). No detection-quality cost (100%
ok-rate on the clean runs). **DEPLOYED** into
`CrossMarkerPerception.__init__`, superseding the 2026-08-06
peripheral-bias-only cal. This closes the "unexplained run-to-run
variance" thread that had blocked the color-gate and camera-Z bisections —
those two remain formally inconclusive (not confirmed OR ruled out), but
are now much lower priority since the dominant variance source is fixed.
Wx/Wy/Wz's own structural collinearity ceiling (a different mechanism,
`_fill_A`'s quadratic/near-degenerate columns) is UNTOUCHED by this fix —
still needs a bigger marker plate if pursued further. See
[[feedback_cross_marker_radial_spread_ceiling]] for the full numeric trace.

**DESIGN DIRECTION UPDATE (2026-08-07): ring/texture-flow approach being
RETIRED, replaced by a 4-corner-ArUco marker layout.** User: the textured
background + dense-GFT-point approach was built specifically to counter
point starvation on the sparse cross+stub shape (the "ring approach" this
whole thread's mechanism analyses assumed) — no longer needed once the
textured area is replaced by 4 corner ArUco tags. NOT YET IMPLEMENTED in
code as of this writing — purely a stated design direction. Implication
for everything above: the `_fill_A` image-Jacobian math is marker-agnostic
(generic 6-DOF flow solve over tracked points), so the Wz/Wx/Wy
collinearity mechanism will likely still apply to WHATEVER point layout
the new marker uses, but should be RE-DERIVED/RE-VERIFIED once built, not
assumed to transfer. The corner-ArUco layout may also enable falling back
on ArUco's own decode-based orientation (removing the stub-asymmetry
dependency `alpha`'s disambiguation currently has, see below) if corner
IDs are used.

**Confirmed s/h/alpha computation methodology (2026-08-07, user-requested
review before the next validation flight) — no code changes, verification
only:** `s` = line-intersection of the two cross-arm lines
(`cross_marker_detector.py::_line_intersection`), degrades gracefully
under partial occlusion. `h` = `cv2.goodFeaturesToTrack` restricted to
`det.isolated_mask` (the SAME-FRAME color-gated+shape-filtered detector
mask, not the whole image) plus the boundary-margin/center-disk
exclusions from the peripheral-bias fix — structurally impossible for a
candidate to land outside the mask; residual risk is mask QUALITY
(anti-aliasing edge noise, or real legs passing the color gate before
shape-filter rejection), not missing ROI restriction. `alpha` = plain
unweighted 2nd-moment (`_unweighted_principal_angle`) over real
arm+stub pixels (no ArUco `[4,3,2,1]` weighting hack), stub resolves the
pi-ambiguity, holds last-good if stub not detected that frame. See
[[feedback_cross_marker_radial_spread_ceiling]] and the io-calibration
skill's new "s/h/alpha computation reference" section for the same
writeup with code line references.

**NEXT STEP (agreed, separate chat): independent multisine validation**
of the currently-deployed cal (`apps/record_cross_marker_validation.py` +
`tools/validate_cross_marker_flow.py`, built 2026-08-02, never actually
run) — the io-calibration skill's train/validate discipline has never
been closed for this pipeline at any point in this whole thread, no
matter how good training-data R^2 has looked. **Also confirmed: NOT ready
for a real closed-loop landing flight test** — `CrossMarkerNode` still
doesn't implement terminal-kick/ring-loom-fusion (own class docstring:
will AttributeError on a real descent), unchanged since 2026-08-01.
