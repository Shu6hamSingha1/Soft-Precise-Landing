---
name: project_cross_marker_pipeline_20260801
description: "Cross+stub fiducial pipeline built 2026-08-01 as a decode-free ArUco alternative — status, architecture, and what's next"
metadata: 
  node_type: memory
  type: project
  originSessionId: dbc47060-f292-4fca-8a3d-75a6066d2b5c
  modified: 2026-08-01T15:08:59.355Z
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
