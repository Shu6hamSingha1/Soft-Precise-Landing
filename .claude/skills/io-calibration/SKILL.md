---
name: io-calibration
description: Calibrate and validate the INPUT (FC command→achieved body-rate/thrust transfer) and OUTPUT (image→[h;w] optical-flow sensor) signals for the PX4/Gazebo PLASMC pipeline. Use when the user asks to calibrate or recalibrate the output/corner/ring sensor cal, the input/thrust/rate cal, derive a calibration matrix, validate a cal, set up validation data, or understand the calibration pipeline, folder layout, or routing. Encodes the two-stage train/test process (phased calibration ≥5 runs → independent multisine + landing validation), the derive/aggregate/validate tools, the data layout, env routing, and the load-bearing facts and gotchas.
---

# Input / Output calibration & validation

The PLASMC pipeline has two signal chains that each need calibration, and the
**same two-stage discipline** applies to both:

1. **CALIBRATION** — *phased* maneuver, **≥5 runs**, decorrelated axes → derive the cal.
2. **VALIDATION** — a **different** maneuver (multisine / multi-axis) + a landing-to-touchdown,
   **1 run each**, recorded to a **separate folder** so it never contaminates the calibration set.

Calibration data and validation data must stay separated: the derive tools default-scan
`calibration_data/`, so a validation run placed there silently corrupts the cal.

## When to invoke
- "Calibrate / recalibrate the output (corner / ring) sensor cal" or "the input (rate/thrust) cal"
- "Derive the calibration matrix", "validate the cal", "why is the cal off"
- "Set up / collect validation data", "where does calibration data live"
- Any mention of `_sensor_cal_hw`, `_sensor_cal_ring`, `_sensor_cal_s`, `M_ring`, `derive_board_cal`,
  `derive_ring_cal`, `aggregate_input_calibration`, gain/lag, thrust slope, `CALIB_MODE`, `CALIB_PARENT`.

## The hard rule: scale-free / depth-free
The control law uses **image-based quantities only** — no Z/altitude/metric data ever enters the
cal that the controller consumes. Truth (Gazebo GT) is allowed ONLY for deriving/validating the
cal, never in the runtime control path. See memory `feedback_scale_free_depth_free`.

## ⚠ 2026-07-02 addendum — currency notes (read before any recal)
- **The cal is WORLD/MARKER-SPECIFIC.** The single-LARGE-marker world (2026-06-23) required an
  in-world re-cal; the live `_sensor_cal_hw` is keyed to the CURRENT world+marker (the prior board
  cal is backed up as `.pre_singlemarker_cal_bak`). Changing marker/world (e.g. the rover's
  platform-mounted 1 m marker) ⇒ re-derive before perception-ON runs. See
  `feedback_single_marker_rank_deficiency`.
- **`FLOW_LAT_REDUCED=1` (baked 2026-06-25) requires a PAIRED recal:** the cal `h_x/h_y` rows are a
  `w_xy` recombination that breaks under the reduced solve (~3× under-read) → record any output-cal
  WITH `FLOW_LAT_REDUCED=1` so the derive yields diagonal rows. See `feedback_lateral_flow_reduced_solve`.
- **Yaw feature cal is RESOLVED, not pending** (2026-07-02): `cal_s[3]=1.0` is CORRECT (alpha tracks
  GT yaw r=1.00); the moving-rover "yaw cal" task dissolved — the only turning-target gap is the
  controller alpha-rate cap (`PLASMC_YAW_ALPHA_MAX_RATE`), which is control, not calibration.
  Supersedes `project_yaw_calibration_pending`. See `feedback_rover_yaw_cal_resolved`.
- **GT-FB bypasses the sensor cal entirely** (`PLASMC_GT_FEEDBACK=1` injects exact V-frame s/h) —
  cal changes are INERT under GT-FB; only perception-ON runs exercise the cal.
- `test_data/Landing_Test/` now holds ~1,500 recordings (the "534" figure below is a stale count).

---

# OUTPUT calibration (image → optical flow `[h; w]`)

**Maneuver:** `apps/record_output_calibration.py`, default **phased** (each axis x→y→z→yaw driven ALONE,
settles between → axes decorrelated). `CALIB_MODE=multisine` = freq-multiplexed (validation only).

**Derive (corner):** `tools/derive_board_cal.py` — full **6×6** `M` s.t. `GT[h;w] = M @ raw`
(corner lstsq is full-rank with fixed geometric h↔w coupling, so M is NOT diagonal). The applied
corner cal **zeros the Wx/Wy rows as a LEVEL-TARGET modeling choice** (`CTRL_ZERO_WXY=1`): the GT
w-axis is set yaw-only `V_w_ug[:,:2]=0`, so those rows come out 0. **NOTE (corrected 2026-06-07):**
roll/pitch rate is NOT geometrically unobservable from image flow — the 2026-06-04 "unobservable"
verdict was OVERTURNED (an under-excitation + V-leveled-frame artifact). wx/wy ARE recoverable with
the multi-marker board's corner SPREAD + adequate excitation; zeroing them is a choice for a level
target, not a necessity. See memory `wxy-unobservable-imu-fusion-deferred`. h-block ≈ identity.

**Derive (ring, texture-free):** `tools/derive_ring_cal.py` — separate **6×6** `M_ring`.
- `RING_CAL_MODE=transfer` (DEFAULT): calibrate the ring against the **already-calibrated CORNER**
  as a transfer standard (`M_ring@ring ≈ M_corner@corner`), both raws SAME-CLOCK in `Img_Data`.
  Avoids the Img/GT cross-clock-alignment smear of `gt` mode, and makes `ring_cal≈corner_cal` so the
  marker→ring handoff is continuous. **Keyed to the current corner M — re-run after changing
  `_sensor_cal_hw`.** Inherits the yaw-only-w (Wx/Wy=0) from the corner standard.
- `RING_CAL_MODE=gt`: GT-direct (the "true" cal). Co-sampled ring (clean) if present, else
  retro-aligns from Img_Data (cross-clock; noisier). `RING_CAL_COSAMPLED_ONLY=auto|1|0`.
- Ring is **NOT depth-mixed** (board coplanar + V-frame leveling → uniform perpendicular depth;
  ring h_z ~ corner h_z r≈0.95). The loom (`Ring Divergence`) is preferred for **robustness**
  (texture-free median, survives marker death), NOT depth-invariance. See memory
  `ring-depth-mixing-falsified`, `ring-flow-calibration`.

**Applied (in `src/img_data.py`):** `_sensor_cal_hw` (corner 6×6), `_sensor_cal_s` (centroid 4-diag),
`_sensor_cal_ring` (ring 6×6). Runtime: `getOptFlowAngVel()` = `_sensor_cal_hw @ corner-KF`;
`getRingFlowAngVel()` = `_sensor_cal_ring @ ring-KF` (ring is a SAFETY NET — control consumes the
corner flow). Raw getters `getRawOptFlowAngVel` / `getRawRingFlowAngVel` (pre-cal) are co-sampled
into the GT dict by `record_output_calibration.py` so the derive tools can fit.

**Validate:** `notebooks/plotter_output_validation.ipynb` — GT-direct, auto-loads the live cal,
checks calibrated corner + ring flow vs GT on a multisine run (per-channel R²) and runs the
landing-to-touchdown check via `tools/validate_output_flow.py::validate(dir)`.

# INPUT calibration (FC command → achieved body-rate / thrust)

**Key difference:** input cal is a **dynamic transfer (gain + LAG)**, not a static matrix. The
headline metric is **lag**, not amplitude — gain≈1 with 287 ms yaw lag is still a failing cal.

**Maneuver:** `apps/record_input_calibration.py` — sends `send_attitude_rate` (body ω + thrust) via a
`cmd_profile` array; records commanded (`gt['Command']`) vs achieved (`tel['Angular Velocity FRD']`).
Thrust mapping: `thrust_norm = 0.738 - B_T/42.3` (B_T in Newtons; see `feedback_input_cal_thrust_units`).

**Aggregate:** `tools/aggregate_input_calibration.py::per_run_metrics(dir, return_series=True)` —
per-axis Pearson r, cross-correlation **lag (ms)**, and **gain**; MAD-trimmed mean across runs.
**Yaw lag is ~5× roll/pitch** (ωz ~280 ms r≈0.76; ωx/ωy ~55–65 ms r≈0.95) — PX4 yaw rate-loop is
slower + EKF heading drift; per-axis matters (`feedback_input_cal_yaw_lag`).

**Actionability is limited:** `MC_*RATE_P` runtime tuning is DEAD (breaks SITL preflight —
`feedback_mc_rate_p_dead`). So input cal produces (a) the thrust slope and (b) a *characterization*
of lag for a feedforward/predictor — it is system-ID, not a corrective gain.

**Validate:** `notebooks/plotter_input_validation.ipynb` — multi-axis run + landing; headlines
per-axis lag, reusing `per_run_metrics`.

---

# Data layout & routing

```
calibration_data/output/   phased runs (derive_board_cal/derive_ring_cal SCAN here)  — gitignored
calibration_data/input/    input-cal runs (aggregate_input_calibration scans here)   — gitignored
validation_data/output_multisine/    output multisine validation                            — gitignored
validation_data/input_multiaxis/  input multi-axis validation                            — gitignored
validation_data/output_landing/   output landing (position-SP descent, flow vs GT)       — gitignored
validation_data/input_landing/    input landing (thrust staircase -> thrust map)          — gitignored
```

**Routing env knobs** (keep validation OUT of `calibration_data/`):
| Knob | Read by | Use |
|---|---|---|
| `CALIB_PARENT` | `run_output_calibration.sh`, `run_input_calibration.sh` | parent dir override (auto-timestamped subdir) |
| `CALIB_OUT_BASE` / `INPUT_CALIB_OUT_BASE` | output/input cal apps | parent dir (app-level) |
| `CALIB_OUT_DIR` / `INPUT_CALIB_OUT_DIR` | apps + launchers | exact path (precedence) |
| `LANDING_OUT_BASE` | `apps/landing_test.py` | route a validation landing |
| `CALIB_MODE` | `record_output_calibration.py` | `phased` (default) \| `multisine` |
| `RING_CAL_MODE` | `derive_ring_cal.py` | `transfer` (default) \| `gt` |

**Collect (Gazebo — the USER runs these; never invoke SITL yourself):**
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
# CALIBRATION (phased, loop until ≥5 valid; cleanup empty dirs — see CLAUDE.md recal workflow)
for i in $(seq 1 10); do timeout 220 bash scripts/run_output_calibration.sh; done
~/ws/scripts/env2025/bin/python3 tools/derive_board_cal.py     # corner M  -> paste into img_data.py
~/ws/scripts/env2025/bin/python3 tools/derive_ring_cal.py      # ring M_ring (transfer) -> paste
# VALIDATION (dedicated apps via CALIB_APP/INPUT_APP override; VALIDATION_PROFILE=multisine|landing)
CALIB_APP=apps/record_output_validation.py VALIDATION_PROFILE=multisine CALIB_PARENT=$PWD/validation_data/output_multisine bash scripts/run_output_calibration.sh
INPUT_APP=apps/record_input_validation.py  VALIDATION_PROFILE=multisine CALIB_PARENT=$PWD/validation_data/input_multiaxis bash scripts/run_input_calibration.sh
# LANDINGS use POSITION-SETPOINT / thrust-staircase descents (NOT the untuned PLASMC
# controller — it has a descent regressor that hovers; run_aruco_landing is for tuning, not validation):
CALIB_APP=apps/record_output_validation.py VALIDATION_PROFILE=landing CALIB_PARENT=$PWD/validation_data/output_landing bash scripts/run_output_calibration.sh   # output: position-SP descent, flow vs GT. CALIB_PARENT routes it OUT of calibration_data/output.
INPUT_APP=apps/record_input_validation.py VALIDATION_PROFILE=landing CALIB_PARENT=$PWD/validation_data/input_landing bash scripts/run_input_calibration.sh   # input: thrust staircase (0.74↓0.01), GT-stop 1m. CALIB_PARENT routes it OUT of calibration_data/input (else it contaminates the rate-cal aggregate).
```
The validation apps (`apps/record_output_validation.py`, `apps/record_input_validation.py`) each fly one of two
`cmd_profiles` (`VALIDATION_PROFILE=multisine|landing`) and route to `validation_data/`. The **landing**
profiles deliberately AVOID the PLASMC controller: output = `send_position_ned` descent; input = a
thrust_norm staircase. (`run_aruco_landing.sh` is the controller — for *tuning*, not validation.)

**Validation maneuver tuning (from the 2026-06-06 SITL shakedown — see `validation-runs-status`):**
the apps run end-to-end, but the DEFAULT amplitudes are too aggressive:
- **output multisine** drops the marker (KLT-fallback spam → align r drops, vertical/yaw R² corrupts).
  Reduce `VAL_MS_AMP_XY` (0.30→~0.15) + `VAL_MS_AMP_YAW_DEG` (15→~8) to keep it framed.
- **input multisine is OPEN-LOOP** (no attitude feedback) → `VAL_RATE_AMP=0.15` rad/s flips the
  drone (impact ~20 s). Drop to `VAL_RATE_AMP≈0.03–0.05`. wx/wy still validated well (r=0.98); wz poor.
These were runnable from an agent session with the Bash sandbox disabled, but per CLAUDE.md the
**user normally drives Gazebo** — prefer handing them the commands.

# Tools inventory
- `tools/derive_board_cal.py` — corner 6×6 M (GT=M@raw)
- `tools/derive_ring_cal.py` — ring M_ring (transfer|gt)
- `tools/aggregate_calibration_phased.py` — diagonal corner cal + GT-signal helpers (compute_gt_signals)
- `tools/aggregate_input_calibration.py` — input per-axis gain/lag (per_run_metrics)
- `tools/validate_output_flow.py` — THE output-flow validation/QA module (single source). Views:
  `prep()` (shared load+cal+align core), `channel_r2(dir)` (GT-direct per-channel — multisine view),
  `validate(dir)` (GT-direct altitude-binned to touchdown), `cross_validate_ring(glob)` (GT-free
  leave-one-run-out ring-cal check — folded in from the former `compare_ring_corner_cal.py`).
  Renamed from `validate_flow_to_touchdown.py`. The validation notebook calls these (no duplication).
- Recording (flight) apps: `apps/record_output_validation.py` / `apps/record_input_validation.py`
  (fly the validation maneuver, save to `validation_data/`) — distinct from the validation TOOL above.
- Notebooks: `plotter_{output,input}_calibration.ipynb` (derive/inspect) + `plotter_{output,input}_validation.ipynb` (validate)

# Load-bearing facts & gotchas
- **Single source of truth:** the validation notebooks + `validate_output_flow.py` **auto-load**
  the cal from `src/img_data.py` (regex-parse the np.array literals). Don't hardcode cal values in
  scripts — edit `img_data.py` and everything tracks it. The corner derivation/inspection notebook
  shows the live matrix on run.
- **Validation is GT-direct only.** The corner-as-transfer-standard trick is a *derivation*
  expedient (ring cal), never a validation method.
- **Signal-floor limits precision:** flow_z / ω_z have ±30% / ±9% CI at default amplitudes; raise
  `CALIB_AMP_Z` / `CALIB_AMP_YAW_DEG` (caveat: over-driving clips the other axes). See
  `reference_aggregate_calibration`.
- **Frame:** the runtime solves flow in the **gravity-leveled VIRTUAL frame** (`_getVirtualPts`);
  validation GT must be V-frame (`feedback_outputcal_flow_validation_vframe`,
  `feedback_vframe_rhs_yaw_only`). camera = body-FRD (no rotation); V = level(camera).
- **1 run validates the SIGNAL** (R²/lag). Landing **performance** (xy/softness) needs n≥5 —
  a clean 1-run cal-validation says nothing about landing quality.
- **Pre-recorded validation data:** the **landing** validation is covered by `test_data/Landing_Test/`
  (534 independent recordings; pick one with non-empty `Ring Opt Flow Ang Vel` for the ring part).
  A clean **multisine** validation has NO pre-recorded option — the only multisine recordings
  (`Obsolete/.../output_pre_fpsfix`) carry the pre-fps-fix leveling bug → false-negative R²; record
  a fresh one. Input multi-axis can interim-validate on a `calibration_data/input/` run (caveat: not
  held out). `validation_data/` is pre-populated with symlinks to the best candidates.

# Known dead-ends (don't retry)
- Refreshing cal via `aggregate_calibration.py` → 7–10× wrong (methodology mismatch;
  `feedback_calibration_lessons`).
- `MC_*RATE_P` via MAVSDK runtime → SITL preflight failure.
- Treating the pre-fps-fix multisine as a validation set → leveling bug, misleading.
- Re-deriving the "ring is depth-mixed" rationale → falsified (`ring-depth-mixing-falsified`).

# When a NEW perception module's cal validates badly, check the raw signal before
# blaming the fit methodology (2026-08-02/03, cross-marker case study)
A derived cal can validate at near-zero/negative R² on independent data for reasons that
have nothing to do with n≥5, purity gating, or fit robustness — if the module computing
the raw signal is itself new (not `img_data.py`), suspect a missing or broken transform
stage before re-tuning the derive tool. Two real bugs found this way in
`cross_marker_perception.py` (a from-scratch module that "mirrors" `img_data.py`'s math
without importing it):
1. **A load-bearing transform step silently dropped in the port.** `img_data.py` always
   reprojects points through `_getVirtualPts` (gravity-leveled V-frame) before computing
   flow; the new module never got that step, so it was comparing raw tilt-contaminated
   flow against tilt-compensated GT. Symptom-free at build time because validation was
   hover-only (near-zero tilt). Fix: diff the new module's per-stage structure against
   the original it claims to mirror, don't just spot-check the final formula.
   See `feedback_missing_vframe_leveling_port` / `feedback_duplicated_math_diff_check`.
2. **dt computed from the wrong clock relative to the state it's dividing.** A per-frame
   dt built from "time since the last function call" is only valid if the tracked state
   also updates every call — if it only updates on success (e.g. gated behind a detector),
   any gap leaves dt reflecting one interval while the tracked displacement spans many.
   Symptom: a fix that should obviously help (like #1) shows no effect, or an
   inconsistent one, because this noise floor dominates. See
   `feedback_dt_staleness_after_detection_dropout`.
Both were confirmed by comparing raw-vs-GT correlation directly (not just the fitted R²)
on independent flights, phase-isolated and restricted to detection-ok samples — a cal
that validates badly but a RAW correlation check that's ALSO near-zero on clean data
points at the raw signal, not the fit.

# Cross-marker output cal: its OWN derive tool, separate data dir
The cross+stub fiducial (`src/cross_marker_perception.py`) is NOT routed through
`img_data.py`/`IMG_PROCESSOR` at all — its own image-Jacobian solve (`_fill_A`, same
6-DOF math as the corner cal but independently implemented) needs its own cal, derived
by `tools/derive_cross_marker_cal.py` (recordings: `apps/record_cross_marker_calibration.py`
→ `calibration_data/output_cross/`, NOT `calibration_data/output/`). Same phased-excitation
/ ≥5-run / MIN_OK_RATE=95%-gate discipline as the ArUco board cal, but it's a genuinely
separate cal pipeline — don't assume the ArUco cal transfers.

## Hz/Wz weakness root-cause chain (2026-08-05→07 — read before re-touching this cal)
**Current deployed state (2026-08-07): R² Hx=0.69 Hy=0.76 Hz=0.48 Wz=0.50** (Wx/Wy
correctly forced 0). Investigated across many stages, each RULING OUT the previous
suspect rather than confirming it — the two weak rows (Hz, Wz) turned out to be **two
different phenomena, not one "hardest to observe" pattern** — don't conflate them:

**Wz is a REAL structural ceiling, unresolved, marker-size-limited.** Its raw columns
(`h1`=Ty, `w0`=Wx in `_fill_A`) are near-perfectly collinear (r=0.98-1.00 in every
excitation phase — two near-zero eigenvalues in the raw 6-column correlation matrix)
because `_fill_A`'s Wx column `-(1+y²)` stays ~constant whenever `|y|` is small, at
this marker's achievable radial spread — numerically aliasing Ty's constant column.
Same underlying cause as Wx/Wy being force-zeroed (genuinely unrecoverable, confirmed
via dedicated rollexc/pitchexc excitation, r=-0.09..-0.15). **The only remaining lever
for Wz/Wx/Wy is a bigger physical marker plate** — not fixable by recal, point-sampling,
or excitation tuning. Not pursued further as of 2026-08-07.

**Hz's weakness was NOT the same phenomenon — it was 3 stacked, fixable bugs, 2 now
fixed, 1 still open:**
1. **NOT data contamination** (ruled out 2026-08-06) — an ad-hoc diagnostic script's
   own alignment bug, not a real z-phase GT contamination. See
   `feedback_cross_marker_radial_spread_ceiling`.
2. **Radial spread — partial fix, doesn't touch Hz.** `_sample_flow_points`
   (`cross_marker_perception.py`) excludes a central disk
   (`CROSS_FLOW_CENTER_EXCLUDE_FRAC=0.35`) + a frame-boundary margin
   (`CROSS_FLOW_BOUNDARY_MARGIN_PX=20`) to bias GFT candidates toward the arm/stub
   tips. DEPLOYED (2026-08-06): fixed Hx/Hy (0.55→0.73, 0.63→0.79, beating even the
   pre-camera-change baseline) but left Hz flat (0.22→0.22) — proved radial spread
   isn't Hz's constraint.
3. **A genuine raw-signal regression, 08-04/05 camera/geometry cluster — STILL NOT
   ROOT-CAUSED.** Re-checked and found the earlier "raw Hz-vs-GT r=0.93-0.96" claim
   was stale: current-era data showed only r=0.07-0.67 (mean~0.5), vs 0.90-0.96 on
   archived pre-08-05 recordings (`calibration_data/output_cross_stale_pre20260805/`).
   Bisected two suspects at n=2 each (color-gate threshold `V<100→V<20/60` via new
   `CROSS_COLOR_GATE_V_MAX` env; camera Z-offset `.15→.20`, isolated via
   `x500_mono_cam_down/model.sdf.bak_before_campos_20260805_010737`) — BOTH
   inconclusive, swamped by huge run-to-run variance. Neither adopted; camera/gate
   left at live defaults. **Note: camera Z=.15 is a DELIBERATE tradeoff (keeps
   landing legs out of the downward FoV), not a revertible regression** — don't
   suggest reverting it without addressing leg visibility first (see below).
4. **The variance itself was the real story — found and FIXED (2026-08-07), Hz
   R² doubled.** `_sample_flow_points` (GFT) was only called on cold-start or when
   the tracked pool dropped below `RESAMPLE_TRIGGER=10` — between those events the
   SAME point set just tracked forward via LK indefinitely. Confirmed: 4/5 pre-fix
   calibration runs had a perfectly CONSTANT `n_kept` for their ENTIRE ~500-frame
   z-phase (zero resamples) — whichever corner subset got picked once, near takeoff,
   persisted the whole flight. Because Hz/Wz's `_fill_A` columns are
   position-WEIGHTED (`[-x,-y]`/`[-y,x]`) while Hx/Hy's are position-INDEPENDENT
   constants, that one-time draw's exact locations disproportionately set the WHOLE
   flight's Hz/Wz quality while Hx/Hy stayed robust — explaining the large,
   previously-unexplained run-to-run spread that made suspect-3's bisection
   inconclusive. **Fix: `RESAMPLE_PERIOD_S=1.0`** (`CROSS_RESAMPLE_PERIOD_S` env)
   forces periodic refresh independent of pool size. Validated n=5: R² Hx=0.69
   Hy=0.76 **Hz=0.48** Wz=0.50 vs the prior Hx=0.73 Hy=0.79 Hz=0.22 Wz=0.53 — Hz MORE
   THAN DOUBLED, everything else within normal noise. DEPLOYED.
5. **Suspect 3 (the raw-signal regression) is STILL OPEN, but CONFIRMED REAL
   (2026-08-10) and 2 of ~4 candidate causes are now RULED OUT with real n≥3 data**
   (the original 08-05 bisection was only n=2 per side, too weak — re-ran properly).
   Re-measured cleanly: z-excitation-phase raw Tz-vs-GT correlation is a TIGHT
   0.90-0.96 on pre-08-05 archived data (`calibration_data/output_cross_stale_pre20260805/`,
   n=5) vs a TIGHT 0.78-0.82 on current data (n=5), same measurement method both
   sides (apples-to-apples) — a real, consistent ~0.15 gap, not a fluke (unlike the
   sign-flip above). Bisected:
   - **Color-gate threshold (100→20): RULED OUT** — reverting it under the CURRENT
     thinner marker line doesn't cause a subtle correlation drop, it breaks
     detection almost completely (0% ok-rate, `color_gate_empty` on ~93% of
     frames). Not a live candidate; the two changes (line-width halving,
     gate-threshold widening) are coupled and can't be independently reverted.
   - **Camera Z offset (.15→.20 revert): RULED OUT** — 3 flights with the camera
     temporarily reverted to the pre-08-05 Z=.20 position (backed up as
     `x500_mono_cam_down/model.sdf.bak_current_z15_20260810`, restored after)
     gave z-phase correlation 0.80/0.87/0.80 — statistically indistinguishable
     from the current Z=.15 baseline (0.78-0.82). No effect.
   - **Tracking-based ROI addition: RULED OUT (2026-08-10)** — 3 flights with it
     disabled (temp toggle `CROSS_DISABLE_TRACK_ROI=1` in
     `cross_marker_perception.py`'s `process_frame`, forces `track_state=None`
     so `cross_marker_detector.detect()` always takes the pre-08-05 full-frame
     path) gave z-phase correlation 0.82/0.84/0.84 — again indistinguishable
     from baseline. **Remove the toggle once this thread is fully closed out**
     (currently still in the file, harmless no-op unless the env var is set).
   - **Line-width halving (independent of the gate threshold — re-tested
     2026-08-10 after realizing the two changes are NOT actually coupled):
     INCONCLUSIVE, not adopted.** Earlier note wrongly assumed line-width and
     gate-threshold couldn't be independently tested; they can — generated a
     full-width mask at the CURRENT hi-res texture's resolution (extract the
     pre-08-05-backup's mask, upscale 3x nearest-neighbor, same method as
     `make_cross_marker_hirestex.py`, fresh speckle background), swapped it in
     via `model.sdf`'s `albedo_map` temporarily (no live-file edits). Result:
     z-phase correlation across 3 flights was **-0.03, +0.60, +0.94** — much
     HIGHER variance than the tight half-width baseline (0.78-0.82), with
     `n_kept` (tracked point count) also swinging 13-33 median across runs vs.
     a narrower baseline range. Mean isn't clearly better and variance is
     worse — full-width line isn't a clean fix; if anything it may destabilize
     which corner subset the periodic-resample (`RESAMPLE_PERIOD_S`, the
     08-07 fix) settles on. NOT adopted; don't re-derive "revert the line
     width" as a fix from this result without addressing the variance first.
   - **Still untested, and NOT a natural next step:** mount-yaw+90° rotation.
     Unlike the other candidates, this isn't a clean isolated toggle — it's
     a coordinate-frame convention several other fixes were built on top of
     (see its own module comment), so reverting it risks reintroducing bugs
     those fixes solved rather than isolating this one. (The `_getVirtualPts`
     axis-sign-flip is similarly a correctness fix for a real bug, not a live
     candidate.)
   **Also tried (2026-08-10): raising `CROSS_FLOW_CENTER_EXCLUDE_FRAC`
   0.35→0.55 (radial-leverage idea — Hz's per-point signal is linear in
   radial distance, `_fill_A`'s Tz column is `[-x,-y]`, so point QUALITY
   should matter more than count). A narrow z-phase-only check looked
   promising (mean 0.804→0.835, n=5) but the WHOLE-FLIGHT joint fit showed
   a net regression across nearly every channel (Hz R² 0.48→0.07, Hx
   0.69→0.48, Hy 0.76→0.65, consistent across all 5 runs) — the more
   aggressive exclusion starves the point pool (n_kept median 10-17→8-10)
   enough to add noise everywhere, outweighing the narrow z-phase gain.
   REVERTED; default stays 0.35. Lesson: always re-derive the whole-flight
   fit before adopting a point-sampling change, a narrow single-phase
   diagnostic can't see costs elsewhere.

   **Also tested (2026-08-10), separate from the 08-04/05 cluster: the
   08-09 hi-res texture swap. RULED OUT, and actually a net POSITIVE.**
   Reverting to pre-hires (current line-width/gate held fixed) for 3
   flights gave z-phase correlation +0.52/+0.57/+0.20 — WORSE than the
   current hires baseline (0.78-0.82), not better. Point-set diagnostics:
   pre-hires had MORE tracked points (n_kept median 24-55) but LOWER radial
   spread (mean 0.09-0.13); current-hires has FEWER points (median 10-17)
   but a higher max spread (0.19-0.26) — a few high-leverage far corners
   from the finer speckle matter more for Tz's position-weighted column
   than raw point count. Don't revert the hi-res texture for this
   regression — it's a red herring in the opposite direction, and an
   implementation audit (same date) also confirmed the Jacobian formula
   and V-frame leveling are correct and identical across all 4 platforms
   (MATLAB, ArUco PX4, cross-marker PX4, hardware) — this is not a code bug.
   **Status as of 2026-08-10: 4 of 5 candidates from the 08-04/05 cluster
   tested, none give a clean fix** (3 ruled out with no effect; line-width
   gives a noisier, not-clearly-better result). The remaining ~0.15 gap
   (0.9→0.8 mean, still real per the apples-to-apples check above) may not
   trace to any single one of these changes — could be a combined/nonlinear
   effect of several small changes together, or something outside this
   candidate list entirely. Given the already-banked win (Hz whole-flight R²
   0.22→0.48 from the resample-variance fix) and the risk/cost of testing the
   one remaining candidate, this is a reasonable point to STOP the bisection
   unless a cheap new idea surfaces — don't keep spending flight batches on
   this without one.
   **⭐ 2026-08-09/10 UPDATE: the "sign flip below 2m" theory was investigated and
   RESOLVED FALSE (not just superseded — actively disproven).** An initial 5-flight
   batch (all launched back-to-back) appeared to show raw pre-cal Tz's correlation
   with GT vertical velocity flipping sign around 2m altitude. A follow-up 3-flight
   batch showed the OPPOSITE sign in the same band, and pooling all 12 available
   landing flights (demeaned per-flight) gives the TRUE population correlation as
   **+0.23 in the 1-2m band — positive, same sign as everywhere else. No inversion
   exists.** Mechanism: the landing-validation maneuver is a near-constant-vz linear
   descent, so within any narrow altitude slice the true signal has almost no
   variance (`std(vz_g)≈0.04-0.05`, barely above the raw measurement's own noise
   floor) — a Pearson correlation there is noise-dominated and its sign is close to
   a coin flip per flight. The original 5/5-negative batch was an unlucky ~3%-chance
   cluster, not a real effect. **Don't re-derive a sign-flip mechanism from a single
   batch on this channel — pool ≥3 independent sessions before trusting its sign.**
   What's still genuinely open, reframed correctly: Hz's raw correlation is modest
   (mean ~0.5, same sign throughout) and R² can still read catastrophically negative
   near touchdown as an artifact of comparing against a near-zero-variance true
   signal there, not because the prediction is pointing the wrong way — check
   bias/noise floor next, not sign. Full trace:
   `PX4_Gazebo/docs/HANDOVER_cross_marker_hz_signflip_20260809.md` (read bottom-up —
   the top sections describe the now-disproven theory) and
   `project_cross_marker_pipeline_20260801` memory.
6. **Hardware (ArUco) shows the OPPOSITE profile** (checked for comparison): `Hz` is
   the *strongest* row on hardware (R²=0.64) while `Hx/Hy` are weak (0.07-0.08,
   real-camera noise). This had already disproven "Hz/Wz are universally the
   hardest-to-observe rows" as a cross-platform law before the 08-07 fix above
   further sharpened the point: don't cite that framing at all going forward — Wz
   is genuinely hard to observe; Hz's past weakness was mostly bugs, not geometry.

## s/h/alpha computation reference (cross-marker pipeline, confirmed 2026-08-07)
- **`s` (centroid):** `cross_marker_detector.py::detect()` fits the two cross-arm
  lines from real detected pixels, `center = _line_intersection(line_i, line_j)`.
  Degrades gracefully under partial occlusion (only needs 2 non-parallel lines
  fittable from whatever real pixels are visible).
- **`h` (optical flow):** `cv2.goodFeaturesToTrack` is called with `mask =
  det.isolated_mask` — the SAME-FRAME color-gated + shape-filtered detector mask,
  not the whole image. Structurally impossible for GFT to place a candidate outside
  that mask; a boundary-margin exclusion (`FLOW_BOUNDARY_MARGIN_PX`) and center-disk
  exclusion (`CROSS_FLOW_CENTER_EXCLUDE_FRAC`) further restrict candidates within it.
  The residual risk is mask-QUALITY (color-gate anti-aliasing edge noise, or the
  drone's own real legs passing the color gate before shape-filter rejection — see
  the "ghost" correction below), not missing ROI restriction.
- **`alpha` (orientation):** `_unweighted_principal_angle()` over
  `line_points_i + line_points_j + stub_points` (real arm+stub pixels, camera-mount-
  yaw-corrected) — a PLAIN unweighted 2nd-moment (no ArUco `[4,3,2,1]` synthetic
  weighting hack, unneeded since the stub is a real geometric asymmetry). The stub
  also resolves the π-ambiguity (`_disambiguate_angle` via stub-centroid-minus-arm-
  centroid vector). Holds last-good alpha if the stub isn't detected that frame.

## "Ghost" correction (2026-08-07) + no SDF-only leg-widening lever
The 2026-08-02 "pre-existing Gazebo camera-render ghost... mirrored duplicate of the
drone's own body" finding (below/in project memory) was a MISDIAGNOSIS — confirmed via
raw disarmed-frame inspection (`apps/diag_raw_image_dump.py`) that the top/bottom-margin
artifact is the drone's REAL landing legs (matches the SDF's own leg collision geometry
almost exactly), not a render duplicate. The existing ghost-defense code
(`_restrict_to_center_roi`, `_reject_blobby_components`) is unaffected — built against
the real artifact, only the naming was wrong. Separately: no SDF-only lever exists to
widen the leg gap for a larger camera Z — `x500_base/meshes/NXP-HGD-CF.dae` is ONE
combined mesh for the whole airframe (no separate leg link/pose); only a full mesh edit
(or a full physically-consistent scale-up: mass+inertia+rotor-arm+thrust together, real
flight-dynamics risk) would do it. User decision: only pursue that if this whole thread
hits a dead end specifically needing more camera height.

## Design direction: cross+stub is the settled marker (CORRECTED 2026-08-08)
No 4-corner-ArUco redesign is planned. An earlier version of this note claimed the
opposite (user moving from the textured cross+stub to a 4-corner-ArUco layout) --
that was a misreading of a typo'd 2026-08-07 user message; the corrected, confirmed
meaning is the reverse: the CURRENT textured cross+stub design is what superseded an
earlier, since-abandoned 4-corner-ArUco concept. Do not plan around, cite, or
re-verify against a 4-corner-ArUco migration for this marker -- none is coming. The
ring/texture-flow approach (built to counter point starvation on the sparse
cross+stub shape) is still not needed per that same 2026-08-07 message, independent
of the corrected/retracted redesign claim.

# Recalibration procedure
1. Identify which chain: OUTPUT (corner/ring flow, ArUco) / OUTPUT (cross-marker,
   separate pipeline — see above) / INPUT (rate/thrust).
2. Record ≥5 phased calibration runs (user, Gazebo) to the default `calibration_data/...`
   (ArUco: `output/`; cross-marker: `output_cross/` — separate dirs, don't mix).
3. Derive: `derive_board_cal.py` (ArUco corner) → paste `_sensor_cal_hw`/`_sensor_cal_s`;
   `derive_ring_cal.py` (ring, keyed to the new corner M) → paste `_sensor_cal_ring`;
   `derive_cross_marker_cal.py` (cross-marker, independent pipeline) → paste into
   `CrossMarkerPerception.__init__`. (Input: `aggregate_input_calibration.py` → thrust
   slope + lag table.)
4. Validate on INDEPENDENT data: multisine + landing (output) / multi-axis + landing (input),
   in `validation_data/`. Notebooks auto-load the new cal.
5. The notebooks + tools auto-load — no manual re-sync. Re-run `derive_ring_cal.py` whenever
   `_sensor_cal_hw` changes (the transfer ring cal is keyed to it).
