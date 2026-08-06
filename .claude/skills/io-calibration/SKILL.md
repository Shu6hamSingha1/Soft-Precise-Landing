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

## Hz/Wz weakness root-cause chain (2026-08-05/06 — read before re-touching this cal)
Applies to `_sensor_cal_hw`'s Hz row (R²~0.22, the weakest of Hx/Hy/Hz) and Wz (large
inter-run STD, coefficients swinging to ~10-13). Investigated in 3 stages, each RULED
OUT the previous stage's suspect rather than confirming it — read in order before
re-diagnosing this from scratch:
1. **NOT data contamination.** An ad-hoc diagnostic script appeared to find z-phase GT
   windows dominated by yaw-rate instead of vz — that "finding" was itself a bug: the
   script truncated `gt['Phase']` via a naive `[:n]` slice instead of applying the same
   `valid`-mask filter `compute_gt_signals()` uses internally (drops scattered
   duplicate-timestamp samples, not a trailing block), desyncing phase labels from the
   GT arrays. Re-checked with correct alignment across all 6 original recordings:
   z/yaw/yawagg phases are genuinely clean in every run. A purity gate built on this
   false premise (`clean_zyaw_mask`, briefly added to `derive_cross_marker_cal.py`) made
   Wz WORSE and was reverted. See `feedback_cross_marker_radial_spread_ceiling`.
2. **Radial spread was the next suspect, per the existing Wx/Wy-unrecoverable finding**
   (same memory): the flow Jacobian's `Hz`/`Wz` columns (`_fill_A`, linear in point
   (x,y)) and `Wx`/`Wy` columns (quadratic) all need tracked points spread far from
   image center; the cross marker's `goodFeaturesToTrack` corners measured only
   ~0.05 mean / ~0.26 max normalized radius (the shape's own strongest corner — the
   cross intersection — dominates the point pool every frame). Fix attempt:
   `_sample_flow_points` (cross_marker_perception.py) now excludes a central disk
   (`CROSS_FLOW_CENTER_EXCLUDE_FRAC`, default 0.35× the mask's own half-extent) from the
   GFT mask, biasing candidates toward arm/stub tips, plus a frame-boundary margin
   (`CROSS_FLOW_BOUNDARY_MARGIN_PX`, default 20px) excluding corners likely to exit
   frame mid-track. Falls back to the unbiased mask if too few peripheral corners
   survive (`MIN_PERIPHERAL_POINTS`, default 4).
3. **Result (n=4 fresh runs, one 75%-ok-rate run correctly gated out): MIXED, not a
   clean win.** Radial spread DID increase as measured (mean p90 0.113→0.184, max p90
   0.289→0.380 — confirms the mechanism is real and the fix works as designed). Hx/Hy
   R² improved substantially (0.55→0.73, 0.63→0.79). **But Hz's R² was UNCHANGED
   (0.22→0.22 to 2 decimal places)** despite the spread increase, and Wz got marginally
   worse (R² 0.57→0.53, inter-run STD worse on some columns, though its own coefficient
   magnitude dropped 12.9→9.4). **Conclusion: radial spread is NOT the (sole) binding
   constraint for Hz specifically** — something else caps vertical-flow observability
   (leading suspects, NOT yet investigated: z-phase excitation amplitude too small
   relative to noise, or `_getVirtualPts`'s perspective-divide noise near-grazing rays
   swamping the loom signal regardless of point spread). The peripheral-bias code is a
   net win for Hx/Hy and doesn't hurt detection ok-rate, but has NOT been deployed to
   the live `_sensor_cal_hw`/`_sensor_cal_s` pending this next investigation — don't
   assume it's live without checking the file's own provenance comment/date.
4. **Hardware (ArUco) shows the OPPOSITE profile** — checked for comparison, not a
   contradiction: `Hz` is the *strongest* row on hardware (R²=0.64) while `Hx/Hy` are
   weak (0.07-0.08, real-camera noise/blur, not geometry) and `Wz` is weak on both
   platforms (0.57 cross-marker vs 0.08 hardware). Don't treat "Hz/Wz are universally
   the hardest-to-observe rows" as a cross-platform law — the weak axes only overlap on
   Wz; Hz's weakness is cross-marker-specific.

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
