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

---

# OUTPUT calibration (image → optical flow `[h; w]`)

**Maneuver:** `apps/output_calibration.py`, default **phased** (each axis x→y→z→yaw driven ALONE,
settles between → axes decorrelated). `CALIB_MODE=multisine` = freq-multiplexed (validation only).

**Derive (corner):** `tools/derive_board_cal.py` — full **6×6** `M` s.t. `GT[h;w] = M @ raw`
(corner lstsq is full-rank with fixed geometric h↔w coupling, so M is NOT diagonal). The applied
corner cal **zeros the Wx/Wy rows** (yaw-only w: roll/pitch rate is geometrically unobservable from
image flow, so the GT w-axis is set yaw-only `V_w_ug[:,:2]=0` → those rows come out 0, consistent
with `CTRL_ZERO_WXY=1`). h-block comes out near-identity from phased data.

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
into the GT dict by `output_calibration.py` so the derive tools can fit.

**Validate:** `notebooks/plotter_output_validation.ipynb` — GT-direct, auto-loads the live cal,
checks calibrated corner + ring flow vs GT on a multisine run (per-channel R²) and runs the
landing-to-touchdown check via `tools/validate_output_flow.py::validate(dir)`.

# INPUT calibration (FC command → achieved body-rate / thrust)

**Key difference:** input cal is a **dynamic transfer (gain + LAG)**, not a static matrix. The
headline metric is **lag**, not amplitude — gain≈1 with 287 ms yaw lag is still a failing cal.

**Maneuver:** `apps/input_calibration.py` — sends `send_attitude_rate` (body ω + thrust) via a
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
validation_data/multisine/    output multisine validation                            — gitignored
validation_data/input_multi/  input multi-axis validation                            — gitignored
validation_data/landing/      landing-to-touchdown (serves BOTH input + output)      — gitignored
```

**Routing env knobs** (keep validation OUT of `calibration_data/`):
| Knob | Read by | Use |
|---|---|---|
| `CALIB_PARENT` | `run_output_calibration.sh`, `run_input_calibration.sh` | parent dir override (auto-timestamped subdir) |
| `CALIB_OUT_BASE` / `INPUT_CALIB_OUT_BASE` | output/input cal apps | parent dir (app-level) |
| `CALIB_OUT_DIR` / `INPUT_CALIB_OUT_DIR` | apps + launchers | exact path (precedence) |
| `LANDING_OUT_BASE` | `apps/landing_test.py` | route a validation landing |
| `CALIB_MODE` | `output_calibration.py` | `phased` (default) \| `multisine` |
| `RING_CAL_MODE` | `derive_ring_cal.py` | `transfer` (default) \| `gt` |

**Collect (Gazebo — the USER runs these; never invoke SITL yourself):**
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
# CALIBRATION (phased, loop until ≥5 valid; cleanup empty dirs — see CLAUDE.md recal workflow)
for i in $(seq 1 10); do timeout 220 bash scripts/run_output_calibration.sh; done
~/ws/scripts/env2025/bin/python3 tools/derive_board_cal.py     # corner M  -> paste into img_data.py
~/ws/scripts/env2025/bin/python3 tools/derive_ring_cal.py      # ring M_ring (transfer) -> paste
# VALIDATION (dedicated apps via CALIB_APP/INPUT_APP override; VALIDATION_PROFILE=multisine|landing)
CALIB_APP=apps/record_output_validation.py VALIDATION_PROFILE=multisine CALIB_PARENT=$PWD/validation_data/multisine bash scripts/run_output_calibration.sh
INPUT_APP=apps/record_input_validation.py  VALIDATION_PROFILE=multisine CALIB_PARENT=$PWD/validation_data/input_multi bash scripts/run_input_calibration.sh
LANDING_OUT_BASE=$PWD/validation_data/landing bash scripts/run_aruco_landing.sh   # landing serves BOTH
```
The validation apps (`apps/record_output_validation.py`, `apps/record_input_validation.py`) each fly one of two
`cmd_profiles` (`VALIDATION_PROFILE=multisine|landing`) and route to `validation_data/`.

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

# Recalibration procedure
1. Identify which chain: OUTPUT (corner/ring flow) or INPUT (rate/thrust).
2. Record ≥5 phased calibration runs (user, Gazebo) to the default `calibration_data/...`.
3. Derive: `derive_board_cal.py` (corner) → paste `_sensor_cal_hw`/`_sensor_cal_s`; then
   `derive_ring_cal.py` (ring, keyed to the new corner M) → paste `_sensor_cal_ring`.
   (Input: `aggregate_input_calibration.py` → thrust slope + lag table.)
4. Validate on INDEPENDENT data: multisine + landing (output) / multi-axis + landing (input),
   in `validation_data/`. Notebooks auto-load the new cal.
5. The notebooks + tools auto-load — no manual re-sync. Re-run `derive_ring_cal.py` whenever
   `_sensor_cal_hw` changes (the transfer ring cal is keyed to it).
