---
name: validation-runs-status
description: "Validation data collection status (2026-06-06): the record_output_validation.py / record_input_validation.py apps WORK end-to-end in SITL, but their default maneuver amplitudes are too aggressive (output marker-loss, input open-loop flip). What's collected, what's usable, and what to tune. Pre-recorded landing validation is available; clean multisine needs a tuned re-record."
metadata:
  node_type: memory
  type: project
  originSessionId: validation-2026-06-06
---

The two-stage calibration/validation pipeline + apps are built (see [[ring-flow-calibration]],
skill `io-calibration`). First SITL shakedown of the validation apps (2026-06-06):

**`apps/record_output_validation.py` (multisine) — RUNS, data mediocre.** Saved 12.4k frames to
`validation_data/output_multisine/`. But the marker drops out repeatedly during the aggressive multisine
(KLT fallback all over the log) → align r=0.61 (vs 0.96 on cal runs), corner R^2 hx=0.69 hy=0.59
but **hz=-0.07, wz=0.17**. The lateral generalizes; vertical/yaw validation is corrupted by
marker loss, NOT necessarily a cal failure. FIX: reduce `VAL_MS_AMP_XY` (0.30→~0.15) and
`VAL_MS_AMP_YAW_DEG` (15→~8) to keep the marker framed, or raise the lateral gate.

**`apps/record_input_validation.py` (multisine) — RUNS, but OPEN-LOOP FLIPS the drone.** Sustained
`VAL_RATE_AMP=0.15` rad/s multisine on all 3 axes tilts the drone → altitude loss → "Impact
detected |a|=82 m/s²" near t=20s. Still, **wx/wy are GOOD** (r=0.98, lag 45 ms — matches cal);
only **wz is poor** (r=0.25). FIX: drop `VAL_RATE_AMP` hard (0.15→~0.03–0.05) and/or shorten
`VAL_INPUT_MS_S`; open-loop body-rate excitation has no attitude feedback so amplitude must stay
tiny. The thrust 'landing' profile (`VALIDATION_PROFILE=landing`) is untested.

**Pre-recorded validation data (no new run needed):** landing-to-touchdown is covered by
`test_data/Landing_Test/` (534 independent recordings; pick one with non-empty ring). Input
multi-axis can interim-validate on a `calibration_data/input/` run (not held out). `validation_data/`
is symlink-populated with the best candidates. The pre-fps-fix multisine
(`Obsolete/.../output_pre_fpsfix`) is leveling-corrupted → false-negative, do NOT use.

**How to run (headless; user drives Gazebo, but it WAS runnable from an agent session with
sandbox disabled):**
```
HEADLESS=1 CALIB_APP=apps/record_output_validation.py VALIDATION_PROFILE=multisine \
  CALIB_PARENT=$PWD/validation_data/output_multisine bash scripts/run_output_calibration.sh
HEADLESS=1 INPUT_APP=apps/record_input_validation.py VALIDATION_PROFILE=multisine VAL_RATE_AMP=0.04 \
  CALIB_PARENT=$PWD/validation_data/input_multiaxis bash scripts/run_input_calibration.sh
```
The apps self-terminate (maneuver→save→launcher cleanup); both runs left no orphaned SITL procs.
