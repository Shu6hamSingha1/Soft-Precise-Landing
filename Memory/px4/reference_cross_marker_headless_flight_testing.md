---
name: reference_cross_marker_headless_flight_testing
description: How to launch headless Gazebo flights (calibration/validation/real landing) against the cross-marker pipeline, plus the launcher fix and the first closed-loop landing's open issue
metadata:
  node_type: memory
  type: reference
---

Full procedure: `PX4_Gazebo/docs/HANDOVER_cross_marker_headless_flight_testing_20260811.md`.

**Quick reference:**
- Calibration flight: `WORLD=cross_marker CALIB_APP=apps/record_cross_marker_calibration.py HEADLESS=1 bash scripts/run_output_calibration.sh`
- Validation flight: same launcher, `CALIB_APP=apps/record_cross_marker_validation.py VALIDATION_PROFILE=multisine|landing`
- **Real landing test (only works as of 2026-08-11):**
  `HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross bash scripts/run_aruco_landing.sh`
  — both env vars required together (`WORLD` picks the Gazebo scene,
  `MARKER_TYPE` picks the perception pipeline in `controller.py`).
- `run_ic_validation.sh` needed no changes — it forwards `WORLD`/`MARKER_TYPE`
  through `run_aruco_landing_retry.sh` automatically.

**Launcher fix (2026-08-11):** `run_aruco_landing.sh` previously hardcoded
`PX4_GZ_WORLD=aruco` and every `/world/aruco/...` bridge topic — added a
`WORLD` env override (default `aruco`, unchanged), mirroring
`run_output_calibration.sh`'s existing pattern. This was the FIRST time the
real PLASMC controller (not a scripted recorder app) was ever run against
the cross-marker pipeline.

**ROOT-CAUSED, that first landing's hard touchdown (541 m/s² impact, 4.27 m/s
rel-vel) — CORRECTED after a methodology check (compare against proper
Z_REG-regularized, time-synced GT, not just the controller's own desired
reference):** two distinct problems. (1) A real, GT-confirmed sign flip —
perceived `h_z` reads +0.27 (ascending) while properly-computed GT `h_z`
(`_compute_gt_flow_zreg`, `Z_REG=0.2`, matching `gt_feedback.py`'s
convention) is -0.78 to -0.86 (descending) at the same instant, ~1.3m
altitude. (2) The DOMINANT problem: perceived `h_z` stays pinned around
-0.15 to -0.27 through the whole terminal second while true GT `h_z` grows
to -5.70 near touchdown (the classic `1/(z+Z_REG)` terminal amplification
this project has documented extensively for ArUco) — the perceived signal
has nowhere near the dynamic range to track the true near-ground blowup, so
the controller sees "close enough to the -0.30 reference" and never brakes
hard. `compute_gt_signals` (used for all the day's earlier calibration
z-phase checks) CANNOT see this at all — it has no Z_REG and hard-NaNs
below 1m depth; `_compute_gt_flow_zreg` is the correct tool for anything
touchdown-adjacent. Perception detection itself was fine (100%);
kappa/s_e_n/CBF all stayed nominal. See the full doc for the diagnostic
trace and the two candidate fixes (a depth-aware fallback/blend near the
ground, or re-deriving the cal specifically for the near-ground regime —
a rate-limiter alone would NOT fix the dominant magnitude problem).

**How to apply:** before any future cross-marker flight, check whether you
need (1) calibration, (2) independent validation, or (3) a real closed-loop
landing — they use different apps and only (3) exercises the actual
controller/CBF/terminal logic. Don't infer landing-quality conclusions from
(1)/(2) data.
