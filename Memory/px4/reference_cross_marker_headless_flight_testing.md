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
rel-vel):** the perceived vertical flow `h_z` went noisy and briefly
WRONG-SIGNED (+0.27 vs. a flat -0.30 reference) right at ~1.5m altitude,
corrupting the vertical-velocity braking command exactly when needed — a
real, live consequence of the Hz-weak-near-the-ground problem this whole
session's calibration work was chasing (the "sign-flips-at-2m" STATISTICAL
theory was debunked, but the underlying low-altitude Hz noisiness was never
in question). Perception itself was fine (100% detection); kappa/s_e_n/CBF
all stayed nominal — this is specifically the terminal vertical-velocity
control loop trusting a bad `h_z` reading. See the full doc for the
diagnostic trace and the proposed next step (a safety-net/rate-limiter on
`h_z` near the ground, analogous to ArUco's ring-loom-fusion fallback,
which this marker's docstring explicitly says isn't implemented).

**How to apply:** before any future cross-marker flight, check whether you
need (1) calibration, (2) independent validation, or (3) a real closed-loop
landing — they use different apps and only (3) exercises the actual
controller/CBF/terminal logic. Don't infer landing-quality conclusions from
(1)/(2) data.
