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

**Open issue from that first landing:** hard touchdown (541 m/s² impact,
4.27 m/s rel-vel) despite clean perception the whole flight (100% detection,
kappa/s_e_n/CBF all nominal, no ratchet/breach/saturation). Live lead:
commanded velocity (`w_u`) was climbing through the terminal descent instead
of braking — likely a descent-pacing/terminal-gating assumption tuned
against ArUco's `MARKER_EXTENT_PX` scale that doesn't transfer to the
cross-marker's different extent growth rate. Not yet root-caused. See the
full doc for the diagnostic trace and next steps.

**How to apply:** before any future cross-marker flight, check whether you
need (1) calibration, (2) independent validation, or (3) a real closed-loop
landing — they use different apps and only (3) exercises the actual
controller/CBF/terminal logic. Don't infer landing-quality conclusions from
(1)/(2) data.
