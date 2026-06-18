---
name: PX4/Gazebo key paths and resources (Ubuntu)
description: Authoritative file and tool locations for the PX4 SITL + Gazebo Harmonic stack on the Ubuntu side
type: reference
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
**Repo tree (Ubuntu, `/home/shubham/Soft-Precise-Landing/`)**
- `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m` — canonical PLASMC reference (single-run; in subfolder, NOT at MATLAB top level)
- `MATLAB/Common/{InitVar,Constants,UAVDyn,RK5,image_feature,kappa_Solver,kappa_a_Solver,traj_Gen,smooth4,sat}.m` — shared sim utilities
- `MATLAB/Comparison/` — 5-controller benchmark + Monte-Carlo
- `MATLAB/Sweeps/` — parameter sweeps
- `PX4_Gazebo/` — Python pipeline. Key files:
  - `landing_test.py` — main landing entry (arm + takeoff + PLASMC landing)
  - `controller.py` — PLASMC port (outer + middle + yaw; PX4 does inner rate)
  - `img_data.py` — ArUco detection + LK optical flow + applied sensor_cal matrices (lines 65–66)
  - `flight_controller.py` — MAVSDK wrapper
  - `gz_subscriber.py` — ROS 2 subs for /pose, /clock, /image
  - `numerical_methods.py` — RK5, smooth4, extrapolate
  - `record_output_calibration.py` — drives sinusoidal motion sweep, records raw image/pose data for sensor-cal recalibration. Has patches: reduced amplitude, post-takeoff hover, asyncio.wait_for, altitude bail.
  - `tips.txt` — manual launch sequence (legacy reference)
- `PX4_Gazebo/run_aruco_landing.sh` — one-command landing launcher (`HEADLESS=1` for offscreen Qt)
- `PX4_Gazebo/run_output_calibration.sh` — one-command sensor-cal recording launcher
- `PX4_Gazebo/measure_image_fps.sh` — measures `/image` topic Hz (Gazebo native + ROS side)
- `PX4_Gazebo/validate_image_feed.sh` — wraps validate_image.py
- `PX4_Gazebo/validate_image.py` — captures sample frame, saves raw + ArUco-annotated PNG
- `PX4_Gazebo/analyze_calibration.py` — derives per-axis median(gt/raw) → proposed sensor_cal matrices for the most recent recording
- `PX4_Gazebo/validate_pose_transforms.py` — 6 sanity checks on the frame conventions used by analyze_calibration
- `PX4_Gazebo/aggregate_calibration.py` — combines all recordings, reports mean/median/std per axis
- `PX4_Gazebo/tune_savgol.py` — sweeps (window, polyorder) over all recordings, picks max-corr config
- `PX4_Gazebo/plotter_output_calibration.ipynb` — interactive notebook: ground-truth vs calibrated-image-side overlays + quality metrics. Cell 8 has savgol params (offline-tuned to 101, 3).
- `PX4_Gazebo/calibration_data/<timestamp>/` — auto-saved recordings (gitignored). Each contains Img_Data.npy, Telemetry_Data.npy, Ground_Truth.npy, Img_Params.txt.
- `PX4_Gazebo/run_logs/` — per-component logs from launchers (gitignored).
- `scripts/` — Python analysis scripts for paper figures (NOT the SITL pipeline)

**External Ubuntu paths**
- `~/PX4-Autopilot/` — PX4 source. SITL binary at `build/px4_sitl_default/bin/px4`.
- `~/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf` — drone model.
- `~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` — **camera; edited to 640×480 @ hfov=1.74 → fx=fy=270**.
- `~/PX4-Autopilot/Tools/simulation/gz/worlds/aruco.sdf` — stationary-target world, gravity `0 0 -9.8`.
- `~/ws/scripts/soft_precise_landing/` — original Python pipeline (working baseline, **do not edit**).
- `~/ws/scripts/env2025/` — Python venv used by all PX4_Gazebo scripts.
- `~/Downloads/QGroundControl.AppImage` — QGC (always launched, offscreen in HEADLESS mode to satisfy the "No connection to GCS" preflight).
- `~/ws/scripts/soft_precise_landing/plotter_output_calibration.ipynb` — reference notebook for sensor-cal analysis (cells 11+16+18 are the core methodology that analyze_calibration.py ports).

**Stack components**
- **uXRCE-DDS agent:** `/usr/local/bin/MicroXRCEAgent udp4 -p 8888` (mandatory before PX4 SITL).
- **PX4 SITL airframes:** 4014 = `x500_mono_cam_down`, 4022 = `rover_aruco`.
- **Worlds:** `aruco` (stationary), `rover` (moving target).
- **ROS 2 bridges:** `ros_gz_bridge parameter_bridge` for `/clock`, `/pose` (PoseArray), `/image`.

**Headless trick**
Gazebo Harmonic doesn't honor PX4's `HEADLESS=1` env var. Use `QT_QPA_PLATFORM=offscreen` on the PX4 SITL launch. Standalone `gz sim -s` breaks the camera plugin on x500_mono_cam_down. QGC must run (offscreen is fine) to satisfy PX4's "No connection to GCS" preflight.

**How to apply**
- Before recommending a MATLAB file path: it's inside a subfolder (Multi_init_cond, Common, Comparison, Sweeps), not top level.
- Before recommending a PX4/Gazebo file: check `PX4_Gazebo/` first (this repo), then `~/ws/scripts/soft_precise_landing/` (baseline).
- Full launch: `./run_aruco_landing.sh` (landing test) or `./run_output_calibration.sh` (sensor cal).
- For calibration analysis: `analyze_calibration.py` → `validate_pose_transforms.py` → `aggregate_calibration.py` → review, then edit `img_data.py:65-66`.
- For savgol tuning: `tune_savgol.py`.
