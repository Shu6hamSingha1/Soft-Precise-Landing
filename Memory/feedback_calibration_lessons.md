---
name: SITL calibration lessons (record_output_calibration.py workflow)
description: How to run sensor calibration reliably in PX4 SITL — common failure modes, what NOT to do, the validated workflow
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
The PX4_Gazebo sensor-calibration pipeline took ~10 attempted runs across a session to produce 5 valid recordings. Key lessons:

## DO

- **Use reduced sweep amplitudes** (XY ≤ 0.7 m, Z ≤ 0.3 m smoothly varying, yaw ≤ 10°). Original 1.0 m / 15° / square-wave-z reliably tripped PX4 preflight (Accelerometer Bias, Velocity Unstable, Attitude failure) → failsafe → MAVSDK deadlock.
- **Hover 5 s after takeoff** before sending offboard setpoints. Without this, PX4's TAKEOFF→HOLD state machine isn't done and the first offboard setpoint hits in a mode-transition window → `flight_mode_manager: Matching flight task was not able to run`.
- **Wrap every MAVSDK `send_position_ned` in `asyncio.wait_for(..., timeout=0.5)`**. PX4 dropping offboard makes the call block forever — `wait_for` lets the loop bail.
- **Bail on altitude drop** (`cur_z - takeoff_z > 2 m`). Detects PX4 entering AUTO_LAND failsafe before MAVSDK times out.
- **Use timestamped output dirs, not a `latest` symlink**. Multiple back-to-back runs need history preserved; `latest` overwrites.
- **Loop run-until-5-valid**, accepting ~50% failure rate. Cheaper than chasing reliability.

## DON'T

- **Don't commit calibration recordings to git** — `.gitignore` already covers `PX4_Gazebo/calibration_data/`.
- **Don't apply the offline-tuned savgol (window=101, polyorder=3)** to the runtime img_data.py. 3.4 s group delay at 30 Hz would destabilize PLASMC. Runtime currently uses (13, 1) — short window, linear polynomial — chosen via lag-aware tuning.
- **Don't trust short-window "improvement" reports from offline analysis** — the real metric for runtime is lag-aware (sliding window, middle sample). The legacy `FILTER_WIN=51` looked good offline (0.41 mean|corr|) but was strictly worse than no filter at runtime (0.24 vs 0.33 baseline) because the centroid signal got pulled out of phase. Always re-run `tune_savgol.py` in runtime mode before applying a new filter to img_data.py.
- **Don't trust the legacy `_sensor_cal_hw = diag(1,1,1,1/3,1/3,1)` and `_sensor_cal_s = diag(1/6,1/6,1,1)`** — those were for a 1280×960 camera and over-scale optical flow by ~10×. Applied 2026-05-12: `diag([0.152, 0.178, 0.065, 0.208, 0.221, 0.244])` and `diag([0.581, 0.581, 1, 1])`.

## Why ~50% of SITL calibration runs still fail

Even with the patches, half the attempted runs produce empty data. The python process hangs at 200% CPU; `wait_for` should bail but somehow doesn't always. Probable cause is MAVSDK's offboard ACK path getting wedged at a layer below the wait_for, plus PX4's preflight tripping randomly on accel bias. Workaround is the run-until-5-valid loop; root-cause debug is deferred.

## How to apply

- When the user asks to recalibrate sensor_cal matrices: use the validated 5-step workflow (run_output_calibration.sh → analyze → validate_pose_transforms → aggregate → apply median to img_data.py).
- When the user asks about calibration failures: the run hung in `send_position_ned`. Check PX4 log for "Failsafe activated" or "Landing at current position".
- When the user asks to add a filter to img_data.py runtime: warn about the lag tradeoff; recommend window ≤ 11, polyorder ≤ 2.
