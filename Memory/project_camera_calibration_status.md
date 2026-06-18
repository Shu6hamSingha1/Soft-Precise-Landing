---
name: camera-sensor-cal-status-px4-gazebo
description: "Current camera resolution, fps performance, applied sensor-cal matrices, the validated calibration workflow, and the 2026-06-01 image-center bug fix"
metadata: 
  node_type: memory
  type: project
  originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Camera

- **640×480 @ fx=fy=270, hfov=1.74 rad** (mono_cam SDF — modified from Gazebo default 1280×960).
- After `cv2.ROTATE_90_CW` in `gz_subscriber.py`, the runtime image is **480 wide × 640 tall**.
- Measured fps: **~62 Hz steady** on both Gazebo native and ROS bridge.

## Image-center bug (FIXED 2026-06-01)

`img_data.py:62` had `self.center = np.array(self._resolution)/2`. `gz_subscriber.py:154` stores `_res = (msg.height, msg.width)` per ROS convention, so this gave `center = (H/2, W/2) = (320, 240)`. But cv2.aruco corners are `(x=col, y=row)`, so the principal point should be `(W/2, H/2) = (240, 320)`. Fix: `self.center = np.array(self._resolution[::-1]) / 2.0`. Same transposition bug existed in `plotter_output_calibration.ipynb` cell 38 (now `center = np.array([240.0, 320.0])`). See [[image-center-bug]] for the full diagnosis.

Effect of the bug:
- Marker reported at normalized (-0.27, +0.30) when actually at ~(0.028, 0.004) under the drone.
- L matrix evaluated at wrong corner coords → strong v_z↔v_x↔v_y multicollinearity (LᵀL[v_z, v_x] = +1.22), making v_z essentially unobservable.
- Plotter cell 38 LHS-RHS rel error: yaw 286% → 39%, z 118% → 37% after fix.

## Sensor calibration matrices — STALE (need re-derivation)

**Currently in `img_data.py:124-125` (carried over from 2026-05-31):**
```python
_sensor_cal_hw = np.diag([0.2138, 0.1186, 2.6344, 0.5050, 0.8256, 10.5153])
_sensor_cal_s  = np.diag([0.7073, 0.6625, 1.0000, 1.0000])
```

These were derived from recordings made with the BUGGY image center. The std-ratio aggregation absorbed the bias as magnitude compensation — but it couldn't fix the cross-axis bleeding (v_z observability problem). After the center fix, the runtime LSTSQ will produce different `raw_hw` values, so these `_sensor_cal_hw` values **no longer correspond to the runtime LSTSQ output**. Signs are still correct, magnitudes may be off by 10-30% per axis until re-derivation.

**Recalibration required.** New recordings must be made with the fixed `img_data.py` and aggregated:
```bash
# Run until you have ≥5 valid post-fix recordings
for i in 1 2 3 4 5 6 7 8 9 10; do
    timeout 220 bash run_output_calibration.sh
    # ...cleanup empty dirs...
done
~/ws/scripts/env2025/bin/python3 aggregate_calibration_phased.py
# Apply the new diag values to img_data.py:124-125 and plotter cell 4
```

## Transformation math state (post 2026-06-01 audit)

The plotter notebooks now use **NED world + FRD body throughout**, and:
- `R_V_from_body = identity` (after the SDF 90° pitch + cv2.ROTATE_90_CW, image_+X = body_+X forward, image_+Y = body_+Y right, image_+Z = body_+Z down).
- `V_input` in the L equation `ṡ = L · V_input` is `[+B_y_g, +B_w_tug]` directly (no rotation, no leading minus) — because B_y_g and B_w_tug are already "target relative to camera" quantities in body-FRD = V-frame.
- Earlier `R_V_from_body = Rz(π) = diag(-1,-1,+1)` + leading-minus formulation in cell 6/38 happened to give the same numerical result on x and y axes by accident, only exposed by ω_z sign flip; see [[so3-quaternion-omega]] for the related GT-side quat-diff fix.

## Calibration workflow

The pipeline uses **phased excitation** (drives each axis alone in sequence: yaw → x → y → z, with brief settles between). Each cal-axis is derived only from the samples tagged with that axis's phase, giving decorrelated lstsq inputs.

1. `cd PX4_Gazebo && bash run_output_calibration.sh` — headless: brings up MicroXRCEAgent + PX4 SITL + 3 bridges + QGC, hovers 5 s post-takeoff, runs phased sweep, auto-saves to `calibration_data/output/<timestamp>/`.
2. ~50% success rate per attempt; cleanup empty dirs. Run until ≥5 valid. The notebook's `_is_valid_run` also rejects overshoot-corrupted recordings (UAV peak-to-peak motion > 3× commanded amplitude).
3. `python3 validate_pose_transforms.py` — 6/6 sanity checks on frame conventions.
4. `python3 aggregate_calibration_phased.py` — canonical aggregator; MAD-trimmed mean + median + std across runs.
5. `python3 tune_savgol.py` — offline-best and runtime-best savgol parameters.
6. `plotter_output_calibration.ipynb` — interactive validation. Cell 38 LHS-RHS uses the same L matrix as `img_data.py:_fill_A`; rel error should now be 30-45% per phase (was 100-286% before the center fix).

## Savgol filter tuning

| Config | Offline mean\|corr\| | Runtime mean\|corr\| | Notes |
|---|---|---|---|
| no filter (baseline) | — | 0.329 | raw → sensor_cal |
| MATLAB (11, 2) | 0.348 | 0.337 | MATLAB Constants.m default |
| Legacy (51, 2) | 0.410 | **0.243** | user's old img_data.py; HURTS runtime |
| Offline best (101, 3) | **0.439** | 0.178 | notebook only — 3.4 s lag |
| **Runtime best (13, 1)** | 0.395 | **0.357** | applied to img_data.py |

- **Offline notebook** (cell 8): `FILTER_WIN=101, POLYORDER=3`.
- **Runtime** (`img_data.py`): `FILTER_WIN=13, FILTER_POLYORDER=1` — lag-aware tuned.
