---
name: feedback_sitl_headless_run
description: "How to run Gazebo SITL landings in headless mode reliably — env vars, retry wrapper, CPU pinning, and common failure modes"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a377a083-d63b-447a-908e-12017cf609f1
---

**Canonical headless SITL landing run (2026-06-09)**

**How to apply:** Always use this pattern for any unattended / batch SITL run.

## Single run (manual)
```bash
cd /home/shubham/Soft-Precise-Landing/PX4_Gazebo/
HEADLESS=1 taskset -c 6-15 bash scripts/run_aruco_landing_retry.sh
```

## n-rep batch loop
```bash
for i in 1 2 3 4 5; do
  echo "=== rep $i ==="
  taskset -c 6-15 bash scripts/run_aruco_landing_retry.sh
  echo "=== rep $i done ==="
done
```
**Never use `set -e` in the wrapper loop.** The retry script exits with code 42 on an IC-convergence flake; `set -e` misinterprets that as a fatal error and kills the remaining reps (confirmed failure 2026-06-09).

## Why each piece is required

| Item | Why |
|---|---|
| `HEADLESS=1` | Sets `QT_QPA_PLATFORM=offscreen` on PX4 SITL. Gazebo's `-s` (standalone server) breaks the downward-camera plugin; use offscreen Qt instead. QGC also launches offscreen to satisfy PX4 preflight GCS check. |
| `taskset -c 6-15` | Prevents CPU starvation that causes lockstep/is_armable race condition. Without it the EKF timing race causes ~30-50% of runs to hang at "waiting for PX4 preflight". CPU 6-15 isolates SITL from the rest of the OS. |
| `run_aruco_landing_retry.sh` | Wraps `run_aruco_landing.sh` with up to `MAX_ATTEMPTS=5` retries on exit-code-42 (IC convergence timeout = SITL startup flake, not a real failure). Without it, ~20-30% of runs fail at IC. |
| `LANDING_AUTOSAVE=1` | Auto-saves results to `test_data/Landing_Test/<timestamp>/`; baked as default in the run script (commit 2e125a9) but set explicitly if unsure. |

## Common failure modes and fixes

| Symptom | Cause | Fix |
|---|---|---|
| Hangs at "waiting for PX4 preflight to clear" | CPU starvation → lockstep race | `taskset -c 6-15` |
| "IC convergence timeout (pos_err=X m)" before controller engages | SITL startup flake (IMU timestamp=0) | Use `run_aruco_landing_retry.sh`; it auto-retries |
| IC yaw limit-cycle (drone sweeps ±30° in yaw forever) | `IC_YAW_SERVO_DMAX_DEG` too large → bang-bang + KF lag | Default is 0.3 (baked `a62c81e`); do not increase |
| `renice` your IDE | Gazebo + ROS + IDE compete for CPU 0-5; `renice 15 $(pgrep code)` or `renice 15 $(pgrep cursor)` frees cycles | Run once per session before batch |
| n-rep loop stops after one IC timeout | `set -e` in wrapper script aborts on exit code 42 | Remove `set -e`; use `run_aruco_landing_retry.sh` |

## Result location
```
PX4_Gazebo/test_data/Landing_Test/<Day Mon DD HH-MM-SS YYYY>/
  Control_Data.npy   — controller state timeseries
  Ground_Truth.npy   — GT['SoftPrecise'] = {xy_err, rel_vel, target_lost, ...}
  Img_Data.npy       — image pipeline state
  Control_Params.npy — parameter snapshot
```
`SoftPrecise['xy_err']` and `SoftPrecise['target_lost']` are the canonical outcome metrics.
