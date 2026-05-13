#!/usr/bin/env bash
# Run input_calibration.py headlessly. Same pattern as run_output_calibration.sh:
# brings up MicroXRCEAgent + PX4 SITL (Qt offscreen) + 3 bridges + QGC, waits
# for preflight, then runs the input-calibration sweep that sends attitude-rate
# commands and records pose response.
set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/run_logs"
mkdir -p "$LOG_DIR"

CALIB_PARENT="$SCRIPT_DIR/calibration_data/input"
mkdir -p "$CALIB_PARENT"
TIMESTAMP="$(date '+%a %b %e %H-%M-%S %Y')"
export INPUT_CALIB_OUT_DIR="${INPUT_CALIB_OUT_DIR:-$CALIB_PARENT/$TIMESTAMP}"
mkdir -p "$INPUT_CALIB_OUT_DIR"
echo "[input-calib] output dir: $INPUT_CALIB_OUT_DIR"

declare -a PIDS=()

cleanup() {
  echo
  echo "[input-calib] cleanup..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  done
  sleep 1
  pkill -9 -f 'input_calibration.py' 2>/dev/null || true
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco' 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  pkill -9 -f 'QGroundControl' 2>/dev/null || true
  echo "[input-calib] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[input-calib] launching $name"
  setsid "$@" > "$logfile" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  sleep 0.3
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[input-calib] $name died:"
    tail -n 20 "$logfile" || true
    exit 1
  fi
}

start_bg microxrce MicroXRCEAgent udp4 -p 8888
sleep 1

echo "[input-calib] starting PX4 SITL (headless Qt)..."
# Truncate log so the preflight-wait grep below only matches THIS run.
: > "$LOG_DIR/px4_sitl.log"
setsid env QT_QPA_PLATFORM=offscreen \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
PIDS+=($!)

echo -n "[input-calib] waiting for Gazebo "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q '/world/aruco/clock'; do
  sleep 1; echo -n "."
  WAITED=$((WAITED + 1))
  [ "$WAITED" -gt 60 ] && { echo " timeout"; exit 1; }
done
echo " up after ${WAITED}s"

start_bg bridge_clock ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  --ros-args -r /world/aruco/clock:=/clock

start_bg bridge_pose ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V \
  --ros-args -r /world/aruco/pose/info:=/pose

start_bg bridge_image ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/image

if [ -x "$HOME/Downloads/QGroundControl.AppImage" ]; then
  QT_QPA_PLATFORM=offscreen start_bg qgc "$HOME/Downloads/QGroundControl.AppImage"
fi

echo -n "[input-calib] waiting for PX4 preflight "
WAITED=0
while ! strings "$LOG_DIR/px4_sitl.log" 2>/dev/null | grep -q 'Ready for takeoff' \
      && [ "$WAITED" -lt 30 ]; do
  sleep 2; echo -n "."
  WAITED=$((WAITED + 2))
done
echo " (${WAITED}s)"

echo "[input-calib] running input_calibration.py (data -> $INPUT_CALIB_OUT_DIR)..."
cd "$SCRIPT_DIR"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
INPUT_CALIB_OUT_DIR="$INPUT_CALIB_OUT_DIR" python3 input_calibration.py
echo "[input-calib] script exit $?"
