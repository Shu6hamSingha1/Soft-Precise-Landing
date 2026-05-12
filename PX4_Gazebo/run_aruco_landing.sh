#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic + ros_gz bridges + the landing
# controller in one shot. Background processes are tracked so Ctrl+C in
# the foreground (the landing_test.py run) cleans up everything.
#
# Scenario: stationary ArUco target, x500_mono_cam_down drone.
# Adapted from tips.txt steps 1–7.

set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/run_logs"
mkdir -p "$LOG_DIR"

declare -a PIDS=()
declare -A NAMES=()

cleanup() {
  echo
  echo "[run] Shutting down background processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      name="${NAMES[$pid]:-pid$pid}"
      echo "[run]   killing $name (pid $pid)"
      kill "$pid" 2>/dev/null
    fi
  done
  # Give them a moment, then SIGKILL stragglers
  sleep 1
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null
    fi
  done
  echo "[run] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[run] launching $name (log: $logfile)"
  "$@" > "$logfile" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  NAMES[$pid]="$name"
  sleep 0.3
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[run] $name died immediately; last lines:"
    tail -n 20 "$logfile" || true
    exit 1
  fi
}

# 1) uXRCE-DDS agent
start_bg microxrce MicroXRCEAgent udp4 -p 8888
sleep 1

# 2) PX4 SITL + Gazebo (Gazebo Harmonic window will appear)
echo "[run] starting PX4 SITL (Gazebo Harmonic should open momentarily)..."
(
  cd "$PX4_DIR"
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  ./build/px4_sitl_default/bin/px4 -i 0
) > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
PIDS+=("$PX4_PID")
NAMES[$PX4_PID]="px4_sitl"

# Wait for the Gazebo /clock topic to come up before bridging.
echo -n "[run] waiting for Gazebo to come up "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q '/world/aruco/clock'; do
  sleep 1
  echo -n "."
  WAITED=$((WAITED + 1))
  if [ "$WAITED" -gt 60 ]; then
    echo " timed out!"
    echo "[run] PX4 SITL did not bring up Gazebo's /world/aruco/clock in 60s."
    echo "[run] Last PX4 SITL log lines:"
    tail -n 30 "$LOG_DIR/px4_sitl.log" || true
    exit 1
  fi
done
echo " up after ${WAITED}s."

# 3) ros_gz bridges
start_bg bridge_clock ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  --ros-args -r /world/aruco/clock:=/clock

start_bg bridge_pose ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V \
  --ros-args -r /world/aruco/pose/info:=/pose

start_bg bridge_image ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/image

# 4) (Optional) QGroundControl — launch only if available, in background.
if [ -x "$HOME/Downloads/QGroundControl.AppImage" ]; then
  start_bg qgc "$HOME/Downloads/QGroundControl.AppImage"
fi

# 5) Run the landing controller in the foreground.
#    Ctrl+C here triggers the cleanup trap and kills all background processes.
echo
echo "[run] All background processes up. Starting landing_test.py..."
echo "[run] Press Ctrl+C to abort and clean everything up."
echo
cd "$SCRIPT_DIR"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python3 landing_test.py
