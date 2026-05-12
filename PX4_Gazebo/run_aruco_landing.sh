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

# HEADLESS=1 -> no Gazebo GUI client, no QGroundControl. PX4's gz launch
# script honors the HEADLESS env var.
HEADLESS="${HEADLESS:-}"
if [ -n "$HEADLESS" ]; then
  echo "[run] HEADLESS mode: Gazebo will run server-only, QGC skipped."
fi

declare -a PIDS=()
declare -A NAMES=()

cleanup() {
  echo
  echo "[run] Shutting down background processes..."
  # Each child started via setsid has its own process group whose PGID
  # equals its leader PID. `kill -- -PGID` signals the whole group, so
  # bridge wrappers + bridge binaries all die together.
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      name="${NAMES[$pid]:-pid$pid}"
      echo "[run]   killing $name (pid $pid + group)"
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
  done
  sleep 1
  # Belt and suspenders: anything still alive in any of our groups gets SIGKILL.
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
    fi
  done
  # Catch any matching strays just in case (orphaned grandchildren).
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco' 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  echo "[run] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[run] launching $name (log: $logfile)"
  # exec replaces the subshell so $! captures the actual binary PID,
  # not the throw-away shell. setsid puts it in its own process group
  # so we can kill the whole group on cleanup.
  setsid "$@" > "$logfile" 2>&1 &
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

# 2) PX4 SITL + Gazebo
# In HEADLESS mode we don't use PX4_GZ_STANDALONE (that mode breaks model
# sensor initialization for camera-bearing models). Instead we let PX4
# spawn gz sim normally and suppress the GUI window via Qt's offscreen
# platform plugin — sensors stay alive, no visible window.
if [ -n "$HEADLESS" ]; then
  echo "[run] starting PX4 SITL (Gazebo Harmonic with Qt offscreen platform)..."
  EXTRA_ENV_HEADLESS="QT_QPA_PLATFORM=offscreen"
else
  echo "[run] starting PX4 SITL (Gazebo Harmonic window should open momentarily)..."
  EXTRA_ENV_HEADLESS=""
fi
setsid env $EXTRA_ENV_HEADLESS \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
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

# 4) QGroundControl — always launch if available (its heartbeats satisfy
# PX4's "No connection to ground control station" preflight check). In
# HEADLESS mode it inherits QT_QPA_PLATFORM=offscreen so no window appears.
if [ -x "$HOME/Downloads/QGroundControl.AppImage" ]; then
  if [ -n "$HEADLESS" ]; then
    QT_QPA_PLATFORM=offscreen start_bg qgc "$HOME/Downloads/QGroundControl.AppImage"
  else
    start_bg qgc "$HOME/Downloads/QGroundControl.AppImage"
  fi
fi

# 4b) Wait for PX4 health to settle (EKF2 converge, baro init, GCS connect).
echo -n "[run] waiting for PX4 preflight to clear "
WAITED=0
while ! strings "$LOG_DIR/px4_sitl.log" 2>/dev/null | grep -q 'Ready for takeoff' \
      && [ "$WAITED" -lt 30 ]; do
  sleep 2
  echo -n "."
  WAITED=$((WAITED + 2))
done
if [ "$WAITED" -ge 30 ]; then
  echo " (timeout, proceeding anyway)"
else
  echo " ready after ${WAITED}s."
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
