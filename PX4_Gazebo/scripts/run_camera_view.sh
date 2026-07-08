#!/usr/bin/env bash
# Bring up the SITL infrastructure (headless PX4, GUI-capable Gazebo
# rendering) and display a live /image feed with FoV gridlines overlaid.
# Runs until the user closes the image window (q/Esc) or Ctrl+C.
set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
mkdir -p "$LOG_DIR"

declare -a PIDS=()
declare -A NAMES=()

cleanup() {
  echo
  echo "[camview] Shutting down background processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      name="${NAMES[$pid]:-pid$pid}"
      echo "[camview]   killing $name (pid $pid + group)"
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
    fi
  done
  pkill -9 -f 'px4_sitl_default/bin/px4'        2>/dev/null || true
  pkill -9 -f 'gz sim'                          2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco'   2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent'                  2>/dev/null || true
  echo "[camview] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[camview] launching $name (log: $logfile)"
  setsid "$@" > "$logfile" 2>&1 &
  local pid=$!
  PIDS+=("$pid"); NAMES[$pid]="$name"
  sleep 0.3
}

start_bg microxrce MicroXRCEAgent udp4 -p 8888
sleep 1

start_bg px4_sitl env QT_QPA_PLATFORM=offscreen \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 -i 0"

IMG_TOPIC='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
echo -n "[camview] waiting for image topic "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q "$IMG_TOPIC"; do
  sleep 1
  echo -n "."
  WAITED=$((WAITED + 1))
  if [ "$WAITED" -gt 60 ]; then
    echo " timed out"
    exit 1
  fi
done
echo " up after ${WAITED}s"

start_bg bridge_image ros2 run ros_gz_bridge parameter_bridge \
  "${IMG_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image" \
  --ros-args -r "${IMG_TOPIC}:=/image"
sleep 3   # let the bridge settle and a few frames flow

echo
echo "[camview] opening live camera view — close the window (q/Esc) or Ctrl+C here to stop."
echo "------------------------------------------------------------"
cd "$SCRIPT_DIR/.."
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python3 apps/camera_view.py
PY_RC=$?
echo "------------------------------------------------------------"
echo "[camview] camera_view.py exit code: $PY_RC"
exit "$PY_RC"
