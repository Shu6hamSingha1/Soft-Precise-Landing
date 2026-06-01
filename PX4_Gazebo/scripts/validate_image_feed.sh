#!/usr/bin/env bash
# Bring up the SITL infrastructure (headless) and run validate_image.py
# to confirm: image dimensions, encoding, content stats, and ArUco
# detectability.
set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
mkdir -p "$LOG_DIR"

declare -a PIDS=()

cleanup() {
  echo
  echo "[validate] cleanup..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  done
  sleep 1
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco' 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  echo "[validate] done."
}
trap cleanup EXIT INT TERM

echo "[validate] starting MicroXRCEAgent..."
setsid MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/microxrce.log" 2>&1 &
PIDS+=($!)
sleep 1

echo "[validate] starting PX4 SITL (headless Qt)..."
setsid env QT_QPA_PLATFORM=offscreen \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
PIDS+=($!)

IMG_TOPIC='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
echo -n "[validate] waiting for image topic "
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

echo "[validate] starting image bridge..."
setsid ros2 run ros_gz_bridge parameter_bridge \
  "${IMG_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image" \
  --ros-args -r "${IMG_TOPIC}:=/image" \
  > "$LOG_DIR/bridge_image.log" 2>&1 &
PIDS+=($!)
sleep 3   # let the bridge settle and a few frames flow

echo
echo "[validate] running validate_image.py ..."
echo "------------------------------------------------------------"
cd "$SCRIPT_DIR/.."
# shellcheck disable=SC1091
source "$VENV/bin/activate"
python3 apps/validate_image.py
PY_RC=$?
echo "------------------------------------------------------------"
echo "[validate] validate_image.py exit code: $PY_RC"
exit "$PY_RC"
