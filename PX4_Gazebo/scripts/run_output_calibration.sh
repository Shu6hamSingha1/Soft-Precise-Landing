#!/usr/bin/env bash
# Run record_output_calibration.py headlessly. Brings up MicroXRCEAgent + PX4 SITL
# (Qt offscreen) + 3 ros_gz bridges + QGC (offscreen, for GCS-link
# preflight), waits for PX4 health, then runs the calibration sweep.
set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
mkdir -p "$LOG_DIR"

# Every run lands in its own timestamped folder so we keep history (matches
# the original ~/ws/scripts/soft_precise_landing/record_output_calibration.py
# convention which used time.ctime()-style names). A `latest` symlink in the
# same parent tracks the most recent run for analyze/validate scripts.
# CALIB_PARENT overridable so a VALIDATION run routes elsewhere (e.g.
# CALIB_PARENT=$PWD/validation_data/output_multisine CALIB_MODE=multisine ...) without
# contaminating the calibration set the derive tools scan.
CALIB_PARENT="${CALIB_PARENT:-$SCRIPT_DIR/../calibration_data/output}"
mkdir -p "$CALIB_PARENT"
TIMESTAMP="$(date '+%a %b %e %H-%M-%S %Y')"
export CALIB_OUT_DIR="${CALIB_OUT_DIR:-$CALIB_PARENT/$TIMESTAMP}"
mkdir -p "$CALIB_OUT_DIR"
echo "[calib] output dir: $CALIB_OUT_DIR"

declare -a PIDS=()

cleanup() {
  echo
  echo "[calib] cleanup..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  done
  sleep 1
  # Hardier kill — record_output_calibration.py can hang past SIGTERM if MAVSDK
  # is awaiting an offboard ACK that PX4 won't send (post-failsafe).
  pkill -9 -f 'record_output_calibration.py' 2>/dev/null || true
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco' 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  pkill -9 -f 'QGroundControl' 2>/dev/null || true
  echo "[calib] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[calib] launching $name"
  setsid "$@" > "$logfile" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  sleep 0.3
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "[calib] $name died:"
    tail -n 20 "$logfile" || true
    exit 1
  fi
}

start_bg microxrce MicroXRCEAgent udp4 -p 8888
sleep 1

echo "[calib] starting PX4 SITL (headless Qt)..."
# Force a clean default-param boot every time. A parameters.bson left truncated
# by a kill -9 (SITL cleanup) loads corrupt -> EKF2/baro get no valid data ->
# "ekf2 missing data" preflight fail -> is_armable never trues (systematic, and
# survives reboots since it's a file on disk). The landing script clears these
# for the same reason (run_aruco_landing.sh); the cal path was missing it.
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/0/parameters.bson"        2>/dev/null
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/0/parameters_backup.bson" 2>/dev/null
# Truncate log so the preflight-wait grep below only matches THIS run.
: > "$LOG_DIR/px4_sitl.log"
setsid env QT_QPA_PLATFORM=offscreen \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 -d -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
PIDS+=($!)

echo -n "[calib] waiting for Gazebo "
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
  # Non-fatal: QGC FUSE-mount failures (stale /tmp/.mount_QGroun* leftovers
  # after many SITL runs) would otherwise abort the entire calibration
  # attempt via start_bg's exit 1. QGC is only here for GCS heartbeat.
  echo "[calib] launching qgc (non-fatal)"
  setsid env QT_QPA_PLATFORM=offscreen "$HOME/Downloads/QGroundControl.AppImage" \
      > "$LOG_DIR/qgc.log" 2>&1 &
  PIDS+=("$!")
fi

echo -n "[calib] waiting for PX4 preflight "
WAITED=0
while ! strings "$LOG_DIR/px4_sitl.log" 2>/dev/null | grep -q 'Ready for takeoff' \
      && [ "$WAITED" -lt 30 ]; do
  sleep 2; echo -n "."
  WAITED=$((WAITED + 2))
done
echo " (${WAITED}s)"

# "Ready for takeoff" is printed before EKF heading-estimate / gyro-bias checks settle,
# so the first arm() attempt frequently gets COMMAND_DENIED. Give PX4 an extra 8 s
# to settle. (flight_controller.py also retries arm internally; this just makes
# the first attempt usually succeed.)
echo "[calib] settling 20s before arm (heading-estimate convergence)..."
sleep 20

echo "[calib] running record_output_calibration.py (data -> $CALIB_OUT_DIR)..."
cd "$SCRIPT_DIR/.."
# shellcheck disable=SC1091
source "$VENV/bin/activate"
# CALIB_APP overridable so this launcher can also fly the VALIDATION app, e.g.
#   CALIB_APP=apps/record_output_validation.py VALIDATION_PROFILE=multisine \
#   CALIB_PARENT=$PWD/validation_data/output_multisine bash scripts/run_output_calibration.sh
CALIB_OUT_DIR="$CALIB_OUT_DIR" python3 "${CALIB_APP:-apps/record_output_calibration.py}"
echo "[calib] script exit $?"
