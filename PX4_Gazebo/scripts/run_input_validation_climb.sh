#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic + ros_gz bridges + the CLIMB-DIRECTION
# thrust-map validation flight (apps/record_input_validation_climb.py).
#
# Purpose: get a real max-deliverable-thrust number from actual SITL
# behavior instead of extrapolating the small-signal input-cal fit (see
# that script's module docstring). Ascends thrust_norm from just above
# hover toward 1.0 and records achieved vertical accel at each step.
#
# Cleanup/start_bg boilerplate copied verbatim from run_aruco_landing.sh
# (docs/SH_REFERENCE.md §2) -- do not diverge without updating both.

set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
mkdir -p "$LOG_DIR"
WORLD="${WORLD:-aruco}"

HEADLESS="${HEADLESS:-}"
if [ -n "$HEADLESS" ]; then
  echo "[run] HEADLESS mode: Gazebo will run server-only, QGroundControl skipped."
fi

declare -a PIDS=()
declare -A NAMES=()

cleanup() {
  echo
  echo "[run] Shutting down background processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      name="${NAMES[$pid]:-pid$pid}"
      echo "[run]   killing $name (pid $pid + group)"
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
    fi
  done
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f "parameter_bridge.*world/$WORLD" 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  pkill -9 -f 'QGroundControl' 2>/dev/null || true
  echo "[run] done."
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"
  echo "[run] launching $name (log: $logfile)"
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

# 0) Clean PX4 param state (see run_aruco_landing.sh's identical step for why).
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/0/parameters.bson"        2>/dev/null
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/0/parameters_backup.bson" 2>/dev/null

# 1) uXRCE-DDS agent
start_bg microxrce MicroXRCEAgent udp4 -p 8888
sleep 1

# 2) PX4 SITL + Gazebo
if [ -n "$HEADLESS" ]; then
  echo "[run] starting PX4 SITL (Gazebo Harmonic with Qt offscreen platform)..."
  EXTRA_ENV_HEADLESS="QT_QPA_PLATFORM=offscreen"
  PX4_DAEMON="-d"
else
  echo "[run] starting PX4 SITL (Gazebo Harmonic window should open momentarily)..."
  EXTRA_ENV_HEADLESS=""
  PX4_DAEMON=""
fi
setsid env $EXTRA_ENV_HEADLESS \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD="$WORLD" \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 $PX4_DAEMON -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
PIDS+=("$PX4_PID")
NAMES[$PX4_PID]="px4_sitl"

echo -n "[run] waiting for Gazebo to come up "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock"; do
  sleep 1
  echo -n "."
  WAITED=$((WAITED + 1))
  if [ "$WAITED" -gt 60 ]; then
    echo " timed out!"
    tail -n 30 "$LOG_DIR/px4_sitl.log" || true
    exit 1
  fi
done
echo " up after ${WAITED}s."

# 3) ros_gz bridges — clock + pose only (this test needs no camera image).
start_bg bridge_clock ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock" \
  --ros-args -r "/world/$WORLD/clock:=/clock"

start_bg bridge_pose ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V" \
  --ros-args -r "/world/$WORLD/pose/info:=/pose"

# 4) QGroundControl (heartbeat only, offscreen, non-fatal)
if [ -x "$HOME/Downloads/QGroundControl.AppImage" ]; then
  echo "[run] launching qgc (log: $LOG_DIR/qgc.log) — non-fatal"
  setsid env QT_QPA_PLATFORM=offscreen "$HOME/Downloads/QGroundControl.AppImage" \
      > "$LOG_DIR/qgc.log" 2>&1 &
  qgc_pid=$!
  PIDS+=("$qgc_pid")
  NAMES[$qgc_pid]="qgc"
fi

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

echo "[run] short settle (5s) before launching the climb-validation script..."
sleep 5

# 5) Run the climb-validation flight in the foreground.
echo
echo "[run] All background processes up. Starting record_input_validation_climb.py..."
echo "[run] Press Ctrl+C to abort and clean everything up."
echo
cd "$SCRIPT_DIR/.."
# shellcheck disable=SC1091
source "$VENV/bin/activate"
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
  set -u
fi
PY_OUT="$LOG_DIR/input_validation_climb.out"
# MAX_DUR_S default 60s (script) + settle 3s + arm/takeoff/land overhead;
# 120s gives ~2x headroom. Override PY_TIMEOUT_S / VAL_CLIMB_MAX_S together
# if you widen the staircase (more steps or longer STEP_S).
PY_TIMEOUT_S="${PY_TIMEOUT_S:-120}"
timeout --kill-after=10 "$PY_TIMEOUT_S" python3 apps/record_input_validation_climb.py 2>&1 | tee "$PY_OUT"
PY_EXIT=${PIPESTATUS[0]}
if [ "$PY_EXIT" = "124" ] || [ "$PY_EXIT" = "137" ]; then
  echo "[run] DETECTED: record_input_validation_climb.py hung past ${PY_TIMEOUT_S}s wall-clock."
  exit 42
fi
exit "$PY_EXIT"
