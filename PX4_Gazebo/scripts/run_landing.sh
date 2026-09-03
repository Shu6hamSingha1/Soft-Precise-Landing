#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic + ros_gz bridges + the landing
# controller in one shot. Background processes are tracked so Ctrl+C in
# the foreground (the landing_test.py run) cleans up everything.
#
# Scenario: stationary ArUco target, x500_mono_cam_down drone.
# Adapted from tips.txt steps 1–7.
#
# WORLD overridable (2026-08-11, matches run_output_calibration.sh's pattern)
# so this launcher can also drive a landing test in the cross-marker world,
# e.g.: WORLD=cross_marker MARKER_TYPE=cross bash run_aruco_landing.sh
# (MARKER_TYPE=cross is read by controller.py, selecting CrossMarkerNode --
# see its own module comment. Model name (x500_mono_cam_down) is unaffected;
# only the world/$WORLD topic-path prefix changes.)

set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
mkdir -p "$LOG_DIR"
WORLD="${WORLD:-aruco}"

# HEADLESS=1 -> no Gazebo GUI client, no QGroundControl. PX4's gz launch
# script honors the HEADLESS env var.
HEADLESS="${HEADLESS:-}"
if [ -n "$HEADLESS" ]; then
  echo "[run] HEADLESS mode: Gazebo will run server-only, QGC skipped."
fi

# CHASE_CAM=1 -> bridge the static chase camera (external view, from the
# chase_cam sensor on the ground_plane link — see world .sdf) and start a
# recorder that writes an mp4 to test_data/Test_Videos/chase_<ts>.mp4 for
# RECORD_S seconds. Ported from run_rover_landing.sh's CHASE_CAM block;
# gate/stop files are read generically by controller.py/landing_test.py.
if [ "${CHASE_CAM:-0}" = "1" ]; then
  export CHASE_GATE_FILE="${CHASE_GATE_FILE:-$LOG_DIR/chase_gate.flag}"
  rm -f "$CHASE_GATE_FILE" 2>/dev/null
  export CHASE_STOP_FILE="${CHASE_STOP_FILE:-$LOG_DIR/chase_stop.flag}"
  rm -f "$CHASE_STOP_FILE" 2>/dev/null
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

# 0) Reset PX4 SITL persistent param state. Without this, params set by
# previous runs (via airframe init, MAVSDK set_param, or pxh> param set)
# persist across launches and can cause hard-to-debug behaviour — e.g.
# carrying a too-permissive EKF2_ABL_LIM into a fresh test, or holding
# stale safety thresholds. Force a clean default-param boot every time.
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/0/parameters.bson"      2>/dev/null
rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/0/parameters_backup.bson" 2>/dev/null

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
  # -d (daemon, no pxh shell): a headless px4 (no TTY) otherwise spams the pxh
  # prompt to its log at ~GB/min. -d keeps the INFO lines we grep. GUI mode keeps
  # the interactive shell.
  PX4_DAEMON="-d"
else
  echo "[run] starting PX4 SITL (Gazebo Harmonic window should open momentarily)..."
  EXTRA_ENV_HEADLESS=""
  PX4_DAEMON=""
fi
# QT_PRELOAD (2026-09-03): gz sim cannot start on this machine without it --
#   libgz-sim8-gz.so: /lib/.../libQt5Quick.so.5: undefined symbol
#   _ZNK16QDoubleValidator8validateER7QStringRi, version Qt_5
# PX4 then sits in "Waiting for Gazebo world..." until it times out, and EVERY rep reports
# landed=NO with no data -- which looks exactly like the change under test broke the
# controller. Cost a 2 h / 40-rep A/B before it was diagnosed. The project's one-off
# harnesses (test_data/Rover_AB_harness/record_robustness_set.sh, spanrescue_ab.sh) already
# carried this; the canonical launcher did not. Set QT_PRELOAD= (empty) to opt out.
# Applies in BOTH modes -- the clash is gz's, not headless-specific.
QT_PRELOAD="${QT_PRELOAD-/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5}"
if [ -n "$QT_PRELOAD" ] && [ ! -e "$QT_PRELOAD" ]; then
  echo "[run] WARN: QT_PRELOAD=$QT_PRELOAD not found — gz may fail to start (Qt5 symbol clash)."
  QT_PRELOAD=""
fi
[ -n "$QT_PRELOAD" ] && EXTRA_ENV_HEADLESS="$EXTRA_ENV_HEADLESS LD_PRELOAD=$QT_PRELOAD"

# Announce what is ACTUALLY being flown. The script's legacy name says "aruco" while WORLD /
# MARKER_TYPE are env-driven, which has caused real doubt about whether an ArUco run happened
# when it had not (2026-09-03). Make the answer unambiguous in every log.
echo "[run] FLYING: WORLD=$WORLD  MARKER_TYPE=${MARKER_TYPE:-<unset:aruco>}"

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

# Wait for the Gazebo /clock topic to come up before bridging.
echo -n "[run] waiting for Gazebo to come up "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock"; do
  sleep 1
  echo -n "."
  WAITED=$((WAITED + 1))
  if [ "$WAITED" -gt 60 ]; then
    echo " timed out!"
    echo "[run] PX4 SITL did not bring up Gazebo's /world/$WORLD/clock in 60s."
    echo "[run] Last PX4 SITL log lines:"
    tail -n 30 "$LOG_DIR/px4_sitl.log" || true
    exit 1
  fi
done
echo " up after ${WAITED}s."

# 3) ros_gz bridges
start_bg bridge_clock ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock" \
  --ros-args -r "/world/$WORLD/clock:=/clock"

start_bg bridge_pose ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V" \
  --ros-args -r "/world/$WORLD/pose/info:=/pose"

start_bg bridge_image ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  --ros-args -r "/world/$WORLD/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/image"

if [ "${CHASE_CAM:-0}" = "1" ]; then
  CHASE_TOPIC_GZ="/world/$WORLD/model/ground_plane/link/link/sensor/chase/image"
  start_bg bridge_chase ros2 run ros_gz_bridge parameter_bridge \
    "${CHASE_TOPIC_GZ}@sensor_msgs/msg/Image@gz.msgs.Image" \
    --ros-args -r "${CHASE_TOPIC_GZ}:=/chase_image"
  start_bg chase_rec bash -c "source '$VENV/bin/activate'; \
    if [ -f \"\$HOME/ros2_ws/install/setup.bash\" ]; then set +u; source \"\$HOME/ros2_ws/install/setup.bash\"; set -u; fi; \
    exec python3 '$SCRIPT_DIR/../apps/record_chase.py'"
fi

# 4) QGroundControl — always launch if available (its heartbeats satisfy
# PX4's "No connection to ground control station" preflight check). QGC
# is always launched OFFSCREEN — the user never interacts with its GUI,
# it's purely there for the heartbeat. This keeps the desktop clean even
# when Gazebo's GUI is visible.
if [ -x "$HOME/Downloads/QGroundControl.AppImage" ]; then
  # Launch QGC non-fatally: FUSE-mount failures (stale /tmp/.mount_QGroun*
  # leftovers after many SITL runs, fusermount "Operation not permitted")
  # would otherwise abort the whole run via start_bg's exit 1. QGC is only
  # here for the heartbeat — non-critical when offboard is the mode.
  echo "[run] launching qgc (log: $LOG_DIR/qgc.log) — non-fatal"
  setsid env QT_QPA_PLATFORM=offscreen "$HOME/Downloads/QGroundControl.AppImage" \
      > "$LOG_DIR/qgc.log" 2>&1 &
  qgc_pid=$!
  PIDS+=("$qgc_pid")
  NAMES[$qgc_pid]="qgc"
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

# "Ready for takeoff" prints before EKF heading-estimate settles, so the
# first arm() can be COMMAND_DENIED. flight_controller.arm_and_takeoff
# polls telemetry.health() for is_armable (60s budget) before arming,
# so we only need a short structural wait here to let the bridges/QGC
# settle into the loop.
echo "[run] short settle (5s) before launching landing_test (arm() polls is_armable)..."
sleep 5

# 5) Run the landing controller in the foreground.
#    Ctrl+C here triggers the cleanup trap and kills all background processes.
echo
echo "[run] All background processes up. Starting landing_test.py..."
echo "[run] Press Ctrl+C to abort and clean everything up."
echo
cd "$SCRIPT_DIR/.."
# shellcheck disable=SC1091
source "$VENV/bin/activate"
# Overlay px4_msgs (for CMD_TRANSPORT=dds, the low-latency uXRCE-DDS rate path).
# Additive over /opt/ros/humble (adds px4_msgs + typesupport); does not change
# rclpy. No-op if absent.
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  set +u   # ros2_ws setup.bash references COLCON_TRACE unbound; don't trip set -u
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
  set -u
fi
PY_OUT="$LOG_DIR/landing_test.out"
# Hard wall-clock timeout to prevent rare hangs (PX4 SITL crash without
# clean exit, MAVSDK deadlock, etc.) from blocking sweep loops indefinitely.
# 180 s is generous: arm settle (~5 s) + IC convergence (up to 30 s) +
# warmup (0.1 s) + landing flight (4-22 s) + cleanup ≈ ~70 s typical max.
# 180 s gives 2.5× headroom. Hang triggers exit code 124; treated as
# lockstep-race retry signal below.
PY_TIMEOUT_S="${PY_TIMEOUT_S:-180}"
PY_SCRIPT="${PY_SCRIPT:-apps/landing_test.py}"
LANDING_AUTOSAVE=1 timeout --kill-after=10 "$PY_TIMEOUT_S" python3 "$PY_SCRIPT" 2>&1 | tee "$PY_OUT"
PY_EXIT=${PIPESTATUS[0]}
# coreutils timeout returns 124 (or 137 with SIGKILL after grace) on timeout
if [ "$PY_EXIT" = "124" ] || [ "$PY_EXIT" = "137" ]; then
  echo "[run] DETECTED: landing_test.py hung past ${PY_TIMEOUT_S}s wall-clock — treating as failure."
  exit 42
fi

# Detect the IMU-timestamp / lockstep race documented at
# discuss.px4.io/t/px4-sitl-multi-vehicle-gazebo-imu-timing-errors/33268.
# When PX4's gz_bridge subscribes to IMU before lockstep has synced with
# the Gazebo clock, accel/gyro is published with timestamp=0, which cascades
# into "ekf2 missing data" → is_armable stays False permanently.
# flight_controller.arm_and_takeoff polls is_armable for 60s; if it times
# out it raises "is_armable did not go True within 60s". Exit code 42
# signals the retry wrapper to restart the whole stack.
if grep -qE 'is_armable did not go True|arm\(\) failed even after is_armable' "$PY_OUT" 2>/dev/null; then
  echo "[run] DETECTED: PX4 lockstep race did not recover (is_armable timeout)."
  echo "[run] Signaling retry with exit code 42."
  exit 42
fi
# IC convergence timeout — drone never settled at the IC pose during the 15s
# pre-controller hover loop. PX4 SITL flakiness; let the retry wrapper boot
# a fresh stack rather than fly garbage from a moving start.
if grep -q 'IC convergence timeout' "$PY_OUT" 2>/dev/null; then
  echo "[run] DETECTED: IC convergence timeout (PX4 SITL didn't settle)."
  echo "[run] Signaling retry with exit code 42."
  exit 42
fi
exit "$PY_EXIT"
