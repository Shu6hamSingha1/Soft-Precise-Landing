#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic + ros_gz bridges + the landing
# controller for the MOVING-target (rover) scenario, in one shot.
# Background processes are tracked so Ctrl+C in the foreground (the
# landing_test.py run) cleans up everything.
#
# Scenario: moving rover_aruco target (airframe 4022, -i 1) + x500_mono_cam_down
# drone (airframe 4014, -i 0), both in the `rover` world. Adapted from the
# stationary launcher scripts/run_aruco_landing.sh and tips.txt steps 1-7
# (moving-target variant).
#
# ============================== STATUS / PREREQUISITES ======================
# Infra blockers RESOLVED 2026-07-01 (see docs/MOVING_TARGET_PREP.md):
#   * Model install: `rover_aruco` lived only in ~/.gazebo/models and PX4's gz
#     could not find it -> installed a copy under
#     ~/PX4-Autopilot/Tools/simulation/gz/models/rover_aruco/.
#   * SDF version: the model declared `<sdf version='1.0'>`, which Gazebo
#     Harmonic (SDF 1.11) cannot convert -> bumped model.sdf + model.config to
#     1.9 in BOTH ~/.gazebo/models/rover_aruco/ and the PX4 copy. Rover now
#     spawns as `rover_aruco_1`.
#   * Pose topic + indices: bridge the FULL /world/rover/pose/info (stable order
#     [ground_plane, rover_aruco_1, x500...] -> target=1, UAV=2, same as aruco),
#     NOT dynamic_pose/info (variable membership -> 375 m IC error). Details below.
#
# STILL OPEN before a meaningful rover landing (see docs/MOVING_TARGET_PREP.md):
#   1. Yaw calibration (cal_s[3]) — inert for the stationary square marker,
#      ACTIVE for a moving/turning target.
#   2. Rover motion source — there is no trajectory plugin / speed knob today.
#      Motion must be commanded externally on -i 1 (QGC mission / MAVLink
#      offboard / manual). See ROVER_DRIVE below (placeholder, default OFF).
#      A stationary rover already exercises the full stack as a first baseline.
# ============================================================================

set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VENV="${VENV:-$HOME/ws/scripts/env2025}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
mkdir -p "$LOG_DIR"

WORLD="rover"

# Pose topic: use the FULL /world/rover/pose/info (below), NOT dynamic_pose/info.
# dynamic_pose/info only carries entities that MOVED that step, so its PoseArray
# membership/order varies frame-to-frame -> fixed indices there point at random
# links during flight (verified 2026-07-01: caused a 375 m IC pos_err). pose/info
# publishes the full pose graph every step (~54 Hz, same rate) with STABLE order
# [ground_plane, rover_aruco_1, x500_mono_cam_down_0, ...links...] -> exactly the
# stationary-aruco layout: target=poses[1], UAV=poses[2]. So the gz_subscriber
# defaults (2/1) already match; export explicitly for clarity.
# The chase_cam (CHASE_CAM=1) is a static WORLD model, so it appears in pose/info
# BEFORE the spawned vehicles: [ground_plane, chase_cam, rover_aruco_1, x500...]
# -> it shifts the vehicle indices by +1 (target 1->2, UAV 2->3). Without this
# the controller reads the chase camera as the target and the rover as the UAV
# (12.35 m frozen IC error / garbage control -> fly-away). Verified 2026-07-02.
if [ "${CHASE_CAM:-0}" = "1" ]; then
  export POSE_IDX_TARGET="${POSE_IDX_TARGET:-2}"
  export POSE_IDX_UAV="${POSE_IDX_UAV:-3}"
else
  export POSE_IDX_TARGET="${POSE_IDX_TARGET:-1}"
  export POSE_IDX_UAV="${POSE_IDX_UAV:-2}"
fi

# GT-feedback marker mount offset: the rover carries the ArUco marker +0.50 m
# above its base (rover_aruco SDF), unlike the flat stationary aruco marker. So
# the GT camera-to-marker depth needs this offset here (gt_feedback default is
# 0.0 for flat-marker worlds). Camera offset (0.20 m) is universal -> its default.
export PLASMC_GT_MARKER_DZ="${PLASMC_GT_MARKER_DZ:-0.5}"

# HEADLESS=1 -> no Gazebo GUI client, no QGroundControl.
HEADLESS="${HEADLESS:-}"
if [ -n "$HEADLESS" ]; then
  echo "[run] HEADLESS mode: Gazebo will run server-only, QGC skipped."
fi

# ROVER_MOTION: 1 = drive the rover along a traj_Gen trajectory via MAVSDK
# offboard (apps/rover_drive.py on udp://:14541); 0 = rover sits still (a valid
# first baseline for the full stack). Trajectory/speed via ROVER_TRAJ /
# ROVER_SPEED_MULT (see apps/rover_drive.py). ROVER_DRIVE overrides with a
# fully custom command if set.
ROVER_MOTION="${ROVER_MOTION:-0}"
ROVER_DRIVE="${ROVER_DRIVE:-}"
if [ -z "$ROVER_DRIVE" ] && [ "$ROVER_MOTION" = "1" ]; then
  ROVER_DRIVE="'$VENV/bin/python3' '$SCRIPT_DIR/../apps/rover_drive.py'"
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
  # Belt and suspenders: matching strays (both PX4 instances, gz, bridges, QGC).
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f "parameter_bridge.*world/$WORLD" 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  pkill -9 -f 'QGroundControl' 2>/dev/null || true
  # The standalone gz server (spawned by the -i 1 rover px4) has repeatedly
  # OUTLIVED the single pkill above; if it survives, the NEXT run attaches to
  # this stale world -> frozen/garbage pose -> 375 m IC error. Verify it's gone
  # and re-kill until it is (bounded), so retries start from a clean slate.
  for _ in 1 2 3 4 5; do
    pgrep -f 'gz sim' >/dev/null 2>&1 || break
    pkill -9 -f 'gz sim' 2>/dev/null || true
    sleep 1
  done
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

# 0a) Stale-server guard: a gz server / px4 left over from a previous (crashed
# or incompletely-cleaned) run poisons this one — the new px4 attaches to the
# old world -> wrong/frozen pose -> 375 m IC error. If anything is already up,
# clear it before starting so every run (and every retry) boots clean.
if pgrep -f 'gz sim' >/dev/null 2>&1 || gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock"; then
  echo "[run] stale sim detected (gz server / $WORLD clock already up) — clearing..."
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    pgrep -f 'gz sim' >/dev/null 2>&1 || break
    pkill -9 -f 'gz sim' 2>/dev/null || true
    sleep 1
  done
  sleep 1
fi

# 0b) Reset PX4 SITL persistent param state for BOTH instances (rootfs/0 and
# rootfs/1) so each run boots from airframe defaults.
for inst in 0 1; do
  rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/$inst/parameters.bson"        2>/dev/null
  rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/$inst/parameters_backup.bson" 2>/dev/null
done

# 1) uXRCE-DDS agent
start_bg microxrce MicroXRCEAgent udp4 -p 8888
sleep 1

# 2a) PX4 SITL instance -i 1 = the ROVER target (spawns the gz sim server +
# the `rover` world). This MUST start first; the UAV instance attaches to the
# already-running gz sim via PX4_GZ_STANDALONE=1.
# Headless: offscreen Qt AND px4 daemon mode (-d, no pxh shell). Without -d a
# headless px4 (no TTY) spams the pxh prompt + clear-line escapes to its log at
# ~GB/min; -d keeps the INFO logs we grep ("Ready for takeoff", lockstep race)
# but drops the interactive shell. GUI mode keeps pxh for interactive debugging.
if [ -n "$HEADLESS" ]; then
  EXTRA_ENV_HEADLESS="QT_QPA_PLATFORM=offscreen"
  PX4_DAEMON="-d"
else
  EXTRA_ENV_HEADLESS=""
  PX4_DAEMON=""
fi
echo "[run] starting PX4 SITL rover (-i 1, airframe 4022, world $WORLD)..."
setsid env $EXTRA_ENV_HEADLESS \
  PX4_SYS_AUTOSTART=4022 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=rover_aruco \
  PX4_GZ_WORLD="$WORLD" \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 $PX4_DAEMON -i 1" \
  > "$LOG_DIR/px4_rover.log" 2>&1 &
ROVER_PID=$!
PIDS+=("$ROVER_PID")
NAMES[$ROVER_PID]="px4_rover"

# Wait for Gazebo /world/rover/clock to come up before launching the UAV.
echo -n "[run] waiting for Gazebo (rover) to come up "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q "/world/$WORLD/clock"; do
  sleep 1
  echo -n "."
  WAITED=$((WAITED + 1))
  if [ "$WAITED" -gt 60 ]; then
    echo " timed out!"
    echo "[run] PX4 rover did not bring up /world/$WORLD/clock in 60s."
    tail -n 30 "$LOG_DIR/px4_rover.log" || true
    exit 1
  fi
done
echo " up after ${WAITED}s."

# 2b) PX4 SITL instance -i 0 = the UAV (x500_mono_cam_down), STANDALONE so it
# attaches to the rover instance's gz sim rather than spawning a second one.
echo "[run] starting PX4 SITL UAV (-i 0, airframe 4014, standalone)..."
setsid env $EXTRA_ENV_HEADLESS \
  PX4_GZ_STANDALONE=1 \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="1,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD="$WORLD" \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 $PX4_DAEMON -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
PX4_PID=$!
PIDS+=("$PX4_PID")
NAMES[$PX4_PID]="px4_sitl"

# Wait for the UAV camera image topic to register (confirms the standalone
# UAV model spawned and its sensors initialized).
echo -n "[run] waiting for UAV camera topic "
WAITED=0
while ! gz topic -l 2>/dev/null | grep -q "x500_mono_cam_down_0/link/camera_link/sensor/imager/image"; do
  sleep 1
  echo -n "."
  WAITED=$((WAITED + 1))
  if [ "$WAITED" -gt 60 ]; then
    echo " timed out!"
    echo "[run] UAV camera topic never registered in 60s."
    tail -n 30 "$LOG_DIR/px4_sitl.log" || true
    exit 1
  fi
done
echo " up after ${WAITED}s."

# 3) ros_gz bridges (rover world; pose is the FULL pose/info — see note above).
start_bg bridge_clock ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock" \
  --ros-args -r "/world/$WORLD/clock:=/clock"

start_bg bridge_pose ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V" \
  --ros-args -r "/world/$WORLD/pose/info:=/pose"

start_bg bridge_image ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  --ros-args -r "/world/$WORLD/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image:=/image"

# 3b) CHASE_CAM=1 -> bridge the static chase camera (external view, from the
# chase_cam model in rover.sdf) and start a recorder that writes an mp4 to
# test_data/Test_Videos/chase_<ts>.mp4 for RECORD_S seconds. Independent of the
# UAV's down-cam recording (IMG_RECORD).
if [ "${CHASE_CAM:-0}" = "1" ]; then
  CHASE_TOPIC_GZ="/world/$WORLD/model/chase_cam/link/link/sensor/chase/image"
  start_bg bridge_chase ros2 run ros_gz_bridge parameter_bridge \
    "${CHASE_TOPIC_GZ}@sensor_msgs/msg/Image@gz.msgs.Image" \
    --ros-args -r "${CHASE_TOPIC_GZ}:=/chase_image"
  # Recorder needs the venv (cv2) on top of the ROS env already in scope.
  start_bg chase_rec bash -c "source '$VENV/bin/activate'; \
    if [ -f \"\$HOME/ros2_ws/install/setup.bash\" ]; then set +u; source \"\$HOME/ros2_ws/install/setup.bash\"; set -u; fi; \
    exec python3 '$SCRIPT_DIR/../apps/record_chase.py'"
fi

# 4) QGroundControl — heartbeat for the UAV's preflight "GCS connected" check.
if [ -x "$HOME/Downloads/QGroundControl.AppImage" ]; then
  echo "[run] launching qgc (log: $LOG_DIR/qgc.log) — non-fatal"
  setsid env QT_QPA_PLATFORM=offscreen "$HOME/Downloads/QGroundControl.AppImage" \
      > "$LOG_DIR/qgc.log" 2>&1 &
  qgc_pid=$!
  PIDS+=("$qgc_pid")
  NAMES[$qgc_pid]="qgc"
fi

# 4b) Wait for the UAV preflight to settle.
echo -n "[run] waiting for UAV preflight to clear "
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

# 4c) Optional: start the rover motion source on -i 1 (MAVSDK offboard, driving
# a traj_Gen trajectory). Set ROVER_MOTION=1 (or a custom ROVER_DRIVE command).
if [ -n "$ROVER_DRIVE" ]; then
  echo "[run] starting rover motion source: $ROVER_DRIVE"
  # Run inside the venv-activated environment (mavsdk); this shell has already
  # sourced $VENV below step 5, but the bg child starts before that — activate
  # explicitly so rover_drive.py finds mavsdk.
  start_bg rover_drive bash -c "source '$VENV/bin/activate'; $ROVER_DRIVE"
else
  echo "[run] NOTE: rover motion OFF (ROVER_MOTION!=1) — rover sits still (valid baseline)."
fi

echo "[run] short settle (5s) before launching landing_test (arm() polls is_armable)..."
sleep 5

# 5) Run the landing controller in the foreground (connects to the UAV on
#    udp://:14540). Ctrl+C triggers cleanup of all background processes.
echo
echo "[run] All background processes up. Starting landing_test.py..."
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
PY_OUT="$LOG_DIR/landing_test.out"
PY_TIMEOUT_S="${PY_TIMEOUT_S:-180}"
PY_SCRIPT="${PY_SCRIPT:-apps/landing_test.py}"
LANDING_AUTOSAVE=1 timeout --kill-after=10 "$PY_TIMEOUT_S" python3 "$PY_SCRIPT" 2>&1 | tee "$PY_OUT"
PY_EXIT=${PIPESTATUS[0]}
if [ "$PY_EXIT" = "124" ] || [ "$PY_EXIT" = "137" ]; then
  echo "[run] DETECTED: landing_test.py hung past ${PY_TIMEOUT_S}s wall-clock — treating as failure."
  exit 42
fi
if grep -qE 'is_armable did not go True|arm\(\) failed even after is_armable' "$PY_OUT" 2>/dev/null; then
  echo "[run] DETECTED: PX4 lockstep race did not recover (is_armable timeout)."
  exit 42
fi
if grep -q 'IC convergence timeout' "$PY_OUT" 2>/dev/null; then
  echo "[run] DETECTED: IC convergence timeout (PX4 SITL didn't settle)."
  exit 42
fi
exit "$PY_EXIT"
