#!/usr/bin/env bash
# Bring up MicroXRCEAgent + PX4 SITL + Gazebo + image bridge (headless),
# wait for the /image topic to come up, then sample its rate for DURATION
# seconds. Cleans up all background processes on exit.

set -u

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/../run_logs"
DURATION="${DURATION:-15}"
mkdir -p "$LOG_DIR"

declare -a PIDS=()

cleanup() {
  echo
  echo "[measure] cleanup..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  done
  sleep 1
  pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge.*world/aruco' 2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent' 2>/dev/null || true
  echo "[measure] done."
}
trap cleanup EXIT INT TERM

# 1) uXRCE-DDS agent
echo "[measure] starting MicroXRCEAgent..."
setsid MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/microxrce.log" 2>&1 &
PIDS+=($!)
sleep 1

# 2) PX4 SITL + Gazebo (headless via Qt offscreen)
echo "[measure] starting PX4 SITL (headless Gazebo)..."
setsid env QT_QPA_PLATFORM=offscreen \
  PX4_SYS_AUTOSTART=4014 \
  PX4_GZ_MODEL_POSE="0,0" \
  PX4_SIM_MODEL=x500_mono_cam_down \
  PX4_GZ_WORLD=aruco \
  bash -c "cd '$PX4_DIR' && exec ./build/px4_sitl_default/bin/px4 -i 0" \
  > "$LOG_DIR/px4_sitl.log" 2>&1 &
PIDS+=($!)

# Wait for Gazebo and the camera image topic to come up
echo -n "[measure] waiting for image topic "
IMG_TOPIC='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
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

# 3) Image bridge so /image becomes a ROS topic
echo "[measure] starting image bridge..."
setsid ros2 run ros_gz_bridge parameter_bridge \
  "${IMG_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image" \
  --ros-args -r "${IMG_TOPIC}:=/image" \
  > "$LOG_DIR/bridge_image.log" 2>&1 &
PIDS+=($!)
sleep 2

# 4) Measure both Gazebo-side and ROS-side rates in parallel
echo "[measure] sampling /image rate via ROS for ${DURATION}s..."
echo
echo "----- Gazebo native topic rate (gz topic --hz) -----"
timeout "$DURATION" gz topic --echo --topic "$IMG_TOPIC" 2>/dev/null \
  | awk 'BEGIN{n=0; t0=0} /^header/{n++; if(t0==0) t0=systime(); t1=systime()} END{dt=(t1-t0); if(dt>0 && n>1) printf("gz received %d messages over %.1fs -> %.2f Hz\n", n, dt, (n-1)/dt); else print "no data"}' &
GZ_PID=$!

echo
echo "----- ROS topic rate (ros2 topic hz /image) -----"
timeout "$DURATION" ros2 topic hz /image 2>&1 | tail -n 30
wait "$GZ_PID" 2>/dev/null || true

echo
echo "[measure] done sampling."
