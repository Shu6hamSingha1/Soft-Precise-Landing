#!/usr/bin/env bash
# Retry-wrapper around run_aruco_landing.sh. Some PX4 SITL launches lose a
# startup race (gz_bridge subscribes to IMU before lockstep scheduler is
# synced to the Gazebo clock → timestamp=0 → permanently denied arming).
# The inner launcher detects this and exits with code 42 so we can restart.
# See discuss.px4.io/t/px4-sitl-multi-vehicle-gazebo-imu-timing-errors/33268.
#
# Forwards all env vars (HEADLESS, LANDING_AUTOSAVE, etc.) and arguments.
# Usage: HEADLESS=1 LANDING_AUTOSAVE=1 ./run_aruco_landing_retry.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-5}"

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  echo "================================================================="
  echo "[retry] Attempt $attempt / $MAX_ATTEMPTS"
  echo "================================================================="

  bash "$SCRIPT_DIR/run_aruco_landing.sh"
  rc=$?

  if [ "$rc" -eq 0 ]; then
    echo "[retry] SUCCESS after attempt $attempt"
    exit 0
  fi
  if [ "$rc" -eq 42 ]; then
    echo "[retry] Attempt $attempt lost the gz_bridge/lockstep race; retrying after 10 s..."
    sleep 10
    continue
  fi
  # Any other non-zero exit (timeout, abort, real error) — don't retry.
  echo "[retry] Attempt $attempt exited rc=$rc (not the race condition); giving up."
  exit "$rc"
done

echo "[retry] All $MAX_ATTEMPTS attempts hit the gz_bridge/lockstep race."
echo "[retry] Apply the upstream GZBridge::imuCallback timestamp guard if this persists."
exit 1
