#!/usr/bin/env bash
# Retry-wrapper around run_rover_landing.sh. PX4 SITL startup is flaky (~50% of
# launches): the gz_bridge/lockstep IMU-timing race (is_armable never goes True)
# and occasional IC-non-settle both make the inner launcher exit 42. This wrapper
# reboots the whole two-instance stack on a 42 and gives up on any other rc.
# See discuss.px4.io/t/px4-sitl-multi-vehicle-gazebo-imu-timing-errors/33268.
#
# Forwards all env vars (HEADLESS, PLASMC_GT_FEEDBACK, ROVER_MOTION, ROVER_TRAJ,
# LANDING_AUTOSAVE, etc.) and arguments to the inner launcher.
# Usage: HEADLESS=1 PLASMC_GT_FEEDBACK=1 ./run_rover_landing_retry.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-5}"

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  echo "================================================================="
  echo "[retry] Attempt $attempt / $MAX_ATTEMPTS"
  echo "================================================================="

  bash "$SCRIPT_DIR/run_rover_landing.sh"
  rc=$?

  if [ "$rc" -eq 0 ]; then
    echo "[retry] SUCCESS after attempt $attempt"
    exit 0
  fi
  if [ "$rc" -eq 42 ]; then
    echo "[retry] Attempt $attempt hit a PX4 SITL startup flake (lockstep race /"
    echo "[retry]   IC non-settle); rebooting the stack after 10 s..."
    sleep 10
    continue
  fi
  echo "[retry] Attempt $attempt exited rc=$rc (not a retriable flake); giving up."
  exit "$rc"
done

echo "[retry] All $MAX_ATTEMPTS attempts hit a startup flake."
exit 1
