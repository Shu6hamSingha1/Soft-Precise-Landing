#!/usr/bin/env bash
# Fill the data gap identified 2026-09-15/16: no moving-target GT-FB run exists on the
# cross-marker (rover_cross) world -- every prior moving-target GT-FB result (Linear 3/3
# on-platform, Circular baseline 0/4, AU_LEAD-tuned Circular 2/3) is on rover_aruco (the
# July 2026-07-02/03 campaign predates the cross_marker model by a month; see
# px4/project_rover_turning_open.md / project_moving_rover_landing_works.md). This harness
# reruns the BASELINE Circular config (yaw ASMC active, no AU_LEAD, no tuning) on
# rover_cross + MARKER_TYPE=cross, GT-FB, to get the cross-marker moving-target data point.
#
# Since GT-FB bypasses the image pipeline, marker geometry shouldn't change the control
# outcome vs the ArUco-rover baseline result (heading-hold-off, yaw-active Circular: FAIL
# ~8.1 m from yaw ramp windup, per project_rover_turning_open.md) -- this test checks that
# assumption directly rather than asserting it.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../scripts" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT="$PROJ/test_data/RoverCross_GTFB_Circular"
N="${N:-3}"
MAXLAUNCH="${MAXLAUNCH:-8}"

valid=$(ls "$OUT" 2>/dev/null | grep -v harness | wc -l); launch=0
echo "############ rover_cross GT-FB Circular baseline (have $valid/$N) ############"
while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
  launch=$((launch+1))
  before=$(ls "$OUT" 2>/dev/null | grep -v harness | wc -l)
  echo "=== launch $launch (valid $valid/$N) $(date +%H:%M:%S) ==="
  env HEADLESS=1 MAX_ATTEMPTS=5 \
      WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross \
      INITIAL_DRONE_ENU="2.0,2.0,5.0" \
      ROVER_MOTION=1 ROVER_TRAJ=Circular \
      PLASMC_GT_FEEDBACK=1 \
      LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$OUT" \
    taskset -c 6-15 bash "$SCRIPT_DIR/run_rover_landing_retry.sh" || true
  for d in "$OUT"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
  after=$(ls "$OUT" 2>/dev/null | grep -v harness | wc -l)
  if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "valid $valid/$N"; fi
done
echo "=== DONE: $valid valid in $launch launches $(date) ==="
