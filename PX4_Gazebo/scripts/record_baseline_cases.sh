#!/usr/bin/env bash
# Record GT-FB onboard + chase + montage videos for a COMPARISON BASELINE controller
# (PLASMC_BASELINE=lin2022|zhang2026|lin2023|cho2022) on ONE case, ONE attempt only
# (no SoftPrecise retry loop -- baselines are not expected/required to land precisely,
# per user direction 2026-09-23). Mirrors record_gtfb_cases.sh's env recipe for the 5
# trajectory cases, and adds the IC1-5 manuscript initial conditions (stationary
# target, WORLD=rover_cross, INITIAL_DRONE_ENU per IC).
#
# Usage: record_baseline_cases.sh <lin2022|zhang2026|lin2023|cho2022> \
#          <IC1|IC2|IC3|IC4|IC5|Static|Linear|Sinusoidal|Circular|Lissajous>
#
# Writes into test_data/RecordBaseline_dev/<BASELINE>/<CASE>/<rep>/ (working dir);
# promote_baseline_to_final.sh copies the single rep into
# test_data/Final/<TAG>-GT/<CASE>/, matching the VISTA-GT layout minus the
# overlay_s_alpha/overlay_h videos (no reusable generator for those; onboard+chase+
# montage only, per user decision 2026-09-23).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"

BASELINE="${1:?usage: record_baseline_cases.sh <lin2022|zhang2026|lin2023|cho2022> <IC1-5|Static|Linear|Sinusoidal|Circular|Lissajous>}"
CASE="${2:?usage: record_baseline_cases.sh <baseline> <case>}"

case "$BASELINE" in
  lin2022|zhang2026|lin2023|cho2022) ;;
  *) echo "unknown baseline $BASELINE" >&2; exit 1 ;;
esac

OUT_ROOT="$PROJ/test_data/RecordBaseline_dev/$BASELINE"
mkdir -p "$OUT_ROOT"

common_env=(
  HEADLESS=1
  LD_PRELOAD="$HOME/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5"
  MARKER_TYPE=cross
  PLASMC_GT_FEEDBACK=1
  "PLASMC_BASELINE=$BASELINE"
  IMG_RECORD=1
  CHASE_CAM=1
  LANDING_AUTOSAVE=1
)
# MAX_ATTEMPTS left at run_rover_landing_retry.sh's default (5) -- that retry is for PX4
# SITL startup FLAKE (rc=42, ~50% base rate per its own comment), not for landing quality;
# we still only ever get ONE landing rep per case (no SoftPrecise-gated relaunch loop).

declare -A IC_ENU=( [IC1]="0.0,0.0,5.0" [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0" )

case "$CASE" in
  IC1|IC2|IC3|IC4|IC5)
    case_env=(WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0 "INITIAL_DRONE_ENU=${IC_ENU[$CASE]}")
    ;;
  Static)
    case_env=(WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0 "INITIAL_DRONE_ENU=2.0,2.0,5.0")
    ;;
  Sinusoidal|Lissajous)
    case_env=(WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=1 "ROVER_TRAJ=$CASE" "INITIAL_DRONE_ENU=2.0,2.0,5.0")
    ;;
  Linear|Circular)
    case_env=(WORLD=rover_cross_deck ROVER_MODEL=rover_ackermann POSE_IDX_UAV=3 POSE_IDX_TARGET=1
              ROVER_MOTION=1 "ROVER_TRAJ=$CASE" DECK_MOTION=1 "INITIAL_DRONE_ENU=2.0,2.0,5.0")
    ;;
  *)
    echo "unknown case $CASE" >&2; exit 1 ;;
esac

OUT="$OUT_ROOT/$CASE"
mkdir -p "$OUT"
VIDEOS="$PROJ/test_data/Test_Videos"
echo "############ BASELINE $BASELINE / $CASE (single attempt, no SP gate) $(date +%H:%M:%S) ############"
_t0=$(date +%s)
env "${common_env[@]}" "${case_env[@]}" LANDING_OUT_BASE="$OUT" \
  taskset -c 6-15 bash "$SCRIPT_DIR/run_rover_landing_retry.sh"
rc=$?

rep=$(ls -td "$OUT"/*/ 2>/dev/null | head -1)
onboard=$(find "$VIDEOS" -maxdepth 1 -name '*.mp4' -not -name 'chase_*' -newermt "@$_t0" -print 2>/dev/null | sort | tail -1)
chase=$(find "$VIDEOS" -maxdepth 1 -name 'chase_*.mp4' -newermt "@$_t0" -print 2>/dev/null | sort | tail -1)

if [ -n "$rep" ] && [ -f "$rep/Ground_Truth.npy" ]; then
  [ -n "$onboard" ] && cp "$onboard" "$rep/onboard_cam.mp4" && echo "    onboard -> $rep/onboard_cam.mp4"
  [ -n "$chase" ] && cp "$chase" "$rep/chase_cam.mp4" && echo "    chase   -> $rep/chase_cam.mp4"
  ~/ws/scripts/env2025/bin/python3 -c "
import numpy as np
g = np.load('$rep/Ground_Truth.npy', allow_pickle=True).item()['SoftPrecise']
print(f'    result: xy_err={g.get(\"xy_err\")} rel_vel={g.get(\"rel_vel\")} precise={g.get(\"precise\")} soft={g.get(\"soft\")}')
" 2>/dev/null
  echo "$rep" > "$OUT/.rep"
else
  echo "    (no rep produced, rc=$rc)"
fi
echo "=== $BASELINE / $CASE DONE rc=$rc rep=$(cat "$OUT/.rep" 2>/dev/null || echo NONE) $(date) ==="
