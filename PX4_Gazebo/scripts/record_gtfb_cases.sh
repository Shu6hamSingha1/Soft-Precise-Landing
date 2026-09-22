#!/usr/bin/env bash
# Record GT-FB raw onboard + chase videos for PLASMC (VISTA) on the manuscript's 5 comparison
# cases (Section experimentation/test-conditions), all from IC2=[2,2,-5]m ENU, as agreed
# 2026-09-22. Retries each case up to MAX_ATTEMPTS times until a SoftPrecise landing, writing
# into test_data/RecordGTFB_dev/<Case>/ (a WORKING dir -- promote_sp_to_final.sh copies only the
# SoftPrecise rep into test_data/Final/<Case>/, per [[feedback_use_defaults]]-style discipline of
# not polluting Final/ with test data).
#
# Cases 2 (Linear) and 5 (Circular) ride the ship-deck (heave/roll/pitch, apps/deck_follower.py +
# the standalone deck_platform model, WORLD=rover_cross_deck / ROVER_MODEL=rover_ackermann).
# Cases 1/3/4 (Static/Sinusoidal/Lissajous) have no deck term (rover_trajectory.deck_state
# returns zero for them) and use the plain WORLD=rover_cross / ROVER_MODEL=rover_cross (rover +
# flat platform, its own pose = target, no deck_platform involved).
#
# ⚠ Linear/Circular ridden this way pre-date the 2026-09-22 rover_drive.py sim-clock pacing fix
# (peer session soft-precise-landing-53) -- speeds recorded here should be re-checked against
# each rep's own Ground_Truth.npy (see tools -- or ad hoc) rather than trusted from
# ROVER_SPEED_MULT alone.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_ROOT="$PROJ/test_data/RecordGTFB_dev"
CASE="${1:?usage: record_gtfb_cases.sh <Static|Linear|Sinusoidal|Lissajous|Circular> [max_attempts]}"
MAX_ATTEMPTS="${2:-5}"

common_env=(
  HEADLESS=1
  LD_PRELOAD="$HOME/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5"
  MARKER_TYPE=cross
  INITIAL_DRONE_ENU="2.0,2.0,5.0"
  PLASMC_GT_FEEDBACK=1
  IMG_RECORD=1
  CHASE_CAM=1
  LANDING_AUTOSAVE=1
  # Recording-acceptance thresholds RELAXED from the manuscript's 0.08m/0.20m/s (2026-09-22,
  # user request): precise<=0.15m, soft<=0.5m/s. LANDING_PRECISE_TOL is honored by
  # apps/landing_test.py itself; the velocity bound is NOT env-overridable there (hardcoded
  # 0.2 m/s), so this script's own SP check below recomputes both from the raw xy_err/rel_vel
  # fields instead of trusting the recorded precise/soft booleans -- landing_test.py's own
  # shared classification logic is left untouched.
  LANDING_PRECISE_TOL=0.15
)
RECORD_VEL_TOL=0.5
RECORD_XY_TOL=0.15

case "$CASE" in
  Static)
    case_env=(WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0)
    ;;
  Sinusoidal|Lissajous)
    case_env=(WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=1 "ROVER_TRAJ=$CASE")
    ;;
  Linear|Circular)
    case_env=(WORLD=rover_cross_deck ROVER_MODEL=rover_ackermann POSE_IDX_UAV=3 POSE_IDX_TARGET=1
              ROVER_MOTION=1 "ROVER_TRAJ=$CASE" DECK_MOTION=1)
    ;;
  *)
    echo "unknown case $CASE" >&2; exit 1 ;;
esac

OUT="$OUT_ROOT/$CASE"
mkdir -p "$OUT"
valid=$(ls "$OUT" 2>/dev/null | grep -vE '\.mp4$|_raw$' | wc -l)
launch=0
echo "############ GT-FB PLASMC $CASE (have $valid reps, target: first SoftPrecise, max $MAX_ATTEMPTS launches) ############"
VIDEOS="$PROJ/test_data/Test_Videos"
while [ "$launch" -lt "$MAX_ATTEMPTS" ]; do
  launch=$((launch+1))
  echo "=== $CASE launch $launch/$MAX_ATTEMPTS $(date +%H:%M:%S) ==="
  _t0=$(date +%s)
  env "${common_env[@]}" "${case_env[@]}" LANDING_OUT_BASE="$OUT" \
    taskset -c 6-15 bash "$SCRIPT_DIR/run_rover_landing_retry.sh"
  # newest rep dir
  rep=$(ls -td "$OUT"/*/ 2>/dev/null | head -1)
  # Onboard/chase videos are NOT written under LANDING_OUT_BASE (hardcoded paths in
  # cross_marker_perception.py / record_chase.py) -- correlate by mtime newer than this
  # launch's start (both files are only finalized at process-exit/touchdown, so this
  # window is exact per launch, not just "most recent ever").
  onboard=$(find "$VIDEOS" -maxdepth 1 -name '*.mp4' -not -name 'chase_*' -newermt "@$_t0" -print 2>/dev/null | sort | tail -1)
  chase=$(find "$VIDEOS" -maxdepth 1 -name 'chase_*.mp4' -newermt "@$_t0" -print 2>/dev/null | sort | tail -1)
  if [ -n "$rep" ] && [ -f "$rep/Ground_Truth.npy" ]; then
    [ -n "$onboard" ] && cp "$onboard" "$rep/onboard_cam.mp4" && echo "    onboard -> $rep/onboard_cam.mp4"
    [ -n "$chase" ] && cp "$chase" "$rep/chase_cam.mp4" && echo "    chase   -> $rep/chase_cam.mp4"
    sp=$(~/ws/scripts/env2025/bin/python3 -c "
import numpy as np
g = np.load('$rep/Ground_Truth.npy', allow_pickle=True).item()['SoftPrecise']
xy, vel = g.get('xy_err'), g.get('rel_vel')
precise, soft = xy is not None and xy <= $RECORD_XY_TOL, vel is not None and vel <= $RECORD_VEL_TOL
print(f'{precise and soft} xy={xy} vel={vel}')
" 2>/dev/null)
    echo "    rep=$rep  precise+soft(<=${RECORD_XY_TOL}m,<=${RECORD_VEL_TOL}m/s)=$sp"
    sp="${sp%% *}"
    if [ "$sp" = "True" ]; then
      echo "=== $CASE: SoftPrecise achieved on launch $launch -> $rep ==="
      echo "$rep" > "$OUT/.softprecise_rep"
      break
    fi
  else
    echo "    (no rep produced this launch)"
  fi
done
echo "=== $CASE DONE: $launch launches, SP rep: $(cat "$OUT/.softprecise_rep" 2>/dev/null || echo NONE) $(date) ==="
