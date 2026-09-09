#!/usr/bin/env bash
# Rover CBF diagnostic sweep -- all 7 rover motion profiles, A/B on the
# moving-target lead (CBF_DRIFT_TAU = 0 vs LEAD_TAU).
#
# The question is NOT landing precision (rover landings are perception-blocked).
# It is: does the visibility CBF trigger correctly under target motion, and does
# the tau*d lead do its job -- keep the measured marker centre inside phi longer,
# fire earlier, reduce drift-off. The analysis pass at the end scores exactly
# that from vis_active / vis_slack / vis_drift / vis_c / vis_gz + marker-in-frame.
#
# One SITL stack at a time -> serial. cross-marker rover world.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"          # .../PX4_Gazebo
PY="$HOME/ws/scripts/env2025/bin/python3"
TS="$(date +%Y%m%d-%H%M%S)"
BUNDLE="$ROOT/test_data/RoverCBFSweep/$TS"
N_REPS="${N_REPS:-3}"
LEAD_TAU="${LEAD_TAU:-0.4}"
TRAJS=(${TRAJS:-Static Linear Circular EightShape Sinusoidal Lissajous CircularYaw})
export HEADLESS=1
export WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross
export ROVER_MOTION=1 ROVER_SPEED_MULT="${ROVER_SPEED_MULT:-1.0}"

mkdir -p "$BUNDLE"
SUMMARY="$BUNDLE/summary.tsv"
printf "traj\tarm\ttau\trep\tsaved\tresult_dir\n" > "$SUMMARY"

run_one() {  # $1 traj  $2 arm(off|lead)  $3 rep
  local traj="$1" arm="$2" rep="$3" tau
  [ "$arm" = off ] && tau=0.0 || tau="$LEAD_TAU"
  local ld="$BUNDLE/_autosave_${arm}"; mkdir -p "$ld"
  local before; before=$(ls -td "$ld/"*/ 2>/dev/null | head -1 || true)
  echo "=== $traj  arm=$arm tau=$tau  rep=$rep  $(date +%H:%M:%S) ==="
  ( cd "$ROOT" && env ROVER_TRAJ="$traj" CBF_DRIFT_TAU="$tau" \
      LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$ld" MAX_ATTEMPTS=5 \
      bash "$ROOT/scripts/run_rover_landing_retry.sh" ) \
      > "$BUNDLE/${traj}_${arm}_rep${rep}.log" 2>&1
  local latest; latest=$(ls -td "$ld/"*/ 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\t%s\t%s\tNO\t-\n" "$traj" "$arm" "$tau" "$rep" >> "$SUMMARY"; return
  fi
  local dst="$BUNDLE/${traj}/${arm}/rep${rep}"; mkdir -p "$(dirname "$dst")"
  cp -r "$latest" "$dst"
  printf "%s\t%s\t%s\t%s\tYES\t%s\n" "$traj" "$arm" "$tau" "$rep" "${traj}/${arm}/rep${rep}" >> "$SUMMARY"
}

echo "[rover_cbf] bundle=$BUNDLE  N_REPS=$N_REPS  LEAD_TAU=$LEAD_TAU  trajs=${TRAJS[*]}"
for traj in "${TRAJS[@]}"; do
  for r in $(seq 1 "$N_REPS"); do
    run_one "$traj" off  "$r"; sleep 2
    run_one "$traj" lead "$r"; sleep 2
  done
done

echo; echo "[rover_cbf] === CBF-behaviour analysis ==="
"$PY" "$ROOT/tools/analyze_rover_cbf_sweep.py" "$BUNDLE" | tee "$BUNDLE/analysis.txt"
echo "[rover_cbf] DONE ($(date +%H:%M:%S))  bundle: test_data/RoverCBFSweep/$TS"
