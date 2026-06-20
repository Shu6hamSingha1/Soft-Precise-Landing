#!/usr/bin/env bash
# Lateral-velocity-arrest sweep on manuscript-combined, IC2 n=5. Two levers:
#   chi_r (surface velocity weight) + h_rd (descent pacing / convergence ordering).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/LatVel_IC2_${name}"; mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1)); local before; before=$(ls "$base" 2>/dev/null|wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null|wc -l)
    [ "$after" -gt "$before" ] && { valid=$((valid+1)); echo "[$name] valid $valid/$N"; }
  done
  echo "=== [$name] DONE: $valid valid in $launch ==="
}
run_arm chir050   env PLASMC_CHI_R_X=0.5 PLASMC_CHI_R_Y=0.5
run_arm chir130   env PLASMC_CHI_R_X=1.3 PLASMC_CHI_R_Y=1.3
echo "ALL ARMS DONE"
