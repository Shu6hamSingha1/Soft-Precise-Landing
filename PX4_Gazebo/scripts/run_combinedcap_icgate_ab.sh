#!/usr/bin/env bash
# Combined+cap vs back-mapped IC2-5 GATE (2026-06-20). Does the command-bounding cap
# rescue the combined-barrier's zeta_r blow-up (-> a_u 130 -> whip-out)?
#   combinedcap — combined-barrier (DEFAULT) + PLASMC_COMMIT_DSD_MAX=0.8
#   backmap     — PLASMC_COMBINED_BARRIER=0 (old back-mapped baseline)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
declare -A IC_ENU=( [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0" )
run_cell () {
  local ic="$1" arm="$2"; shift 2
  local base="$PROJ/test_data/CombCap_${ic}_${arm}"; mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1)); local before; before=$(ls "$base" 2>/dev/null|wc -l)
    echo "=== [$ic/$arm] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU="${IC_ENU[$ic]}" LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null|wc -l)
    [ "$after" -gt "$before" ] && { valid=$((valid+1)); echo "[$ic/$arm] valid $valid/$N"; }
  done
  echo "=== [$ic/$arm] DONE: $valid valid in $launch launches ==="
}
for ic in IC2 IC5 IC3 IC4; do
  run_cell "$ic" combinedcap  env PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_DSD_MAX=0.8
  run_cell "$ic" backmap      env PLASMC_COMBINED_BARRIER=0
done
echo "ALL CELLS DONE"
