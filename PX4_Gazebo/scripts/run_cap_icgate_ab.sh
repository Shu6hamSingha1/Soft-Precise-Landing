#!/usr/bin/env bash
# Command-bounding IC2-5 GATE A/B (2026-06-20). For each off-center IC, run baseline
# vs cap0.8 (PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_DSD_MAX=0.8). The mandatory pre-bake
# check: does the IC1 lateral-wall fix transfer off-center without regression? Judge by
# td_lat variance + flyaway. N valid reps per (IC,arm). IC1 already done (cap sweep).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-12}"

declare -A IC_ENU
IC_ENU[IC2]="2.0,2.0,5.0"
IC_ENU[IC3]="-2.0,2.0,5.0"
IC_ENU[IC4]="2.0,2.0,7.0"
IC_ENU[IC5]="2.0,2.0,3.0"

run_cell () {
  local ic="$1" arm="$2"; shift 2
  local base="$PROJ/test_data/CapGate_${ic}_${arm}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$ic/$arm] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU="${IC_ENU[$ic]}" LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$ic/$arm] valid $valid/$N"; fi
  done
  echo "=== [$ic/$arm] DONE: $valid valid in $launch launches ==="
}

for ic in IC2 IC5 IC3 IC4; do          # IC2 (off-center) + IC5 (canary) first
  run_cell "$ic" baseline
  run_cell "$ic" cap08  env PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_DSD_MAX=0.8
done
echo "ALL CELLS DONE"
