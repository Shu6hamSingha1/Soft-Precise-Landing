#!/usr/bin/env bash
# Loom cal-fix closed-loop A/B (2026-06-20). IC1, cap0.8 CONTAINS the lateral wall so
# the loom's TERMINAL effect (vz/balloon/touchdown) is judgeable. Arms:
#   capbase   — cap0.8, joint-pinv loom (loom off)
#   caploom   — cap0.8 + FLOW_LOOM_DECOUPLE=1 (clean moment loom, cal-bypass fix)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/LoomCal_IC1_${name}"; mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1)); local before; before=$(ls "$base" 2>/dev/null|wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=0,0,5 LANDING_OUT_BASE="$base" PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_DSD_MAX=0.8 "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null|wc -l)
    [ "$after" -gt "$before" ] && { valid=$((valid+1)); echo "[$name] valid $valid/$N"; }
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}
run_arm capbase
run_arm caploom  env FLOW_LOOM_DECOUPLE=1
echo "ALL ARMS DONE"
