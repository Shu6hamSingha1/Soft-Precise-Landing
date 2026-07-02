#!/usr/bin/env bash
# Size-normalized corner MOMENT loom re-test (2026-06-22). The terminal vertical launch is driven by
# ArUco PRIMARY-MARKER SWITCHING: the board's nested markers hand off as primary at the deck ->
# corner spread/extent jumps -> the loom (d/dt of spread) spikes -> launch. The size-normalized
# moment loom M/sz²=(f/Z)² is marker-INDEPENDENT -> should stay CONTINUOUS across switches.
# FLOW_LOOM_DECOUPLE=1 applies the moment loom (size-normalized by primary marker size) on V_v[2].
# Re-test (prior MomLoom A/B judged xy=noise + was before the switch mechanism was understood); JUDGE
# on terminal-loom continuity across switches + VERTICAL launches.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"; MAXLAUNCH="${MAXLAUNCH:-40}"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/MomLoomSz_${name}"; mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1)); local before; before=$(ls "$base" 2>/dev/null|wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null|wc -l); [ "$after" -gt "$before" ] && { valid=$((valid+1)); echo "[$name] valid $valid/$N"; }
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}
run_arm momentsz  env FLOW_LOOM_DECOUPLE=1
run_arm default
echo "ALL ARMS DONE"
