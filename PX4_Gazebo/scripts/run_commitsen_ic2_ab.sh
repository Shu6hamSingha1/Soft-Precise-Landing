#!/usr/bin/env bash
# Centered-gating terminal-commit IC2 A/B (2026-06-22). The extent-only commit caps
# the catastrophic tail (gate+commit: xy std 19.4->2.8, worst 66->11.7m vs gate-only)
# but commits on ALTITUDE alone -> fires while still off-center (LAT@commit up to 2.74m)
# -> coasts to that offset -> 0/15 sub-meter. Centered-gating (PLASMC_COMMIT_SEN=0.3)
# adds a |s_e_n|<0.3 condition to the latch: commit only when BOTH low (extent>65) AND
# centered -> should recover precision while keeping the tail capped.
#   gatecommit_sen : DESCENT_GATE + COMMIT_EXTENT=65 + COMMIT_LAT_TAPER + COMMIT_SEN=0.3
#   gate           : DESCENT_GATE only (the stack baseline)
# gatecommit_sen FIRST (de-risk: watch that the sen-gate doesn't BLOCK the commit ->
# fly-aways return; s_e_n may be unreliable at the Z~0.43m commit altitude). N=15.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/CommitSen_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

# gate + commit + centered-gating FIRST (de-risk)
run_arm gatecommit_sen  env PLASMC_DESCENT_GATE=1 PLASMC_COMMIT_EXTENT=65 PLASMC_COMMIT_LAT_TAPER=1 PLASMC_COMMIT_SEN=0.3
# gate-only baseline
run_arm gate            env PLASMC_DESCENT_GATE=1
echo "ALL ARMS DONE"
