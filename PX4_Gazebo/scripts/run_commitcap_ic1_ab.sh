#!/usr/bin/env bash
# terminal-commit IC1 A/B (2026-06-19). N valid reps per arm at IC1=(0,0,5),
# headless, taskset 6-15. Arms: baseline (joint-lstsq loom) vs loomdec
# (FLOW_LOOM_DECOUPLE=1, marker moment-trace divergence, gain 1.15). Tests
# whether the decoupled loom improves terminal touchdown.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-14}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/CommitCap_IC1_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=0,0,5 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm baseline
run_arm commit   env PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_DSD_MAX=0.6
echo "ALL ARMS DONE"
