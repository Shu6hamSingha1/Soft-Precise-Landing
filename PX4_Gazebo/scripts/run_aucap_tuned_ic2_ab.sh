#!/usr/bin/env bash
# Terminal a_u-cap A/B on the TUNED manuscript-combined config, IC2 n=5. The failures are
# all terminal-1/Z (converge then breach at 0.3-0.7m, a_u spikes 176-572). Terminal commit
# cap (PLASMC_COMMIT_AU_MAX, gated on marker-fill) bounds the spike; approach untouched.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/AuCapT_IC2_${name}"; mkdir -p "$base"
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
run_arm nocap
run_arm cap15   env PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_AU_MAX=15
run_arm cap30   env PLASMC_COMMIT_EXTENT=50 PLASMC_COMMIT_AU_MAX=30
echo "ALL ARMS DONE"
