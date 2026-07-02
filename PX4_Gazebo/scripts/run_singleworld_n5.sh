#!/usr/bin/env bash
# Single-LARGE-marker world, MATCHED-cal n=5 test (2026-06-23). World=single ~1m ArUco (id0);
# cal re-derived in-world (Hx/Hy R^2 0.63/0.62 vs nested-board 0.07). Controller: lock + scaled-quad
# dense + moment loom + visibility-margin ring-switch. The fair test the nested-board lock couldn't give.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAX="${MAX:-16}"; base="$PROJ/test_data/SingleWorldRecal_test"; mkdir -p "$base"
v=0; i=0
while [ "$v" -lt "$N" ] && [ "$i" -lt "$MAX" ]; do
  i=$((i+1)); b=$(ls "$base" 2>/dev/null|wc -l); echo "=== run $i (valid $v/$N) ==="
  env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1 \
    taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
  for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
  a=$(ls "$base" 2>/dev/null|wc -l); [ "$a" -gt "$b" ] && { v=$((v+1)); echo "valid $v/$N"; }
done
echo "=== SINGLE-WORLD n=5 DONE: $v valid in $i runs ==="
