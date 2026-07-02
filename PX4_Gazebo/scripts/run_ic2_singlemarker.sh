#!/usr/bin/env bash
# IC2 baseline on the CURRENT single-marker config (2026-06-23): 1m ArUco world + matched cal +
# baked PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1. Re-confirms the cal/config is applied, and is
# the baseline BEFORE the gyro-compensated centroid-rate observer (the altitude-starvation fix).
# Same config as SingleWorldRecal_test (2/5 sub). N=5.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAX="${MAX:-16}"; base="$PROJ/test_data/IC2_SingleMarker"; mkdir -p "$base"
v=0; i=0
while [ "$v" -lt "$N" ] && [ "$i" -lt "$MAX" ]; do
  i=$((i+1)); b=$(ls "$base" 2>/dev/null|wc -l); echo "=== run $i (valid $v/$N) ==="
  env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1 \
    taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
  for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
  a=$(ls "$base" 2>/dev/null|wc -l); [ "$a" -gt "$b" ] && { v=$((v+1)); echo "valid $v/$N"; }
done
echo "=== IC2 SINGLE-MARKER DONE: $v valid in $i runs ==="
