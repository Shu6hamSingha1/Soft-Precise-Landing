#!/usr/bin/env bash
# Centroid-rate observer IC2 A/B (2026-06-23). Tests the gyro-compensated decoded-corner centroid-rate
# observer (PLASMC_CENTROID_RATE=1) — fills the altitude velocity gap (Nfc=0 -> no flow -> drift -> fly-away)
# with h_x,h_y from d(centroid)/dt - L_w·w(gyro) - loom. vs the baseline (lstsq lateral, off).
#   observer : PLASMC_CENTROID_RATE=1   (decoded-corner centroid-rate lateral)
#   baseline : default (LK lstsq lateral)
# observer FIRST (de-risk: watch for sensible flow, no crashes, the Nfc=0 fly-aways dropping). N=5.
# Both on the baked single-marker config (1m world, matched cal, moment loom). IC2=(2,2,5).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAX="${MAX:-16}"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/CentroidRate_IC2_${name}"; mkdir -p "$base"
  local v=0 i=0
  while [ "$v" -lt "$N" ] && [ "$i" -lt "$MAX" ]; do
    i=$((i+1)); local b; b=$(ls "$base" 2>/dev/null|wc -l); echo "=== [$name] run $i (valid $v/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local a; a=$(ls "$base" 2>/dev/null|wc -l); [ "$a" -gt "$b" ] && { v=$((v+1)); echo "[$name] valid $v/$N"; }
  done
  echo "=== [$name] DONE: $v valid in $i runs ==="
}
run_arm observer  env PLASMC_CENTROID_RATE=1
run_arm baseline  env
echo "ALL ARMS DONE"
