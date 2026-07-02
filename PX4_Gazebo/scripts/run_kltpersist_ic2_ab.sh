#!/usr/bin/env bash
# KLT partial-decode persistence IC2 A/B (2026-06-21) — "keep the sensor alive".
# Tests PLASMC_KLT_PERSIST=1 (now incl. the partial-decode path: carry the prev
# frame's extra corners by 1-frame LK when the board only partially decodes, so
# N Flow Corners stays high -> conditioned flow lstsq -> no spurious lateral flow
# -> no terminal kick). Targets the DECODE-FAIL half of the E_z=0.5 residual
# fly-aways (corners in-FoV but ArUco can't decode the outer markers; reps 13.3/
# 7.0/3.3). The GEOMETRIC half (corners off-image: 15.8/4.5/11.9) is NOT covered
# here -- that needs the descent-gate/commit (separate track).
#
# Base = current baked defaults (E_z=0.5 already baked). Single knob.
# Judge by: fly-away rate + terminal N Flow Corners (did persistence lift it?).
# N>=15 (stochastic terminal residual; judge variance/fly-rate, not median).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/KltPersist_IC2_${name}"
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

# baseline = baked defaults (E_z=0.5, persist OFF)
run_arm baseline
# persist = partial-decode KLT persistence ON
run_arm persist  env PLASMC_KLT_PERSIST=1
echo "ALL ARMS DONE"
