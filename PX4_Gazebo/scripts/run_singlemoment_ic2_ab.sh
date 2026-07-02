#!/usr/bin/env bash
# Single-marker + corner MOMENT loom IC2 A/B (2026-06-22). The single-marker pinv-loom run failed
# (rank-deficiency noise -> vz launches persist). The MOMENT loom sidesteps the pinv entirely:
# loom = -1/2 d(lnM)/dt = direct area-rate scalar (NO matrix inversion -> NO rank-deficiency). On the
# LOCKED single marker it's also switch-free + ID/size-independent (same marker -> s cancels in M1/M0).
#   single_moment : PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1  (lock + corner moment loom)
#   momentsz      : FLOW_LOOM_DECOUPLE=1                          (multi-marker moment loom = current best)
# single_moment FIRST. N=5 (user: n=5 enough). Judge: vertical launches + terminal loom continuity.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-16}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/SingleMoment_${name}"
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

run_arm single_moment  env PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1
run_arm momentsz       env FLOW_LOOM_DECOUPLE=1
echo "ALL ARMS DONE"
