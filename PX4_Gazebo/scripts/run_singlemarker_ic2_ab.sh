#!/usr/bin/env bash
# Single-marker + ring-switch IC2 A/B (2026-06-22). Eliminates the nested-ArUco primary-marker
# SWITCHING (root of the terminal loom spike -> vertical launch) by locking the loom/centroid to
# ONE marker (re-lock only when it disappears), feeding the flow with baseline scaled-quad DENSE
# points (~sqrt(N) loom-noise reduction), and switching the loom to the RING when the marker is
# NOT VISIBLE (KLT out-of-bounds = left FoV). vs the size-normalized moment loom (FLOW_LOOM_DECOUPLE,
# the current vertical-channel fix). single arm FIRST (de-risk: watch lock/dense/ring-switch).
# Judge: vertical launches + terminal loom continuity.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"; MAXLAUNCH="${MAXLAUNCH:-40}"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/SingleMarker_${name}"; mkdir -p "$base"
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
run_arm single    env PLASMC_SINGLE_MARKER=1
run_arm momentsz  env FLOW_LOOM_DECOUPLE=1
echo "ALL ARMS DONE"
