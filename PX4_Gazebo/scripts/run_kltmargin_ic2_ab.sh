#!/usr/bin/env bash
# KLT partial-decode persistence + EDGE-MARGIN gate, IC2 A/B (2026-06-22).
# Follow-up to run_kltpersist_ic2_ab.sh: that A/B confirmed persistence triples
# terminal corner availability (5.8->18.4) and tightens the decode-fail flavor
# (sub-meter 5->9/15, median 2.11->0.70) BUT fattened the tail (74m) by carrying
# near-edge/drifting corners on the GEOMETRIC flavor (marker leaving FoV). The
# edge margin (PLASMC_KLT_PERSIST_MARGIN_PX=40, now default) drops carried corners
# within 40px of any edge, so persistence backs off as the marker leaves -> should
# KEEP the tight cluster while killing the tail. Fresh paired baseline (run-to-run
# baseline variance is large -> same-session baseline needed).
# Base = baked defaults (E_z=0.5). N>=15. Judge: fly-rate + terminal N Flow Corners.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/KltMargin_IC2_${name}"
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

# baseline = baked defaults (persist OFF)
run_arm baseline
# margin = partial-decode persistence ON with the 40px edge margin (now default)
run_arm margin  env PLASMC_KLT_PERSIST=1
echo "ALL ARMS DONE"
