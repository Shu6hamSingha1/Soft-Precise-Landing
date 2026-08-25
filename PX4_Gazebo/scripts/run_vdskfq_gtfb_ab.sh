#!/usr/bin/env bash
# VDS_KF_Q A/B under PLASMC_GT_FEEDBACK=1, IC2, both marker types (2026-08-20).
#
# Context: project_20260817_crossmarker_descent_stall_investigation.md found a
# repeatable ~0.487m GT-feedback descent-stall (kappa-leakage fixed-point
# attractor), marker-type-agnostic (ArUco hits it too, 1/6 escape vs the
# historically-documented 56-76% SP rate). The one identified, untested config
# gap vs. that historical campaign: current default PLASMC_VDS_KF_Q=10 (baked
# 2026-07-04 for a DIFFERENT reason -- terminal-deck fly-away severity, see
# controller.py:536 comment) vs. the older VDS_KF_Q=1. This harness tests
# whether q=1 closes the descent-stall/escape-rate gap. Priority #1 from that
# session's handoff.
#
# NOTE: q=10 was baked because q=1 masked drift-term noise on a DIFFERENT bug
# (terminal-deck fly-aways, now separately understood, not necessarily fixed).
# If q=1 escapes the stall here, re-check fly-away severity before adopting.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-15}"
IC="2.0,2.0,5.0"   # IC2

run_arm () {
  local name="$1" world="$2" marker="$3"; shift 3
  local base="$PROJ/test_data/VdsKfqGTFB_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 WORLD="$world" MARKER_TYPE="$marker" INITIAL_DRONE_ENU="$IC" \
        LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 PLASMC_GT_FEEDBACK=1 \
        LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

# baseline = current baked default (VDS_KF_Q=10)
run_arm aruco_q10 aruco arucotag        env PLASMC_VDS_KF_Q=10
run_arm aruco_q1  aruco arucotag        env PLASMC_VDS_KF_Q=1
run_arm cross_q10 cross_marker cross    env PLASMC_VDS_KF_Q=10
run_arm cross_q1  cross_marker cross    env PLASMC_VDS_KF_Q=1
echo "ALL ARMS DONE"
