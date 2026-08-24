#!/usr/bin/env bash
# PLASMC_TERMINAL_COMMIT A/B under PLASMC_GT_FEEDBACK=1, IC2, both marker types (2026-08-20).
#
# Context: project_20260817_crossmarker_descent_stall_investigation.md found a
# repeatable ~0.487m GT-feedback descent-stall (kappa-leakage fixed-point
# attractor). Comparing controller.py against the 06-30 SP-achieving snapshot
# (486f713, 19/25 SP) found two parameters that regressed away from that state:
# PLASMC_VDS_KF_Q (tested separately, run_vdskfq_gtfb_ab.sh) and
# PLASMC_TERMINAL_COMMIT, baked ON at the SP breakthrough (06-29) then baked
# OFF 4 days later (78c5309, 07-03) for a MOVING-target reason (rover open-loop
# hold develops uncorrected offset on a curving/translating deck) -- that
# commit's own comment asserts commit-off is "neutral-to-better" on the
# STATIONARY platform but this was never re-validated with a fresh IC1-5 SP
# gate. This harness runs that missing single-factor A/B: commit ON (=1,
# restoring the 06-29/SP-era default) vs commit OFF (=0, current default).
#
# Single-factor: PLASMC_VDS_KF_Q left at whatever default controller.py
# currently carries (10) in both arms here -- don't conflate with the VDS_KF_Q
# result. Run AFTER run_vdskfq_gtfb_ab.sh; if that sweep finds q=1 helps,
# consider a follow-up combined arm (commit=1 + q=1) once both singles are read.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-15}"
IC="2.0,2.0,5.0"   # IC2

run_arm () {
  local name="$1" world="$2" marker="$3"; shift 3
  local base="$PROJ/test_data/TerminalCommitGTFB_IC2_${name}"
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

# baseline = current baked default (commit OFF)
run_arm aruco_commit0 aruco arucotag        env PLASMC_TERMINAL_COMMIT=0
run_arm aruco_commit1 aruco arucotag        env PLASMC_TERMINAL_COMMIT=1
run_arm cross_commit0 cross_marker cross    env PLASMC_TERMINAL_COMMIT=0
run_arm cross_commit1 cross_marker cross    env PLASMC_TERMINAL_COMMIT=1
echo "ALL ARMS DONE"
