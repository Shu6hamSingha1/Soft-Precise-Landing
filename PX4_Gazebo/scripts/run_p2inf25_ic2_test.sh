#!/usr/bin/env bash
# P2INF_XY=2.5 IC2 test (2026-08-28) -- the tighter P2INF_X/Y=0.5 inherited
# from Dgate_IC2_baseline (never validated for lateral, Gazebo default is
# 2.5) was traced as the direct mechanical trigger for the funnel breach in
# KFAuLead_IC2_combined's outlier run: s_e_n was small and still converging
# right up to the moment p(t) hit its 0.5 floor -- the funnel getting tight,
# not the tracking error growing, is what pushed h_e/p toward the
# containment threshold. Testing the actual validated default (2.5) here,
# same IC2 GT-feedback base otherwise, no KF/AU_LEAD changes (isolate this
# one variable first).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-15}"

BASE_ENV="PLASMC_GT_FEEDBACK=1 PLASMC_P20_Z=10 PLASMC_OMEGA_X=0.1 PLASMC_OMEGA_Y=0.1 \
PLASMC_OMEGA_Z=0.1 PLASMC_GAMMA_Y=0.5 PLASMC_E_X=1 PLASMC_E_Y=1 PLASMC_E_Z=0.5 \
PLASMC_N_Z=0.1 PLASMC_P_X=1.5 PLASMC_P_Y=1.5 PLASMC_KAPPA0_X=0.125 PLASMC_KAPPA0_Y=0.125 \
PLASMC_KAPPA0_Z=0.25 TD_DEBUG=1 PLASMC_P2INF_X=2.5 PLASMC_P2INF_Y=2.5"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/P2inf25_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" $BASE_ENV "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm baseline
echo "ARM DONE"
