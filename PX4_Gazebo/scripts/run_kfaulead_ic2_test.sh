#!/usr/bin/env bash
# Combined KF-retune + AU_LEAD IC2 test (2026-08-28) -- AU_LEAD alone (2026-08-28
# aulead_ic2_test) gave 4/5 clean but 1/5 catastrophic (a_u to 1e6): the lead
# compensator amplifies the discrete jump the containment-freeze/ungated-dh_d bug
# produces. Retuning the dh_d KF first (validated 2026-08-28, kfretune_ic2_ab: 5/5
# clean, a_u thrash ~100x smaller) should remove that discontinuity BEFORE AU_LEAD
# sees it. Testing both together.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-15}"

BASE_ENV="PLASMC_GT_FEEDBACK=1 PLASMC_P20_Z=10 PLASMC_P2INF_X=0.5 PLASMC_P2INF_Y=0.5 \
PLASMC_OMEGA_X=0.1 PLASMC_OMEGA_Y=0.1 PLASMC_OMEGA_Z=0.1 PLASMC_GAMMA_Y=0.5 \
PLASMC_E_X=1 PLASMC_E_Y=1 PLASMC_E_Z=0.5 PLASMC_N_Z=0.1 PLASMC_P_X=1.5 PLASMC_P_Y=1.5 \
PLASMC_KAPPA0_X=0.125 PLASMC_KAPPA0_Y=0.125 PLASMC_KAPPA0_Z=0.25 TD_DEBUG=1 \
PLASMC_DERIV_KF_Q=1.0 PLASMC_DERIV_KF_R=0.05 PLASMC_AU_LEAD=1"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/KFAuLead_IC2_${name}"
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

run_arm combined
echo "ARM DONE"
