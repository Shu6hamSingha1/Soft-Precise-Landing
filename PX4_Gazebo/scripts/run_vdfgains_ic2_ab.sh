#!/usr/bin/env bash
# Combined-barrier: MANUSCRIPT gains (vdf-aligned, new default) vs old HOT gains, IC2 n=5.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
KF="PLASMC_COMBINED_BARRIER=1 PLASMC_VDS_KF=1 PLASMC_DHD_KF=1 PLASMC_DW_KF=1"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/VdfGains_IC2_${name}"; mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1)); local before; before=$(ls "$base" 2>/dev/null|wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" $KF "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null|wc -l)
    [ "$after" -gt "$before" ] && { valid=$((valid+1)); echo "[$name] valid $valid/$N"; }
  done
  echo "=== [$name] DONE: $valid valid in $launch ==="
}
run_arm manuscript
run_arm hotgains  env PLASMC_GAMMA_X=2.0 PLASMC_GAMMA_Y=2.0 PLASMC_KAPPA0_X=0.5 PLASMC_KAPPA0_Y=0.5 PLASMC_XI2_X=0.6 PLASMC_XI2_Y=0.6
echo "ALL ARMS DONE"
