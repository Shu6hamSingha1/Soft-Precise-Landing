#!/usr/bin/env bash
# Combined + KF + global a_u_xy cap SWEEP at IC2 (2026-06-20). Targets the off-center
# APPROACH over-aggression (53% breach at altitude). All arms: combined-barrier + the
# validated KFs (VDS/DHD/DW). Sweep PLASMC_AU_MAX_XY ∈ {none, 15, 10, 6} + terminal cap.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
KF="PLASMC_COMBINED_BARRIER=1 PLASMC_VDS_KF=1 PLASMC_DHD_KF=1 PLASMC_DW_KF=1"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/AuMax_IC2_${name}"; mkdir -p "$base"
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
run_arm kfonly
run_arm aumax15  env PLASMC_AU_MAX_XY=15
run_arm aumax10  env PLASMC_AU_MAX_XY=10
run_arm aumax6   env PLASMC_AU_MAX_XY=6
echo "ALL ARMS DONE"
