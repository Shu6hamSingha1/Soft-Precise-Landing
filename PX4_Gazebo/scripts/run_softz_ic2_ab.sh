#!/usr/bin/env bash
# Vertical-softness A/B on tuned manuscript-combined, IC2 n=5. The manuscript gains reverted
# the soft-config z params (KAPPA0_z 1.0->0.25, E_z 0.5->1.0, P2INF_z 0.5->1.5) -> hard vz~2.6.
# softz: keep manuscript xy explicit, restore soft-config z (gave vel 1.06 in feedback_descent_softness).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"; MAXLAUNCH="${MAXLAUNCH:-12}"
# soft-config z + manuscript xy (all axes explicit so the auto-apply is bypassed cleanly)
SOFTZ="PLASMC_KAPPA0_X=0.125 PLASMC_KAPPA0_Y=0.125 PLASMC_KAPPA0_Z=1.0 \
PLASMC_E_X=1.0 PLASMC_E_Y=1.0 PLASMC_E_Z=0.1 \
PLASMC_XI2_X=0.2 PLASMC_XI2_Y=0.2 PLASMC_XI2_Z=0.6 \
PLASMC_P2INF_X=0.5 PLASMC_P2INF_Y=0.5 PLASMC_P2INF_Z=1.0"
run_arm () {
  local name="$1"; shift; local base="$PROJ/test_data/SoftZ_IC2_${name}"; mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1)); local before; before=$(ls "$base" 2>/dev/null|wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" $@ \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null|wc -l)
    [ "$after" -gt "$before" ] && { valid=$((valid+1)); echo "[$name] valid $valid/$N"; }
  done
  echo "=== [$name] DONE: $valid valid in $launch ==="
}
run_arm base
run_arm softz  $SOFTZ
echo "ALL ARMS DONE"
