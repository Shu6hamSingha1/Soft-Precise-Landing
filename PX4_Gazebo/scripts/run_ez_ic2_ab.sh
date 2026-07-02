#!/usr/bin/env bash
# E_z IC2 A/B (2026-06-21) — combined-barrier gain retune.
# Tests the MATLAB CB57 lever ported to PX4: E_z 1.0->0.5 engages kappa's
# switching damping on the terminal Z limit cycle (locked out at E_z=1.0 because
# the cycle rings inside the boundary layer |sigma_z|/E_z<=0.2). Better h_ez
# convergence -> constant loom -> less terminal 1/Z positive feedback -> fewer
# terminal-1/Z fly-aways (the current 1/5 IC2 residual).
#
# Combined mode auto-sets E=diag(1,1,1) ONLY if no PLASMC_E_* is set, so the
# ez05 arm MUST pass ALL THREE (E_X=1.0 E_Y=1.0 E_Z=0.5) to isolate the Z change
# (E_Z alone would silently revert X/Y to the base default 1.5).
#
# Terminal-1/Z is STOCHASTIC (~1/5) -> N>=15, judge by FLY-AWAY RATE + terminal
# vz variance, NOT median xy (skill meta-finding: lateral outcome is
# stochasticity-dominated; n=5 measures noise).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/Ez_IC2_${name}"
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

# baseline = current baked combined defaults (E_z=1.0)
run_arm baseline
# ez05 = E_z 1.0->0.5, X/Y held at the combined-default 1.0 (isolation)
run_arm ez05  env PLASMC_E_X=1.0 PLASMC_E_Y=1.0 PLASMC_E_Z=0.5
echo "ALL ARMS DONE"
