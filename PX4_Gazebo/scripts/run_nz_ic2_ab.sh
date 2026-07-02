#!/usr/bin/env bash
# N_z VDF-parity A/B (2026-06-22) — isolate the descent κ-adaptation rate on the CLEAN
# combined baseline (NO descent-gate, NO commit). N_z was a stale 2026-06-13 soft-config
# leftover (0.1) that combined-mode never aligned to MATLAB vdf_params (P.N=diag([.02,.02,.02])).
# N is the κ-ODE adapt rate (dκ/dt=Θ·N·G·|σ|−N·P·κ); κ_eq is N-independent, but N_z=0.1 makes
# κ_z race 5× faster -> RUNAWAY on the terminal Z funnel breach. MATLAB lowered N_z (0.05->0.02)
# to tame exactly that (CB43/CB44). Targets the VERTICAL terminal fly-aways (committed-centered
# z-launches, |vz|→12) the centered-gating A/B isolated.
#   nz010 : PLASMC_N_Z=0.1   (the old/current-baked value)
#   nz002 : default 0.02     (VDF parity, the new baked default)
# nz002 FIRST. N=15. Judge: did N_z=0.02 reduce the vertical fly-aways / cap the descent tail?
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/Nz_IC2_${name}"
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

# nz002 (VDF parity, new default) FIRST
run_arm nz002
# nz010 (old value, env override)
run_arm nz010  env PLASMC_N_Z=0.1
echo "ALL ARMS DONE"
