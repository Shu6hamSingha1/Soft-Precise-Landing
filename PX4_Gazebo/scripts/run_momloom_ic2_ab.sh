#!/usr/bin/env bash
# Moment-loom IC2 A/B (2026-06-21) — perception lever for the terminal-1/Z residual.
# Tests the MATLAB CB49 fix ported to PX4 (img_data.py FLOW_LOOM_DECOUPLE): replace
# the ill-conditioned pinv(L_s) loom V_v[2]=vz/z (the sigma_min~2 weak depth mode,
# whose SVD tolerance sign-flips/under-reports near ground) with the SCALE-FREE
# moment area-rate loom -1/2 d(ln M)/dt (M = 2nd-moment corner spread). Rotation-
# immune, well-conditioned, no inversion. MATLAB: 92->95/100, lower breach. PX4
# offline-verified vs GT loom on a centered descent: corr 0.16->0.85, rmse 0.88->0.06.
# Attacks the loom sign-flip that the MATLAB CB46 trace identified as the TRIGGER of
# the terminal-1/Z launch (the residual that survives chi_z=0.1).
#
# Single knob: FLOW_LOOM_DECOUPLE=1 (overrides ONLY V_v[2]; lateral h_x/h_y + omega
# stay from the lstsq). FLOW_LOOM_GAIN stays 1.0 (CB51: >1.0 HURTS, 1.0=95 best);
# FLOW_LSTSQ_DROP_LOOM_COL stays off (MATLAB reduced-pinv = no closed-loop win).
#
# Terminal-1/Z is STOCHASTIC (~1/5) -> N>=15, judge by FLY-AWAY RATE + terminal vz
# variance, NOT median xy (skill meta-finding).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/MomLoom_IC2_${name}"
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

# Base = the validated E_z=0.5 win (2026-06-21, xy std 29.5->4.6, fly 5/15->2/15);
# this A/B isolates the moment loom ON TOP of E_z=0.5 to attack the 2/15 residual
# fly-aways (MATLAB CB46 trace: surviving terminal-1/Z launches are loom-driven).
EZ="PLASMC_E_X=1.0 PLASMC_E_Y=1.0 PLASMC_E_Z=0.5"
# base = E_z=0.5, pinv loom
run_arm base    env $EZ
# momloom = E_z=0.5 + scale-free moment area-rate loom on V_v[2]
run_arm momloom env $EZ FLOW_LOOM_DECOUPLE=1
echo "ALL ARMS DONE"
