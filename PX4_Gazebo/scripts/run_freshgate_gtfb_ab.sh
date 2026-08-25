#!/usr/bin/env bash
# PLASMC_FRESH_GATE_INTEG A/B under PLASMC_GT_FEEDBACK=1, IC2, ArUco (2026-08-20).
#
# THE HYPOTHESIS THAT SURVIVED. History bisection 486f713(19/25 SP) -> HEAD, after
# VDS_KF_Q / TERMINAL_COMMIT / Z_REG / CBF_CORNERS_STALE were each tested and
# FALSIFIED (all 0 SP; the ~0.486 m fixed point was invariant to every one of
# them, and to marker type -- which is itself the clue: the cause is not a
# control gain).
#
# MEASURED: at the stall, izeta_z is EXACTLY frozen -- 1 distinct value across
# 1557 samples (~20 s), delta 0.000000 -- while zeta_z = +0.4055, a large
# persistent error it should be integrating. Not anti-windup (|izeta_z|=0.181 vs
# _izeta_clamp=5.0). The drone sits at 0.486 m, 4.8 cm lateral (a near-perfect SP
# position), commanding a genuine descent (B_T=+0.73 => thrust_norm 0.721 vs hover
# 0.738) and simply never breaks the kappa-leakage equilibrium, because the
# integral term that would wind up and break it is switched off.
#
# CAUSE: the conditional-integration gate added 2026-07-30 (ABSENT at 486f713 --
# verified by git) freezes izeta/is_e_n on FEATURE_PTS_FRESH, a PERCEPTION flag.
# Under GT_FEEDBACK the s/h being integrated come from GROUND TRUTH, so this gates
# the control law on a signal unrelated to the data it is integrating. The gate is
# correct in intent for the perception-ON hardware case it was written for
# (project_pi_coast_root_cause); it is simply wrong to apply under GT/pos feedback.
#
# ARMS: gate1 = current default (gate ON, expect the stall to reproduce)
#       gate0 = PLASMC_FRESH_GATE_INTEG=0 (gate bypassed)
# Scored on SoftPrecise ONLY -- and cross-checked for the frozen-GT false-SP
# pattern (target_lost=True + bit-identical trailing GT, see
# feedback_false_sp_frozen_gt): a tiny xy_err on frozen GT is NOT a landing.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-12}"
IC="2.0,2.0,5.0"   # IC2

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/FreshGateGTFB_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        PLASMC_GT_FEEDBACK=1 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm gate1  env PLASMC_FRESH_GATE_INTEG=1
run_arm gate0  env PLASMC_FRESH_GATE_INTEG=0
echo "ALL ARMS DONE"
