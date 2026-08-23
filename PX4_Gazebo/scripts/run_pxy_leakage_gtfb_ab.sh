#!/usr/bin/env bash
# P_xy (kappa-leakage) A/B under PLASMC_GT_FEEDBACK=1, IC4 (2026-08-19).
#
# Context: hardware PLASMC_HW_POS_FEEDBACK flights (2026-08-17/18) traced a_u/kappa
# detonation spikes to kappa's leakage term (P) being too slow on X/Y relative to
# real error-recovery timescale (tau_xy=1/(N*P)=6.67s at the HARDWARE default
# P_xy=1.5, vs tau_z=2.0s at P_z=5.0) -- kappa keeps ratcheting up for seconds after
# s_e_n has already started converging, then detonates the undamped switching term
# 3+s late. Gazebo's controller.py ALREADY carries a fix for the identical mechanism
# (self._P comment, 2026-07-22 bake): P_xy 1.5->2.5, paired with XI2_xy 0.6->1.0,
# validated n=5 IC1-5 GT-FB (0 severe fly-aways, 2 SP) -- but that fix was apparently
# never ported to Hardware/scripts/controller.py, which still has P_xy=1.5.
#
# This harness does NOT re-test the already-validated XI2_xy=1.0+P_xy=2.5 combo --
# it tests going HIGHER than that (P_xy in [3.0, 5.0], the user's requested range),
# XI2_xy left at its current baked default, against the CURRENT KP_X/Y=3.0 default
# (KP was independently retuned 5->3 on 2026-06-14; the P_xy=1.5 hardware value was
# tuned paired with the OLD KP=5, never reconciled with KP=3 -- this harness runs
# entirely under the current KP=3 regime).
#
# Methodology matches the tuning guide's own precedent for this exact investigation
# line ("GT-FB, IC4 n=2; pending IC2-5 n>=5 + perception-ON") -- quick focused A/B,
# read by MECHANISM (kappa trajectory, s_e_n, sigma), not just SP rate at this n.
# IC2-5 n>=5 gate is a follow-up if this shows a real effect worth pursuing further.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-2}"
MAXLAUNCH="${MAXLAUNCH:-8}"
IC="2.0,2.0,7.0"   # IC4

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/PxyLeakageGTFB_IC4_${name}"
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

# baseline = current baked default (P_xy=2.5, the already-fixed Gazebo value)
run_arm baseline
# p30 = P_xy 2.5->3.0
run_arm p30  env PLASMC_P_X=3.0 PLASMC_P_Y=3.0
# p50 = P_xy 2.5->5.0 (matches Z's leakage rate)
run_arm p50  env PLASMC_P_X=5.0 PLASMC_P_Y=5.0
echo "ALL ARMS DONE"
