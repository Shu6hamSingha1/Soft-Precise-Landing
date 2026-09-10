#!/usr/bin/env bash
# XIR / XI2 handoff validation -- GT-FB, ROVER WORLD ONLY (2026-09-10).
# Source: docs/HANDOFF_xir_xih_moving_target.md (c0198d5b).
#
# MATLAB (realistic plant) says PLASMC_XIR_{X,Y}=0.20 + PLASMC_XI2_{X,Y}=0.20
# lands soft-precise on BOTH stationary and moving targets, where the current
# baked XIR=0.10 / XI2=1.0 fails the moving regime (terminal s_e_n grows back,
# walks the marker out of frame). Mechanism is PURE CONTROL-SIDE (position-funnel
# width == barrier gain G_r ~ 2/p_r; XIR sets p_r decay). GT-FB strips the
# rover perception (detector-collapse) confound so this isolates the funnel claim.
#
# Cells: 2 arms {base, cand} x 2 motion {moving, static} x 2 IC {IC2, IC1}.
#   base  = PLASMC_XIR_{X,Y}=0.10  PLASMC_XI2_{X,Y}=1.0   (current baked)
#   cand  = PLASMC_XIR_{X,Y}=0.20  PLASMC_XI2_{X,Y}=0.20  (handoff candidate)
#   moving = rover Circular @ SPEED_MULT default (nominal); static = ROVER_MOTION=0
#   IC2 = MATLAB discriminator (off-center); IC1 = kappa-leakage-drift canary
#         the handoff explicitly says to watch when dropping XI2 to 0.20.
# All other params baked. Everything routed through run_rover_landing_retry.sh
# (PX4 SITL two-instance startup is ~50% flaky; retry reboots on rc=42).
#
# Discriminator to check afterwards (NOT the SP verdict alone): does terminal
# |s_e_xy| converge and STAY converged through the last phase of descent, and
# is there any FoV breach? Plus IC1: kappa_xy <= ~0.5 (no ratchet), a_u_xy bounded.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../scripts" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_ROOT="$PROJ/test_data/XirXi2_RoverGTFB"
N="${N:-3}"
MAXLAUNCH="${MAXLAUNCH:-10}"

declare -A IC_ENU=( [IC1]="0.0,0.0,5.0" [IC2]="2.0,2.0,5.0" )

run_cell () {
  local arm="$1" motion="$2" ic="$3"
  local xir xi2
  if [ "$arm" = "base" ]; then xir=0.10; xi2=1.0; else xir=0.20; xi2=0.20; fi
  local rmotion; [ "$motion" = "moving" ] && rmotion=1 || rmotion=0
  local base="$OUT_ROOT/${motion}_${arm}_${ic}"
  mkdir -p "$base"
  local valid launch
  valid=$(ls "$base" 2>/dev/null | wc -l); launch=0
  echo "############ CELL ${motion}/${arm}/${ic}  XIR=$xir XI2=$xi2 ROVER_MOTION=$rmotion  (have $valid/$N) ############"
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [${motion}/${arm}/${ic}] launch $launch (valid $valid/$N) $(date +%H:%M:%S) ==="
    env HEADLESS=1 MAX_ATTEMPTS=5 \
        INITIAL_DRONE_ENU="${IC_ENU[$ic]}" \
        ROVER_MOTION="$rmotion" \
        PLASMC_GT_FEEDBACK=1 \
        PLASMC_XIR_X="$xir" PLASMC_XIR_Y="$xir" \
        PLASMC_XI2_X="$xi2" PLASMC_XI2_Y="$xi2" \
        LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$base" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_rover_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[${motion}/${arm}/${ic}] valid $valid/$N"; fi
  done
  echo "=== CELL ${motion}/${arm}/${ic} DONE: $valid valid in $launch launches ==="
}

# Most-informative cells first: the moving discriminator on IC2, both arms.
run_cell cand moving IC2
run_cell base moving IC2
run_cell cand moving IC1
run_cell base moving IC1
run_cell cand static IC2
run_cell base static IC2
run_cell cand static IC1
run_cell base static IC1
echo "ALL CELLS DONE $(date)"
