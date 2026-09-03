#!/usr/bin/env bash
# Q8 step 1b — headroom re-test. 0.48 rad/s (the documented turning case) is at the SO(3)
# sin(dpsi) ceiling (~4% headroom) where FF is provably useless (see
# project_q8_yaw_ff_dead_sin_ceiling). 0.15 rad/s turned out to be inside native tracking
# (98% already, no lag to remove) AND hit an unrelated terminal a_u blow-up
# (project_joint_qp_nonconvergence_kappa_ratchet) that dominated the n=1 landing-outcome read.
# This run: 0.30 rad/s (predicted 30 deg steady lag, 50% headroom -- a real test of "does FF
# shrink genuine lag"), n=3/arm so one terminal flake can't dominate, GT-feedback, cross-marker.
set -u
cd "$HOME/Soft-Precise-Landing/PX4_Gazebo"
export HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross
export PLASMC_GT_FEEDBACK=1 PLASMC_GT_SPIN_WZ=0.30
OUT="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Q8_SpinFF_r30"; mkdir -p "$OUT"
N_REPS=3
ko(){ for r in 1 2 3; do
        pids=$(ps -eo pid,args | grep -aE 'run_ic_validation|run_landing|px4_sitl_default/bin/px4|gz sim|gz-sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|landing_test' | grep -av grep | awk '{print $1}')
        [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null || true; done; sleep 2
      done; sleep 3; }
gate(){ if ss -lun 2>/dev/null | grep -q ':8888'; then echo "ABORT: udp:8888 bound"; exit 1; fi; }

for arm in off ff; do
  [ "$arm" = "ff" ] && export PLASMC_YAW_WT_FF=0.30 || unset PLASMC_YAW_WT_FF
  for rep in $(seq 1 $N_REPS); do
    ko; gate
    echo "########## ARM=$arm rep=$rep  WT_FF=${PLASMC_YAW_WT_FF:-0}  $(date +%H:%M:%S) ##########"
    before=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
    timeout 400 env INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 \
        bash scripts/run_landing_retry.sh > "$OUT/${arm}_rep${rep}.log" 2>&1
    latest=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
    if [ -n "$latest" ] && [ "$latest" != "$before" ]; then
      cp -r "$latest" "$OUT/${arm}_rep${rep}"; echo "  saved -> $OUT/${arm}_rep${rep}"
    else
      echo "  NO REP"
    fi
  done
done
ko
echo "########## Q8 N3 PROBE COMPLETE $(date +%H:%M:%S) ##########"
