#!/usr/bin/env bash
# Q8 step 1 — perfect-knowledge test. GT-FB, synthetic in-place target spin.
#   arm "off" : no feedforward           -> expect the documented e_a ramp-windup
#   arm "ff"  : EXACT injected rate fed forward -> does perfect knowledge fix it?
set -u
cd "$HOME/Soft-Precise-Landing/PX4_Gazebo"
export HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross
export PLASMC_GT_FEEDBACK=1 PLASMC_GT_SPIN_WZ=0.48      # 0.48 rad/s = 27 deg/s, the documented turning case
OUT="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Q8_SpinFF"; mkdir -p "$OUT"
ko(){ for r in 1 2 3; do
        pids=$(ps -eo pid,args | grep -aE 'run_ic_validation|run_landing|px4_sitl_default/bin/px4|gz sim|gz-sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|landing_test' | grep -av grep | awk '{print $1}')
        [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null || true; done; sleep 2
      done; sleep 3; }
for arm in off ff; do
  ko
  if ss -lun 2>/dev/null | grep -q ':8888'; then echo "ABORT: udp:8888 bound"; exit 1; fi
  [ "$arm" = "ff" ] && export PLASMC_YAW_WT_FF=0.48 || unset PLASMC_YAW_WT_FF
  echo "########## ARM=$arm  WT_FF=${PLASMC_YAW_WT_FF:-0}  $(date +%H:%M:%S) ##########"
  before=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
  timeout 400 env INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 \
      bash scripts/run_landing_retry.sh > "$OUT/${arm}.log" 2>&1
  latest=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
  if [ -n "$latest" ] && [ "$latest" != "$before" ]; then cp -r "$latest" "$OUT/$arm"; echo "  saved -> $OUT/$arm"; else echo "  NO REP"; fi
done
ko
echo "########## Q8 PROBE COMPLETE $(date +%H:%M:%S) ##########"
