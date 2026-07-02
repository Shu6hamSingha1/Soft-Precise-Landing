#!/usr/bin/env bash
# Controlled A/B for the terminal fly-away: same GT-FB baked config, only the
# WORLD differs. Arm A = stationary aruco, Arm B = rover (stationary). n=3 each.
# Each rep's Ground_Truth autosaves to a per-arm folder for offline metric
# extraction. Not tracked (diagnostic one-off).
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-3}"
export HEADLESS=1 PLASMC_GT_FEEDBACK=1 PY_TIMEOUT_S=150 MAX_ATTEMPTS=3 LANDING_AUTOSAVE=1

stray_clean() {
  # Kill any lingering sim procs so port 8888 / gz world are free before a rep.
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null
  sleep 2
}

run_arm() {
  local arm="$1" launcher="$2"; shift 2
  local out="$LOG/ab_$arm"
  rm -rf "$out"; mkdir -p "$out"
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "===== ARM $arm rep $rep/$N ====="
    LANDING_OUT_BASE="$out" "$@" bash "$SD/scripts/$launcher" \
      > "$LOG/ab_${arm}_${rep}.out" 2>&1
    echo "[ab] $arm rep $rep rc=$?"
    # keep disk in check
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    sleep 3
  done
}

cd "$SD"
run_arm aruco run_aruco_landing_retry.sh
run_arm rover run_rover_landing_retry.sh env ROVER_MOTION=0
echo "===== AB DONE ====="
ls -d "$LOG"/ab_aruco/*/ "$LOG"/ab_rover/*/ 2>/dev/null | wc -l
