#!/usr/bin/env bash
# Re-run the ROVER arm WITH the landing platform (n=3), same GT-FB config, to
# test whether the platform fixes the terminal fly-away.
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-3}"
export HEADLESS=1 PLASMC_GT_FEEDBACK=1 PY_TIMEOUT_S=150 MAX_ATTEMPTS=3 LANDING_AUTOSAVE=1
out="$LOG/ab_rover_plat"; rm -rf "$out"; mkdir -p "$out"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
cd "$SD"
for rep in $(seq 1 "$N"); do
  stray_clean
  echo "===== ROVER+PLATFORM rep $rep/$N ====="
  LANDING_OUT_BASE="$out" env ROVER_MOTION=0 bash "$SD/scripts/run_rover_landing_retry.sh" \
    > "$LOG/ab_rplat_${rep}.out" 2>&1
  echo "[ab] rplat rep $rep rc=$?"
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
done
echo "===== ROVER+PLATFORM DONE ====="
