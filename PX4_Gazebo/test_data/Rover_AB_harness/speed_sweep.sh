#!/usr/bin/env bash
# Moving-rover SPEED SWEEP: Linear trajectory, GT-FB, SPEED_MULT cells at n reps
# each. Per-rep verdict printed inline; reps autosaved per-cell for offline
# analysis. (0.3 cell already done earlier: 3/3 on-platform.)
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-3}"
CELLS="${CELLS:-0.5 0.7 1.0}"

stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}

for mult in $CELLS; do
  out="$LOG/spd_$mult"; rm -rf "$out"; mkdir -p "$out"
  echo "================ SPEED_MULT=$mult (v=$(echo "$mult" | awk '{printf "%.2f", $1*1.556}') m/s) ================"
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "----- mult=$mult rep $rep/$N -----"
    HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Linear \
      ROVER_SPEED_MULT="$mult" PLASMC_YAW_ALPHA_FILT=0 \
      PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" \
      bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/spd_${mult}_${rep}.out" 2>&1
    rc=$?
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    if [ -z "$d" ]; then echo "[RESULT] mult=$mult rep$rep: NODATA rc=$rc"; continue; fi
    /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$mult" "$rep" <<'PY'
import sys, numpy as np, os
d, mult, rep = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    up, tp, T = gt['UAV Pose'], gt['Target Pose'], np.array(gt['Time'], float)
    n = min(len(up), len(tp), len(T))
    uz = np.array([p.position.z for p in up[:n]])
    ux = np.array([p.position.x for p in up[:n]]); uy = np.array([p.position.y for p in up[:n]])
    tx = np.array([p.position.x for p in tp[:n]]); ty = np.array([p.position.y for p in tp[:n]])
    lat = np.hypot(ux-tx, uy-ty); imin = int(np.argmin(uz))
    dt = np.gradient(T[:n]); dt[dt <= 0] = 1e-3
    rs = np.hypot(np.gradient(ux-tx)/dt, np.gradient(uy-ty)/dt)
    rs = np.convolve(rs, np.ones(7)/7, 'same')
    tmove = np.hypot(tx[-1]-tx[0], ty[-1]-ty[0])
    ok = (0.35 < uz[imin] < 0.95) and lat[imin] < 0.3 and uz.max() < 15
    near = (0.35 < uz[imin] < 0.95) and lat[imin] < 1.0 and uz.max() < 15
    v = 'ON-PLATFORM' if ok else ('NEAR-MISS' if near else 'FAIL')
    print(f"[RESULT] mult={mult} rep{rep}: {v} min_alt={uz[imin]:.2f} lat={lat[imin]:.3f} "
          f"relspd={rs[imin]:.2f} peak={uz.max():.1f} rover_moved={tmove:.1f}")
except Exception as e:
    print(f"[RESULT] mult={mult} rep{rep}: ERR {e}")
PY
  done
done
echo "================ SWEEP DONE ================"
