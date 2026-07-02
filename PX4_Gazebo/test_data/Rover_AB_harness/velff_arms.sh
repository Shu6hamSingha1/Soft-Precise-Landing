#!/usr/bin/env bash
# Target-motion velocity-FF arms (PLASMC_TGT_VEL_FF=1):
#   circvff: Circular r=0.8 + yaw FF + vel FF (n=N) -> the curved-lag fix test
#   linvff:  Linear 0.3 + both FF (n=1)             -> regression (FF ~no-op)
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-3}"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
run_arm() {
  local arm="$1" reps="$2"; shift 2
  local out="$LOG/velff_$arm"; mkdir -p "$out"
  for rep in $(seq 1 "$reps"); do
    stray_clean
    echo "----- $arm rep $rep/$reps -----"
    env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 PLASMC_YAW_ALPHA_FILT=0 \
        PLASMC_YAW_OMEGA_D_FF=1 PLASMC_TGT_VEL_FF=1 PLASMC_GT_TGT_LEAD=0.8 \
        PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" "$@" \
        bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/velff_${arm}_${rep}.out" 2>&1
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    [ -z "$d" ] && { echo "[RESULT] $arm rep$rep: NODATA"; continue; }
    /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$arm" "$rep" <<'PY'
import sys, numpy as np, os
d, arm, rep = sys.argv[1], sys.argv[2], sys.argv[3]
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
    m = (uz < 3) & (uz > 1)
    stlag = lat[m].mean() if m.sum() > 5 else float('nan')
    ok = (0.35 < uz[imin] < 0.95) and lat[imin] < 0.3 and uz.max() < 15
    v = 'ON-PLATFORM' if ok else ('NEAR' if (lat[imin] < 1.0 and uz.max() < 15 and 0.35 < uz[imin] < 0.95) else 'MISS/FAIL')
    print(f"[RESULT] {arm} rep{rep}: {v} min_alt={uz[imin]:.2f} lat={lat[imin]:.3f} "
          f"relspd={rs[imin]:.2f} steady_lag={stlag:.2f} peak={uz.max():.1f}")
except Exception as e:
    print(f"[RESULT] {arm} rep{rep}: ERR {e}")
PY
  done
}
run_arm circvff "$N" ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0
run_arm linvff 1 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3
echo "VELFF DONE"
