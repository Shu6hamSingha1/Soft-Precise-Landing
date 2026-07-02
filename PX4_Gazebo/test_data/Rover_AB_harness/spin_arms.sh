#!/usr/bin/env bash
# Pure-spin isolation arms (rover PARKED, synthetic target spin 0.48 rad/s):
#   B: heading-hold (u_a=0)      -> isolates the spurious-w_z pathway (mech 2)
#   A: yaw SMC active (defaults) -> isolates the yaw ramp windup (mech 1)
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-2}"

stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}

run_arm() {
  local arm="$1"; shift
  local out="$LOG/spin_$arm"; mkdir -p "$out"
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "----- spin_$arm rep $rep/$N -----"
    env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=0 PLASMC_GT_SPIN_WZ=0.48 \
        PLASMC_YAW_ALPHA_FILT=0 PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 \
        LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" "$@" \
        bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/spin_${arm}_${rep}.out" 2>&1
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    [ -z "$d" ] && { echo "[RESULT] spin_$arm rep$rep: NODATA"; continue; }
    /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$arm" "$rep" <<'PY'
import sys, numpy as np, os
d, arm, rep = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    c = np.load(os.path.join(d, 'Control_Data.npy'), allow_pickle=True).item()
    up, tp = gt['UAV Pose'], gt['Target Pose']; n = min(len(up), len(tp))
    uz = np.array([p.position.z for p in up[:n]])
    ux = np.array([p.position.x for p in up[:n]]); uy = np.array([p.position.y for p in up[:n]])
    tx = np.array([p.position.x for p in tp[:n]]); ty = np.array([p.position.y for p in tp[:n]])
    lat = np.hypot(ux-tx, uy-ty); imin = int(np.argmin(uz))
    ua = np.array(c['u_a(t)']).reshape(-1)
    ea = np.degrees(np.array(c['e_a(t)']).reshape(-1))
    ok = (0.35 < uz[imin] < 0.95) and lat[imin] < 0.3 and uz.max() < 15
    v = 'ON-PLATFORM' if ok else ('NEAR' if (lat[imin] < 1.0 and uz.max() < 15 and 0.35 < uz[imin] < 0.95) else 'MISS/FAIL')
    print(f"[RESULT] spin_{arm} rep{rep}: {v} min_alt={uz[imin]:.2f} lat={lat[imin]:.3f} "
          f"peak={uz.max():.1f} ua_max={np.abs(ua).max():.3f} e_a_max|{np.abs(ea).max():.0f}| end {ea[-1]:+.0f}")
except Exception as e:
    print(f"[RESULT] spin_{arm} rep{rep}: ERR {e}")
PY
  done
}

# B: heading-hold + spin
run_arm hold PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0
# A: yaw SMC active + spin
run_arm active
echo "SPIN ARMS DONE"
