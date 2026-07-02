#!/usr/bin/env bash
# CYCLE-AMPLITUDE lever test: heading-hold Circular + P2INF_xy widening.
# Baseline = yawhold_arm_n3 (P2INF vdf 1.0: 0/4, cycle amp ~0.7, e-rot 1.11).
# Cells 2.0 / 3.0 (the stationary "0 fly-away" lever). All 3 axes pinned
# (single-axis env bypasses the VDF auto-align -> hot defaults).
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-3}"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
for val in 2.0 3.0; do
  out="$LOG/p2inf_$val"; mkdir -p "$out"
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "----- p2inf=$val rep $rep/$N -----"
    env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 \
        PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0 \
        PLASMC_P2INF_X="$val" PLASMC_P2INF_Y="$val" PLASMC_P2INF_Z=1.5 \
        PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" \
        bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/p2inf_${val}_${rep}.out" 2>&1
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    [ -z "$d" ] && { echo "[RESULT] p2inf=$val rep$rep: NODATA"; continue; }
    /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$val" "$rep" <<'PY'
import sys, numpy as np, os
d, val, rep = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    up, tp, T = gt['UAV Pose'], gt['Target Pose'], np.array(gt['Time'], float)
    n = min(len(up), len(tp), len(T)); T = T[:n] - T[0]
    uz = np.array([p.position.z for p in up[:n]])
    ux = np.array([p.position.x for p in up[:n]]); uy = np.array([p.position.y for p in up[:n]])
    tx = np.array([p.position.x for p in tp[:n]]); ty = np.array([p.position.y for p in tp[:n]])
    ex = ux - tx; ey = uy - ty; lat = np.hypot(ex, ey); imin = int(np.argmin(uz))
    m = (uz < 3.5) & (uz > 0.8)
    if m.sum() > 30:
        ang = np.unwrap(np.arctan2(ey[m], ex[m])); dt = np.gradient(T[m]); dt[dt <= 0] = 1e-3
        erot = float(np.median(np.gradient(ang) / dt)); emean = float(lat[m].mean())
        latm = lat[m]; trend = np.convolve(latm, np.ones(51)/51, 'same')
        oscstd = float(np.std(latm - trend))
    else:
        erot = emean = oscstd = float('nan')
    ok = (0.35 < uz[imin] < 0.95) and lat[imin] < 0.3 and uz.max() < 15
    v = 'ON-PLATFORM' if ok else ('NEAR' if (lat[imin] < 1.0 and uz.max() < 15 and 0.35 < uz[imin] < 0.95) else 'MISS/FAIL')
    print(f"[RESULT] p2inf={val} rep{rep}: {v} lat={lat[imin]:.3f} min_alt={uz[imin]:.2f} "
          f"| e_mean={emean:.2f} e_rot={erot:+.2f} osc_std={oscstd:.3f} peak={uz.max():.1f}")
except Exception as e:
    print(f"[RESULT] p2inf={val} rep{rep}: ERR {e}")
PY
  done
done
echo "P2INF DONE"
