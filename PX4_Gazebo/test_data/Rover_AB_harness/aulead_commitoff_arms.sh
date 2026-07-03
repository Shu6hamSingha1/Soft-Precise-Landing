#!/usr/bin/env bash
# PLASMC_AU_LEAD test (user-approved 2026-07-03): lateral phase-lead on I_a xy.
# Sized by the corrected cycle mechanism: damping needs command quadrature
# chi > W*tau ~ 25-40 deg at W* = 1.3-1.7 rad/s. Arms:
#   0.9_3.5 = +35 deg @1.4 (HF x3.9)   |   0.7_5.0 = +48 deg @1.4 (HF x7.1)
# Heading-hold Circular (comparability with yawhold_arm_n3 / cycle_gain_sweep /
# dhd_src_sweep). Baseline: 0/4, e_rot 1.11, A~0.45, P_cyc>0.
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
OUTBASE="$SD/test_data/Rover_Turning/aulead_commitoff"
LOG="$SD/test_data/Rover_AB_harness"
N="${N:-3}"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
val=0.9_3.5
wz="${val%_*}"; wp="${val#*_}"
for RATIO in 0.5; do
  out="$OUTBASE/lead_${val}_r${RATIO}"; mkdir -p "$out"
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "----- lead=$val rep $rep/$N -----"
    env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 \
        PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0 \
        PLASMC_AU_LEAD=1 PLASMC_AU_LEAD_WZ="$wz" PLASMC_AU_LEAD_WP="$wp" PLASMC_AU_LEAD_RATIO="$RATIO" PLASMC_TERMINAL_COMMIT=0 \
        PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" \
        bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/aulead_${val}_r${RATIO}_${rep}.out" 2>&1
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    [ -z "$d" ] && { echo "[RESULT] lead=$val rep$rep: NODATA"; continue; }
    grep -m1 "PLASMC_AU_LEAD" "$LOG/aulead_${val}_r${RATIO}_${rep}.out" || echo "[WARN] lead=$val rep$rep: AU_LEAD banner MISSING"
    /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "${val}_r${RATIO}" "$rep" <<'PY'
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
    print(f"[RESULT] lead={val} rep{rep}: {v} lat={lat[imin]:.3f} min_alt={uz[imin]:.2f} "
          f"| e_mean={emean:.2f} e_rot={erot:+.2f} osc_std={oscstd:.3f} peak={uz.max():.1f}")
except Exception as e:
    print(f"[RESULT] lead={val} rep{rep}: ERR {e}")
PY
  done
done
echo "AULEAD ARMS DONE"
