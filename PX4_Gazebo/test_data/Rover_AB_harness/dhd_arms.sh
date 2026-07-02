#!/usr/bin/env bash
# DHD-SOURCE lever test (the COMPLETE k_r job): heading-hold Circular, keep the k_r
# recovery FUNCTION in h_d (HD_KR=0.5 default) but drop its barrier-inflated branch
# from dh_d (c3) via PLASMC_DHD_SRC. Clone of hdkr_arms.sh (canonical A/B pattern).
#   nokr = differentiate h_d_noS + p_10*S_r*dp_r (drop only the -k_r*zeta_r/g_r branch)
#   nos  = differentiate h_d_noS only (pre-06-29 s_ddot-drop of the whole rate term)
# Baselines for comparison (same binary, 2026-07-02): yawhold_arm_n3 (full+kr0.5:
# 0/4, e_mean 0.70, e_rot 1.11) and cycle_gain_sweep hdkr_0 (1/3 ON 0.200, e_rot 0.77-0.89).
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
OUTBASE="$SD/test_data/Rover_Turning/dhd_src_sweep"
LOG="$SD/test_data/Rover_AB_harness"
N="${N:-3}"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
for val in nokr nos; do
  out="$OUTBASE/dhd_$val"; mkdir -p "$out"
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "----- dhd=$val rep $rep/$N -----"
    env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 \
        PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0 \
        PLASMC_DHD_SRC="$val" \
        PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" \
        bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/dhd_${val}_${rep}.out" 2>&1
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    [ -z "$d" ] && { echo "[RESULT] dhd=$val rep$rep: NODATA"; continue; }
    grep -m1 "PLASMC_DHD_SRC" "$LOG/dhd_${val}_${rep}.out" || echo "[WARN] dhd=$val rep$rep: DHD_SRC banner MISSING"
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
    print(f"[RESULT] dhd={val} rep{rep}: {v} lat={lat[imin]:.3f} min_alt={uz[imin]:.2f} "
          f"| e_mean={emean:.2f} e_rot={erot:+.2f} osc_std={oscstd:.3f} peak={uz.max():.1f}")
except Exception as e:
    print(f"[RESULT] dhd={val} rep{rep}: ERR {e}")
PY
  done
done
echo "DHD ARMS DONE"
