#!/usr/bin/env bash
# CONTROL: AU_LEAD OFF + TERMINAL_COMMIT=0, heading-hold Circular (isolates commit-off
# from the lead). Compare to baseline yawhold_arm_n3 (AU_LEAD off + commit ON, 0/4, e_rot 1.11).
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
OUTBASE="$SD/test_data/Rover_Turning/leadoff_commitoff"
LOG="$SD/test_data/Rover_AB_harness"
N="${N:-3}"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
out="$OUTBASE"; mkdir -p "$out"
for rep in $(seq 1 "$N"); do
  stray_clean
  echo "----- leadoff_commitoff rep $rep/$N -----"
  env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 \
      PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0 \
      PLASMC_TERMINAL_COMMIT=0 \
      PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" \
      bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/leadoff_commitoff_${rep}.out" 2>&1
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
  d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
  [ -z "$d" ] && { echo "[RESULT] rep$rep: NODATA"; continue; }
  /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$rep" <<'PY'
import sys, numpy as np, os
d, rep = sys.argv[1], sys.argv[2]
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
        latm = lat[m]; trend = np.convolve(latm, np.ones(51)/51, 'same'); oscstd = float(np.std(latm - trend))
    else: erot = emean = oscstd = float('nan')
    ok = (0.35 < uz[imin] < 0.95) and lat[imin] < 0.3 and uz.max() < 15
    v = 'ON-PLATFORM' if ok else ('NEAR' if (lat[imin] < 1.0 and uz.max() < 15 and 0.35 < uz[imin] < 0.95) else 'MISS/FAIL')
    print(f"[RESULT] leadoff_commitoff rep{rep}: {v} lat={lat[imin]:.3f} min_alt={uz[imin]:.2f} "
          f"| e_mean={emean:.2f} e_rot={erot:+.2f} osc_std={oscstd:.3f} peak={uz.max():.1f}")
except Exception as e:
    print(f"[RESULT] rep{rep}: ERR {e}")
PY
done
echo "LEADOFF_COMMITOFF DONE"
