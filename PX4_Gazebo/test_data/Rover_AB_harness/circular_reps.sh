#!/usr/bin/env bash
# TURNING-rover validation: Circular r=0.8, wz=0.48 rad/s (~27 deg/s, 0.38 m/s
# tangential), GT-FB, YAW_ALPHA_FILT=0 (the 0.30 rad/s alpha cap would clamp the
# genuine 0.48 turn; GT-FB has no corruption to reject). n reps, per-rep verdict.
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-3}"
out="$LOG/circ_1.0"; rm -rf "$out"; mkdir -p "$out"

stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}

for rep in $(seq 1 "$N"); do
  stray_clean
  echo "----- circular rep $rep/$N -----"
  HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Circular \
    ROVER_SPEED_MULT=1.0 PLASMC_YAW_ALPHA_FILT=0 \
    PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" \
    bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/circ_${rep}.out" 2>&1
  rc=$?
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
  d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
  if [ -z "$d" ]; then echo "[RESULT] circ rep$rep: NODATA rc=$rc"; continue; fi
  /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$rep" <<'PY'
import sys, numpy as np, os
d, rep = sys.argv[1], sys.argv[2]
try:
    gt = np.load(os.path.join(d, 'Ground_Truth.npy'), allow_pickle=True).item()
    up, tp, T = gt['UAV Pose'], gt['Target Pose'], np.array(gt['Time'], float)
    n = min(len(up), len(tp), len(T))
    uz = np.array([p.position.z for p in up[:n]])
    ux = np.array([p.position.x for p in up[:n]]); uy = np.array([p.position.y for p in up[:n]])
    tx = np.array([p.position.x for p in tp[:n]]); ty = np.array([p.position.y for p in tp[:n]])
    lat = np.hypot(ux-tx, uy-ty); imin = int(np.argmin(uz))
    tmove = np.hypot(tx[-1]-tx[0], ty[-1]-ty[0])
    ok = (0.35 < uz[imin] < 0.95) and lat[imin] < 0.3 and uz.max() < 15
    near = (0.35 < uz[imin] < 0.95) and lat[imin] < 1.0 and uz.max() < 15
    v = 'ON-PLATFORM' if ok else ('NEAR-MISS' if near else 'FAIL')
    # yaw tracking from Control_Data e_a
    try:
        c = np.load(os.path.join(d, 'Control_Data.npy'), allow_pickle=True).item()
        ea = np.degrees(np.array(c['e_a(t)']).reshape(-1))
        yawinfo = f"e_a max|{np.abs(ea).max():.0f}| end {ea[-1]:+.0f} deg"
    except Exception:
        yawinfo = "e_a n/a"
    print(f"[RESULT] circ rep{rep}: {v} min_alt={uz[imin]:.2f} lat={lat[imin]:.3f} "
          f"peak={uz.max():.1f} rover_moved={tmove:.1f} | {yawinfo}")
except Exception as e:
    print(f"[RESULT] circ rep{rep}: ERR {e}")
PY
done
echo "CIRC DONE"
