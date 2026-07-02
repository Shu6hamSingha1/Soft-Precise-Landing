#!/usr/bin/env bash
# Run chase-recorded rover+platform landings until one lands cleanly on the
# platform (min_alt 0.4-0.9 m, lateral < 1 m, no fly-away). Keep that chase mp4.
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
MAXTRIES="${MAXTRIES:-5}"
for try in $(seq 1 "$MAXTRIES"); do
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
  echo "===== CHASE try $try/$MAXTRIES ====="
  # Record BOTH first-person (drone down-cam via IMG_RECORD) and third-person
  # (chase cam). Both mp4s land in test_data/Test_Videos/ (chase_* = 3rd person,
  # the other timestamp = drone cam).
  HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=0 CHASE_CAM=1 RECORD_S=35 \
    IMG_RECORD=1 IMG_RECORD_TAIL_S=3 \
    PY_TIMEOUT_S=150 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 \
    bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/chase_try_${try}.out" 2>&1
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
  # evaluate the latest landing
  d=$(ls -dt "$SD"/test_data/Landing_Test/*/ 2>/dev/null | head -1)
  chase=$(ls -t "$SD"/test_data/Test_Videos/chase_*.mp4 2>/dev/null | head -1)
  verdict=$(/home/shubham/ws/scripts/env2025/bin/python3 - "$d" <<PY
import sys,numpy as np,os
d=sys.argv[1]
try:
    gt=np.load(os.path.join(d,'Ground_Truth.npy'),allow_pickle=True).item()
    up=gt['UAV Pose']; tp=gt['Target Pose']
    z=np.array([p.position.z for p in up]); x=np.array([p.position.x for p in up]); y=np.array([p.position.y for p in up])
    tx=np.array([p.position.x for p in tp]); ty=np.array([p.position.y for p in tp])
    n=min(len(z),len(tx)); lat=np.hypot(x[:n]-tx[:n],y[:n]-ty[:n]); imin=int(np.argmin(z[:n]))
    clean = (0.35<z[imin]<0.95) and lat[imin]<1.0 and z.max()<15
    print(f"{'CLEAN' if clean else 'DIRTY'} min_alt={z[imin]:.2f} lat={lat[imin]:.2f} peak={z.max():.1f}")
except Exception as e:
    print("ERR",e)
PY
)
  drone=$(ls -t "$SD"/test_data/Test_Videos/*.mp4 2>/dev/null | grep -v '/chase_' | head -1)
  echo "[chase] try $try -> $verdict"
  echo "         chase=$chase"
  echo "         drone=$drone"
  if echo "$verdict" | grep -q CLEAN; then
    echo "CLEAN_CHASE=$chase"
    echo "CLEAN_DRONE=$drone"
    break
  fi
done
echo "===== CHASE LOOP DONE ====="
