#!/usr/bin/env bash
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
run_one() {
  local tag="$1"; shift
  stray_clean
  echo "----- cfree $tag -----"
  env HEADLESS=1 PLASMC_GT_FEEDBACK=1 COMPASS_FREE_VALIDATE=1 PY_TIMEOUT_S=180 MAX_ATTEMPTS=5 \
      LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$LOG/cfree" "$@" \
      bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/cfree_${tag}.out" 2>&1
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
  d=$(ls -dt "$LOG"/cfree/*/ 2>/dev/null | head -1)
  grep -q 'EKF2_MAG_TYPE = 5' "$LOG/cfree_${tag}.out" && MAGOFF=yes || MAGOFF=NO
  /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$tag" "$MAGOFF" <<'PY'
import sys, numpy as np, os
d, tag, magoff = sys.argv[1], sys.argv[2], sys.argv[3]
gt=np.load(os.path.join(d,'Ground_Truth.npy'),allow_pickle=True).item()
c=np.load(os.path.join(d,'Control_Data.npy'),allow_pickle=True).item()
up,tp=gt['UAV Pose'],gt['Target Pose']; n=min(len(up),len(tp))
uz=np.array([p.position.z for p in up[:n]])
ux=np.array([p.position.x for p in up[:n]]); uy=np.array([p.position.y for p in up[:n]])
tx=np.array([p.position.x for p in tp[:n]]); ty=np.array([p.position.y for p in tp[:n]])
lat=np.hypot(ux-tx,uy-ty); imin=int(np.argmin(uz))
ea=np.degrees(np.array(c['e_a(t)']).reshape(-1))
tmove=np.hypot(tx[-1]-tx[0],ty[-1]-ty[0])
ok=(0.35<uz[imin]<0.95) and lat[imin]<0.3 and uz.max()<15
print(f"[RESULT] cfree {tag}: {'CLEAN' if ok else 'FAIL'} min={uz[imin]:.2f} lat={lat[imin]:.3f} "
      f"e_a_end {ea[-1]:+.1f} rover_moved={tmove:.1f} magoff={magoff}")
PY
}
run_one stat2 ROVER_MOTION=0
run_one moving ROVER_MOTION=1 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3 PLASMC_YAW_ALPHA_FILT=0
echo "CFREE DONE"
