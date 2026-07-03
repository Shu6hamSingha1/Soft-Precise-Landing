#!/usr/bin/env bash
# Record a MOVING-platform landing (chase + onboard down-cam + GT) and build the
# synced animated-graph montage for the first ON-PLATFORM landing.
# Usage: LABEL=linear   CONFIG_ENV="ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3" bash montage_landing.sh
#        LABEL=circular  CONFIG_ENV="ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 PLASMC_AU_LEAD=1 PLASMC_AU_LEAD_WZ=0.9 PLASMC_AU_LEAD_WP=3.5 PLASMC_AU_LEAD_RATIO=0.5 PLASMC_TERMINAL_COMMIT=0 PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0" bash montage_landing.sh
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
VID="$SD/test_data/Test_Videos"; LT="$SD/test_data/Landing_Test"
LABEL="${LABEL:?set LABEL}"; CONFIG_ENV="${CONFIG_ENV:-}"; MAXTRY="${MAXTRY:-8}"
OUTBASE="$LT"     # landing_test autosaves here; down-cam video also -> Test_Videos
LOG="$SD/test_data/Rover_AB_harness"
newest(){ ls -t $1 2>/dev/null | head -1; }
stray_clean(){
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server|record_chase.py'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
for try in $(seq 1 "$MAXTRY"); do
  stray_clean
  pre_chase=$(newest "$VID/chase_*.mp4"); pre_run=$(ls -dt "$LT"/*/ 2>/dev/null|head -1)
  pre_drone=$(ls -t "$VID"/*.mp4 2>/dev/null | grep -vE 'chase|montage' | head -1)
  echo "----- montage $LABEL try $try/$MAXTRY -----"
  env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 CHASE_CAM=1 IMG_RECORD=1 RECORD_S=15 \
      $CONFIG_ENV PY_TIMEOUT_S=180 MAX_ATTEMPTS=4 LANDING_AUTOSAVE=1 \
      bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/montage_${LABEL}_${try}.out" 2>&1
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
  run=$(ls -dt "$LT"/*/ 2>/dev/null | head -1)
  chase=$(newest "$VID/chase_*.mp4")
  drone=$(ls -t "$VID"/*.mp4 2>/dev/null | grep -vE 'chase|montage' | head -1)
  [ "$run" = "$pre_run" ] && { echo "  NO RUN"; continue; }
  [ "$chase" = "$pre_chase" ] && { echo "  NO CHASE VIDEO"; continue; }
  [ "$drone" = "$pre_drone" ] && { echo "  NO DRONE VIDEO"; continue; }
  res=$(/home/shubham/ws/scripts/env2025/bin/python3 - "$run" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
    gt=np.load(os.path.join(d,'Ground_Truth.npy'),allow_pickle=True).item()
    up,tp=gt['UAV Pose'],gt['Target Pose']; n=min(len(up),len(tp))
    uz=np.array([p.position.z for p in up[:n]])
    ex=np.array([p.position.x for p in up[:n]])-np.array([p.position.x for p in tp[:n]])
    ey=np.array([p.position.y for p in up[:n]])-np.array([p.position.y for p in tp[:n]])
    lat=np.hypot(ex,ey); i=int(np.argmin(uz))
    print(f"{'ON' if (0.35<uz[i]<0.95 and lat[i]<0.3 and uz.max()<15) else 'NO'} {lat[i]:.3f}")
except Exception: print("NO err")
PY
)
  echo "  result: $res | run=$(basename "$run") chase=$(basename "$chase") drone=$(basename "$drone")"
  if [[ "$res" == ON* ]]; then
    out="$VID/montage_${LABEL}.mp4"
    /home/shubham/ws/scripts/env2025/bin/python3 "$SD/tools/make_landing_montage.py" \
        --chase "$chase" --drone "$drone" --run "$run" --out "$out" --tail-s 1.0 2>&1 | grep -E 'montage\]' | head -2
    echo "MONTAGE_${LABEL} -> test_data/Test_Videos/montage_${LABEL}.mp4 (lat ${res#ON })"
    exit 0
  fi
done
echo "MONTAGE_${LABEL}_FAILED (no on-platform landing in $MAXTRY tries)"
