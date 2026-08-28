#!/usr/bin/env bash
# Record successful moving-platform landings (chase-cam external view) and montage them.
# Usage: LABEL=linear  CONFIG_ENV="ROVER_SPEED_MULT=0.3" NEED=3 bash record_montage.sh
#        LABEL=circular CONFIG_ENV="ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 PLASMC_AU_LEAD=1 PLASMC_AU_LEAD_WZ=0.9 PLASMC_AU_LEAD_WP=3.5 PLASMC_AU_LEAD_RATIO=0.5 PLASMC_TERMINAL_COMMIT=0 PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0" NEED=3 bash record_montage.sh
# Keeps chase mp4s of ON-PLATFORM landings into a labeled dir; ffmpeg-concats at the end.
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
VID="$SD/test_data/Test_Videos"
LABEL="${LABEL:?set LABEL}"
CONFIG_ENV="${CONFIG_ENV:-}"
NEED="${NEED:-3}"
MAXTRY="${MAXTRY:-8}"
KEEP="$VID/montage_${LABEL}"; mkdir -p "$KEEP"
OUTBASE="$SD/test_data/Rover_Turning/montage_${LABEL}"; mkdir -p "$OUTBASE"
LOG="$SD/test_data/Rover_AB_harness"
stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py|rover_drive.py|mavsdk_server|record_chase.py'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}
got=0
for try in $(seq 1 "$MAXTRY"); do
  [ "$got" -ge "$NEED" ] && break
  stray_clean
  ts_before=$(ls -t "$VID"/chase_*.mp4 2>/dev/null | head -1)
  echo "----- $LABEL try $try/$MAXTRY (have $got/$NEED) -----"
  env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 CHASE_CAM=1 IMG_RECORD=1 RECORD_S=28 \
      $CONFIG_ENV \
      PY_TIMEOUT_S=180 MAX_ATTEMPTS=4 LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$OUTBASE" \
      bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/montage_${LABEL}_${try}.out" 2>&1
  rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
  d=$(ls -dt "$OUTBASE"/*/ 2>/dev/null | head -1)
  vid=$(ls -t "$VID"/chase_*.mp4 2>/dev/null | head -1)
  [ -z "$d" ] && { echo "  NODATA"; continue; }
  [ "$vid" = "$ts_before" ] && { echo "  NO NEW VIDEO"; continue; }
  res=$(/home/shubham/ws/scripts/env2025/bin/python3 - "$d" <<'PY'
import sys, numpy as np, os
d=sys.argv[1]
try:
    gt=np.load(os.path.join(d,'Ground_Truth.npy'),allow_pickle=True).item()
    up,tp=gt['UAV Pose'],gt['Target Pose']; n=min(len(up),len(tp))
    uz=np.array([p.position.z for p in up[:n]])
    ex=np.array([p.position.x for p in up[:n]])-np.array([p.position.x for p in tp[:n]])
    ey=np.array([p.position.y for p in up[:n]])-np.array([p.position.y for p in tp[:n]])
    lat=np.hypot(ex,ey); i=int(np.argmin(uz))
    ok=(0.35<uz[i]<0.95) and lat[i]<0.3 and uz.max()<15
    print(f"{'ON' if ok else 'NO'} {lat[i]:.3f}")
except Exception as e: print(f"NO err")
PY
)
  echo "  result: $res  video: $(basename "$vid")"
  if [[ "$res" == ON* ]]; then
    got=$((got+1))
    cp "$vid" "$KEEP/${LABEL}_$(printf %02d $got)_lat${res#ON }.mp4"
    echo "  KEPT ($got/$NEED)"
  fi
done
echo "=== $LABEL collected $got clips ==="
# montage: normalize + concat kept clips
cd "$KEEP"
clips=( $(ls -1 ${LABEL}_*.mp4 2>/dev/null) )
if [ "${#clips[@]}" -ge 1 ]; then
  : > concat.txt
  for c in "${clips[@]}"; do
    ffmpeg -y -i "$c" -vf "scale=960:-2,setsar=1,fps=30" -an -c:v libx264 -pix_fmt yuv420p "norm_$c" >/dev/null 2>&1
    echo "file 'norm_$c'" >> concat.txt
  done
  ffmpeg -y -f concat -safe 0 -i concat.txt -c copy "$VID/montage_${LABEL}.mp4" >/dev/null 2>&1 \
    && echo "MONTAGE -> test_data/Test_Videos/montage_${LABEL}.mp4 (${#clips[@]} clips)" \
    || echo "MONTAGE FAILED"
else
  echo "NO CLIPS to montage"
fi
echo "MONTAGE_${LABEL}_DONE"
