#!/usr/bin/env bash
# Re-record rover_cross GT-FB flights WITH RAW FRAMES (IMG_RECORD=1) so the cross
# detector can be replayed offline (tools/validate_detector_gt.py). Motivation
# (2026-09-23): RecordGTFB_dev/IC_RoverCross has only lossy mp4s; IC5 (3 m start)
# begins with the marker cropped at the frame edge and the tracked point on the
# plate corner; IC3/IC4 flicker with the marker fully in view.
#
#   ICS="IC5 IC3" N=3 bash test_data/RoverIC_raw/record_rover_ic_raw.sh
#   (add ROVER_MOTION=1 ROVER_TRAJ=<type> for a moving rover; default = static)
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
ICS="${ICS:-IC5 IC3}"; N="${N:-3}"; TAG="${TAG:-rover}"
OUT="$SD/test_data/RoverIC_raw/${TAG}"; mkdir -p "$OUT" "$SD/run_logs"
QT_PRELOAD=/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5
declare -A ENU=( [IC1]="0.0,0.0,5.0" [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0" )
if [ "${FORCE:-0}" != "1" ]; then
  _busy=$(ps -eo args | grep -aE 'px4_sitl_default/bin/px4|gz-sim|MicroXRCEAgent|landing_test\.py' | grep -av grep | head -3)
  [ -n "$_busy" ] && { echo "REFUSING: SITL already running:" >&2; echo "$_busy" | cut -c1-100 >&2; exit 3; }
fi
SITL_PAT='px4_sitl_default/bin/px4|gz sim|gz-sim|ign gazebo|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'
ko(){
  for r in 1 2 3; do
    pids=$(ps -eo pid,args|grep -aE "$SITL_PAT"|grep -av grep|awk '{print $1}')
    [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; sleep 3
}
for ic in $ICS; do for rep in $(seq 1 "$N"); do
  echo "[rover-raw] === $ic rep $rep/$N ==="
  ko
  B=$(ls -td "$SD/test_data/Test_Videos/"*_raw 2>/dev/null|head -1)
  env LD_PRELOAD="$QT_PRELOAD" HEADLESS=1 WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross \
      ROVER_MOTION="${ROVER_MOTION:-0}" ${ROVER_TRAJ:+ROVER_TRAJ=$ROVER_TRAJ} \
      PLASMC_GT_FEEDBACK=1 IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0 \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 INITIAL_DRONE_ENU="${ENU[$ic]}" \
      timeout 260 bash "$SD/scripts/run_rover_landing_retry.sh" > "$SD/run_logs/roverraw_${ic}_${rep}.out" 2>&1
  R=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
  F=$(ls -td "$SD/test_data/Test_Videos/"*_raw 2>/dev/null|head -1)
  if [ -z "$F" ] || [ "$F" = "$B" ]; then echo "[rover-raw] $ic rep $rep: NO FRAMES"; continue; fi
  D="$OUT/${ic}_rep${rep}"; mkdir -p "$D/frames"
  for f in Ground_Truth.npy Img_Data.npy Control_Data.npy Img_Params.txt; do [ -f "$R/$f" ] && cp "$R/$f" "$D/"; done
  cp "$F"/f*.png "$D/frames/" 2>/dev/null
  [ -f "$F/frames.tsv" ] && cp "$F/frames.tsv" "$D/frames/"      # exact frame<->Img_Data pairing (since 2026-09-24)
  # marker height for the true-bearing reference (validate_detector_gt.py reads meta.json;
  # default 0.0 is WRONG for rover_cross, whose marker sits 0.5 m above the rover origin)
  printf '{\n "world": "rover_cross",\n "marker_dz": 0.5,\n "case": "%s %s"\n}\n' \
      "${ROVER_TRAJ:-Static}" "$ic" > "$D/meta.json"
  echo "[rover-raw] $ic rep $rep: $(ls "$D/frames" | wc -l) frames <- $(basename "$R")"
done; done
ko; echo done >> "$SD/run_logs/roverraw_${TAG}.flag"
