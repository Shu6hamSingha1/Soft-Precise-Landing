#!/usr/bin/env bash
# Record N independent GT-feedback flights on a chosen scene (default cm_col),
# each with raw frames saved, so EVERY detector variant can be re-scored offline
# against the SAME real trajectories via tools/validate_detector_gt.py -- no
# closed-loop confound (detector failures can't change the flight path under
# GT-FB), and no need to fly twice per arm (one recording answers every variant).
#
# Pattern copied from test_data/Rover_AB_harness/record_robustness_set.sh (which
# recorded ONE flight per scene, n=1) -- this does N independent flights of ONE
# scene, for a much larger offline-comparable sample than any single-flight
# recording or the small-n perception-mode SITL A/Bs run so far this thread.
#
# PLASMC_GT_FEEDBACK=1 ON PURPOSE -- the detector under test must NOT gate the
# flight. CROSS_RING_OVERLAY_DBG=0 -> clean frames (no drawn debug pixels).
#
#   WORLD=cm_col    N=5 bash test_data/GtfbMulti_col/record_col_gtfb_multi.sh
#   WORLD=cm_dim    N=5 TAG=dim bash test_data/GtfbMulti_col/record_col_gtfb_multi.sh
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
SCENE="${WORLD:-cm_col}"; N="${N:-5}"; TAG="${TAG:-col}"
OUT="$SD/test_data/GtfbMulti_${TAG}"; mkdir -p "$OUT"
QT_PRELOAD=/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5
# ⛔⛔ PER-REP FOREIGN-SESSION CHECK -- see CmInv_AB_harness/cminv_ab.sh for the
# full incident/rationale.
if [ "${FORCE:-0}" != "1" ]; then
  _busy=$(ps -eo args | grep -aE 'px4_sitl_default/bin/px4|gz-sim|MicroXRCEAgent|landing_test\.py' | grep -av grep | head -3)
  if [ -n "$_busy" ]; then
    echo "REFUSING TO START: SITL is already running (another session?):" >&2
    echo "$_busy" | cut -c1-100 >&2
    echo "Wait for it to finish, or re-run with FORCE=1 if you are certain it is yours." >&2
    exit 3
  fi
fi
SITL_PAT='px4_sitl_default/bin/px4|gz sim|gz-sim|ign gazebo|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'
foreign_check(){
  local ctx="$1"
  local p=$(ps -eo args | grep -aE "$SITL_PAT" | grep -av grep | head -3)
  if [ -n "$p" ]; then
    echo "REFUSING TO CONTINUE ($ctx): SITL processes survived our own cleanup --" >&2
    echo "this can only mean something else is running them, not us:" >&2
    echo "$p" | cut -c1-100 >&2
    echo "Stopping rather than risk killing a live session. No further ko()/launch will run." >&2
    exit 4
  fi
}
ko(){
  for r in 1 2 3; do
    pids=$(ps -eo pid,args|grep -aE "$SITL_PAT"|grep -av grep|awk '{print $1}')
    [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null
  for w in 1 2 3 4 5 6; do gz topic -l 2>/dev/null | grep -q '/clock' || break; sleep 2; done
  for w in 1 2 3 4 5; do
    hp=$(ss -ulpn 2>/dev/null | awk '/:8888 /{print}')
    [ -z "$hp" ] && break
    fuser -k 8888/udp 2>/dev/null; sleep 2
  done
  sleep 3
  foreign_check "post-cleanup, before next launch"
}
for rep in $(seq 1 "$N"); do
  echo "[gtfb-multi:$TAG] === rep $rep/$N (world $SCENE) ==="
  ko
  B=$(ls -td "$SD/test_data/Test_Videos/"*_raw 2>/dev/null|head -1)
  env LD_PRELOAD="$QT_PRELOAD" HEADLESS=1 WORLD="$SCENE" MARKER_TYPE=cross \
      PLASMC_GT_FEEDBACK=1 IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0 \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 INITIAL_DRONE_ENU="2.0,2.0,5.0" \
      timeout 260 bash "$SD/scripts/run_landing_retry.sh" \
      > "$SD/run_logs/gtfb_${TAG}_${rep}.out" 2>&1
  R=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
  F=$(ls -td "$SD/test_data/Test_Videos/"*_raw 2>/dev/null|head -1)
  if [ -z "$F" ] || [ "$F" = "$B" ]; then echo "[gtfb-multi:$TAG] rep $rep: NO FRAMES"; continue; fi
  D="$OUT/rep${rep}"; mkdir -p "$D/frames"
  for f in Ground_Truth.npy Img_Data.npy Control_Data.npy Img_Params.txt; do
    [ -f "$R/$f" ] && cp "$R/$f" "$D/"; done
  cp "$F"/f*.png "$D/frames/" 2>/dev/null
  echo "[gtfb-multi:$TAG] rep $rep: $(ls "$D/frames" | wc -l) frames  <- $(basename "$R")"
done
ko; echo "GTFB_MULTI_DONE" >> "$SD/run_logs/gtfb_${TAG}.flag"
