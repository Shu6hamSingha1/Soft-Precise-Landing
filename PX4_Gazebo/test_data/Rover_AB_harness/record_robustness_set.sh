#!/usr/bin/env bash
# Record the cross-marker ROBUSTNESS eval set: one descent per scene variant.
# Flies under PLASMC_GT_FEEDBACK=1 ON PURPOSE -- the detector under test must NOT gate the
# flight, or the conditions where it is blind (inverted polarity, chromatic) never take off
# and yield no frames. GT-FB gives every variant the SAME trajectory, so offline detector
# scores are comparable across scenes.
# CROSS_RING_OVERLAY_DBG=0 -> clean frames (the existing DetectorFrameset is overlay-
# contaminated: ~3-5% drawn pixels, 15-21% of the adaptive-gate mask; see
# feedback_detector_offline_replay_gotchas).
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
OUT="$SD/test_data/RobustnessFrameset"; mkdir -p "$OUT"
QT_PRELOAD=/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5
ko(){
  for r in 1 2 3; do
    pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|gz-sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}')
    [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null
  for w in 1 2 3 4 5 6; do gz topic -l 2>/dev/null | grep -q '/clock' || break; sleep 2; done
  sleep 3
}
# tag:world  -- base first so it is the matched reference
VARIANTS="${VARIANTS:-base:cross_marker dim:cm_dim bright:cm_bright lowsun:cm_lowsun inv:cm_inv col:cm_col darkbg:cm_darkbg}"
for spec in $VARIANTS; do
  tag="${spec%%:*}"; world="${spec##*:}"
  echo "[robust] === $tag (world $world)"
  ko
  B=$(ls -td "$SD/test_data/Test_Videos/"*_raw 2>/dev/null|head -1)
  env LD_PRELOAD="$QT_PRELOAD" HEADLESS=1 WORLD="$world" MARKER_TYPE=cross \
      PLASMC_GT_FEEDBACK=1 IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0 \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 INITIAL_DRONE_ENU="2.0,2.0,5.0" \
      timeout 260 bash "$SD/scripts/run_aruco_landing_retry.sh" \
      > "$SD/run_logs/robust_${tag}.out" 2>&1
  R=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
  F=$(ls -td "$SD/test_data/Test_Videos/"*_raw 2>/dev/null|head -1)
  if [ -z "$F" ] || [ "$F" = "$B" ]; then echo "[robust] $tag: NO FRAMES"; continue; fi
  D="$OUT/$tag"; mkdir -p "$D/frames"
  for f in Ground_Truth.npy Img_Data.npy Control_Data.npy Img_Params.txt; do
    [ -f "$R/$f" ] && cp "$R/$f" "$D/"; done
  cp "$F"/f*.png "$D/frames/" 2>/dev/null
  echo "[robust] $tag: $(ls "$D/frames" | wc -l) frames  <- $(basename "$R")"
done
ko; echo ROBUST_DONE > "$SD/run_logs/robust.flag"
