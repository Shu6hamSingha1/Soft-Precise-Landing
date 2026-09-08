#!/usr/bin/env bash
set -u
SCRIPT_DIR="/home/shubham/Soft-Precise-Landing/PX4_Gazebo/scripts"
OUT_DIR="/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/OverfillCapture_IC2to5"
declare -A IC_ENU
IC_ENU[IC2]="2.0,2.0,5.0"
IC_ENU[IC3]="-2.0,2.0,5.0"
IC_ENU[IC4]="2.0,2.0,7.0"
IC_ENU[IC5]="2.0,2.0,3.0"
for ic in IC2 IC3 IC4 IC5; do
  enu="${IC_ENU[$ic]}"
  echo "=== $ic (ENU $enu) ==="
  before=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/"*/ 2>/dev/null | head -1 || true)
  before_raw=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Test_Videos/"*_raw/ 2>/dev/null | head -1 || true)
  env HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross \
      INITIAL_DRONE_ENU="$enu" IMG_RECORD=1 LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$OUT_DIR/${ic}.log" 2>&1
  latest=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/"*/ 2>/dev/null | head -1 || true)
  latest_raw=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Test_Videos/"*_raw/ 2>/dev/null | head -1 || true)
  if [ -n "$latest" ] && [ "$latest" != "$before" ]; then
    cp -r "$latest" "$OUT_DIR/${ic}_data"
    echo "  -> data: $OUT_DIR/${ic}_data"
  fi
  if [ -n "$latest_raw" ] && [ "$latest_raw" != "$before_raw" ]; then
    echo "$ic|$latest_raw" >> "$OUT_DIR/raw_dirs.txt"
  fi
done
echo "DONE"
