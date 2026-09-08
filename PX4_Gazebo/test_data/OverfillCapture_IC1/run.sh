#!/usr/bin/env bash
set -u
SCRIPT_DIR="/home/shubham/Soft-Precise-Landing/PX4_Gazebo/scripts"
OUT_DIR="/home/shubham/Soft-Precise-Landing/PX4_Gazebo/test_data/OverfillCapture_IC1"
N_REPS="${N_REPS:-3}"
for rep in $(seq 1 "$N_REPS"); do
  echo "=== rep $rep ==="
  before=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/"*/ 2>/dev/null | head -1 || true)
  env HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross \
      INITIAL_DRONE_ENU="0.0,0.0,5.0" IMG_RECORD=1 LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$OUT_DIR/rep${rep}.log" 2>&1
  latest=$(ls -td "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/"*/ 2>/dev/null | head -1 || true)
  if [ -n "$latest" ] && [ "$latest" != "$before" ]; then
    cp -r "$latest" "$OUT_DIR/rep${rep}_data"
    echo "  -> captured to $OUT_DIR/rep${rep}_data"
  else
    echo "  -> no new Landing_Test dir found"
  fi
done
echo "DONE"
