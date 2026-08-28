#!/usr/bin/env bash
# Batch driver (2026-08-28): record a landing + build the finalized 2-PiP montage
# (per Memory/px4/reference_finalized_montage_video_layout.md) for IC1-5 (at the
# default Linear rover trajectory) plus one run per remaining rover motion type
# (at default IC1). Reuses montage_landing_cross.sh for the record+basic-montage
# step, then re-builds the montage with BOTH overlay PiPs + the low-angle chase-cam
# + progressive touchdown-crop (this session's validated recipe).
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
cd "$SD"
LOG="$SD/test_data/Rover_AB_harness"
VID="$SD/test_data/Test_Videos"
PY=/home/shubham/ws/scripts/env2025/bin/python3

run_one() {
  local label="$1" cfg="$2"
  echo "=================================================================="
  echo "BATCH: $label  ($cfg)"
  echo "=================================================================="
  LABEL="$label" CONFIG_ENV="$cfg" MAXTRY=6 bash "$LOG/montage_landing_cross.sh" \
      > "$LOG/batch_${label}.out" 2>&1
  if ! grep -q "^MONTAGE_${label} ->" "$LOG/batch_${label}.out"; then
    echo "BATCH: $label FAILED (no on-platform landing) -- see $LOG/batch_${label}.out"
    return 1
  fi
  local run chase drone
  run=$(ls -dt "$SD/test_data/Landing_Test_Cross"/*/ | head -1)
  chase=$(ls -t "$VID"/chase_*.mp4 | head -1)
  drone=$(ls -t "$VID"/*.mp4 | grep -vE 'chase|montage|overlay' | head -1)
  echo "BATCH: $label ON-platform -> run=$run chase=$chase drone=$drone"
  "$PY" tools/overlay_image_features.py --video "$drone" --run "$run" \
      --out "$VID/overlay_${label}_s_alpha.mp4" --channels s,alpha > "$LOG/batch_${label}_overlay1.out" 2>&1
  "$PY" tools/overlay_image_features.py --video "$drone" --run "$run" \
      --out "$VID/overlay_${label}_h.mp4" --channels h > "$LOG/batch_${label}_overlay2.out" 2>&1
  "$PY" tools/make_landing_montage.py \
      --chase "$chase" \
      --drone "$VID/overlay_${label}_s_alpha.mp4" \
      --drone2 "$VID/overlay_${label}_h.mp4" \
      --run "$run" \
      --out "$VID/montage_${label}_final.mp4" \
      --tail-s 1.0 --chase-crop-touchdown 0.45 --chase-crop-ramp-s 3.0 \
      > "$LOG/batch_${label}_montage.out" 2>&1
  echo "BATCH: $label -> $VID/montage_${label}_final.mp4"
  return 0
}

# ---- IC1-5 sweep, default (Linear) rover trajectory ----
run_one "ic1" "INITIAL_DRONE_ENU=0.0,0.0,5.0 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3"
run_one "ic2" "INITIAL_DRONE_ENU=2.0,2.0,5.0 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3"
run_one "ic3" "INITIAL_DRONE_ENU=-2.0,2.0,5.0 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3"
run_one "ic4" "INITIAL_DRONE_ENU=2.0,2.0,7.0 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3"
run_one "ic5" "INITIAL_DRONE_ENU=2.0,2.0,3.0 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3"

# ---- remaining rover motions, default IC1 ----
run_one "motion_circular" "ROVER_TRAJ=Circular ROVER_SPEED_MULT=1.0 PLASMC_AU_LEAD=1 PLASMC_AU_LEAD_WZ=0.9 PLASMC_AU_LEAD_WP=3.5 PLASMC_AU_LEAD_RATIO=0.5 PLASMC_TERMINAL_COMMIT=0 PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0"
run_one "motion_circularyaw" "ROVER_TRAJ=CircularYaw ROVER_SPEED_MULT=1.0 PLASMC_AU_LEAD=1 PLASMC_AU_LEAD_WZ=0.9 PLASMC_AU_LEAD_WP=3.5 PLASMC_AU_LEAD_RATIO=0.5 PLASMC_TERMINAL_COMMIT=0 PLASMC_YAW_ALPHA_FILT=0 PLASMC_YAW_GAMMA=0 PLASMC_YAW_KAPPA0=0 PLASMC_YAW_OMEGA=0 PLASMC_YAW_N=0"
run_one "motion_sinusoidal" "ROVER_TRAJ=Sinusoidal ROVER_SPEED_MULT=0.3"
run_one "motion_eightshape" "ROVER_TRAJ=EightShape ROVER_SPEED_MULT=0.3"
run_one "motion_lissajous" "ROVER_TRAJ=Lissajous ROVER_SPEED_MULT=0.3"
run_one "motion_static" "ROVER_TRAJ=Static ROVER_SPEED_MULT=0.3"

echo "=================================================================="
echo "BATCH DONE"
echo "=================================================================="
