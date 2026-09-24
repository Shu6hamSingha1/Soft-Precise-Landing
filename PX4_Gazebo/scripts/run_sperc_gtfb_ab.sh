#!/usr/bin/env bash
# s-from-perception vs s-from-GT A/B under GT-FB (2026-09-23).
#
# Question: with every other signal on Gazebo ground truth, can the image feature s (centroid xy)
# be taken from the PERCEPTION pipeline instead of the analytic GT without losing landing
# performance? If arm sperc ~= arm gt over the same ICs, s is perception-ready.
#
# Arms (stationary cross-marker, PLASMC_GT_FEEDBACK=1, all defaults otherwise):
#   gt     : GT_ABLATE unset  -> full GT-FB (s, h, hz, yaw, wz all GT)
#   sperc  : GT_ABLATE=h,hz,yaw,wz -> GT everywhere EXCEPT s (perception centroid xy)
#   gthold : full GT, but GT s refreshed only when perception finishes a frame, sampled at that
#            frame's capture stamp (PLASMC_GT_S_HOLD=stamp) -- perception's TIMING, GT's VALUES.
#            gthold ~ sperc => staleness is the cause; gthold ~ gt => close-range s noise is.
#   gtbear : full GT, but s = true bearing x/z (PLASMC_GT_S_Z_REG=0.02) instead of x/(z+0.2) --
#            what a camera actually measures. gtbear ~ sperc => the gap is the 1/z terminal gain
#            of the true bearing, not perception noise.
# (controller.py ~L2411: channels listed in GT_ABLATE come from GT, the rest from perception.
#  wx,wy are zeroed by W_XY_DEROT default in both arms; feature_param[2] is untouched by either.)
#
# Usage: HEADLESS=1 N=5 bash scripts/run_sperc_gtfb_ab.sh                 # stationary, IC1-5
#        IC_SEL="2" ARMS="sperc" ...                                        # subset
#        CASES="Sinusoidal Circular" ARMS="sperc gthold" ...                # MOVING target (rover)
#        EXTRA_ENV="IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0" ...              # clean onboard frames (Test_Videos/*_raw)
# Moving cases (2026-09-24) reuse scripts/record_gtfb_cases.sh's per-case env verbatim (IC2 start,
# rover_cross / rover_cross_deck worlds, cross marker, legacy ROVER_CTRL=pos -- which tracks for
# Sinusoidal/Circular per project_20260922_ackermann_rover_loops_not_tracking). Verify target
# motion from each rep's own Ground_Truth Target Pose; never trust ROVER_SPEED_MULT.
# Output: $OUT_ROOT/<arm>_IC<k>/ or $OUT_ROOT/<arm>_<Case>/ ; summary tail block below.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-12}"
OUT_ROOT="${OUT_ROOT:-$PROJ/test_data/SPercGTFB_AB}"
ARMS="${ARMS:-gt sperc}"
CASES="${CASES:-}"
# IC1-5 in ENU
IC_LIST=("0.0,0.0,5.0" "2.0,2.0,5.0" "-2.0,2.0,5.0" "2.0,2.0,7.0" "2.0,2.0,3.0")
IC_SEL="${IC_SEL:-1 2 3 4 5}"
QT_PRELOAD="$HOME/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5"

# run_arm <label> <launcher> <env...> : loop-until-N-valid into $OUT_ROOT/<label>/
run_arm () {
  local label="$1" launcher="$2"; shift 2
  local base="$OUT_ROOT/${label}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$label] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 LD_PRELOAD="$QT_PRELOAD" MARKER_TYPE=cross \
        LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 PLASMC_GT_FEEDBACK=1 \
        LANDING_OUT_BASE="$base" ${EXTRA_ENV:-} "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/$launcher" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); fi
  done
  echo "=== [$label] DONE: $valid valid in $launch launches ==="
}

arm_env () {   # per-arm env, identical for stationary and moving
  case "$1" in
    gt)     echo "" ;;
    sperc)  echo "GT_ABLATE=h,hz,yaw,wz" ;;
    gthold) echo "PLASMC_GT_S_HOLD=stamp" ;;
    gtbear) echo "PLASMC_GT_S_Z_REG=0.02" ;;    # GT s as the TRUE bearing x/z (h keeps Z_REG=0.2)
    *)      echo "UNKNOWN_ARM" ;;
  esac
}

if [ -z "$CASES" ]; then
  for k in $IC_SEL; do
    ic="${IC_LIST[$((k-1))]}"
    # interleave arms per IC so drift in the sim/PX4 state hits all arms equally
    for arm in $ARMS; do
      # shellcheck disable=SC2046
      run_arm "${arm}_IC${k}" run_aruco_landing_retry.sh WORLD=cross_marker INITIAL_DRONE_ENU="$ic" $(arm_env "$arm")
    done
  done
else
  for c in $CASES; do
    case "$c" in
      Static)     cenv="WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=0" ;;
      Sinusoidal|Lissajous)
                  cenv="WORLD=rover_cross ROVER_MODEL=rover_cross ROVER_MOTION=1 ROVER_TRAJ=$c" ;;
      Linear|Circular)
                  cenv="WORLD=rover_cross_deck ROVER_MODEL=rover_ackermann POSE_IDX_UAV=3 POSE_IDX_TARGET=1 ROVER_MOTION=1 ROVER_TRAJ=$c DECK_MOTION=1" ;;
      *) echo "unknown case $c" >&2; continue ;;
    esac
    for arm in $ARMS; do
      # shellcheck disable=SC2086,SC2046
      run_arm "${arm}_${c}" run_rover_landing_retry.sh $cenv INITIAL_DRONE_ENU="2.0,2.0,5.0" $(arm_env "$arm")
    done
  done
fi

echo "--- summary (xy_err m / rel_vel m/s per rep) ---"
~/ws/scripts/env2025/bin/python3 - "$OUT_ROOT" <<'EOF'
import sys, glob, os, numpy as np
root = sys.argv[1]
for d in sorted(glob.glob(os.path.join(root, "*_*"))):
    rows = []
    for rep in sorted(glob.glob(os.path.join(d, "*/"))):
        f = os.path.join(rep, "Ground_Truth.npy")
        if not os.path.exists(f): continue
        g = np.load(f, allow_pickle=True).item().get("SoftPrecise", {})
        rows.append((g.get("xy_err"), g.get("rel_vel")))
    if not rows: continue
    xy = np.array([r[0] for r in rows if r[0] is not None], float)
    sp = sum(1 for r in rows if r[0] is not None and r[1] is not None and r[0] <= 0.15 and r[1] <= 0.5)
    spm = sum(1 for r in rows if r[0] is not None and r[1] is not None and r[0] <= 0.08 and r[1] <= 0.2)
    print(f"{os.path.basename(d):14s} n={len(rows)} SP(<=0.15m,<=0.5m/s)={sp}/{len(rows)} "
          f"SP(manuscript <=0.08m,<=0.2m/s)={spm}/{len(rows)} "
          f"xy_err mean={xy.mean():.3f} med={np.median(xy):.3f} max={xy.max():.3f}")
EOF
