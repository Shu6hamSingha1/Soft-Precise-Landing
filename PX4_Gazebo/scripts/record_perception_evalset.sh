#!/usr/bin/env bash
# Record the PerceptionEvalSet static-scene tags (2026-09-24) for the locked-design cross
# detector rewrite (Memory/px4/feedback_cross_detector_robustness_requirement.md: build the eval
# set BEFORE the front end; score accuracy + flight outcome, never detect-rate alone).
#
# One GT-FB descent per tag (PLASMC_GT_FEEDBACK=1: the detector under test must not gate the
# flight, so every scene gets a full descent to touchdown), IMG_RECORD=1 with
# CROSS_RING_OVERLAY_DBG=0 (clean frames) + the frames.tsv stamp sidecar. Each rep goes to its
# own LANDING_OUT_BASE; tools/build_evalset_tag.py pairs it with its raw dir BY STAMP and writes
# test_data/PerceptionEvalSet/<tag>/ (+ meta.json with world/marker_dz).
#
# Lighting/polarity/colour variants: re-recorded as rob_* tags (RobustnessFrameset/inv's pairing is broken).
# Moving-target tags are assembled from scripts/run_sperc_gtfb_ab.sh ARMS=gt CASES=... reps.
#
# Usage: bash scripts/record_perception_evalset.sh            # all TAGS below
#        TAGS="rover_static_IC2" bash scripts/record_perception_evalset.sh
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
PY="$HOME/ws/scripts/env2025/bin/python3"
OUT="$PROJ/test_data/PerceptionEvalSet"
WORK="$OUT/_reps"
QT_PRELOAD="$HOME/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5"
MAXLAUNCH="${MAXLAUNCH:-3}"
# tag|launcher|world|marker_dz|IC(ENU)|extra env
SPECS=(
  "flat_IC1|run_aruco_landing_retry.sh|cross_marker|0.0|0.0,0.0,5.0|"
  "flat_IC3|run_aruco_landing_retry.sh|cross_marker|0.0|-2.0,2.0,5.0|"
  "flat_IC4|run_aruco_landing_retry.sh|cross_marker|0.0|2.0,2.0,7.0|"
  "clutter_IC2|run_aruco_landing_retry.sh|cross_marker_clutter|0.0|2.0,2.0,5.0|"
  "rover_static_IC2|run_rover_landing_retry.sh|rover_cross|0.5|2.0,2.0,5.0|ROVER_MODEL=rover_cross ROVER_MOTION=0"
  "rover_static_IC4|run_rover_landing_retry.sh|rover_cross|0.5|2.0,2.0,7.0|ROVER_MODEL=rover_cross ROVER_MOTION=0"
  # Lighting / polarity / colour scenes, RE-RECORDED with the frames.tsv sidecar (2026-09-24):
  # test_data/RobustnessFrameset/inv pairs frames to Img_Data by a tail offset that is WRONG
  # for that recording (f150 shows ~2 m, the offset lands at GT alt 4.9 m) -- every inv score
  # to date, legacy and stroke, was measured against the wrong GT. Same scenes, IC2.
  "rob_base|run_aruco_landing_retry.sh|cross_marker|0.0|2.0,2.0,5.0|"
  "rob_dim|run_aruco_landing_retry.sh|cm_dim|0.0|2.0,2.0,5.0|"
  "rob_bright|run_aruco_landing_retry.sh|cm_bright|0.0|2.0,2.0,5.0|"
  "rob_lowsun|run_aruco_landing_retry.sh|cm_lowsun|0.0|2.0,2.0,5.0|"
  "rob_inv|run_aruco_landing_retry.sh|cm_inv|0.0|2.0,2.0,5.0|"
  "rob_col|run_aruco_landing_retry.sh|cm_col|0.0|2.0,2.0,5.0|"
  "rob_darkbg|run_aruco_landing_retry.sh|cm_darkbg|0.0|2.0,2.0,5.0|"
)
TAGS="${TAGS:-}"

if ps -eo cmd | grep -E "bin/px4 |gz sim|landing_test.py" | grep -qv grep; then
  echo "SITL already running (another session?) -- refusing to launch" >&2; exit 1
fi

for spec in "${SPECS[@]}"; do
  IFS='|' read -r tag launcher world mdz ic extra <<< "$spec"
  if [ -n "$TAGS" ] && ! echo " $TAGS " | grep -q " $tag "; then continue; fi
  [ -f "$OUT/$tag/meta.json" ] && { echo "[evalset] $tag exists, skipping"; continue; }
  base="$WORK/$tag"; mkdir -p "$base"
  launch=0
  while [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [evalset $tag] launch $launch/$MAXLAUNCH $(date +%H:%M:%S) ==="
    # shellcheck disable=SC2086
    env HEADLESS=1 LD_PRELOAD="$QT_PRELOAD" WORLD="$world" MARKER_TYPE=cross \
        INITIAL_DRONE_ENU="$ic" PLASMC_GT_FEEDBACK=1 IMG_RECORD=1 CROSS_RING_OVERLAY_DBG=0 \
        LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 LANDING_OUT_BASE="$base" $extra \
      taskset -c 6-15 bash "$SCRIPT_DIR/$launcher" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then
      rep=$(ls -td "$base"/*/ | head -1)
      "$PY" "$PROJ/tools/build_evalset_tag.py" "$rep" "$OUT" "$tag" --world "$world" --marker-dz "$mdz" \
        && break
    fi
  done
done
echo "EVALSET RECORD DONE"
