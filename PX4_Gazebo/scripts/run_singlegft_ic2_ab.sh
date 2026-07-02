#!/usr/bin/env bash
# Single-marker GFT + moment loom IC2 A/B (2026-06-22). Follow-up to single_moment (scaled-quad):
# the scaled-quad dense points sit on marker EDGES -> aperture problem -> noisy LK -> rank-deficiency
# noise (loom_noise = input_noise/sigma_min^2). GFT finds TEXTURE CORNERS (2D-trackable, no aperture)
# -> lower input LK noise -> lower loom/lateral noise even with the same sigma_min. Tests whether
# cutting the input noise (GFT) closes the gap to the multi-marker moment loom.
#   single_gft : PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1  (lock + GFT dense + corner moment loom)
#   momentsz   : FLOW_LOOM_DECOUPLE=1                          (multi-marker moment loom = current best)
# single_gft FIRST. N=5. Judge: vz launches + sub-meter + lateral flow noise vs scaled-quad (1/5 vert,
# 0/5 sub) and momentsz (0/5 vert, 2/5 sub).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-16}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/SingleGFT_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm single_gft  env PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1
run_arm momentsz    env FLOW_LOOM_DECOUPLE=1
echo "ALL ARMS DONE"
