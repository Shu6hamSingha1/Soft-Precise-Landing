#!/usr/bin/env bash
# Combined-barrier IC2 A/B harness (2026-06-19). Runs N valid reps per arm at
# IC2=(2,2,5), headless, taskset to cores 6-15. Arms:
#   backmap   — PLASMC_COMBINED_BARRIER unset (back-map baseline wall)
#   combined  — PLASMC_COMBINED_BARRIER=1, default chi_r=0.85
#   chir      — PLASMC_COMBINED_BARRIER=1, chi_r bumped (PLASMC_CHI_R_X/Y)
# Each rep autosaves a timestamped subdir under its arm base. A rep counts as
# valid if the launcher returns rc=0 AND a non-empty subdir appeared.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
CHIR="${CHIR:-1.3}"
MAXLAUNCH="${MAXLAUNCH:-14}"   # cap launches per arm (flake guard)

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/CB_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    # prune empty dirs
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm backmap
run_arm combined  env PLASMC_COMBINED_BARRIER=1
run_arm chir      env PLASMC_COMBINED_BARRIER=1 PLASMC_CHI_R_X="$CHIR" PLASMC_CHI_R_Y="$CHIR"
echo "ALL ARMS DONE"
