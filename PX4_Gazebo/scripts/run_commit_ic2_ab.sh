#!/usr/bin/env bash
# Terminal-commit STACK IC2 A/B (2026-06-22). The descent-gate centers the drone to
# the deck (commit-ready 14/15 IC2) but the final ~0.3m destabilizes (1/Z-corrupted
# lateral flow -> a_u spike -> launch). This A/B stacks the terminal-commit on the gate:
#   gate-only   : PLASMC_DESCENT_GATE=1
#   gate+commit : + PLASMC_COMMIT_EXTENT=50 (fires at the centered-low state, Z~0.5m,
#                 median extent 57 there vs 33 at Z>0.8m) + PLASMC_COMMIT_LAT_TAPER=1
#                 (ramp a_u_xy->0 once committed: stop steering on the garbage flow,
#                 coast + descend level).
# Hypothesis: gate solves the approach race, commit handles the last 0.3m -> closes the
# geometric tail. gate+commit arm FIRST (de-risk: watch for off-center coast-out/hover).
# N>=15. Judge: fly-away/tail count + did committed reps land near the committed offset.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/Commit_IC2_${name}"
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

# gate+commit FIRST (de-risk the new taper). COMMIT_EXTENT=65 (was 50 -> fired too
# early at Z~1.2m in 4/9 de-risk reps -> premature lateral cutoff/coast-out; 65 = the
# Z<0.4m p25 extent, fires at the true centered-low state ~0.45m).
run_arm gatecommit  env PLASMC_DESCENT_GATE=1 PLASMC_COMMIT_EXTENT=65 PLASMC_COMMIT_LAT_TAPER=1
# gate-only (the stack's baseline)
run_arm gate        env PLASMC_DESCENT_GATE=1
echo "ALL ARMS DONE"
