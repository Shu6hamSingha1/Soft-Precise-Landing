#!/usr/bin/env bash
# Single-marker GFT + moment loom + lstsq-RCOND IC2 A/B (2026-06-22). Runs AFTER the GFT A/B.
# rcond truncates singular values below rcond*sigma_max -> caps the 1/sigma_min^2 rank-deficiency
# noise amplification on the weak modes, BUT biases (attenuates) the magnitude (MATLAB: loom RMSE
# 0.78->0.42 but magnitude 0.90->0.66 at rcond=3e-2). Isolates rcond ON TOP of GFT.
#   gft_rcond3e2 : PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1 FLOW_LSTSQ_RCOND=3e-2
#   gft          : PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1   (rcond default 1e-3)
# gft_rcond3e2 FIRST. N=5. Judge: does spike-suppression beat the magnitude-attenuation cost
# (vz launches down WITHOUT lateral under-report / sluggish convergence)?
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-16}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/SingleRcond_${name}"
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

# single-marker = scaled-quad dense (GFT reverted 2026-06-22, regressed). rcond ON TOP of scaled-quad.
run_arm sq_rcond3e2  env PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1 FLOW_LSTSQ_RCOND=3e-2
run_arm sq           env PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1
echo "ALL ARMS DONE"
