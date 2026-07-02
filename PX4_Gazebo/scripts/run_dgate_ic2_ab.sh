#!/usr/bin/env bash
# Descent-gate IC2 A/B (2026-06-22) — attack the geometric fly-away at its CAUSE.
# h_ref_eff = h_rd * g(|s_e_n|): slow descent when off-center so the lateral offset
# nulls BEFORE Z (and the FoV footprint ~0.89*Z) shrinks -> marker stays centered ->
# corners stay decoded -> lateral flow stays HONEST (GT-verified honest at altitude,
# corrupted <0.8m) -> no terminal over-reaction/launch. Removes the geometric-tail
# variance source (which is ALSO why n=15 perception A/Bs were unresolvable).
#
# RATE-LIMITED (PLASMC_DGATE_TAU) to avoid the 2026-05-18 failure (varying h_ref ->
# dh_d transient -> c-term destabilization -> 12m IC5 runaway). g_min floor avoids
# permanent hover. GATE ARM RUNS FIRST to surface any hover/runaway early.
#
# Base = baked defaults (E_z=0.5). N>=15. Judge by: fly-away/tail count + FoV
# retention (terminal N Flow Corners, |s_e_n| at low alt) + does descent complete
# (final_alt low, flight_s not blown up = no hover). Default gate params:
# slo=0.4 shi=0.8 gmin=0.15 tau=0.5.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-15}"
MAXLAUNCH="${MAXLAUNCH:-40}"

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/Dgate_IC2_${name}"
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

# gate arm FIRST (early de-risk: watch for hover/runaway)
run_arm gate      env PLASMC_DESCENT_GATE=1
# baseline = baked defaults (gate OFF)
run_arm baseline
echo "ALL ARMS DONE"
