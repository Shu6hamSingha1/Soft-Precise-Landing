#!/usr/bin/env bash
# IC2-5 SITL A/B gate for CBF_JQP_RELIEF_REF_AZ0 (joint-QP descent-rate relief
# self-inflation fix, cbf_visibility.py, pdf open item #2). Cross-marker world
# (the hard rule + where the terminal-stall chain was diagnosed). Wraps
# run_cbf_ab.sh once per IC; arms interleaved per rep inside that harness.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TS="$(date +%Y%m%d-%H%M%S)"
N_REPS="${N_REPS:-5}"
export HEADLESS=1
declare -A IC_ENU=( [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0" )
for ic in IC2 IC3 IC4 IC5; do
  echo "############## $ic  ($(date +%H:%M:%S)) ##############"
  IC="${IC_ENU[$ic]}" \
  N_REPS="$N_REPS" \
  BUNDLE_NAME="JQP_ReliefRefAz0_AB/${TS}/${ic}" \
  EXTRA_ENV="WORLD=cross_marker MARKER_TYPE=cross" \
  ARMS="relief_off:CBF_JQP_RELIEF_REF_AZ0=0|relief_on:CBF_JQP_RELIEF_REF_AZ0=1" \
    bash "$SCRIPT_DIR/run_cbf_ab.sh"
done
echo "############## GATE DONE  ($(date +%H:%M:%S)) ##############"
echo "bundles under: test_data/JQP_ReliefRefAz0_AB/${TS}/"
