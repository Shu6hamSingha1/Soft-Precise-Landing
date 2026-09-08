#!/usr/bin/env bash
# IC2-5 SITL A/B gate for CBF_AZ_COST_GAIN (joint-QP descent-rate relief, §3.1 of
# CBF_visibility.pdf). Q: is the relief needed at all? gain5 = current default,
# gain0 = relief disabled (eq-6 unchanged). Cross-marker world; base = post-e173b05c
# (loom fix in, CROSS_TZ_VETO_R_MULT=1.0) so no frozen-loom confound. Arms
# interleaved per rep by run_cbf_ab.sh.
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
  BUNDLE_NAME="JQP_AzCostGain_AB/${TS}/${ic}" \
  EXTRA_ENV="WORLD=cross_marker MARKER_TYPE=cross" \
  ARMS="gain5:CBF_AZ_COST_GAIN=5|gain0:CBF_AZ_COST_GAIN=0" \
    bash "$SCRIPT_DIR/run_cbf_ab.sh"
done
echo "############## GATE DONE  ($(date +%H:%M:%S)) ##############"
echo "bundles under: test_data/JQP_AzCostGain_AB/${TS}/"
