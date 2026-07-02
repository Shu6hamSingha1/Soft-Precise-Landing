#!/usr/bin/env bash
# Single-marker output re-cal (2026-06-23). Records phased output-cal runs in the SINGLE-MARKER
# world with PLASMC_SINGLE_MARKER=1 so the RAW flow matches what the controller consumes (the
# board-derived _sensor_cal_hw mis-scales the single-marker flow — GT-flow slope 1.77 vs 0.50).
# Loops until >=5 valid recordings (~50% fail; cleanup empties). Then derive_board_cal + derive_ring_cal.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT="$PROJ/calibration_data/output"; NEED="${NEED:-5}"; MAX="${MAX:-14}"
n=0; i=0
while [ "$n" -lt "$NEED" ] && [ "$i" -lt "$MAX" ]; do
  i=$((i+1)); echo "=== recal run $i (valid $n/$NEED) ==="
  env HEADLESS=1 PLASMC_SINGLE_MARKER=1 FLOW_LOOM_DECOUPLE=1 taskset -c 6-15 timeout 230 bash "$SCRIPT_DIR/run_output_calibration.sh" || true
  for d in "$OUT"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
  n=$(ls -d "$OUT"/*/ 2>/dev/null | wc -l)
done
echo "=== RECAL RECORDING DONE: $n valid in $i runs ==="
