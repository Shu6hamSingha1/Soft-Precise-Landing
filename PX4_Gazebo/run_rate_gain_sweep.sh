#!/usr/bin/env bash
# Sweep PX4 MC_*RATE_P/I/D scale and run the impulse-response test at each
# setting.  Then pick the best-looking scale and run N=5 landings to see
# the effect on landing precision.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/RateGainSweep/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"

# Sweep values — start at 1.0 baseline, try modest 1.5x first.  Higher
# scales (2.5x+) risk instability so they go last and only if 1.5x/2x
# show clean improvement.
SCALES=("1.0" "1.5" "2.0")

SUMMARY="$BUNDLE_DIR/sweep_summary.tsv"
printf "scale\tt_d_pitch_ms\ttau_pitch_ms\tlag63_pitch_ms\tlanded\tnote\n" > "$SUMMARY"

for s in "${SCALES[@]}"; do
  echo
  echo "=================================================================="
  echo "  IMPULSE RESPONSE at PLASMC_PX4_RATE_SCALE=$s"
  echo "=================================================================="
  LOG="$BUNDLE_DIR/impulse_scale_${s}.log"
  before=$(ls -t "$HOME/ws/Test_Data/ImpulseResponse/" 2>/dev/null | head -1 || true)
  PLASMC_PX4_RATE_SCALE="$s" HEADLESS=1 \
    bash "$SCRIPT_DIR/run_impulse_response.sh" > "$LOG" 2>&1
  latest=$(ls -t "$HOME/ws/Test_Data/ImpulseResponse/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t-\t-\t-\tNO\timpulse run did not produce data\n" "$s" >> "$SUMMARY"
    continue
  fi
  # Run analyzer on the fresh impulse log
  REPORT=$("$HOME/ws/scripts/env2025/bin/python3" "$SCRIPT_DIR/analyze_impulse_response.py" \
           "$HOME/ws/Test_Data/ImpulseResponse/$latest/impulse_log.npy" 2>&1)
  echo "$REPORT" | tee "$BUNDLE_DIR/analyze_scale_${s}.log"
  # Extract pitch summary line "pitch  N   X ± Y   X ± Y   X ± Y   X ± Y"
  pitch_line=$(echo "$REPORT" | grep -E '^pitch' | head -1)
  if [ -n "$pitch_line" ]; then
    # Parse "pitch  N  td ± tdstd  τ ± τstd  lag63 ± std  lag95 ± std"
    # → numeric fields are at $3, $6, $9, $12 (skipping the ± glyph)
    t_d=$(echo "$pitch_line"  | awk '{print $3}')
    tau=$(echo "$pitch_line"  | awk '{print $6}')
    lag63=$(echo "$pitch_line"| awk '{print $9}')
    printf "%s\t%s\t%s\t%s\tYES\timpulse OK\n" "$s" "$t_d" "$tau" "$lag63" >> "$SUMMARY"
  else
    printf "%s\t-\t-\t-\tYES\tpitch summary line missing — see %s\n" \
           "$s" "$LOG" >> "$SUMMARY"
  fi
  sleep 5
done

echo
echo "=================================================================="
echo "  SWEEP RESULTS"
echo "=================================================================="
column -t -s $'\t' "$SUMMARY"
echo
echo "Per-scale impulse logs: $BUNDLE_DIR/impulse_scale_*.log"
echo "Per-scale analyzer outputs: $BUNDLE_DIR/analyze_scale_*.log"
