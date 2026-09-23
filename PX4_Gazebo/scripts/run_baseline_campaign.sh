#!/usr/bin/env bash
# Drive the full comparative-baseline recording campaign: 4 baselines x 10 cases,
# ONE attempt each (no SoftPrecise retry gate, per user direction 2026-09-23), promoting
# each rep into test_data/Final/<TAG>-GT/<case>/ immediately after it completes so
# progress survives a mid-campaign interruption.
#
# Video scope (2026-09-23, user-agreed): onboard_cam.mp4 + chase_cam.mp4 + a montage
# (raw onboard PiP + chase + GT plot panel) -- NO s/alpha or h/w feature-overlay PiPs
# for the baselines (that decision predated finding tools/overlay_image_features.py;
# left as-is to match what was actually agreed, not re-litigated here).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJ"

declare -A TAG=( [lin2022]="LIN2022-GT" [zhang2026]="ZHANG2026-GT" [lin2023]="LIN2023-GT" [cho2022]="CHO2022-GT" )
CASES=(IC1 IC2 IC3 IC4 IC5 Static Linear Sinusoidal Circular Lissajous)

for bl in lin2022 zhang2026 lin2023 cho2022; do
  for c in "${CASES[@]}"; do
    echo "================================================================="
    echo "=== CAMPAIGN: $bl / $c  $(date) ==="
    echo "================================================================="
    bash "$SCRIPT_DIR/record_baseline_cases.sh" "$bl" "$c"

    devdir="test_data/RecordBaseline_dev/$bl/$c"
    rep=$(cat "$devdir/.rep" 2>/dev/null)
    if [ -z "$rep" ] || [ ! -d "$rep" ]; then
      echo "!!! $bl/$c: NO REP PRODUCED -- skipping promotion, continuing campaign"
      continue
    fi

    out="test_data/Final/${TAG[$bl]}/$c"
    mkdir -p "$out/dataset"
    cp "$rep"/*.npy "$out/dataset/" 2>/dev/null
    cp "$rep"/Img_Params.txt "$out/dataset/" 2>/dev/null
    [ -f "$rep/onboard_cam.mp4" ] && cp "$rep/onboard_cam.mp4" "$out/${c}_onboard_cam.mp4"
    [ -f "$rep/chase_cam.mp4" ] && cp "$rep/chase_cam.mp4" "$out/${c}_chase_cam.mp4"

    if [ -f "$out/${c}_onboard_cam.mp4" ] && [ -f "$out/${c}_chase_cam.mp4" ]; then
      ~/ws/scripts/env2025/bin/python3 tools/make_landing_montage.py \
        --chase "$out/${c}_chase_cam.mp4" \
        --drone "$out/${c}_onboard_cam.mp4" \
        --run "$out/dataset" \
        --out "$out/${c}_montage.mp4" 2>&1 | tail -3
    else
      echo "!!! $bl/$c: missing onboard/chase video, skipping montage"
    fi
    echo "=== $bl/$c PROMOTED -> $out $(date) ==="
  done
done
echo "################ CAMPAIGN DONE $(date) ################"
