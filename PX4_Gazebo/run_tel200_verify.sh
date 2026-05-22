#!/usr/bin/env bash
# Verify telemetry rate change: N=3 landings with 200 Hz odometry/IMU,
# then re-run Phase 2 lag analyzer to compare against the 60 Hz baseline.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/Tel200Verify/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"
IC="0.0,0.0,5.0"; N_REPS="${N_REPS:-3}"

export LANDING_REF_RAD_OPT_FLOW=-0.70
export IMG_FILTER_WIN=7

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy\tvel\ttarget_lost\tresult_dir\n" > "$SUMMARY"

for r in $(seq 1 "$N_REPS"); do
  dst="$BUNDLE_DIR/rep${r}"
  echo "=== Tel200  rep=$r/$N_REPS ==="
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\n" "$r" >> "$SUMMARY"; sleep 2; continue
  fi
  cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
tl = int(sp.get('target_lost', False))
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{tl}")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$r" "$m" "$(basename "$dst")" >> "$SUMMARY"
  sleep 2
done

echo
column -t -s $'\t' "$SUMMARY"
echo
echo "Bundle: $BUNDLE_DIR"

# Now show Telemetry sample count — 60 Hz → ~240 samples in 4s; 200 Hz → ~800
~/ws/scripts/env2025/bin/python3 << PY
import numpy as np, glob, os
reps = sorted(glob.glob("$BUNDLE_DIR/rep*"))
print()
print("Telemetry sample counts (200 Hz target):")
for d in reps:
    if not os.path.isdir(d): continue
    td = np.load(f"{d}/Telemetry_Data.npy", allow_pickle=True).item()
    n = len(td.get("Angular Velocity Body", []))
    print(f"  {os.path.basename(d)}: {n} ω samples")
PY

# Run latency analyzer
echo
~/ws/scripts/env2025/bin/python3 "$SCRIPT_DIR/analyze_loop_latency.py" "$BUNDLE_DIR" 2>&1 | head -30
