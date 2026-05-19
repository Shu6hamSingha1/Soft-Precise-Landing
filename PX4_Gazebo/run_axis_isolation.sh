#!/usr/bin/env bash
# Axis-isolation diagnostic for IC 1. Runs landing under each combination
# of body-rate axis disabled to identify which DOF degrades precision.
#
# Axis labels: wx = roll rate, wy = pitch rate, wz = yaw rate.
#
# Eight configs (boolean = "active" → 1, "zeroed" → 0 in env vars):
#   #1  (wx=1 wy=1 wz=1)  baseline (full controller)
#   #2  (wx=0 wy=1 wz=1)  no roll
#   #3  (wx=1 wy=0 wz=1)  no pitch
#   #4  (wx=1 wy=1 wz=0)  no yaw
#   #5  (wx=0 wy=0 wz=1)  vertical-only (current LANDING_VERTICAL_ONLY=1)
#   #6  (wx=0 wy=1 wz=0)  pitch-only
#   #7  (wx=1 wy=0 wz=0)  roll-only
#   #8  (wx=0 wy=0 wz=0)  no rate control (free-falling attitude)
#
# Each config runs N_REPS times (default 3) on IC 1. Aggregated mean xy
# per config tells us which axis contributes most to lateral drift.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/AxisIsolation/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

N_REPS="${N_REPS:-3}"
IC="${IC:-0.0,0.0,5.0}"

# Each row: name, NO_WX, NO_WY, NO_WZ
CONFIGS=(
  "full         0 0 0"
  "no_wx        1 0 0"
  "no_wy        0 1 0"
  "no_wz        0 0 1"
  "vertical    1 1 0"
  "pitch_only  1 0 1"
  "roll_only   0 1 1"
  "free        1 1 1"
)

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "config\tno_wx\tno_wy\tno_wz\trep\tlanded\txy_err_m\trel_vel_mps\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local cfg_name="$1" no_wx="$2" no_wy="$3" no_wz="$4" rep="$5"
  local dst="$BUNDLE_DIR/${cfg_name}_rep${rep}"
  echo
  echo "=========================================================="
  echo "  config=$cfg_name (NO_WX=$no_wx NO_WY=$no_wy NO_WZ=$no_wz)  rep=$rep"
  echo "=========================================================="

  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)

  env LANDING_NO_WX="$no_wx" LANDING_NO_WY="$no_wy" LANDING_NO_WZ="$no_wz" \
      INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" \
      > "$dst.log" 2>&1
  local rc=$?

  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\t%s\t%s\t%s\tNO\t-\t-\t-\t-\t-\n" \
      "$cfg_name" "$no_wx" "$no_wy" "$no_wz" "$rep" >> "$SUMMARY"
    return
  fi
  local src="$HOME/ws/Test_Data/Landing_Test/$latest"
  cp -r "$src" "$dst"

  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
try:
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    sp = gt.get("SoftPrecise", {})
    if sp:
        print(f"{sp.get('xy_err', 0):.4f}\t{sp.get('rel_vel', 0):.4f}\t"
              f"{int(sp.get('precise', False))}\t{int(sp.get('soft', False))}")
    else:
        print("0\t0\t0\t0")
except Exception:
    print("0\t0\t0\t0")
PY
)
  printf "%s\t%s\t%s\t%s\t%s\tYES\t%s\t%s\n" \
    "$cfg_name" "$no_wx" "$no_wy" "$no_wz" "$rep" "$metrics" "$(basename "$dst")" >> "$SUMMARY"
}

for row in "${CONFIGS[@]}"; do
  read -r cfg no_wx no_wy no_wz <<<"$row"
  for r in $(seq 1 "$N_REPS"); do
    run_one "$cfg" "$no_wx" "$no_wy" "$no_wz" "$r"
    sleep 2
  done
done

echo
echo "=========================================================="
echo "=== Axis-isolation complete: $BUNDLE_DIR"
echo "=========================================================="
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-config aggregate (N=$N_REPS each):"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
agg = defaultdict(lambda: dict(n=0, soft=0, prec=0, xy=[], vel=[]))
for r in rows:
    if r['landed'] != 'YES': continue
    k = r['config']
    a = agg[k]; a['n'] += 1
    a['soft'] += int(r['soft']); a['prec'] += int(r['precise'])
    try: a['xy'].append(float(r['xy_err_m'])); a['vel'].append(float(r['rel_vel_mps']))
    except: pass

print(f"\n  {'config':<12}  {'n':>3}  {'soft':>4}  {'mean xy':>8} {'max xy':>8}  {'mean vel':>9} {'max vel':>9}")
for k in ["full","no_wx","no_wy","no_wz","vertical","pitch_only","roll_only","free"]:
    a = agg.get(k)
    if not a or a['n'] == 0:
        print(f"  {k:<12}  no successful landings")
        continue
    mxy = sum(a['xy'])/a['n']
    maxxy = max(a['xy'])
    mvel = sum(a['vel'])/a['n']
    maxvel = max(a['vel'])
    print(f"  {k:<12}  {a['n']:>3}  {a['soft']:>4}  {mxy:>8.3f} {maxxy:>8.3f}  {mvel:>9.3f} {maxvel:>9.3f}")
PY
