#!/usr/bin/env bash
# Load-bearing parameter sweep — SITL port of the manuscript's 33-axis deep
# sweep, restricted to the LOAD-BEARING parameters identified in Section S3-A
# of the supplement (Table S1):
#   - K_rp      (Lateral PID proportional)         via PLASMC_KP_SCALE
#   - p_2_0     (Initial optic-flow funnel)        via PLASMC_P20_SCALE
#   - p_2_∞     (Terminal optic-flow funnel)       via PLASMC_P2INF_SCALE
#   - 𝒳 (Omega) (Sliding-surface integrator gain)  via PLASMC_OMEGA_SCALE
# (k_R is also load-bearing in MATLAB but N/A in SITL — PX4 handles attitude.)
#
# Per the manuscript's methodology: each axis perturbed by {0.5, 0.75, 1.25,
# 1.5}, replayed on all 5 ICs. Total runs = 4 axes × 4 multipliers × 5 ICs
# = 80, plus a ×1.0 baseline (5 ICs) for reference.
#
# Output: ~/ws/Test_Data/LoadBearing_Sweep/<ts>/<param>_<mult>/IC_<idx>/
# plus a single rolled-up summary.tsv at the bundle root.
#
# Usage:
#   HEADLESS=1 ./run_loadbearing_sweep.sh
#   HEADLESS=1 PARAMS="KP_SCALE OMEGA_SCALE" ./run_loadbearing_sweep.sh
#   HEADLESS=1 MULTIPLIERS="0.75 1.25" ./run_loadbearing_sweep.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/LoadBearing_Sweep/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

# ICs mirror multi_Init_Var.m (ENU)
ICS=(
  "0.0,0.0,5.0"
  "2.0,2.0,5.0"
  "-2.0,2.0,5.0"
  "2.0,2.0,7.0"
  "2.0,2.0,3.0"
)
IC_NAMES=("IC1_origin5" "IC2_pos5" "IC3_neg5" "IC4_high7" "IC5_low3")

PARAMS="${PARAMS:-KP_SCALE P20_SCALE P2INF_SCALE OMEGA_SCALE}"
MULTIPLIERS="${MULTIPLIERS:-0.5 0.75 1.25 1.5}"
INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "param\tmult\tic_idx\tic_name\tic_enu\tlanded\txy_err_m\trel_vel_mps\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_config() {
  local param="$1" mult="$2" ic_idx="$3" ic_name="$4" ic="$5"
  local env_var="PLASMC_${param}"
  local dst_dir="$BUNDLE_DIR/${param}_${mult}/${ic_name}"
  mkdir -p "$(dirname "$dst_dir")"

  echo
  echo "=================================================================="
  echo "  param=$param mult=$mult IC=$ic_name ($ic)"
  echo "=================================================================="

  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)

  env "${env_var}=${mult}" INITIAL_DRONE_ENU="$ic" \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" \
      > "$dst_dir.log" 2>&1
  local rc=$?

  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    echo "[sweep] no new result dir — run failed"
    printf "%s\t%s\t%s\t%s\t%s\tNO\t-\t-\t-\t-\t-\n" \
      "$param" "$mult" "$ic_idx" "$ic_name" "$ic" >> "$SUMMARY"
    return
  fi

  local src="$HOME/ws/Test_Data/Landing_Test/$latest"
  cp -r "$src" "$dst_dir"

  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst_dir" << 'PY'
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
    "$param" "$mult" "$ic_idx" "$ic_name" "$ic" "$metrics" "$(basename "$dst_dir")" >> "$SUMMARY"
}

# Baseline pass first (×1.0 on every parameter — single 5-IC sweep)
if [ "$INCLUDE_BASELINE" = "1" ]; then
  for i in "${!ICS[@]}"; do
    run_config "BASELINE" "1.0" "$((i+1))" "${IC_NAMES[$i]}" "${ICS[$i]}"
    sleep 2
  done
fi

# Param × multiplier × IC grid
for param in $PARAMS; do
  for mult in $MULTIPLIERS; do
    for i in "${!ICS[@]}"; do
      run_config "$param" "$mult" "$((i+1))" "${IC_NAMES[$i]}" "${ICS[$i]}"
      sleep 2
    done
  done
done

echo
echo "=================================================================="
echo "=== Load-bearing sweep complete — bundle: $BUNDLE_DIR"
echo "=================================================================="
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-(param, mult) aggregate:"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
agg = defaultdict(lambda: dict(soft=0, prec=0, sp=0, n=0, xy=[], vel=[]))
for r in rows:
    if r['landed'] != 'YES': continue
    k = (r['param'], r['mult'])
    a = agg[k]
    a['n'] += 1
    a['soft'] += int(r['soft'])
    a['prec'] += int(r['precise'])
    a['sp']   += int(r['precise']) * int(r['soft'])
    try:
        a['xy'].append(float(r['xy_err_m']))
        a['vel'].append(float(r['rel_vel_mps']))
    except: pass
print(f"\n{'param':<14} {'mult':<6} {'land':>6} {'soft':>6} {'prec':>6} {'SP':>4}"
      f"  {'mean xy':>9} {'max xy':>9} {'mean vel':>10}")
print("-"*78)
for k, a in sorted(agg.items()):
    mxy = sum(a['xy'])/len(a['xy']) if a['xy'] else 0
    mxymax = max(a['xy']) if a['xy'] else 0
    mvel = sum(a['vel'])/len(a['vel']) if a['vel'] else 0
    print(f"{k[0]:<14} {k[1]:<6} {a['n']:>6} {a['soft']:>6} {a['prec']:>6} {a['sp']:>4}"
          f"  {mxy:>9.3f} {mxymax:>9.3f} {mvel:>10.3f}")
PY
