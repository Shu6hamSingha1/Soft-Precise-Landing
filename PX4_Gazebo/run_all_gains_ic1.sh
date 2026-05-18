#!/usr/bin/env bash
# All-gains tuning sweep, IC 1 only — single-IC version of the manuscript's
# 33-axis sweep (Section S3-A). Covers every tunable scalar gain via the
# PLASMC_*_SCALE env vars added to controller.py.
#
# IC 1 = (0,0,5) ENU: drone directly above marker. Provides a clean
# sensitivity baseline without confounding effects from lateral offset.
#
# 16 axes × 4 multipliers = 64 runs + 1 baseline ≈ 65 runs, ~1.5-2 hours.
#
# Manuscript classification (Supplement S3-A):
#   LOAD-BEARING (high impact): KP, P20, P2INF, OMEGA
#   PARETO (low impact, may trade time vs precision): KI, KD, XI2, GAMMA, E, N, P, KAPPA0
#   ROBUST (very low impact): YAW_OMEGA, YAW_GAMMA, YAW_N, YAW_P, YAW_KAPPA0, YAW_E
#   STRUCTURAL: HRD (not multiplicative — separate handling)
#
# Usage:
#   HEADLESS=1 ./run_all_gains_ic1.sh
#   HEADLESS=1 PARAMS="KP_SCALE OMEGA_SCALE" ./run_all_gains_ic1.sh
#   HEADLESS=1 MULTIPLIERS="0.5 2.0" ./run_all_gains_ic1.sh

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/AllGains_IC1/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

# Fixed to IC 1: (0,0,5) ENU
IC="0.0,0.0,5.0"

# All tunable scalar axes. Yaw set is "robust" but included for completeness.
PARAMS="${PARAMS:-KP_SCALE KI_SCALE KD_SCALE \
                  XI2_SCALE P20_SCALE P2INF_SCALE \
                  OMEGA_SCALE GAMMA_SCALE E_SCALE \
                  N_SCALE P_SCALE KAPPA0_SCALE \
                  YAW_OMEGA_SCALE YAW_GAMMA_SCALE YAW_N_SCALE \
                  YAW_P_SCALE YAW_KAPPA0_SCALE YAW_E_SCALE \
                  RHOFOV0_SCALE RHOFOVINF_SCALE LFOV_SCALE THETACAP_SCALE}"

MULTIPLIERS="${MULTIPLIERS:-0.5 0.75 1.25 1.5}"
INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "param\tmult\tlanded\txy_err_m\trel_vel_mps\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_config() {
  local param="$1" mult="$2"
  local env_var="PLASMC_${param}"
  local dst_dir="$BUNDLE_DIR/${param}_${mult}"
  echo
  echo "=========================================================="
  echo "  param=$param  mult=$mult"
  echo "=========================================================="

  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)

  env "${env_var}=${mult}" INITIAL_DRONE_ENU="$IC" \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" \
      > "$dst_dir.log" 2>&1
  local rc=$?

  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    echo "[ic1-sweep] no new result dir — run failed"
    printf "%s\t%s\tNO\t-\t-\t-\t-\t-\n" "$param" "$mult" >> "$SUMMARY"
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
  printf "%s\t%s\tYES\t%s\t%s\n" "$param" "$mult" "$metrics" "$(basename "$dst_dir")" >> "$SUMMARY"
}

if [ "$INCLUDE_BASELINE" = "1" ]; then
  run_config "BASELINE" "1.0"
  sleep 2
fi

for param in $PARAMS; do
  for mult in $MULTIPLIERS; do
    run_config "$param" "$mult"
    sleep 2
  done
done

echo
echo "=========================================================="
echo "=== All-gains IC 1 sweep complete: $BUNDLE_DIR"
echo "=========================================================="
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-axis ranking (best multiplier by score = xy + 5·vel):"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
per_param = defaultdict(list)
baseline = None
for r in rows:
    if r['landed'] != 'YES': continue
    try:
        xy = float(r['xy_err_m']); vel = float(r['rel_vel_mps'])
    except: continue
    score = xy + 5*vel
    if r['param'] == 'BASELINE':
        baseline = (xy, vel, score)
    else:
        per_param[r['param']].append((float(r['mult']), xy, vel, score,
                                       int(r['soft']), int(r['precise'])))

if baseline:
    print(f"\n  BASELINE: xy={baseline[0]:.3f}, vel={baseline[1]:.3f}, score={baseline[2]:.3f}\n")

print(f"  {'param':<18} {'best_mult':>9} {'best_xy':>9} {'best_vel':>9} {'best_score':>10} {'best Δ':>8}")
print("  " + "-"*70)
ranking = []
for p, runs in per_param.items():
    runs.sort(key=lambda r: r[3])   # by score
    best = runs[0]
    delta = best[3] - baseline[2] if baseline else 0
    ranking.append((delta, p, best))
ranking.sort()
for delta, p, best in ranking:
    flag = "↓" if delta < 0 else "↑"
    print(f"  {p:<18} {best[0]:>9.2f} {best[1]:>9.3f} {best[2]:>9.3f} {best[3]:>10.3f} {delta:>+7.3f}{flag}")
PY
