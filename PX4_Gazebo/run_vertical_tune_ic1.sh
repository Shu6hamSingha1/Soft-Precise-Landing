#!/usr/bin/env bash
# Vertical-landing IC 1 parameter sweep — FULL controller (lateral PID
# active, wx/wy enabled). IC 1 = drone directly over marker, so the
# "vertical landing" scenario is just descent + yaw + light lateral
# correction for noise. Sweeps z-axis SMC + yaw + lateral PID gains.
#
# Sweeps each parameter through {0.5, 0.75, 1.25, 1.5} multipliers on
# IC 1 (0,0,5) ENU with full controller. ~19 axes × 4 multipliers + 1
# baseline + 4 N_Z scalars ≈ 81 runs (~2 hours).

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/VerticalTune_IC1/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"

# z-channel-relevant axes (no lateral PID since vertical-only).
# N_Z is a SCALAR override (not a multiplier), handled separately.
PARAMS="${PARAMS:-KP_SCALE KI_SCALE KD_SCALE \
                  OMEGA_SCALE GAMMA_SCALE E_SCALE \
                  N_SCALE P_SCALE KAPPA0_SCALE \
                  P20_SCALE P2INF_SCALE XI2_SCALE \
                  YAW_OMEGA_SCALE YAW_GAMMA_SCALE YAW_N_SCALE \
                  YAW_P_SCALE YAW_KAPPA0_SCALE YAW_E_SCALE}"
MULTIPLIERS="${MULTIPLIERS:-0.5 0.75 1.25 1.5}"
INCLUDE_BASELINE="${INCLUDE_BASELINE:-1}"

# Also sweep N_Z directly (scalar, not multiplier).
N_Z_VALUES="${N_Z_VALUES:-0.005 0.01 0.04 0.08}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "param\tmult\tlanded\txy_err_m\trel_vel_mps\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_config() {
  local param="$1" mult="$2" env_var="$3" env_value="$4"
  local dst_dir="$BUNDLE_DIR/${param}_${mult}"
  echo
  echo "============================================================"
  echo "  param=$param  mult=$mult  (${env_var}=${env_value})"
  echo "============================================================"

  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)

  if [ -n "$env_var" ]; then
    env "${env_var}=${env_value}" INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" \
        > "$dst_dir.log" 2>&1
  else
    env INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" \
        > "$dst_dir.log" 2>&1
  fi
  local rc=$?

  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    echo "[vert-tune] no new result dir — run failed"
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
  run_config "BASELINE" "1.0" "" ""
  sleep 2
fi

# N_Z is a scalar override (not multiplier)
for v in $N_Z_VALUES; do
  run_config "N_Z" "$v" "PLASMC_N_Z" "$v"
  sleep 2
done

# Multiplier-based axes
for param in $PARAMS; do
  for mult in $MULTIPLIERS; do
    run_config "$param" "$mult" "PLASMC_${param}" "$mult"
    sleep 2
  done
done

echo
echo "============================================================"
echo "=== Vertical tune sweep complete: $BUNDLE_DIR"
echo "============================================================"
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-axis best-multiplier ranking (score = xy + 5·vel):"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
baseline = None
per_param = defaultdict(list)
for r in rows:
    if r['landed'] != 'YES': continue
    try: xy, vel = float(r['xy_err_m']), float(r['rel_vel_mps'])
    except: continue
    score = xy + 5*vel
    if r['param'] == 'BASELINE':
        baseline = (xy, vel, score)
    else:
        per_param[r['param']].append((r['mult'], xy, vel, score, int(r['soft']), int(r['precise'])))
print(f"\nBASELINE: xy={baseline[0]:.3f}, vel={baseline[1]:.3f}, score={baseline[2]:.3f}\n")
print(f"{'param':<18} {'mult':>6} {'xy':>7} {'vel':>7} {'score':>7} {'soft':>4} {'Δ':>7}")
print("-"*65)
ranking = []
for p, runs in per_param.items():
    runs.sort(key=lambda r: r[3])
    best = runs[0]
    delta = best[3] - baseline[2]
    ranking.append((delta, p, best))
ranking.sort()
for delta, p, best in ranking:
    print(f"{p:<18} {best[0]:>6} {best[1]:>7.3f} {best[2]:>7.3f} {best[3]:>7.3f} {best[4]:>4} {delta:>+7.3f}")
PY
