#!/usr/bin/env bash
# Paired A/B validation of the cbf2 CBF fixes (2026-06-14) at one IC.
# Each ARM is a full env set (two CBF flags toggled together) — a single-knob
# run_knob_sweep can't express this in one bundle, so this is a dedicated harness
# but reuses run_knob_sweep.sh's per-rep landing harness + summary VERBATIM
# (docs/SH_REFERENCE.md §4, §6, §9). Arms are INTERLEAVED per rep so SITL temporal
# drift (flake clustering) is shared across arms, not confounded with one arm.
#
# Default arms (the decisive headline — old CBF vs both fixes):
#   cbf_old    CBF_LW_ROT=0 CBF_RD3_DIRECT=0   (== verbatim pre-fix; parity-proven)
#   cbf_fixed  CBF_LW_ROT=1 CBF_RD3_DIRECT=1   (the new default)
# Override ARMS for the 4-cell decomposition (A-only / B-only / interaction):
#   ARMS="cbf_old:CBF_LW_ROT=0 CBF_RD3_DIRECT=0|A_only:CBF_LW_ROT=1 CBF_RD3_DIRECT=0|B_only:CBF_LW_ROT=0 CBF_RD3_DIRECT=1|cbf_fixed:CBF_LW_ROT=1 CBF_RD3_DIRECT=1"
#
# Usage:
#   bash scripts/run_cbf_ab.sh                      # IC2, n=5, old vs fixed, headless
#   IC=0.0,0.0,5.0 N_REPS=3 bash scripts/run_cbf_ab.sh   # IC1 regression check
#
# Knobs:
#   ARMS       '|'-separated  name:ENV_ASSIGNS  (ENV_ASSIGNS = space-sep KEY=VAL)
#   IC         spawn pose ENU (default IC2 "2.0,2.0,5.0" — the lateral-wall cell)
#   N_REPS     reps per arm (default 5; n>=5 per feedback_sensitivity_sweep_methodology)
#   EXTRA_ENV  held constant across EVERY rep of EVERY arm (apples-to-apples)
#   HEADLESS   default 1 (offscreen Qt; feedback_sitl_headless_run)
#   BUNDLE_NAME  test_data sub-dir (default CBF_AB)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_NAME="${BUNDLE_NAME:-CBF_AB}"
BUNDLE_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/${BUNDLE_NAME}/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="${IC:-2.0,2.0,5.0}"
N_REPS="${N_REPS:-5}"
EXTRA_ENV="${EXTRA_ENV:-}"
export HEADLESS="${HEADLESS:-1}"
LAND_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test"
ARMS="${ARMS:-cbf_old:CBF_LW_ROT=0 CBF_RD3_DIRECT=0|cbf_fixed:CBF_LW_ROT=1 CBF_RD3_DIRECT=1}"

# Parse ARMS into parallel name / env arrays
declare -a ARM_NAME ARM_ENV
IFS='|' read -r -a _arms <<< "$ARMS"
for a in "${_arms[@]}"; do
  ARM_NAME+=("${a%%:*}")
  ARM_ENV+=("${a#*:}")
done

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "arm\trep\tlanded\txy_err\trel_vel\tflight_s\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"
echo "[cbf_ab] IC=$IC N_REPS=$N_REPS HEADLESS=$HEADLESS bundle=$BUNDLE_DIR"
for i in "${!ARM_NAME[@]}"; do echo "[cbf_ab] arm ${ARM_NAME[$i]}: ${ARM_ENV[$i]}"; done

run_one() {
  local arm="$1" arm_env="$2" rep="$3"
  local dst="$BUNDLE_DIR/${arm}/rep${rep}"
  mkdir -p "$(dirname "$dst")"
  echo "=== $arm  rep=$rep  $(date +%H:%M:%S) ==="
  # no-save detection: latest recording dir (DIRECTORIES ONLY, §pitfall 9) before run
  local before
  before=$(ls -td "$LAND_DIR/"*/ 2>/dev/null | head -1 || true)
  env $EXTRA_ENV $arm_env \
      INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -td "$LAND_DIR/"*/ 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\tNO\t-\t-\t-\t-\t-\t-\t-\n" "$arm" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$latest" "$dst"
  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d  = sys.argv[1]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
u  = gt["UAV Pose"]; fs = len(u)/60.0
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{fs:.1f}\t"
      f"{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t"
      f"{int(sp.get('target_lost',False))}")
PY
)
  printf "%s\t%s\tYES\t%s\t%s\n" "$arm" "$rep" "$m" "$(basename "$dst")" >> "$SUMMARY"
}

# Interleave arms within each rep so SITL drift is shared, not confounded.
for r in $(seq 1 "$N_REPS"); do
  for i in "${!ARM_NAME[@]}"; do
    run_one "${ARM_NAME[$i]}" "${ARM_ENV[$i]}" "$r"; sleep 2
  done
done

echo; echo "[cbf_ab] === raw summary ==="
column -t -s $'\t' "$SUMMARY"

echo; echo "[cbf_ab] === per-arm aggregate (clean GT softness recomputed by analyzer) ==="
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
agg = defaultdict(lambda: dict(n=0, xy=[], ve=[], soft=0, prec=0, sp=0, tl=0, nl=0))
for r in rows:
    a = agg[r['arm']]
    if r['landed'] != 'YES':
        a['nl'] += 1; continue
    a['n'] += 1
    a['soft'] += int(r['soft']); a['prec'] += int(r['precise'])
    a['sp']   += int(int(r['soft']) and int(r['precise']))
    a['tl']   += int(r['target_lost'])
    try:
        a['xy'].append(float(r['xy_err'])); a['ve'].append(float(r['rel_vel']))
    except ValueError:
        pass
print(f"  {'arm':<10} {'n':>3} {'SP':>3} {'PREC':>4} {'SOFT':>4} {'TL':>3} {'noland':>6}  "
      f"{'mean xy':>8} {'min xy':>8} {'max xy':>8}  {'mean vel':>9}")
for v, a in agg.items():
    xy = np.array(a['xy']) if a['xy'] else np.array([np.nan])
    ve = np.array(a['ve']) if a['ve'] else np.array([np.nan])
    print(f"  {v:<10} {a['n']:>3} {a['sp']:>3} {a['prec']:>4} {a['soft']:>4} {a['tl']:>3} {a['nl']:>6}  "
          f"{np.nanmean(xy):>8.3f} {np.nanmin(xy):>8.3f} {np.nanmax(xy):>8.3f}  {np.nanmean(ve):>9.3f}")
print("  NOTE: rel_vel is EKF-at-contact (false-soft); judge softness by clean GT vz.")
PY
