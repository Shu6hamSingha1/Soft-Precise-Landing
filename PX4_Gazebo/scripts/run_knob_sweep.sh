#!/usr/bin/env bash
# Generic single-knob landing sweep at one IC. Drives run_aruco_landing_retry.sh
# N_REPS times per value, routes each value's reps into its own sub-bundle, and
# aggregates SP/xy/vel per value. Reusable across the tuning campaign — do NOT
# fork a per-knob variant (docs/SH_REFERENCE.md §4, §9).
#
# Usage:
#   # baseline (no knob varied), IC1, n=5:
#   bash scripts/run_knob_sweep.sh
#   # sweep the centroid-KF measurement noise, IC1, n=5:
#   SWEEP_KNOB=IMG_FEAT_KF_R SWEEP_VALS="0.001 0.004 0.02" bash scripts/run_knob_sweep.sh
#   # hold extra config constant across every rep:
#   EXTRA_ENV="LANDING_REF_RAD_OPT_FLOW=-0.42 IMG_FEATURE_FILTER=kf" \
#     SWEEP_KNOB=PLASMC_KP_X SWEEP_VALS="9 12 14" N_REPS=5 bash scripts/run_knob_sweep.sh
#
# Knobs:
#   SWEEP_KNOB  env var to vary (empty => single "baseline" group, no override)
#   SWEEP_VALS  space-separated values for SWEEP_KNOB (required iff SWEEP_KNOB set)
#   EXTRA_ENV   extra KEY=VAL assignments applied to EVERY rep (held constant)
#   IC          spawn pose ENU (default IC1 "0.0,0.0,5.0"; see §5 for the set)
#   N_REPS      reps per value (default 5)
#   BUNDLE_NAME test_data sub-dir name (default KnobSweep)
#
# NOTE: this is an IC1 (or single-IC) sweeper. Per memory feedback_ic_validation,
# gate any promising value through `bash scripts/run_ic_validation.sh` (do NOT set
# LANDING_OUT_BASE for that — it self-bundles from the default Landing_Test/).
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_NAME="${BUNDLE_NAME:-KnobSweep}"
BUNDLE_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/${BUNDLE_NAME}/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

SWEEP_KNOB="${SWEEP_KNOB:-}"
IC="${IC:-0.0,0.0,5.0}"
N_REPS="${N_REPS:-5}"
EXTRA_ENV="${EXTRA_ENV:-}"
LAND_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test"

if [ -n "$SWEEP_KNOB" ]; then
  if [ -z "${SWEEP_VALS:-}" ]; then
    echo "[sweep] SWEEP_KNOB=$SWEEP_KNOB set but SWEEP_VALS empty — nothing to sweep." >&2
    exit 2
  fi
  read -r -a VALS <<< "$SWEEP_VALS"
else
  VALS=("__baseline__")
fi

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "knob\tval\trep\tlanded\txy_err\trel_vel\tflight_s\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"
echo "[sweep] knob='${SWEEP_KNOB:-<none>}' vals='${SWEEP_VALS:-baseline}' IC=$IC N_REPS=$N_REPS"
echo "[sweep] extra='$EXTRA_ENV'  bundle=$BUNDLE_DIR"

run_one() {
  local val="$1" rep="$2"
  local tag; if [ "$val" = "__baseline__" ]; then tag="baseline"; else tag="${SWEEP_KNOB}=${val}"; fi
  local dst="$BUNDLE_DIR/${tag}/rep${rep}"
  mkdir -p "$(dirname "$dst")"
  echo "=== $tag  rep=$rep  $(date +%H:%M:%S) ==="

  # Detect a no-save fail: capture latest recording dir (DIRECTORIES ONLY, §pitfall 9) before the run.
  local before
  before=$(ls -td "$LAND_DIR/"*/ 2>/dev/null | head -1 || true)

  local knob_assign=()
  [ "$val" != "__baseline__" ] && knob_assign=("$SWEEP_KNOB=$val")
  env $EXTRA_ENV "${knob_assign[@]}" \
      INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1

  local latest
  latest=$(ls -td "$LAND_DIR/"*/ 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\t%s\tNO\t-\t-\t-\t-\t-\t-\t-\n" "${SWEEP_KNOB:-baseline}" "$val" "$rep" >> "$SUMMARY"
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
  printf "%s\t%s\t%s\tYES\t%s\t%s\n" "${SWEEP_KNOB:-baseline}" "$val" "$rep" "$m" "$(basename "$dst")" >> "$SUMMARY"
}

for val in "${VALS[@]}"; do
  for r in $(seq 1 "$N_REPS"); do run_one "$val" "$r"; sleep 2; done
done

echo; echo "[sweep] === raw summary ==="
column -t -s $'\t' "$SUMMARY"

echo; echo "[sweep] === per-value aggregate ==="
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
agg = defaultdict(lambda: dict(n=0, xy=[], ve=[], soft=0, prec=0, sp=0, tl=0, nl=0))
for r in rows:
    a = agg[r['val']]
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
print(f"  {'val':<10} {'n':>3} {'SP':>3} {'PREC':>4} {'SOFT':>4} {'TL':>3} {'noland':>6}  "
      f"{'mean xy':>8} {'min xy':>8} {'max xy':>8}  {'mean vel':>9}")
for v, a in agg.items():
    xy = np.array(a['xy']) if a['xy'] else np.array([np.nan])
    ve = np.array(a['ve']) if a['ve'] else np.array([np.nan])
    print(f"  {v:<10} {a['n']:>3} {a['sp']:>3} {a['prec']:>4} {a['soft']:>4} {a['tl']:>3} {a['nl']:>6}  "
          f"{np.nanmean(xy):>8.3f} {np.nanmin(xy):>8.3f} {np.nanmax(xy):>8.3f}  {np.nanmean(ve):>9.3f}")
PY
