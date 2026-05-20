#!/usr/bin/env bash
# Sweep DH_D_MAX (the cap on smooth4(dh_d)) to test whether the c-term
# saturation in the SMC is a precision-variance source.
#
# Background (from latest landing run, Wed May 20 15-00-29 2026):
#   - dh_d hits the ±20 cap on 280/828 samples (33.8%) — the SMC c-term
#     is saturated 1/3 of the time. Either the cap is too tight (and
#     SMC is clipped from tracking real flow-derivative dynamics), or
#     it's load-bearing as a rate limiter. This sweep finds out.
#
# Test plan:
#   Values: 20 (baseline), 30, 50, 100, 1e9 (effectively no cap)
#   Reps:   3 per value (for variance)
#   IC:     1 (origin, 5m)
#   Total:  5 × 3 = 15 runs (~25 min)

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/DhDMaxSweep/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"
N_REPS="${N_REPS:-3}"
VALUES="${VALUES:-20 30 50 100 1e9}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "dh_d_max\trep\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tsat_frac_pct\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local val="$1" rep="$2"
  local dst="$BUNDLE_DIR/dhdmax_${val}_rep${rep}"
  echo
  echo "=========================================================="
  echo "  PLASMC_DH_D_MAX=$val  rep=$rep"
  echo "=========================================================="

  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)

  env PLASMC_DH_D_MAX="$val" INITIAL_DRONE_ENU="$IC" \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1

  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\n" "$val" "$rep" >> "$SUMMARY"
    return
  fi
  local src="$HOME/ws/Test_Data/Landing_Test/$latest"
  cp -r "$src" "$dst"

  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" "$val" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]; cap = float(sys.argv[2])
try:
    gt   = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    ctrl = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    sp   = gt.get("SoftPrecise", {})
    uav  = gt["UAV Pose"]; tgt = gt["Target Pose"]
    x_err = uav[-1].position.x - tgt[-1].position.x
    y_err = uav[-1].position.y - tgt[-1].position.y
    # Saturation fraction: how often does dh_d hit the cap?
    dh_d = np.array(ctrl["dh_d(t)"])
    sat_frac = float((np.abs(dh_d) >= 0.999 * cap).sum()) / max(dh_d.size, 1) * 100.0
    xy_err = sp.get("xy_err", 0.0)
    rel_vel = sp.get("rel_vel", 0.0)
    prec = int(sp.get("precise", False))
    soft = int(sp.get("soft", False))
    print(f"{xy_err:.4f}\t{x_err:.4f}\t{y_err:.4f}\t{rel_vel:.4f}\t{sat_frac:.1f}\t{prec}\t{soft}")
except Exception as e:
    print(f"0\t0\t0\t0\t0\t0\t0")
PY
)
  printf "%s\t%s\tYES\t%s\t%s\n" "$val" "$rep" "$metrics" "$(basename "$dst")" >> "$SUMMARY"
}

for val in $VALUES; do
  for r in $(seq 1 "$N_REPS"); do
    run_one "$val" "$r"
    sleep 2
  done
done

echo
echo "=========================================================="
echo "=== DH_D_MAX sweep complete: $BUNDLE_DIR"
echo "=========================================================="
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-cap aggregate (N=$N_REPS each):"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
agg = defaultdict(lambda: dict(n=0, soft=0, prec=0, xy=[], vel=[], sat=[], xerr=[], yerr=[]))
for r in rows:
    if r['landed'] != 'YES': continue
    k = r['dh_d_max']
    a = agg[k]; a['n'] += 1
    a['soft'] += int(r['soft']); a['prec'] += int(r['precise'])
    try:
        a['xy'].append(float(r['xy_err_m']))
        a['vel'].append(float(r['rel_vel_mps']))
        a['sat'].append(float(r['sat_frac_pct']))
        a['xerr'].append(float(r['x_err_m']))
        a['yerr'].append(float(r['y_err_m']))
    except: pass

import numpy as np
print(f"\n  {'cap':>6} {'n':>3} {'soft':>4} {'prec':>4}  {'mean xy':>8} {'max xy':>8} {'std xy':>7}  {'mean vel':>8}  {'sat%':>6}")
for k in sorted(agg.keys(), key=lambda x: float(x)):
    a = agg[k]
    if a['n'] == 0: continue
    xy = np.array(a['xy'])
    print(f"  {k:>6} {a['n']:>3} {a['soft']:>4} {a['prec']:>4}  {xy.mean():>8.3f} {xy.max():>8.3f} {xy.std():>7.3f}  {np.mean(a['vel']):>8.3f}  {np.mean(a['sat']):>6.1f}")

print(f"\n  Best mean xy:    cap={min(agg.items(), key=lambda kv: np.mean(kv[1]['xy']) if kv[1]['xy'] else 1e9)[0]}")
print(f"  Best max  xy:    cap={min(agg.items(), key=lambda kv: max(kv[1]['xy']) if kv[1]['xy'] else 1e9)[0]}")
print(f"  Lowest variance: cap={min(agg.items(), key=lambda kv: np.std(kv[1]['xy']) if len(kv[1]['xy'])>1 else 1e9)[0]}")
PY
