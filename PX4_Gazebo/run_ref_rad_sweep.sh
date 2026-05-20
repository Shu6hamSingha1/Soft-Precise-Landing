#!/usr/bin/env bash
# Sweep REF_RAD_OPT_FLOW (descent-rate reference for IBVS h_d). Hypothesis:
# in the grace=1.0s data, the BEST rep descended in 8.2s while the WORST
# took 13.9s — same gains, just emergent dynamics. Faster descent → less
# time at low altitude → less drift accumulation.
#
# MATLAB used -0.42 (current default). Try faster: -0.55, -0.70.
# Trade-off: faster descent → harder touchdown (worse SOFT) but maybe
# tighter precision because less drift window.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/RefRadSweep/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"
N_REPS="${N_REPS:-3}"
VALUES="${VALUES:--0.42 -0.55 -0.70}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "ref_rad\trep\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tflight_s\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local val="$1" rep="$2"
  local dst="$BUNDLE_DIR/ref_${val}_rep${rep}"
  echo "=== REF_RAD=$val rep=$rep ==="
  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env LANDING_REF_RAD_OPT_FLOW="$val" INITIAL_DRONE_ENU="$IC" \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\n" "$val" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
u = gt["UAV Pose"]; t = gt["Target Pose"]
xe = u[-1].position.x - t[-1].position.x
ye = u[-1].position.y - t[-1].position.y
flight_s = len(u) / 60.0
print(f"{sp.get('xy_err',0):.4f}\t{xe:.4f}\t{ye:.4f}\t{sp.get('rel_vel',0):.4f}\t{flight_s:.1f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}")
PY
)
  printf "%s\t%s\tYES\t%s\t%s\n" "$val" "$rep" "$m" "$(basename "$dst")" >> "$SUMMARY"
}

for val in $VALUES; do
  for r in $(seq 1 "$N_REPS"); do run_one "$val" "$r"; sleep 2; done
done

echo
column -t -s $'\t' "$SUMMARY"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
from collections import defaultdict
agg = defaultdict(lambda: dict(xy=[], vel=[], fs=[], soft=0, prec=0, n=0))
for r in csv.DictReader(open(sys.argv[1]), delimiter='\t'):
    if r['landed'] != 'YES': continue
    k = r['ref_rad']; a = agg[k]; a['n'] += 1
    a['soft'] += int(r['soft']); a['prec'] += int(r['precise'])
    try:
        a['xy'].append(float(r['xy_err_m']))
        a['vel'].append(float(r['rel_vel_mps']))
        a['fs'].append(float(r['flight_s']))
    except: pass
print(f"\n  {'ref_rad':>8} {'n':>3} {'soft':>4}  {'mean xy':>8} {'max xy':>8} {'std xy':>7}  {'mean vel':>9}  {'mean flight':>11}")
for k in sorted(agg.keys(), key=lambda x: -float(x)):
    a = agg[k]
    if a['n'] == 0: continue
    xy = np.array(a['xy']); vl = np.array(a['vel']); fs = np.array(a['fs'])
    print(f"  {k:>8} {a['n']:>3} {a['soft']:>4}  {xy.mean():>8.3f} {xy.max():>8.3f} {xy.std():>7.3f}  {vl.mean():>9.3f}  {fs.mean():>11.1f}")
PY
