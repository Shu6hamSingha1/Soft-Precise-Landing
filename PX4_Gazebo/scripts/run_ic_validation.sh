#!/usr/bin/env bash
# Validate current finalized config (cap=50, grace=1.0s, REF_RAD=-0.70,
# κ_0×1.25) across MATLAB's standard 5-IC set. All session tuning was
# done on IC1 — check that the precision improvements transfer to the
# other ICs (off-center, different altitudes) and don't regress them.
#
# MATLAB ICs in NED (Multi_init_cond/multi_Init_Var.m:27-33):
#   IC1: (0, 0, -5)  → ENU (0, 0, 5)
#   IC2: (2, 2, -5)  → ENU (2, 2, 5)  off-center, same alt
#   IC3: (2,-2, -5)  → ENU (-2, 2, 5) opposite side
#   IC4: (2, 2, -7)  → ENU (2, 2, 7)  higher
#   IC5: (2, 2, -3)  → ENU (2, 2, 3)  lower
#
# IC5 is the canary for REF_RAD=-0.70 — only 3 m of altitude means the
# fast descent may not give IBVS enough time to converge. IC4 stresses
# the opposite (longer descent, more drift opportunity).

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/ICValidation/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

N_REPS="${N_REPS:-2}"

declare -A IC_ENU
IC_ENU[IC2]="2.0,2.0,5.0"
IC_ENU[IC3]="-2.0,2.0,5.0"
IC_ENU[IC4]="2.0,2.0,7.0"
IC_ENU[IC5]="2.0,2.0,3.0"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "ic\tinit_enu\trep\tlanded\txy_err_m\trel_vel_mps\tflight_s\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local ic="$1" rep="$2"
  local enu="${IC_ENU[$ic]}"
  local dst="$BUNDLE_DIR/${ic}_rep${rep}"
  echo "=== $ic (ENU $enu) rep=$rep ==="
  local before
  before=$(ls -t "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/" 2>/dev/null | head -1 || true)
  env INITIAL_DRONE_ENU="$enu" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -t "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\t%s\tNO\t-\t-\t-\t-\t-\t-\n" "$ic" "$enu" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/$latest" "$dst"
  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
u = gt["UAV Pose"]
fs = len(u) / 60.0
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{fs:.1f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}")
PY
)
  printf "%s\t%s\t%s\tYES\t%s\t%s\n" "$ic" "$enu" "$rep" "$m" "$(basename "$dst")" >> "$SUMMARY"
}

for ic in IC2 IC3 IC4 IC5; do
  for r in $(seq 1 "$N_REPS"); do run_one "$ic" "$r"; sleep 2; done
done

echo
column -t -s $'\t' "$SUMMARY"

"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
from collections import defaultdict
agg = defaultdict(lambda: dict(n=0, xy=[], vel=[], fs=[], soft=0, prec=0))
for r in csv.DictReader(open(sys.argv[1]), delimiter='\t'):
    if r['landed'] != 'YES': continue
    k = r['ic']; a = agg[k]; a['n'] += 1
    a['soft'] += int(r['soft']); a['prec'] += int(r['precise'])
    try:
        a['xy'].append(float(r['xy_err_m']))
        a['vel'].append(float(r['rel_vel_mps']))
        a['fs'].append(float(r['flight_s']))
    except: pass

import os as _os
_ref = _os.environ.get("LANDING_REF_RAD_OPT_FLOW", "default")
print(f"\n  Per-IC aggregate (config: cap=50, grace=1.0s, REF_RAD={_ref}, κ_0×1.25)")
print(f"  {'ic':<5} {'n':>3} {'soft':>4} {'prec':>4}  {'mean xy':>8} {'max xy':>8}  {'mean vel':>9}  {'mean s':>8}")
for k in sorted(agg.keys()):
    a = agg[k]
    if a['n'] == 0: continue
    print(f"  {k:<5} {a['n']:>3} {a['soft']:>4} {a['prec']:>4}  {np.mean(a['xy']):>8.3f} {np.max(a['xy']):>8.3f}  {np.mean(a['vel']):>9.3f}  {np.mean(a['fs']):>8.1f}")

print(f"\n  IC1 reference (8 reps total this session at this config): mean xy=0.21 (excl outlier), max=1.85")
PY
