#!/usr/bin/env bash
# Validate gain-scheduling: IC1 n=5 quick check + full IC2-5 validation.
# Should recover IC2-5 (K_rp_far=9 takes effect there) AND maintain
# the K_rp_close=4 benefit at IC1.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/SchedValidate/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"
export LANDING_REF_RAD_OPT_FLOW=-0.70
export IMG_FILTER_WIN=7

# IC1 quick check (n=5)
echo "=== IC1 (n=5) ==="
IC="0.0,0.0,5.0"
SUMMARY="$BUNDLE_DIR/ic1.tsv"
printf "rep\tlanded\txy\tvel\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"
for r in $(seq 1 5); do
  dst="$BUNDLE_DIR/ic1_rep${r}"
  echo "--- IC1 rep=$r/5 ---"
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\n" "$r" >> "$SUMMARY"; sleep 2; continue
  fi
  cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
gt = np.load(os.path.join(sys.argv[1], "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t{int(sp.get('target_lost',False))}")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$r" "$m" "$(basename "$dst")" >> "$SUMMARY"
  sleep 2
done

# IC2-5 (n=2 each)
declare -A IC_ENU
IC_ENU[IC2]="2.0,2.0,5.0"
IC_ENU[IC3]="-2.0,2.0,5.0"
IC_ENU[IC4]="2.0,2.0,7.0"
IC_ENU[IC5]="2.0,2.0,3.0"
IC_SUMMARY="$BUNDLE_DIR/ic2to5.tsv"
printf "ic\trep\tlanded\txy\tvel\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$IC_SUMMARY"
for ic in IC2 IC3 IC4 IC5; do
  enu="${IC_ENU[$ic]}"
  for r in 1 2; do
    dst="$BUNDLE_DIR/${ic}_rep${r}"
    echo "--- $ic ($enu) rep=$r ---"
    before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
    env INITIAL_DRONE_ENU="$enu" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
    latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
    if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
      printf "%s\t%s\tNO\t-\t-\t-\t-\t-\t-\n" "$ic" "$r" >> "$IC_SUMMARY"; sleep 2; continue
    fi
    cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
    m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
gt = np.load(os.path.join(sys.argv[1], "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t{int(sp.get('target_lost',False))}")
PY
)
    printf "%s\t%s\tYES\t%s\t%s\n" "$ic" "$r" "$m" "$(basename "$dst")" >> "$IC_SUMMARY"
    sleep 2
  done
done

echo
echo "=== IC1 RESULTS ==="
column -t -s $'\t' "$SUMMARY"
echo
echo "=== IC2-5 RESULTS ==="
column -t -s $'\t' "$IC_SUMMARY"

~/ws/scripts/env2025/bin/python3 << PYEOF
import csv, numpy as np
print()
print("Summary (gain-scheduling enabled):")
ic1_rows = [r for r in csv.DictReader(open("$SUMMARY"), delimiter='\t') if r['landed']=='YES']
if ic1_rows:
    xy = np.array([float(r['xy']) for r in ic1_rows])
    ve = np.array([float(r['vel']) for r in ic1_rows])
    prec = sum(int(r['precise']) for r in ic1_rows); soft = sum(int(r['soft']) for r in ic1_rows)
    sp = sum(1 for r in ic1_rows if int(r['precise']) and int(r['soft']) and not int(r['target_lost']))
    print(f"  IC1 (n={len(ic1_rows)}): PREC={prec} SOFT={soft} SP={sp}  xy_mean={xy.mean():.3f}  xy_min={xy.min():.4f}  vel_mean={ve.mean():.3f}")
    print()
    print("  IC1 reference points:")
    print(f"    OLD default (K_rp=9, P_z=5):  PREC=1 SOFT=2 SP=1 xy_mean=0.328 xy_min=0.039")
    print(f"    K_rp=4 + P_z=2.5 (no sched):  PREC=1 SOFT=3 SP=1 xy_mean=0.465 xy_min=0.026")

ic_rows = list(csv.DictReader(open("$IC_SUMMARY"), delimiter='\t'))
ic_rows = [r for r in ic_rows if r['landed']=='YES']
from collections import defaultdict
by_ic = defaultdict(list)
for r in ic_rows: by_ic[r['ic']].append(r)
print()
print("  IC2-5 (gain scheduling):")
for ic in sorted(by_ic):
    rs = by_ic[ic]
    xy = np.array([float(r['xy']) for r in rs])
    print(f"    {ic}: n={len(rs)}, xy_mean={xy.mean():.3f}, xy_max={xy.max():.3f}")
print()
print("  IC2-5 reference (NO scheduling, K_rp=4 only):")
print(f"    IC2: mean=2.16, IC3: 1.91, IC4: 2.65, IC5: 1.72")
print(f"  IC2-5 reference (OLD default K_rp=9):")
print(f"    IC2: mean=1.64, IC3: 1.79, IC4: 1.59, IC5: 2.88")
PYEOF
