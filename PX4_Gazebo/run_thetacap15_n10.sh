#!/usr/bin/env bash
# Test: PLASMC_THETACAP_SCALE=1.5 (60° → 90° soft cone ceiling).
# Different mechanism from RHOFOVINF — relaxes the tilt-saturation cap
# so the SMC can demand more lateral acceleration in the final phase,
# providing more braking authority on lateral velocity.
#
# Hypothesis: PRECISE-only reps fail SOFT because lateral velocity isn't
# braked.  More tilt authority = more lateral force available.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/ThetaCap15/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"
IC="0.0,0.0,5.0"; N_REPS="${N_REPS:-10}"

export LANDING_REF_RAD_OPT_FLOW=-0.70
export IMG_FILTER_WIN=7
export PLASMC_THETACAP_SCALE=1.5

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy\tvel\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"
for r in $(seq 1 "$N_REPS"); do
  dst="$BUNDLE_DIR/rep${r}"
  echo "=== THETACAP×1.5  rep=$r/$N_REPS ==="
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
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t"
      f"{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t"
      f"{int(sp.get('target_lost',False))}")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$r" "$m" "$(basename "$dst")" >> "$SUMMARY"
  sleep 2
done
echo
column -t -s $'\t' "$SUMMARY"
echo

~/ws/scripts/env2025/bin/python3 << PYEOF
import csv, numpy as np, os
rows = [r for r in csv.DictReader(open("$SUMMARY"), delimiter='\t') if r['landed']=='YES']
if not rows: exit()
xy = np.array([float(r['xy']) for r in rows])
ve = np.array([float(r['vel']) for r in rows])
tl = np.array([int(r['target_lost']) for r in rows])
prec = sum(int(r['precise']) for r in rows); soft = sum(int(r['soft']) for r in rows)
sp = sum(1 for r in rows if int(r['precise']) and int(r['soft']) and not int(r['target_lost']))
vh_ends = []; vz_ends = []
for r in rows:
    try:
        d = os.path.join("$BUNDLE_DIR", r['result_dir'])
        td = np.load(os.path.join(d, "Telemetry_Data.npy"), allow_pickle=True).item()
        v = td["Velocity Body"][-1]
        vh_ends.append(float(np.hypot(v.x_m_s, v.y_m_s)))
        vz_ends.append(float(v.z_m_s))
    except Exception: pass
print(f"THETACAP × 1.5  (n={len(rows)})")
print(f"  PRECISE={prec}  SOFT={soft}  SOFT+PRECISE={sp}  TL={int(tl.sum())}")
print(f"  xy:  mean={xy.mean():.3f}  std={xy.std():.3f}  min={xy.min():.4f}  max={xy.max():.3f}")
print(f"  vel: mean={ve.mean():.3f}")
if vh_ends:
    print(f"  vh_end:  mean={np.mean(vh_ends):.3f}  min={min(vh_ends):.3f}")
    print(f"  vz_end:  mean={np.mean(vz_ends):.3f}")
print()
print("Reference points:")
print(f"  Interventions only:        PREC=1 SOFT=2 SP=1 xy_mean=0.328 xy_min=0.039 vh_end~1.2")
print(f"  RHOFOVINF × 0.5:           PREC=0 SOFT=4 SP=0 xy_mean=0.317 xy_min=0.110 vh_end=0.80")
print(f"  RHOFOVINF × 0.7:           PREC=1 SOFT=2 SP=0 xy_mean=0.321 xy_min=0.077 vh_end=0.99")
PYEOF
