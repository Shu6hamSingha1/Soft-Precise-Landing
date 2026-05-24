#!/usr/bin/env bash
# Stack test: RHOFOVINF × 0.5 (for softness) + KP_X/Y × 1.25 (to recover
# precision).  Last untried single-experiment that could plausibly move
# off the precision-softness frontier.
#
# Risk: KP boosts historically broke at × 1.5 (instability cascade);
# × 1.25 is the conservative version, below that threshold.  RHOFOVINF
# × 0.5 we've shown is stable on its own.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/RhoFov05_KP125/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"
IC="0.0,0.0,5.0"; N_REPS="${N_REPS:-10}"

export LANDING_REF_RAD_OPT_FLOW=-0.70
export IMG_FILTER_WIN=7
export PLASMC_RHOFOVINF_SCALE=0.5
export PLASMC_KP_X_SCALE=1.25
export PLASMC_KP_Y_SCALE=1.25

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy\tvel\tprecise\tsoft\ttarget_lost\tresult_dir\n" > "$SUMMARY"
for r in $(seq 1 "$N_REPS"); do
  dst="$BUNDLE_DIR/rep${r}"
  echo "=== RHOFOV0.5+KP1.25  rep=$r/$N_REPS ==="
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
print(f"RHOFOVINF × 0.5 + KP_X/Y × 1.25  (n={len(rows)})")
print(f"  PRECISE={prec}  SOFT={soft}  SOFT+PRECISE={sp}  TL={int(tl.sum())}")
print(f"  xy:  mean={xy.mean():.3f}  std={xy.std():.3f}  min={xy.min():.4f}  max={xy.max():.3f}")
print(f"  vel: mean={ve.mean():.3f}")
if vh_ends:
    print(f"  vh_end:  mean={np.mean(vh_ends):.3f}  min={min(vh_ends):.3f}  max={max(vh_ends):.3f}")
    print(f"  vz_end:  mean={np.mean(vz_ends):.3f}")
print()
print("Reference points (the frontier so far):")
print(f"  Interventions only:        PREC=1 SOFT=2 SP=1 xy_mean=0.328 xy_min=0.039 vh_end~1.2")
print(f"  RHOFOVINF × 0.5:           PREC=0 SOFT=4 SP=0 xy_mean=0.317 xy_min=0.110 vh_end=0.80")
print(f"  THETACAP × 1.5:            PREC=0 SOFT=1 SP=0 xy_mean=0.315 xy_min=0.099 vh_end=0.94 TL=1")
PYEOF
