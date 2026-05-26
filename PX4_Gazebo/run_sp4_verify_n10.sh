#!/usr/bin/env bash
# Verify the SP-producing config from coord descent (SP #4): n=10 at IC1.
# Tests true SP-rate at this config, not just the lucky single-rep result.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/SP4Verify/${TIMESTAMP}"; mkdir -p "$BUNDLE_DIR"
IC="0.0,0.0,5.0"; N_REPS="${N_REPS:-10}"

# Coord descent state: E_X × 68.84, E_Z × 0.1067 + current defaults
export LANDING_REF_RAD_OPT_FLOW=-0.70
export IMG_FILTER_WIN=7
export PLASMC_KP_SCHED_ENABLE=0
export PLASMC_E_X_SCALE=68.84
export PLASMC_E_Z_SCALE=0.1067

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy\tvel\tprecise\tsoft\ttarget_lost\tvh0\tresult_dir\n" > "$SUMMARY"
for r in $(seq 1 "$N_REPS"); do
  dst="$BUNDLE_DIR/rep${r}"
  echo "=== SP4 verify rep=$r/$N_REPS ==="
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=4 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\t-\n" "$r" >> "$SUMMARY"; sleep 2; continue
  fi
  cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
gt = np.load(os.path.join(sys.argv[1], "Ground_Truth.npy"), allow_pickle=True).item()
td = np.load(os.path.join(sys.argv[1], "Telemetry_Data.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
v0 = td["Velocity Body"][0]
vh0 = (v0.x_m_s**2 + v0.y_m_s**2)**0.5
print(f"{sp.get('xy_err',0):.4f}\t{sp.get('rel_vel',0):.4f}\t"
      f"{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}\t"
      f"{int(sp.get('target_lost',False))}\t{vh0:.4f}")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$r" "$m" "$(basename "$dst")" >> "$SUMMARY"
  sleep 2
done
echo
column -t -s $'\t' "$SUMMARY"
~/ws/scripts/env2025/bin/python3 << PYEOF
import csv, numpy as np
rows = [r for r in csv.DictReader(open("$SUMMARY"), delimiter='\t') if r['landed']=='YES']
if not rows: exit()
xy = np.array([float(r['xy']) for r in rows])
ve = np.array([float(r['vel']) for r in rows])
vh0 = np.array([float(r['vh0']) for r in rows])
prec = sum(int(r['precise']) for r in rows); soft = sum(int(r['soft']) for r in rows)
sp = sum(1 for r in rows if int(r['precise']) and int(r['soft']) and not int(r['target_lost']))
tl = sum(int(r['target_lost']) for r in rows)
print()
print(f"SP4 verification (E_X × 68.84, E_Z × 0.1067):")
print(f"  n={len(rows)}  PREC={prec}  SOFT={soft}  SP={sp}  TL={tl}")
print(f"  xy:  mean={xy.mean():.3f}  min={xy.min():.4f}  max={xy.max():.3f}")
print(f"  vel: mean={ve.mean():.3f}")
print(f"  vh0: mean={vh0.mean():.4f}  min={vh0.min():.4f}  max={vh0.max():.4f}")
print()
print(f"  Per-rep:")
for r in rows:
    sp_mark = "★" if int(r['precise']) and int(r['soft']) and not int(r['target_lost']) else " "
    print(f"    {sp_mark} rep {r['rep']}: xy={r['xy']} vel={r['vel']} vh0={r['vh0']} prec={r['precise']} soft={r['soft']}")
PYEOF
