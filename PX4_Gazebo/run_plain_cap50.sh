#!/usr/bin/env bash
# Confirm plain cap=50 (no gain overrides) with 5 reps to definitively
# compare against the 5-rep combo test. The original 3-rep result was
# mean xy=0.447 / std=0.157 — needs more samples for confidence.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/PlainCap50/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"
N_REPS="${N_REPS:-5}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tsat_frac_pct\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local rep="$1"
  local dst="$BUNDLE_DIR/plain_rep${rep}"
  echo "=== Plain cap=50 rep=$rep ==="
  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\n" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
ctrl = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
u = gt["UAV Pose"]; t = gt["Target Pose"]
xe = u[-1].position.x - t[-1].position.x
ye = u[-1].position.y - t[-1].position.y
dh = np.array(ctrl["dh_d(t)"])
sat = float((np.abs(dh) >= 49.95).sum())/max(dh.size,1)*100.0
print(f"{sp.get('xy_err',0):.4f}\t{xe:.4f}\t{ye:.4f}\t{sp.get('rel_vel',0):.4f}\t{sat:.1f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$rep" "$m" "$(basename "$dst")" >> "$SUMMARY"
}

for r in $(seq 1 "$N_REPS"); do run_one "$r"; sleep 2; done

echo
column -t -s $'\t' "$SUMMARY"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
done = [r for r in csv.DictReader(open(sys.argv[1]), delimiter='\t') if r['landed']=='YES']
if not done: print("no landings"); sys.exit(0)
xy = np.array([float(r['xy_err_m']) for r in done])
ve = np.array([float(r['rel_vel_mps']) for r in done])
sat = np.array([float(r['sat_frac_pct']) for r in done])
soft = sum(int(r['soft']) for r in done); prec = sum(int(r['precise']) for r in done)
spr = sum(1 for r in done if int(r['soft']) and int(r['precise']))
print(f"\n  reps={len(done)}, soft={soft}, prec={prec}, soft+prec={spr}")
print(f"  xy_err:  mean={xy.mean():.3f}, max={xy.max():.3f}, std={xy.std():.3f}")
print(f"  rel_vel: mean={ve.mean():.3f}, max={ve.max():.3f}")
print(f"  sat%:    mean={sat.mean():.1f}")
PY
