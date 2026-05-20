#!/usr/bin/env bash
# Retest the previously-best per-axis combo on top of the new cap=50
# default. Before the cap was raised, this combo never reached PRECISE
# because dh_d was saturating 34-50% of samples. Now that the cap is at
# 50 and saturation has fallen to ~29%, the combo's behavior may differ.
#
# Combo:
#   KP_X_SCALE=1.5   (best single-axis pitch-precision tuning)
#   KP_Y_SCALE=0.75  (best single-axis roll-precision tuning)
#   PLASMC_N_Z=0.005 (best yaw adaptive-rate)
# Plus baseline DH_D_MAX=50 (now the default).
#
# 5 reps on IC1 for variance.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/ComboOnCap50/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"
N_REPS="${N_REPS:-5}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tsat_frac_pct\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local rep="$1"
  local dst="$BUNDLE_DIR/combo_rep${rep}"
  echo "=========================================================="
  echo "  Combo rep=$rep (KP_X=1.5, KP_Y=0.75, N_Z=0.005, cap=50)"
  echo "=========================================================="
  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env PLASMC_KP_X_SCALE=1.5 PLASMC_KP_Y_SCALE=0.75 PLASMC_N_Z=0.005 \
      INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\n" "$rep" >> "$SUMMARY"
    return
  fi
  local src="$HOME/ws/Test_Data/Landing_Test/$latest"
  cp -r "$src" "$dst"
  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
try:
    gt   = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    ctrl = np.load(os.path.join(d, "Control_Data.npy"), allow_pickle=True).item()
    sp   = gt.get("SoftPrecise", {})
    uav  = gt["UAV Pose"]; tgt = gt["Target Pose"]
    x_err = uav[-1].position.x - tgt[-1].position.x
    y_err = uav[-1].position.y - tgt[-1].position.y
    dh_d = np.array(ctrl["dh_d(t)"])
    sat = float((np.abs(dh_d) >= 49.95).sum()) / max(dh_d.size, 1) * 100.0
    print(f"{sp.get('xy_err',0):.4f}\t{x_err:.4f}\t{y_err:.4f}\t{sp.get('rel_vel',0):.4f}\t{sat:.1f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}")
except Exception:
    print("0\t0\t0\t0\t0\t0\t0")
PY
)
  printf "%s\tYES\t%s\t%s\n" "$rep" "$metrics" "$(basename "$dst")" >> "$SUMMARY"
}

for r in $(seq 1 "$N_REPS"); do
  run_one "$r"
  sleep 2
done

echo
echo "=========================================================="
echo "=== Combo-on-cap50 complete: $BUNDLE_DIR"
echo "=========================================================="
column -t -s $'\t' "$SUMMARY"

"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv, numpy as np
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
done = [r for r in rows if r['landed']=='YES']
if not done:
    print("No landings"); sys.exit(0)
xy = np.array([float(r['xy_err_m']) for r in done])
ve = np.array([float(r['rel_vel_mps']) for r in done])
sat = np.array([float(r['sat_frac_pct']) for r in done])
soft = sum(int(r['soft']) for r in done)
prec = sum(int(r['precise']) for r in done)
spr = sum(1 for r in done if int(r['soft']) and int(r['precise']))
print(f"\n  reps={len(done)}, soft={soft}, prec={prec}, soft+prec={spr}")
print(f"  xy_err:  mean={xy.mean():.3f}, max={xy.max():.3f}, std={xy.std():.3f}")
print(f"  rel_vel: mean={ve.mean():.3f}, max={ve.max():.3f}")
print(f"  sat%:    mean={sat.mean():.1f}")
PY
