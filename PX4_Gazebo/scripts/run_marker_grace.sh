#!/usr/bin/env bash
# Test the marker-loss grace period. Plain cap=50, grace=0.5s default,
# 5 reps on IC1. Expectation: heavy-tailed outliers (xy > 1.5m) disappear
# because brief marker dropouts no longer trigger open-loop fallback.
#
# Outlier diagnosis: PlainCap50/20260520-164103/plain_rep2 (xy=2.19m) had
# "LANDING PAD NOT VISIBLE" twice during flight; the second triggered
# 5s of open-loop final descent with zero body rates, drifting on
# residual momentum. Grace period prevents this premature commitment.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/MarkerGrace/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"
N_REPS="${N_REPS:-5}"
GRACE="${GRACE:-0.5}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tn_dropouts\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local rep="$1"
  local dst="$BUNDLE_DIR/grace_rep${rep}"
  echo "=== grace=$GRACE rep=$rep ==="
  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  env LANDING_MARKER_LOSS_GRACE="$GRACE" INITIAL_DRONE_ENU="$IC" \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\n" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$HOME/ws/Test_Data/Landing_Test/$latest" "$dst"
  # Count "NOT VISIBLE" dropouts during flight
  local dropouts
  dropouts=$(grep "LANDING PAD NOT VISIBLE" "$dst.log" 2>/dev/null | wc -l)
  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" "$dropouts" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]; dropouts = sys.argv[2]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
u = gt["UAV Pose"]; t = gt["Target Pose"]
xe = u[-1].position.x - t[-1].position.x
ye = u[-1].position.y - t[-1].position.y
print(f"{sp.get('xy_err',0):.4f}\t{xe:.4f}\t{ye:.4f}\t{sp.get('rel_vel',0):.4f}\t{dropouts}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}")
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
dr = np.array([int(r['n_dropouts']) for r in done])
soft = sum(int(r['soft']) for r in done); prec = sum(int(r['precise']) for r in done)
spr = sum(1 for r in done if int(r['soft']) and int(r['precise']))
print(f"\n  reps={len(done)}  soft={soft}  prec={prec}  soft+prec={spr}")
print(f"  xy_err:    mean={xy.mean():.3f}  max={xy.max():.3f}  std={xy.std():.3f}")
print(f"  rel_vel:   mean={ve.mean():.3f}  max={ve.max():.3f}")
print(f"  dropouts:  mean={dr.mean():.1f}  max={dr.max()}")
print(f"\n  CMP to PlainCap50 (no grace): mean=0.66, max=2.19, std=0.66")
PY
