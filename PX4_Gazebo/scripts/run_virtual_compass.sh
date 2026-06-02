#!/usr/bin/env bash
# Validate the virtual-compass + SO(3) yaw control path against the
# legacy Euler-PD + direct-u_a path. Manuscript Section III-B1/B2 says
# psi_d should evolve from image-based alpha_e via the leakage ASMC,
# then enter R_d via the heading vector for SO(3) tracking.
#
# IC1 only (centered start at 5m). Each rep uses the same retry wrapper.

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/VirtualCompass/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"
N_REPS="${N_REPS:-3}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "rep\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tflight_s\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local rep="$1"
  local dst="$BUNDLE_DIR/vcompass_rep${rep}"
  echo "=== virtual_compass=1 rep=$rep ==="
  local before
  before=$(ls -t "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/" 2>/dev/null | head -1 || true)
  env LANDING_VIRTUAL_COMPASS=1 INITIAL_DRONE_ENU="$IC" \
      LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
      bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  local latest
  latest=$(ls -t "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\n" "$rep" >> "$SUMMARY"
    return
  fi
  cp -r "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/$latest" "$dst"
  local m
  m=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
sp = gt.get("SoftPrecise", {})
u = gt["UAV Pose"]; t = gt["Target Pose"]
xe = u[-1].position.x - t[-1].position.x
ye = u[-1].position.y - t[-1].position.y
fs = len(u)/60.0
print(f"{sp.get('xy_err',0):.4f}\t{xe:.4f}\t{ye:.4f}\t{sp.get('rel_vel',0):.4f}\t{fs:.1f}\t{int(sp.get('precise',False))}\t{int(sp.get('soft',False))}")
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
soft = sum(int(r['soft']) for r in done); prec = sum(int(r['precise']) for r in done)
print(f"\n  Virtual-compass IC1 (cap=50, grace=1.0s, REF_RAD=-0.42, K_R=diag(5,5,5))")
print(f"  reps={len(done)}  soft={soft}  prec={prec}")
print(f"  xy_err:  mean={xy.mean():.3f}  max={xy.max():.3f}  std={xy.std():.3f}")
print(f"  rel_vel: mean={ve.mean():.3f}  max={ve.max():.3f}")
print(f"\n  Legacy Euler-PD IC1 ref (5-rep MarkerGrace at -0.42): mean xy=0.49, max=0.77, std=0.24")
PY
