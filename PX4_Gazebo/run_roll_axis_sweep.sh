#!/usr/bin/env bash
# Sweep each Y-axis controller gain individually on IC 1, measure impact
# on roll motion (wx, phi) and X-precision (ENU x_err at touchdown).
#
# Mapping (from controller.py:643-652 inverse kinematics):
#   I_a[1] (east = ENU x) → roll (phi)   → wx command
#   I_a[0] (north = ENU y) → pitch (theta) → wy command
# So ROLL motion is driven by SMC's Y-axis output (index 1 of a_u),
# and that ROLL controls X-position precision in Gazebo ENU.
#
# Axes swept (per-axis Y-scalers, multiplier-based):
#   KP, KI, KD          (outer PID, image-y error)
#   OMEGA, GAMMA, E     (middle SMC)
#   N, P, KAPPA0        (adaptive ODE)
#   XI2, P20, P2INF     (funnel envelope)
# 12 axes × 4 multipliers + 1 baseline = 49 runs (~1.5 hr)

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/RollAxisSweep/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"

PARAMS="${PARAMS:-KP KI KD OMEGA GAMMA E N P KAPPA0 XI2 P20 P2INF}"
MULTIPLIERS="${MULTIPLIERS:-0.5 0.75 1.25 1.5}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "param\tmult\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tmax_phi_deg\tmax_wx_dps\trms_wx_dps\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local param="$1" mult="$2"
  local env_var="PLASMC_${param}_Y_SCALE"
  local dst="$BUNDLE_DIR/${param}_Y_${mult}"
  echo
  echo "=========================================================="
  echo "  param=${param}_Y  mult=${mult}  (${env_var}=${mult})"
  echo "=========================================================="

  local before
  before=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)

  if [ "$param" = "BASELINE" ]; then
    env INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  else
    env "${env_var}=${mult}" INITIAL_DRONE_ENU="$IC" \
        LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" > "$dst.log" 2>&1
  fi

  local latest
  latest=$(ls -t "$HOME/ws/Test_Data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    printf "%s\t%s\tNO\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\n" "$param" "$mult" >> "$SUMMARY"
    return
  fi
  local src="$HOME/ws/Test_Data/Landing_Test/$latest"
  cp -r "$src" "$dst"

  # Compute x_err, y_err, max|phi|, max|wx|, rms|wx| from saved data.
  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
try:
    gt  = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    tel = np.load(os.path.join(d, "Telemetry_Data.npy"), allow_pickle=True).item()
    sp  = gt.get("SoftPrecise", {})
    uav = gt["UAV Pose"]; tgt = gt["Target Pose"]
    # x_err and y_err at TOUCHDOWN (last sample = same instant as SoftPrecise snapshot)
    x_err = uav[-1].position.x - tgt[-1].position.x
    y_err = uav[-1].position.y - tgt[-1].position.y
    # Roll trajectory: extract from quaternion w,x,y,z (PX4)
    q_list = tel["Quaternion"]
    phi = []
    for q in q_list:
        sinr = 2.0*(q.w*q.x + q.y*q.z)
        cosr = 1.0 - 2.0*(q.x*q.x + q.y*q.y)
        phi.append(np.arctan2(sinr, cosr))
    phi = np.array(phi)
    # Body-frame angular rate (roll axis): FRD frame .forward is x (roll axis)
    aw = tel["Angular Velocity FRD"]
    wx = np.array([a.forward_rad_s for a in aw])
    max_phi_deg = float(np.max(np.abs(np.rad2deg(phi))))
    max_wx_dps  = float(np.max(np.abs(np.rad2deg(wx))))
    rms_wx_dps  = float(np.sqrt(np.mean(np.rad2deg(wx)**2)))
    xy_err = sp.get("xy_err", 0.0)
    rel_vel = sp.get("rel_vel", 0.0)
    prec = int(sp.get("precise", False))
    soft = int(sp.get("soft", False))
    print(f"{xy_err:.4f}\t{x_err:.4f}\t{y_err:.4f}\t{rel_vel:.4f}\t{max_phi_deg:.2f}\t{max_wx_dps:.2f}\t{rms_wx_dps:.2f}\t{prec}\t{soft}")
except Exception as e:
    print(f"0\t0\t0\t0\t0\t0\t0\t0\t0")
PY
)
  printf "%s\t%s\tYES\t%s\t%s\n" "$param" "$mult" "$metrics" "$(basename "$dst")" >> "$SUMMARY"
}

run_one "BASELINE" "1.0"
sleep 2

for param in $PARAMS; do
  for mult in $MULTIPLIERS; do
    run_one "$param" "$mult"
    sleep 2
  done
done

echo
echo "=========================================================="
echo "=== Roll-axis sweep complete: $BUNDLE_DIR"
echo "=========================================================="
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-axis impact on ROLL & X-PRECISION (mult that minimizes |x_err|):"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
from collections import defaultdict
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
baseline = None
per_param = defaultdict(list)
for r in rows:
    if r['landed'] != 'YES': continue
    try:
        m = r['mult']; xe = float(r['x_err_m']); ye = float(r['y_err_m'])
        xy = float(r['xy_err_m']); vel = float(r['rel_vel_mps'])
        mphi = float(r['max_phi_deg']); mwx = float(r['max_wx_dps']); rwx = float(r['rms_wx_dps'])
        soft = int(r['soft']); prec = int(r['precise'])
    except: continue
    rec = (m, xe, ye, xy, vel, mphi, mwx, rwx, soft, prec)
    if r['param'] == 'BASELINE': baseline = rec
    else: per_param[r['param']].append(rec)

if baseline:
    print(f"\n  BASELINE: |x_err|={abs(baseline[1]):.3f}, |y_err|={abs(baseline[2]):.3f}, ",
          f"max|phi|={baseline[5]:.1f}°, max|wx|={baseline[6]:.1f}°/s, rms|wx|={baseline[7]:.1f}°/s")
    print(f"            xy_err={baseline[3]:.3f}, vel={baseline[4]:.3f}\n")

print(f"  {'param':<8} {'best_m':>7} {'|x_err|':>7} {'max|phi|':>9} {'max|wx|':>8} {'rms|wx|':>8} {'xy_err':>7}")
print("  " + "-"*70)
for p, runs in per_param.items():
    runs.sort(key=lambda r: abs(r[1]))
    best = runs[0]
    print(f"  {p:<8} {best[0]:>7} {abs(best[1]):>7.3f} {best[5]:>9.1f} {best[6]:>8.1f} {best[7]:>8.1f} {best[3]:>7.3f}")
PY
