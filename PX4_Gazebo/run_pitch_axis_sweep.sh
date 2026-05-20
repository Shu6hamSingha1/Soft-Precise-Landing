#!/usr/bin/env bash
# Sweep each X-axis controller gain individually on IC 1, measure impact
# on pitch motion (wy, theta) and Y-precision (ENU y_err at touchdown).
#
# Mapping (from controller.py:643-652 inverse kinematics):
#   I_a[0] (north = ENU y) → pitch (theta) → wy command
#   I_a[1] (east  = ENU x) → roll  (phi)   → wx command
# PITCH motion is driven by SMC's X-axis output (index 0 of a_u),
# and that PITCH controls Y-position precision in Gazebo ENU.
#
# Axes swept (per-axis X-scalers, multiplier-based):
#   KP, KI, KD          (outer PID, image-x error)
#   OMEGA, GAMMA, E     (middle SMC)
#   N, P, KAPPA0        (adaptive ODE)
#   XI2, P20, P2INF     (funnel envelope)
# 12 axes × 4 multipliers + 1 baseline = 49 runs (~1.5 hr)

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/ws/Test_Data/PitchAxisSweep/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"

IC="0.0,0.0,5.0"

PARAMS="${PARAMS:-KP KI KD OMEGA GAMMA E N P KAPPA0 XI2 P20 P2INF}"
MULTIPLIERS="${MULTIPLIERS:-0.5 0.75 1.25 1.5}"

SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "param\tmult\tlanded\txy_err_m\tx_err_m\ty_err_m\trel_vel_mps\tmax_theta_deg\tmax_wy_dps\trms_wy_dps\tprecise\tsoft\tresult_dir\n" > "$SUMMARY"

run_one() {
  local param="$1" mult="$2"
  local env_var="PLASMC_${param}_X_SCALE"
  local dst="$BUNDLE_DIR/${param}_X_${mult}"
  echo
  echo "=========================================================="
  echo "  param=${param}_X  mult=${mult}  (${env_var}=${mult})"
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

  # Compute x_err, y_err, max|theta|, max|wy|, rms|wy| from saved data.
  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
try:
    gt  = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    tel = np.load(os.path.join(d, "Telemetry_Data.npy"), allow_pickle=True).item()
    sp  = gt.get("SoftPrecise", {})
    uav = gt["UAV Pose"]; tgt = gt["Target Pose"]
    x_err = uav[-1].position.x - tgt[-1].position.x
    y_err = uav[-1].position.y - tgt[-1].position.y
    # Pitch trajectory: extract from quaternion
    q_list = tel["Quaternion"]
    theta = []
    for q in q_list:
        sinp = 2.0*(q.w*q.y - q.z*q.x)
        sinp = np.clip(sinp, -1.0, 1.0)
        theta.append(np.arcsin(sinp))
    theta = np.array(theta)
    # Body-frame angular rate (pitch axis): FRD .right is y (pitch axis)
    aw = tel["Angular Velocity FRD"]
    wy = np.array([a.right_rad_s for a in aw])
    max_theta_deg = float(np.max(np.abs(np.rad2deg(theta))))
    max_wy_dps    = float(np.max(np.abs(np.rad2deg(wy))))
    rms_wy_dps    = float(np.sqrt(np.mean(np.rad2deg(wy)**2)))
    xy_err = sp.get("xy_err", 0.0)
    rel_vel = sp.get("rel_vel", 0.0)
    prec = int(sp.get("precise", False))
    soft = int(sp.get("soft", False))
    print(f"{xy_err:.4f}\t{x_err:.4f}\t{y_err:.4f}\t{rel_vel:.4f}\t{max_theta_deg:.2f}\t{max_wy_dps:.2f}\t{rms_wy_dps:.2f}\t{prec}\t{soft}")
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
echo "=== Pitch-axis sweep complete: $BUNDLE_DIR"
echo "=========================================================="
column -t -s $'\t' "$SUMMARY"

echo
echo "Per-axis impact on PITCH & Y-PRECISION (mult that minimizes |y_err|):"
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
        mth = float(r['max_theta_deg']); mwy = float(r['max_wy_dps']); rwy = float(r['rms_wy_dps'])
        soft = int(r['soft']); prec = int(r['precise'])
    except: continue
    rec = (m, xe, ye, xy, vel, mth, mwy, rwy, soft, prec)
    if r['param'] == 'BASELINE': baseline = rec
    else: per_param[r['param']].append(rec)

if baseline:
    print(f"\n  BASELINE: |x_err|={abs(baseline[1]):.3f}, |y_err|={abs(baseline[2]):.3f}")
    print(f"            max|theta|={baseline[5]:.1f}°, max|wy|={baseline[6]:.1f}°/s, rms|wy|={baseline[7]:.1f}°/s")
    print(f"            xy={baseline[3]:.3f}, vel={baseline[4]:.3f}\n")

print(f"  {'param':<8} {'best_m':>7} {'|y_err|':>7} {'max|theta|':>11} {'max|wy|':>8} {'rms|wy|':>8} {'xy_err':>7}")
print("  " + "-"*72)
ranking = []
for p, runs in per_param.items():
    runs.sort(key=lambda r: abs(r[2]))
    best = runs[0]
    ranking.append((abs(best[2]), p, best))
ranking.sort()
for _, p, best in ranking:
    print(f"  {p:<8} {best[0]:>7} {abs(best[2]):>7.3f} {best[5]:>11.1f} {best[6]:>8.1f} {best[7]:>8.1f} {best[3]:>7.3f}")

print(f"\n=== All configs with |y_err| < 0.08 m (PRECISE in y) ===")
all_runs = [(p, *r) for p, runs in per_param.items() for r in runs]
all_runs.sort(key=lambda r: abs(r[3]))
n = 0
for p, m, xe, ye, xy, vel, mth, mwy, rwy, soft, prec in all_runs:
    if abs(ye) < 0.08:
        n += 1
        print(f"  {p:<8} {m}: y_err={ye:+.3f}, x_err={xe:+.3f}, max|theta|={mth:.1f}°, max|wy|={mwy:.1f}°/s, xy={xy:.3f}, vel={vel:.3f}")
print(f"  ({n} configs achieve |y_err| < 0.08 m)")

print(f"\n=== Pitch motion vs |y_err| correlation ===")
import numpy as np
yes = [abs(r[3]) for r in all_runs]
mwys = [r[7] for r in all_runs]
rwys = [r[8] for r in all_runs]
mths = [r[6] for r in all_runs]
print(f"  corr(max|wy|, |y_err|):     {np.corrcoef(mwys, yes)[0,1]:+.3f}")
print(f"  corr(rms|wy|, |y_err|):     {np.corrcoef(rwys, yes)[0,1]:+.3f}")
print(f"  corr(max|theta|, |y_err|):  {np.corrcoef(mths, yes)[0,1]:+.3f}")
PY
