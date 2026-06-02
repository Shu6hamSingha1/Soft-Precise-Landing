#!/usr/bin/env bash
# Multi-IC SITL landing sweep — Python port of MATLAB Multi_init_cond/multi_Init_Var.m.
# Sweeps the 5 nominal initial conditions over a stationary ArUco target and
# collects the soft/precise classification per run.
#
# MATLAB ICs (NED, line 27-33 of multi_Init_Var.m) → ENU equivalent:
#   p0 NED        ENU (Gazebo world)
#   ──────────    ───────────────────
#   ( 0, 0,-5)    (0, 0, 5)
#   ( 2, 2,-5)    (2, 2, 5)
#   ( 2,-2,-5)    (-2, 2, 5)
#   ( 2, 2,-7)    (2, 2, 7)
#   ( 2, 2,-3)    (2, 2, 3)
# (NED→ENU: ENU_x = NED_y, ENU_y = NED_x, ENU_z = -NED_z)
#
# Each IC drives one run of run_aruco_landing_retry.sh (with its built-in
# IC-convergence + retry-on-PX4-flake handling). Results from each run land
# in ~/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/<timestamp>/ and get aggregated by name into
# this script's summary at the end. We also copy each successful run dir into
# a multi-IC bundle for compact downstream analysis.
#
# Usage:
#   HEADLESS=1 ./run_multi_ic_landing.sh
#   HEADLESS=1 PER_IC_REPEATS=3 ./run_multi_ic_landing.sh   # 3 runs per IC

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BUNDLE_DIR="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Multi_IC/${TIMESTAMP}"
mkdir -p "$BUNDLE_DIR"
PER_IC_REPEATS="${PER_IC_REPEATS:-1}"

# IC list mirrors MATLAB multi_Init_Var.m line 27-33, in ENU (x, y, z) = (east, north, up).
ICS=(
  "0.0,0.0,5.0"
  "2.0,2.0,5.0"
  "-2.0,2.0,5.0"
  "2.0,2.0,7.0"
  "2.0,2.0,3.0"
)

# Summary line per run: ic_index | ic | result_dir | classification
SUMMARY="$BUNDLE_DIR/summary.tsv"
printf "ic_idx\trepeat\tic_enu\tresult_dir\tlanded\txy_err_m\trel_vel_mps\tprecise\tsoft\n" > "$SUMMARY"

run_one() {
  local ic_idx="$1" repeat="$2" ic="$3"
  echo
  echo "=============================================================="
  echo "=== IC $ic_idx/${#ICS[@]} repeat $repeat/$PER_IC_REPEATS  ENU=$ic"
  echo "=============================================================="

  # Snapshot mtime of latest Landing_Test dir BEFORE the run, so we can detect
  # the new one after.
  local before
  before=$(ls -t "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/" 2>/dev/null | head -1 || true)

  INITIAL_DRONE_ENU="$ic" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
    bash "$SCRIPT_DIR/run_aruco_landing_retry.sh"
  local rc=$?
  echo "[multi_ic] run rc=$rc"

  # Find the new run dir (most recent that's different from `before`)
  local latest
  latest=$(ls -t "$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/" 2>/dev/null | head -1 || true)
  if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
    echo "[multi_ic] no new result dir — run probably failed"
    printf "%d\t%d\t%s\tFAIL\t0\t-\t-\t-\t-\n" \
      "$ic_idx" "$repeat" "$ic" >> "$SUMMARY"
    return
  fi

  local src="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Landing_Test/$latest"
  local dst="$BUNDLE_DIR/IC_${ic_idx}_rep_${repeat}"
  cp -r "$src" "$dst"
  echo "[multi_ic] copied $latest → $dst"

  # Extract SoftPrecise from saved Ground_Truth.npy
  local metrics
  metrics=$("$HOME/ws/scripts/env2025/bin/python3" - "$dst" << 'PY'
import sys, numpy as np, os
d = sys.argv[1]
try:
    gt = np.load(os.path.join(d, "Ground_Truth.npy"), allow_pickle=True).item()
    sp = gt.get("SoftPrecise", {})
    if sp:
        print(f"{sp.get('xy_err', 0):.4f}\t{sp.get('rel_vel', 0):.4f}\t"
              f"{int(sp.get('precise', False))}\t{int(sp.get('soft', False))}")
    else:
        print("0\t0\t0\t0")
except Exception as e:
    print("0\t0\t0\t0")
PY
)
  printf "%d\t%d\t%s\t%s\tYES\t%s\n" \
    "$ic_idx" "$repeat" "$ic" "$(basename "$dst")" "$metrics" >> "$SUMMARY"
}

for i in "${!ICS[@]}"; do
  for rep in $(seq 1 "$PER_IC_REPEATS"); do
    run_one "$((i + 1))" "$rep" "${ICS[$i]}"
    sleep 3
  done
done

echo
echo "=============================================================="
echo "=== Multi-IC sweep complete — bundle: $BUNDLE_DIR"
echo "=============================================================="
column -t -s $'\t' "$SUMMARY"
echo
echo "Pass-rate breakdown:"
"$HOME/ws/scripts/env2025/bin/python3" - "$SUMMARY" << 'PY'
import sys, csv
rows = list(csv.DictReader(open(sys.argv[1]), delimiter='\t'))
n_total = len(rows)
landed = [r for r in rows if r['landed'] == 'YES']
soft   = [r for r in landed if r['soft']    == '1']
prec   = [r for r in landed if r['precise'] == '1']
sp     = [r for r in landed if r['soft'] == '1' and r['precise'] == '1']
print(f"  Total runs:       {n_total}")
print(f"  Landed:           {len(landed)}/{n_total}")
print(f"  Soft:             {len(soft)}/{len(landed) if landed else 1}")
print(f"  Precise:          {len(prec)}/{len(landed) if landed else 1}")
print(f"  Soft + Precise:   {len(sp)}/{len(landed) if landed else 1}")
PY
