#!/usr/bin/env bash
# Batch landing-rate runner: run N reps of a config on rover+platform (GT-FB),
# record each outcome, report the clean-platform-landing rate. Configs are passed
# as "label|EXTRA_ENV..." lines in CFGS. Not tracked (tuning diagnostic).
set -u
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo
LOG=/tmp/claude-1001/-home-shubham-Soft-Precise-Landing/2c6cf0e5-24a9-4219-9bc7-0508d2378bdf/scratchpad
N="${N:-6}"

stray_clean() {
  for p in $(pgrep -f 'build/px4_sitl_default/bin/px4|landing_test.py'); do kill -9 $p 2>/dev/null; done
  for p in $(pgrep -f 'gz sim'); do grep -qa claude /proc/$p/cmdline 2>/dev/null || kill -9 $p 2>/dev/null; done
  pkill -9 -f MicroXRCEAgent 2>/dev/null; sleep 2
}

# configs: label then extra env assignments (space-separated).
# Pin all 3 axes of P2INF (Z=1.5 = the combined auto-align vdf value) to avoid the
# single-axis auto-align bypass. PRINF is lateral-only (2 axes).
declare -a CFGS=(
  "baseline|"
  "p2inf2|PLASMC_P2INF_X=2.0 PLASMC_P2INF_Y=2.0 PLASMC_P2INF_Z=1.5"
  "prinf1|PLASMC_PRINF_X=1.0 PLASMC_PRINF_Y=1.0"
)

for cfg in "${CFGS[@]}"; do
  label="${cfg%%|*}"; extra="${cfg#*|}"
  out="$LOG/lr_$label"; rm -rf "$out"; mkdir -p "$out"
  echo "===================== CONFIG $label  ($extra) ====================="
  for rep in $(seq 1 "$N"); do
    stray_clean
    echo "----- $label rep $rep/$N -----"
    env HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=0 PY_TIMEOUT_S=150 MAX_ATTEMPTS=4 \
        LANDING_AUTOSAVE=1 LANDING_OUT_BASE="$out" $extra \
        bash "$SD/scripts/run_rover_landing_retry.sh" > "$LOG/lr_${label}_${rep}.out" 2>&1
    rm -f "$SD"/run_logs/px4_*.log 2>/dev/null
    d=$(ls -dt "$out"/*/ 2>/dev/null | head -1)
    /home/shubham/ws/scripts/env2025/bin/python3 - "$d" "$label" "$rep" <<'PY'
import sys,numpy as np,os
d,label,rep=sys.argv[1],sys.argv[2],sys.argv[3]
try:
    gt=np.load(os.path.join(d,'Ground_Truth.npy'),allow_pickle=True).item()
    up=gt['UAV Pose']; tp=gt['Target Pose']
    z=np.array([p.position.z for p in up]); x=np.array([p.position.x for p in up]); y=np.array([p.position.y for p in up])
    tx=np.array([p.position.x for p in tp]); ty=np.array([p.position.y for p in tp])
    n=min(len(z),len(tx)); lat=np.hypot(x[:n]-tx[:n],y[:n]-ty[:n]); imin=int(np.argmin(z[:n]))
    clean = (0.35<z[imin]<0.95) and lat[imin]<1.0 and z.max()<15
    print(f"[RESULT] {label} rep{rep}: {'CLEAN' if clean else 'FAIL'} min={z[imin]:.2f} lat@min={lat[imin]:.2f} pk={z.max():.1f}")
except Exception as e:
    print(f"[RESULT] {label} rep{rep}: NODATA {e}")
PY
  done
done
echo "===================== BATCH DONE ====================="
