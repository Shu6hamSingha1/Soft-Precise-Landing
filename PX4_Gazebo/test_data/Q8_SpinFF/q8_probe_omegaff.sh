#!/usr/bin/env bash
# Q8 step 2 -- re-test the EXISTING PLASMC_YAW_OMEGA_D_FF knob (default OFF) at the sin-ceiling
# rate (0.48 rad/s, the documented turning-rover case). Added AFTER -K_R@e_R, clipped only at
# W_U_MAX=2.0 (not the 0.5 sin-ceiling); sourced from the ASMC's OWN _ua_psid, not an
# independently-injected estimate, so it should not double-count the way the falsified
# PLASMC_YAW_WT_FF did (see project_q8_yaw_ff_harmful_with_headroom).
#
# 2026-09-04 REWRITE: the first run of this probe silently attributed 3 of 6 "reps" to a
# CONCURRENT session's detector-robustness work (WORLD=cm_inv, CROSS_GATE_MODE=ensemble) --
# before/latest dir-diff has no way to tell "my newest rep" from "anyone's newest rep" when
# another session writes to the same Landing_Test/ during the window. Fix: verify each copied
# rep's OWN Control_Params.npy overrides match what THIS run actually set, before accepting it;
# reject + retry (bounded) otherwise. See docs/SH_REFERENCE.md pitfall 10 addendum.
set -u
cd "$HOME/Soft-Precise-Landing/PX4_Gazebo"
export HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross
export PLASMC_GT_FEEDBACK=1 PLASMC_GT_SPIN_WZ=0.48
OUT="$HOME/Soft-Precise-Landing/PX4_Gazebo/test_data/Q8_SpinFF_omegaff"; mkdir -p "$OUT"
N_REPS=3
MAXLAUNCH="${MAXLAUNCH:-8}"     # per arm; SITL ~50% flaky + rejects cost an extra attempt

ko(){ for r in 1 2 3; do
        pids=$(ps -eo pid,args | grep -aE 'run_ic_validation|run_landing|px4_sitl_default/bin/px4|gz sim|gz-sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|landing_test' | grep -av grep | awk '{print $1}')
        [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null || true; done; sleep 2
      done; sleep 3; }
gate(){ if ss -lun 2>/dev/null | grep -q ':8888'; then echo "ABORT: udp:8888 bound"; exit 1; fi; }

# verify_attribution DIR KEY=VAL [KEY=VAL ...] -- exits 0 iff DIR/Control_Params.npy's
# Config.overrides matches every given KEY=VAL exactly (this run's own env, not a stale/foreign
# rep's). Prints a one-line reason to stderr on mismatch.
verify_attribution() {
  local dir="$1"; shift
  "$HOME/ws/scripts/env2025/bin/python3" - "$dir" "$@" <<'PY'
import sys, numpy as np
d = sys.argv[1]
# "!KEY" (no '=') means KEY must be ABSENT from overrides; "KEY=VAL" means KEY must equal VAL.
absent = [a[1:] for a in sys.argv[2:] if a.startswith('!')]
expected = dict(kv.split('=', 1) for kv in sys.argv[2:] if not kv.startswith('!'))
try:
    cp = np.load(f"{d}/Control_Params.npy", allow_pickle=True).item()
    ov = cp.get("Config", {}).get("overrides", {})
except Exception as e:
    print(f"NOATTR ({e})", file=sys.stderr); sys.exit(1)
bad = {k: ov.get(k) for k, v in expected.items() if ov.get(k) != v}
present = [k for k in absent if k in ov]
if bad or present:
    print(f"MISMATCH bad={bad} unexpectedly-present={present} "
          f"(not this run's rep -- expected {expected}, absent {absent})", file=sys.stderr)
    sys.exit(1)
PY
}

for arm in off on; do
  [ "$arm" = "on" ] && export PLASMC_YAW_OMEGA_D_FF=1 || unset PLASMC_YAW_OMEGA_D_FF
  saved=0; launch=0
  while [ "$saved" -lt "$N_REPS" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    ko; gate
    echo "########## ARM=$arm launch=$launch (saved=$saved/$N_REPS)  OMEGA_D_FF=${PLASMC_YAW_OMEGA_D_FF:-0}  $(date +%H:%M:%S) ##########"
    before=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
    timeout 400 env INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 \
        bash scripts/run_landing_retry.sh > "$OUT/${arm}_launch${launch}.log" 2>&1
    latest=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
    if [ -z "$latest" ] || [ "$latest" = "$before" ]; then
      echo "  NO REP (SITL flake)"; continue
    fi
    cand="$OUT/${arm}_cand${launch}"; cp -r "$latest" "$cand"
    if [ "$arm" = "on" ]; then
      check_args=("PLASMC_GT_SPIN_WZ=0.48" "PLASMC_YAW_OMEGA_D_FF=1")
    else
      check_args=("PLASMC_GT_SPIN_WZ=0.48" "!PLASMC_YAW_OMEGA_D_FF")
    fi
    if reason=$(verify_attribution "$cand" "${check_args[@]}" 2>&1); then
      saved=$((saved+1)); mv "$cand" "$OUT/${arm}_rep${saved}"
      echo "  saved -> $OUT/${arm}_rep${saved}"
    else
      rm -rf "$cand"
      echo "  REJECTED: $reason"
    fi
  done
  [ "$saved" -lt "$N_REPS" ] && echo "  WARNING: only $saved/$N_REPS saved for arm=$arm (launch budget $MAXLAUNCH exhausted)"
done
ko
echo "########## Q8 OMEGA_D_FF PROBE COMPLETE $(date +%H:%M:%S) ##########"
