#!/usr/bin/env bash
# Validate PLASMC_YAW_RATE_LAW (2026-09-04 new yaw-rate law) per its own design memory's
# methodology: (1) stationary regression -- must be a no-op on the working IC1-5 case;
# (2) ceiling case (0.48 rad/s, GT-exact w_z) vs today's OMEGA_D_FF result; (3) beyond the
# sin(dpsi) ceiling (0.60 rad/s) -- unreachable by any prior mechanism.
set -u
cd "$HOME/Soft-Precise-Landing/PX4_Gazebo"

ko(){ for r in 1 2 3; do
        pids=$(ps -eo pid,args | grep -aE 'run_ic_validation|run_landing|px4_sitl_default/bin/px4|gz sim|gz-sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|landing_test' | grep -av grep | awk '{print $1}')
        [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null || true; done; sleep 2
      done; sleep 3; }
gate(){ if ss -lun 2>/dev/null | grep -q ':8888'; then echo "ABORT: udp:8888 bound"; exit 1; fi; }
verify_attribution() {
  local dir="$1"; shift
  "$HOME/ws/scripts/env2025/bin/python3" - "$dir" "$@" <<'PY'
import sys, numpy as np
d = sys.argv[1]
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
    print(f"MISMATCH bad={bad} unexpectedly-present={present}", file=sys.stderr); sys.exit(1)
PY
}
run_rep() {
  # run_rep OUTDIR TAG N_REPS MAXLAUNCH ENU EXTRA_CHECK... (EXTRA_CHECK = verify_attribution args)
  local out="$1" tag="$2" nreps="$3" maxl="$4" enu="$5"; shift 5
  mkdir -p "$out"
  local saved=0 launch=0
  while [ "$saved" -lt "$nreps" ] && [ "$launch" -lt "$maxl" ]; do
    launch=$((launch+1)); ko; gate
    echo "########## $tag launch=$launch (saved=$saved/$nreps)  $(date +%H:%M:%S) ##########"
    before=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
    timeout 400 env INITIAL_DRONE_ENU="$enu" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=3 \
        bash scripts/run_landing_retry.sh > "$out/${tag}_launch${launch}.log" 2>&1
    latest=$(ls -td test_data/Landing_Test/*/ 2>/dev/null | head -1)
    if [ -z "$latest" ] || [ "$latest" = "$before" ]; then echo "  NO REP"; continue; fi
    cand="$out/${tag}_cand${launch}"; cp -r "$latest" "$cand"
    if reason=$(verify_attribution "$cand" "$@" 2>&1); then
      saved=$((saved+1)); mv "$cand" "$out/${tag}_rep${saved}"; echo "  saved -> $out/${tag}_rep${saved}"
    else
      rm -rf "$cand"; echo "  REJECTED: $reason"
    fi
  done
  [ "$saved" -lt "$nreps" ] && echo "  WARNING: only $saved/$nreps for $tag"
}

export HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross

# ── Phase 1: stationary regression (real perception, no GT-FB, no spin) ──
OUT1="test_data/YawRateLaw/stationary"
declare -A IC_ENU=([IC1]="0.0,0.0,5.0" [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0")
export PLASMC_YAW_RATE_LAW=1
for ic in IC1 IC2 IC3 IC4 IC5; do
  run_rep "$OUT1" "$ic" 1 3 "${IC_ENU[$ic]}" "PLASMC_YAW_RATE_LAW=1"
done
unset PLASMC_YAW_RATE_LAW

# ── Phase 2: ceiling case, GT-exact w_z, off vs new-law, n=3 each ──
OUT2="test_data/YawRateLaw/ceiling048"
export PLASMC_GT_FEEDBACK=1 PLASMC_GT_SPIN_WZ=0.48
for arm in off on; do
  [ "$arm" = "on" ] && export PLASMC_YAW_RATE_LAW=1 || unset PLASMC_YAW_RATE_LAW
  chk=("PLASMC_GT_SPIN_WZ=0.48"); [ "$arm" = "on" ] && chk+=("PLASMC_YAW_RATE_LAW=1") || chk+=("!PLASMC_YAW_RATE_LAW")
  run_rep "$OUT2" "$arm" 3 8 "0.0,0.0,5.0" "${chk[@]}"
done
unset PLASMC_YAW_RATE_LAW

# ── Phase 3: beyond the old ceiling (0.60 rad/s), new-law only, n=3 ──
OUT3="test_data/YawRateLaw/beyond060"
export PLASMC_GT_SPIN_WZ=0.60 PLASMC_YAW_RATE_LAW=1
run_rep "$OUT3" "on" 3 8 "0.0,0.0,5.0" "PLASMC_GT_SPIN_WZ=0.60" "PLASMC_YAW_RATE_LAW=1"
unset PLASMC_YAW_RATE_LAW PLASMC_GT_SPIN_WZ PLASMC_GT_FEEDBACK

ko
echo "########## YAW_RATE_LAW PROBE COMPLETE $(date +%H:%M:%S) ##########"
