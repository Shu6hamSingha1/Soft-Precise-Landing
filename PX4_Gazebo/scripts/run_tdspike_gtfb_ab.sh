#!/usr/bin/env bash
# PLASMC_TD_SPIKE sweep under PLASMC_GT_FEEDBACK=1, IC2, ArUco (2026-08-23).
#
# THE CONFIRMED REGRESSION. Full history bisection 486f713(19/25 SP) -> HEAD,
# after VDS_KF_Q / TERMINAL_COMMIT / Z_REG / CBF_CORNERS_STALE-freshness-gate
# were each tested and falsified (0 SP; the descent stall was invariant to all
# of them and to marker type). A wholesale 486f713 controller.py+cbf_visibility.py
# +img_data.py+gt_feedback.py revert (compat-shimmed for 2 API additions the
# current apps/landing_test.py calls) was then run n=5 and CONFIRMED, with
# tight reproducibility (min_alt 0.4868-0.4871m, std 0.0001m across all 5):
# the SAME physical/perceptual event (a real loom-inversion near 0.487m,
# apparently a near-field marker-overflow artifact -- ground truth confirms
# genuine ground contact is ~0.01m, matching 1340 archived historical PRECISE
# landings' median min_alt of -0.0088m) occurs in BOTH eras. The touchdown
# latch is what differs:
#
#   486f713 _touchdownDetect: latches on h_z > 0.0 (ANY positive/inverted loom)
#     for _td_frames(3) frames -> fires immediately on the weak inversion,
#     disarms, scores SOFT+PRECISE while the drone is still ~0.487m airborne.
#   HEAD _touchdownDetect (PLASMC_TD_SPIKE, controller.py): requires a genuine
#     SPIKE, h_z > 0.5, explicitly NOT merely h_z>0 per its own docstring.
#     Its own init comment: "PROVISIONAL, n=2 evidence -- validate at n>=5".
#     That validation was never done. h(t) at the stall sits at ~[0,-0.001,0.0002]
#     -- nowhere near 0.5 -- so the latch never fires; the descent loop keeps
#     running an ever-decaying command until Gazebo's ODE physics engine
#     auto-disables the near-stationary rigid body (confirmed via an
#     independent-process /pose probe: messages keep arriving at 50Hz, n_poses
#     stable, content bit-frozen for 17.5s while /clock keeps advancing) and
#     the 25s descent-stall watchdog aborts.
#
# So the HISTORICAL "SP" wasn't a deeper/better landing -- it accepted the
# SAME shallow event as a landing. The open question THIS sweep answers: what
# TD_SPIKE value is actually correct now? 0.0 restores the exact 486f713
# behavior (known to reproducibly disarm ~10cm+ above true ground -- see
# min_alt 0.487m vs the true ~0.01m rest height); the current 0.5 never fires.
# Sweep the range to find where the real vertical-reversal-at-contact signal
# actually lives, not just to restore the old (also-imperfect) value blindly.
#
# Scored on SoftPrecise's min_alt/precise/soft, NOT stall-rate (a "completion"
# via a kappa/a_u detonation crash is not a landing -- see project session
# notes on the false-completion pattern) -- inspect min_alt against the true
# ~0.01m ground-contact reference before trusting a `precise=True` at face value.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-12}"
IC="2.0,2.0,5.0"   # IC2

run_arm () {
  local name="$1" spike="$2"
  local base="$PROJ/test_data/TdSpikeGTFB_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        PLASMC_GT_FEEDBACK=1 PLASMC_TD_SPIKE="$spike" LANDING_OUT_BASE="$base" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm td000 0.0    # exact 486f713 behavior
run_arm td010 0.1
run_arm td030 0.3
run_arm td050 0.5    # current default (never fires per this session's evidence)
echo "ALL ARMS DONE"
