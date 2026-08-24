#!/usr/bin/env bash
# PLASMC_GT_Z_REG sweep under PLASMC_GT_FEEDBACK=1, IC2, ArUco (2026-08-20).
#
# HYPOTHESIS (history bisection, this session): the GT-FB depth floor is
# DOUBLE-COUNTED since 7830ef6 (2026-07-02), and that is the regression that
# cost the 06-30 campaign's IC1-5 SP (19/25, project_why_sp_achieved -- which
# names Z_REG as THE #1 enabler).
#
#   At 486f713 (the SP-achieving snapshot): W_x_tu = target_origin - uav_base,
#   and Z_REG=0.2 was chosen EXPLICITLY as a fudge standing in for the
#   un-applied camera mount offset -- its own comment said so: "base_link min-z
#   is ~0.1 m, but the camera is mounted +0.20 m above base_link ... so 0.2 is
#   the conservative depth floor (between the base_link 0.1 and the camera 0.3)."
#
#   7830ef6 then applied the mount offsets EXPLICITLY (W_x_tu = marker - camera,
#   _CAM_OFF_FLU=+0.15 m), making that fudge redundant -- but left Z_REG at 0.2.
#   The file's own note admits it: "Z_REG is a pure depth floor ... no longer a
#   fudge that also absorbs the mount offset. Measured min first-descent
#   camera-marker depth ~0.10 m, so Z_REG may warrant revisiting (0.2 -> ~0.1);
#   left at 0.2 PENDING A SWEEP." That sweep was never run. This is it.
#
# Measured effect at the observed 0.486 m stall plateau: effective depth is
# 0.486+0.15+0.20=0.836 m now vs 0.486+0.00+0.20=0.686 m in the SP era -> HEAD
# under-reads the loom 1/Z by 1.22x exactly where the descent stalls. An
# under-read loom = the controller under-perceives its own descent -> under-
# brakes/under-drives the z axis, which is the kappa_z leakage fixed point
# (kappa_z pinned 0.2516 cross / 0.119 aruco) this session traced.
#
# Arms: 0.2 (current default/baseline), 0.1 (the file's own suggested value,
# matching the measured ~0.10 m true camera-marker floor), 0.05 (probe past it
# -- if the trend continues, the floor is the binding term, not a coincidence).
#
# READ BY SoftPrecise, NOT stall-rate: this session established that escaping
# the stall via a kappa/a_u detonation (a_u ~300-440, 3-7 m/s impact) still
# counts as "not stalled" while being a CRASH. Every rep must be scored on
# precise/soft from Ground_Truth.npy, never on whether the watchdog fired.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$SCRIPT_DIR/.." && pwd)"
N="${N:-5}"
MAXLAUNCH="${MAXLAUNCH:-12}"
IC="2.0,2.0,5.0"   # IC2

run_arm () {
  local name="$1"; shift
  local base="$PROJ/test_data/ZregGTFB_IC2_${name}"
  mkdir -p "$base"
  local valid=0 launch=0
  while [ "$valid" -lt "$N" ] && [ "$launch" -lt "$MAXLAUNCH" ]; do
    launch=$((launch+1))
    local before; before=$(ls "$base" 2>/dev/null | wc -l)
    echo "=== [$name] launch $launch (valid $valid/$N) ==="
    env HEADLESS=1 INITIAL_DRONE_ENU="$IC" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
        PLASMC_GT_FEEDBACK=1 LANDING_OUT_BASE="$base" "$@" \
      taskset -c 6-15 bash "$SCRIPT_DIR/run_aruco_landing_retry.sh" || true
    for d in "$base"/*/; do [ -d "$d" ] && [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d"; done
    local after; after=$(ls "$base" 2>/dev/null | wc -l)
    if [ "$after" -gt "$before" ]; then valid=$((valid+1)); echo "[$name] valid $valid/$N"; fi
  done
  echo "=== [$name] DONE: $valid valid in $launch launches ==="
}

run_arm z020  env PLASMC_GT_Z_REG=0.2
run_arm z010  env PLASMC_GT_Z_REG=0.1
run_arm z005  env PLASMC_GT_Z_REG=0.05
echo "ALL ARMS DONE"
