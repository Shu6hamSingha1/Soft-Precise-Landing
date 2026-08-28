#!/usr/bin/env bash
# FLOW CLAMP investigation: full perception (no GT-FB), observer+DESCENT_GATE baseline.
# Run A (clamp_on): clamp ACTIVE, log raw pre-clamp values whenever borderline.
# Run B (clamp_off): clamp BYPASSED (diagnostic only).
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/clampdbg.tsv"; printf "run\trep\ttag\tclassif_relvel\tminalt_xy\tmin_alt\tpeak\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
run_batch(){
  local name="$1"; shift
  for rep in 1 2 3 4 5; do
    ko
    env "$@" PLASMC_DESCENT_GATE=1 FLOW_CLAMP_DBG=1 INITIAL_DRONE_ENU="2.0,2.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/clampdbg_${name}_$rep.log" 2>&1
    tag=$(grep -aoE 'classification: [A-Z_+-]+' "$SD/run_logs/clampdbg_${name}_$rep.log"|tail -1|awk '{print $2}')
    crv=$(grep -aoE 'rel_vel=[0-9.]+ m/s \[' "$SD/run_logs/clampdbg_${name}_$rep.log"|tail -1|grep -oE '[0-9.]+'|head -1)
    hon=$(grep -a 'Honest precision @ min-alt' "$SD/run_logs/clampdbg_${name}_$rep.log"|tail -1)
    mxy=$(echo "$hon"|grep -oE 'xy=[0-9.-]+'|head -1|cut -d= -f2); malt=$(echo "$hon"|grep -oE 'min_alt=[0-9.-]+'|head -1|cut -d= -f2)
    L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
    pk=$("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);n=min(len(U),len(T));print("%.2f"%max(np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)))
except:print("NA")
PY
)
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$name" "$rep" "${tag:-?}" "${crv:-?}" "${mxy:-?}" "${malt:-?}" "$pk" >> "$RES"
    if [ "$(echo "$pk > 3.5" | bc -l 2>/dev/null || echo 0)" = "1" ]; then echo "FLYAWAY_${name}_rep${rep}" >> "$RES"; fi
  done
}
run_batch "clamp_on"
run_batch "clamp_off" FLOW_CLAMP_OFF=1
ko; echo "CLAMPDBG_DONE" >> "$RES"
