#!/usr/bin/env bash
# Ring-commit A/B v2 (merged overflow-signature handover gate). IC2 perception x5, GT-scored.
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/rc2.tsv"; printf "rep\tclassif_xy\tminalt_xy\tmin_alt\tpeak\tHANDOVER\tRINGCOMMIT\trec\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4 5; do
  ko
  PLASMC_TERMINAL_RING_COMMIT=1 PLASMC_DESCENT_GATE=1 HANDOVER_DBG=1 INITIAL_DRONE_ENU="2.0,2.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/rc2_$rep.log" 2>&1
  cxy=$(grep -aoE 'xy_err=[0-9.]+' "$SD/run_logs/rc2_$rep.log"|tail -1|cut -d= -f2)
  mxy=$(grep -aoE 'Honest precision @ min-alt: xy=[0-9.-]+' "$SD/run_logs/rc2_$rep.log"|tail -1|grep -oE '[0-9.-]+$')
  malt=$(grep -aoE 'min_alt=[0-9.-]+' "$SD/run_logs/rc2_$rep.log"|tail -1|cut -d= -f2)
  ho=$(grep -ac '\[HANDOVER\] latched' "$SD/run_logs/rc2_$rep.log")
  rc=$(grep -ac 'RING-COMMIT:' "$SD/run_logs/rc2_$rep.log")
  L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
  pk=$("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);n=min(len(U),len(T));print("%.2f"%max(np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)))
except:print("NA")
PY
)
  printf "%s\t%s\t%s\t%s\t%s\t%sho\t%src\t%s\n" "$rep" "${cxy:-NA}" "${mxy:-NA}" "${malt:-NA}" "$pk" "$ho" "$rc" "$(basename "$L")" >> "$RES"
done
ko; echo "RC2_DONE" >> "$RES"
