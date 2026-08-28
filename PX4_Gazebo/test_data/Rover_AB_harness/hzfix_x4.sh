#!/usr/bin/env bash
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/hzfix_x4.tsv"; printf "rep\txy_err\tclass\tpeak_lat\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4; do
  ko
  INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 PLASMC_GT_FEEDBACK=1 GT_ABLATE=s,h,yaw,wz FLOW_DH_MAX=0 FLOW_DS_MAX=0 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/hzfix_$rep.log" 2>&1
  xy=$(grep -aoE 'xy_err=[0-9.]+' "$SD/run_logs/hzfix_$rep.log"|tail -1|cut -d= -f2)
  cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$SD/run_logs/hzfix_$rep.log"|tail -1|awk '{print $3}')
  L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
  pk=$("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);n=min(len(U),len(T));print("%.2f"%max(np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)))
except:print("NA")
PY
)
  printf "%s\t%s\t%s\t%s\n" "$rep" "${xy:-NA}" "${cls:-NA}" "$pk" >> "$RES"
done
ko; echo "HZFIX_X4_DONE" >> "$RES"
