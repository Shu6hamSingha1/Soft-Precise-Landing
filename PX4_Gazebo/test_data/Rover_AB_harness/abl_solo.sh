#!/usr/bin/env bash
# REVERSE ablation: perception for ONE channel, GT for the REST — isolate each perception
# signal's individual damage against the clean 0.042m GT baseline. FAIL => that channel is bad.
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/abl_solo.tsv"
printf "perception_channel\txy_err\trel_vel\tclass\tpeak_lat\n" > "$RES"
killorphans(){ for r in 1 2 3; do
  pids=$(ps -eo pid,args | grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test' | grep -av grep | awk '{print $1}')
  [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
# GT_ABLATE list = all channels EXCEPT the one we keep from perception
declare -A GTREST=([s]="h,hz,yaw,wz" [h]="s,hz,yaw,wz" [yaw]="s,h,hz,wz" [hz]="s,h,yaw,wz" [wz]="s,h,hz,yaw")
for ch in wz yaw h hz s; do
  killorphans
  INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 \
    PLASMC_GT_FEEDBACK=1 GT_ABLATE="${GTREST[$ch]}" FLOW_DH_MAX=0 FLOW_DS_MAX=0 HEADLESS=1 \
    timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/abl_solo_$ch.log" 2>&1
  xy=$(grep -aoE 'xy_err=[0-9.]+' "$SD/run_logs/abl_solo_$ch.log" | tail -1 | cut -d= -f2)
  rv=$(grep -aoE 'rel_vel=[0-9.]+' "$SD/run_logs/abl_solo_$ch.log" | tail -1 | cut -d= -f2)
  cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$SD/run_logs/abl_solo_$ch.log" | tail -1 | awk '{print $3}')
  latest=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null | head -1)
  peak=$("$HOME/ws/scripts/env2025/bin/python3" - "$latest" << 'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
    gt=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item()
    U=np.array(gt["UAV Pose"],dtype=object);T=np.array(gt["Target Pose"],dtype=object)
    n=min(len(U),len(T));print("%.2f"%max(np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)))
except Exception: print("NA")
PY
)
  printf "perc-%s-only\t%s\t%s\t%s\t%s\n" "$ch" "${xy:-NA}" "${rv:-NA}" "${cls:-NA}" "$peak" >> "$RES"
done
killorphans
echo "ABL_SOLO_DONE" >> "$RES"
