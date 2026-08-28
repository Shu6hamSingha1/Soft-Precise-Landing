#!/usr/bin/env bash
# V_ds KF q A/B: q=10 (original low-lag) x5 IC2 perception. Compare to kfq (q=1 default).
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/vdsq.tsv"; printf "rep\txy_err\tclass\tpeak\ttermosc\trec\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4 5; do
  ko
  PLASMC_VDS_KF_Q=10 INITIAL_DRONE_ENU="2.0,2.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/vdsq_$rep.log" 2>&1
  xy=$(grep -aoE 'xy_err=[0-9.]+' "$SD/run_logs/vdsq_$rep.log"|tail -1|cut -d= -f2)
  cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$SD/run_logs/vdsq_$rep.log"|tail -1|awk '{print $3}')
  L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
  read pk to < <("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);n=min(len(U),len(T))
 alt=np.array([U[i].position.z-T[i].position.z for i in range(n)]);lat=np.array([np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)])
 term=(alt>0.05)&(alt<0.6)   # terminal-osc proxy: std of lateral in the last 0.6m
 to=np.std(lat[term]) if term.sum()>5 else 0.0
 print("%.2f %.3f"%(lat.max(),to))
except:print("NA NA")
PY
)
  printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$rep" "${xy:-NA}" "${cls:-NA}" "$pk" "$to" "$(basename "$L")" >> "$RES"
done
ko; echo "VDSQ_DONE" >> "$RES"
