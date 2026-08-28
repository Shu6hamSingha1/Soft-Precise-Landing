#!/usr/bin/env bash
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/gtfb_x4.tsv"; printf "rep\toff0\toff1.9\tpeak\tclass\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4; do
  ko
  INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 PLASMC_GT_FEEDBACK=1 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/gtfb_$rep.log" 2>&1
  cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$SD/run_logs/gtfb_$rep.log"|tail -1|awk '{print $3}')
  L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
  read o0 o19 pk < <("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);tg=np.asarray(g["Time"]).ravel();n=min(len(U),len(T),len(tg));t0=tg[0]
 dx=np.array([U[i].position.x-T[i].position.x for i in range(n)]);dy=np.array([U[i].position.y-T[i].position.y for i in range(n)]);lat=np.hypot(dx,dy)
 i19=int(np.argmin(np.abs((tg[:n]-t0)-1.9)));print("%.2f %.2f %.2f"%(lat[0],lat[i19],lat.max()))
except Exception as e:print("NA NA NA")
PY
)
  printf "%s\t%s\t%s\t%s\t%s\n" "$rep" "$o0" "$o19" "$pk" "${cls:-NA}" >> "$RES"
done
ko; echo "GTFB_X4_DONE" >> "$RES"
