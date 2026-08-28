#!/usr/bin/env bash
# GT-FB @ IC2 (off-center start) x4: measure perception-vs-GT CENTROID error off-center.
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/scent_x4.tsv"; printf "rep\tpeak\tclass\tsx_perc\tsx_GT\tsx_err\tsx_corr\tsy_corr\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4; do
  ko
  INITIAL_DRONE_ENU="2.0,2.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 PLASMC_GT_FEEDBACK=1 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/scent_$rep.log" 2>&1
  cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$SD/run_logs/scent_$rep.log"|tail -1|awk '{print $3}')
  L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
  line=$("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
sys.path.insert(0,'/home/shubham/Soft-Precise-Landing/PX4_Gazebo/src')
from gt_feedback import GTFeedback
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();dd=np.load(os.path.join(d,"Img_Data.npy"),allow_pickle=True).item()
 U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);tg=np.asarray(g["Time"]).ravel()
 ps=np.asarray(dd["Feature Params"]);n=min(len(U),len(T),len(tg),len(ps))
 dx=np.array([U[i].position.x-T[i].position.x for i in range(n)]);dy=np.array([U[i].position.y-T[i].position.y for i in range(n)]);lat=np.hypot(dx,dy)
 gf=GTFeedback();gsx=np.full(n,np.nan);gsy=np.full(n,np.nan)
 for i in range(n):
  try:s4,_=gf.update(U[i],T[i],float(tg[i]));gsx[i]=s4[0];gsy[i]=s4[1]
  except:pass
 m=(lat[:n]>0.30)&np.isfinite(gsx)&np.isfinite(ps[:n,0])
 px,gx=ps[:n,0][m],gsx[m];py,gy=ps[:n,1][m],gsy[m]
 cx=np.corrcoef(gx,px)[0,1] if (m.sum()>8 and np.std(px)>1e-6) else 0
 cy=np.corrcoef(gy,py)[0,1] if (m.sum()>8 and np.std(py)>1e-6) else 0
 print("%.2f\t%.3f\t%.3f\t%+.3f\t%.2f\t%.2f"%(lat.max(),np.mean(px),np.mean(gx),np.mean(px-gx),cx,cy))
except Exception as e:print("NA\tNA\tNA\tNA\tNA\tNA")
PY
)
  printf "%s\t%s\t%s\t%s\n" "$rep" "$(echo "$line"|cut -f1)" "${cls:-NA}" "$(echo "$line"|cut -f2-)" >> "$RES"
done
ko; echo "SCENT_X4_DONE" >> "$RES"
