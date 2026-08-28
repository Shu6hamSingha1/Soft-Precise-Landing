#!/usr/bin/env bash
# repro rep3-class flyaway: same config as lr4 (paired + arm-mask + moment), debug on, x4 tries.
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/lr4dbg.tsv"; printf "rep\ttag\tpeak\tfires\n" > "$RES"
ko(){ for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4; do
  ko
  PLASMC_LOOM_RING_ON_LOSS=1 PLASMC_RING_ARM_MASK=1 PLASMC_RING_PAIRED=1 RING_LOOM_SOURCE=moment PLASMC_DESCENT_GATE=1 LOOM_RING_DBG=1 INITIAL_DRONE_ENU="2.0,2.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/lr4dbg_$rep.log" 2>&1
  tag=$(grep -aoE 'classification: [A-Z_+-]+' "$SD/run_logs/lr4dbg_$rep.log"|tail -1|awk '{print $2}')
  nf=$(grep -ac '\[lr\]' "$SD/run_logs/lr4dbg_$rep.log")
  L=$(ls -td "$SD/test_data/Landing_Test/"*/|head -1)
  pk=$("$HOME/ws/scripts/env2025/bin/python3" - "$L" <<'PY'
import sys,numpy as np,os
d=sys.argv[1]
try:
 g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item();U=np.array(g["UAV Pose"],dtype=object);T=np.array(g["Target Pose"],dtype=object);n=min(len(U),len(T));print("%.2f"%max(np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)))
except:print("NA")
PY
)
  printf "%s\t%s\t%s\t%s\n" "$rep" "${tag:-?}" "$pk" "$nf" >> "$RES"
  if [ "$(echo "$pk > 3.5" | bc -l 2>/dev/null || echo 0)" = "1" ]; then echo "FLYAWAY_CAUGHT rep$rep" >> "$RES"; break; fi
done
ko; echo "LR4DBG_DONE" >> "$RES"
