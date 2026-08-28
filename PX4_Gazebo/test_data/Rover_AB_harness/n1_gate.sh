#!/usr/bin/env bash
# n=1 confirmation gate (IC1-5), observer baked on. Writes results incrementally so a
# dropped orchestrator still leaves per-IC results + logs. Kills orphans between ICs.
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/n1gate_results.tsv"
printf "ic\tenu\txy_err\trel_vel\tclass\n" > "$RES"
declare -A IC=([IC1]="0.0,0.0,5.0" [IC2]="2.0,2.0,5.0" [IC3]="-2.0,2.0,5.0" [IC4]="2.0,2.0,7.0" [IC5]="2.0,2.0,3.0")
killorphans(){ for r in 1 2 3; do
  pids=$(ps -eo pid,args | grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test' | grep -av grep | awk '{print $1}')
  [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for ic in IC1 IC2 IC3 IC4 IC5; do
  killorphans
  INITIAL_DRONE_ENU="${IC[$ic]}" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 \
    timeout 230 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/n1_$ic.log" 2>&1
  xy=$(grep -aoE 'xy_err=[0-9.]+' "$SD/run_logs/n1_$ic.log" | tail -1 | cut -d= -f2)
  rv=$(grep -aoE 'rel_vel=[0-9.]+' "$SD/run_logs/n1_$ic.log" | tail -1 | cut -d= -f2)
  cls=$(grep -aoE 'Landing classification: [A-Z_]+' "$SD/run_logs/n1_$ic.log" | tail -1 | awk '{print $3}')
  printf "%s\t%s\t%s\t%s\t%s\n" "$ic" "${IC[$ic]}" "${xy:-NA}" "${rv:-NA}" "${cls:-NA}" >> "$RES"
done
killorphans
echo "N1GATE_DONE" >> "$RES"
