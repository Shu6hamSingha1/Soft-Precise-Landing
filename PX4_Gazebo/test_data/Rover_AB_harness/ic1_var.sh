#!/usr/bin/env bash
# IC1 variance quantification: 4 reps, KF observer (baked), incremental results.
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
RES="$SD/run_logs/ic1_variance.tsv"
printf "rep\txy_err\trel_vel\tclass\n" > "$RES"
killorphans(){ for r in 1 2 3; do
  pids=$(ps -eo pid,args | grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|record_output|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test' | grep -av grep | awk '{print $1}')
  [ -z "$pids" ] && break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done; rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null; }
for rep in 1 2 3 4; do
  killorphans
  INITIAL_DRONE_ENU="0.0,0.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 \
    timeout 230 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/ic1var_$rep.log" 2>&1
  xy=$(grep -aoE 'xy_err=[0-9.]+' "$SD/run_logs/ic1var_$rep.log" | tail -1 | cut -d= -f2)
  rv=$(grep -aoE 'rel_vel=[0-9.]+' "$SD/run_logs/ic1var_$rep.log" | tail -1 | cut -d= -f2)
  cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$SD/run_logs/ic1var_$rep.log" | tail -1 | awk '{print $3}')
  printf "%s\t%s\t%s\t%s\n" "$rep" "${xy:-NA}" "${rv:-NA}" "${cls:-NA}" >> "$RES"
done
killorphans
echo "IC1_VARIANCE_DONE" >> "$RES"
