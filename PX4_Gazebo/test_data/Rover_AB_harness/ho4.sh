#!/usr/bin/env bash
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
for r in 1 2 3; do pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}'); [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2; done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null
PLASMC_TERMINAL_RING_COMMIT=1 PLASMC_DESCENT_GATE=1 HANDOVER_DBG=1 IMG_RECORD=1 INITIAL_DRONE_ENU="2.0,2.0,5.0" LANDING_AUTOSAVE=1 MAX_ATTEMPTS=5 HEADLESS=1 timeout 235 bash scripts/run_aruco_landing_retry.sh > "$SD/run_logs/ho4.log" 2>&1
echo "HO4_DONE $(ls -td $SD/test_data/Test_Videos/*.mp4 2>/dev/null | head -1)" >> "$SD/run_logs/ho4.tsv"
