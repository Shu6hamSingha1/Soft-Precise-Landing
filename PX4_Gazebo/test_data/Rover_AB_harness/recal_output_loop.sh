#!/usr/bin/env bash
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
stray_clean() {
  for round in 1 2 3 4; do
    local pids; pids=$(pgrep -f 'build/px4_sitl_default/bin/px4'; pgrep -f 'gz sim'; pgrep -x MicroXRCEAgent; pgrep -f parameter_bridge; pgrep -f mavsdk_server; pgrep -f 'ros2 run ros_gz')
    [ -z "$pids" ] && break
    for pid in $pids; do pgid=$(ps -o pgid= -p "$pid" 2>/dev/null|tr -d ' '); [ -n "$pgid" ] && kill -9 -"$pgid" 2>/dev/null; kill -9 "$pid" 2>/dev/null; done
    sleep 2
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* 2>/dev/null   # clear leaked DDS shm
  sleep 3
  # LOAD GATE: lockstep is timing-sensitive -> wait for a calm machine (1-min load < 5)
  for w in $(seq 1 24); do l=$(cut -d. -f1 /proc/loadavg); [ "$l" -lt 5 ] && break; sleep 5; done
}
for i in $(seq 1 30); do
  stray_clean
  HEADLESS=1 FLOW_LAT_REDUCED=1 PLASMC_CENTROID_RATE=1 timeout 240 bash scripts/run_output_calibration.sh > "$SD/run_logs/recal_$i.log" 2>&1
  for d in calibration_data/output/*/; do [ -z "$(ls "$d" 2>/dev/null)" ] && rmdir "$d" 2>/dev/null; done
  n=$(ls calibration_data/output 2>/dev/null | wc -l)
  why=""; grep -q "is_armable did not go True" "$SD/run_logs/recal_$i.log" 2>/dev/null && why=" (lockstep)"; grep -q "Unable to get simulation time" "$SD/run_logs/recal_$i.log" 2>/dev/null && why="$why (sim-clock)"
  echo "[recal] attempt $i -> $n valid$why | load $(cut -d' ' -f1 /proc/loadavg) shm $(ls /dev/shm|grep -c fastrtps)"
  [ "$n" -ge 5 ] && break
done
stray_clean; echo "RECAL_RECORDING_DONE ($(ls calibration_data/output 2>/dev/null | wc -l) valid runs)"
