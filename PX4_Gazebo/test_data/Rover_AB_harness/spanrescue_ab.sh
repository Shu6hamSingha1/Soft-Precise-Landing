#!/usr/bin/env bash
# A/B gate for CROSS_CENTROID_SPAN_RESCUE (landed f49f567f, DEFAULT OFF) on
# rover_cross, PERCEPTION mode (no GT anywhere), STATIC rover, offset IC.
# Offline eval predicted detOK 28.5%->94.8% (rover_IC2, descent band). This asks
# whether that converts into landings, and whether the ~24% off-by->0.15 centroids
# it also admits hurt the controller (CROSS_S_JUMP_GATE is the outlier defence).
# Arms INTERLEAVED per rep so SITL temporal drift is shared, not confounded.
# Follows docs/SH_REFERENCE.md §10 + the local one-off shape (perc_base.sh).
#   IC=2.0,2.0,5.0 N=5 bash test_data/Rover_AB_harness/spanrescue_ab.sh
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
IC="${IC:-2.0,2.0,5.0}"; N="${N:-5}"; TAG="${TAG:-ic2}"
# WORLD_KIND=rover (default, two-instance rover_cross) | flat (single-instance
# cross_marker) -- the flat arm is the must-not-regress-clean control.
WORLD_KIND="${WORLD_KIND:-rover}"
if [ "$WORLD_KIND" = "flat" ]; then
  LAUNCH="scripts/run_aruco_landing_retry.sh"; SURF=0.0
  WORLD_ENV="WORLD=cross_marker MARKER_TYPE=cross"
elif [ "$WORLD_KIND" = "clutter" ]; then
  LAUNCH="scripts/run_aruco_landing_retry.sh"; SURF=0.0
  WORLD_ENV="WORLD=cross_marker_clutter MARKER_TYPE=cross"
else
  LAUNCH="scripts/run_rover_landing_retry.sh"; SURF=0.50
  WORLD_ENV="WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross ROVER_MOTION=0"
fi
RES="$SD/run_logs/spanrescue_${TAG}.tsv"
printf "arm\trep\txy_err\tclass\tmin_h_pad\tlat_end\tdetOK_desc\trescue\trec\n" > "$RES"
ko(){
  # Kill every gz-sim variant (the server is NOT matched by the 'gz sim' CLI pattern)
  # and WAIT until no /clock topic is advertised. ⛔ NOTE: this was written to fix an
  # apparent stale-server-on-world-switch bug (flat-after-flat 3/10 launch failures vs
  # flat-after-rover 7/9). That theory was WRONG -- this did not fix it, and a rover
  # probe afterwards failed identically, so the breakage was GLOBAL (SITL down from
  # ~21:41 on 2026-09-02) and the pattern was just ORDERING. Kept because it is
  # harmless and stricter, NOT because it fixes anything observed.
  for r in 1 2 3; do
    pids=$(ps -eo pid,args|grep -aE 'px4_sitl_default/bin/px4|gz sim|gz-sim|ign gazebo|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'|grep -av grep|awk '{print $1}')
    [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done
  for pat in gz-sim-server gz-sim-gui; do
    gp=$(ps -eo pid,args|grep -a "$pat"|grep -av grep|awk '{print $1}'); for p in $gp; do kill -9 "$p" 2>/dev/null; done
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null
  for w in 1 2 3 4 5 6 7 8; do gz topic -l 2>/dev/null | grep -q '/clock' || break; sleep 2; done
  sleep 3
}
for rep in $(seq 1 "$N"); do
  for arm in off on; do
    v=0; [ "$arm" = on ] && v=1
    ko
    L0=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    # Qt5 WORKAROUND (2026-09-03): system libqt5gui5 (esm3) does not export
    # QDoubleValidator::validate that libqt5quick5 (esm1) needs, so `gz sim` aborts and
    # PX4 can never bring up a world. Preload the PyQt5-bundled Qt5Gui, which does export
    # it. This is a WORKAROUND, not a repair -- the real fix is aligning the apt packages.
    env LD_PRELOAD="${QT_PRELOAD:-/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5}" \
        HEADLESS=1 $WORLD_ENV LANDING_AUTOSAVE=1 MAX_ATTEMPTS=4 \
        INITIAL_DRONE_ENU="$IC" CROSS_CENTROID_SPAN_RESCUE="$v" \
        timeout 260 bash "$SD/$LAUNCH" \
        > "$SD/run_logs/spanrescue_${TAG}_${arm}_${rep}.out" 2>&1
    LG="$SD/run_logs/spanrescue_${TAG}_${arm}_${rep}.out"
    xy=$(grep -aoE 'xy_err=[0-9.]+' "$LG"|tail -1|cut -d= -f2)
    cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$LG"|tail -1|awk '{print $3}')
    rs=$(grep -a 'SPAN RESCUE' "$LG"|tail -1|sed -E 's/.*failed ([0-9]+)x -> rescued ([0-9]+).*OVERFILL[^ ]* consulted ([0-9]+) rescued ([0-9]+).*/\1|\2|\3|\4/')
    [ -z "$rs" ] && rs="0|0|0|0"
    L=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    if [ "$L" = "$L0" ]; then rec="NO_NEW_REC"; mh="NA"; le="NA"; dk="NA"; else
      rec="$(basename "$L")"
      read -r mh le dk <<<"$("$HOME/ws/scripts/env2025/bin/python3" - "$L" "$SURF" <<'PY'
import sys,os,numpy as np,warnings
warnings.filterwarnings('ignore')
d=sys.argv[1]; SURF=float(sys.argv[2])
try:
    g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item()
    U=g["UAV Pose"];T=g["Target Pose"];n=min(len(U),len(T))
    h=np.array([U[i].position.z-T[i].position.z-SURF for i in range(n)])
    lat=np.array([np.hypot(U[i].position.x-T[i].position.x,U[i].position.y-T[i].position.y) for i in range(n)])
    tt=np.asarray(g["Time"],float)[:n]; St=g["Start Time"]
    im=np.load(os.path.join(d,"Img_Data.npy"),allow_pickle=True).item()
    it=np.asarray(im["Time"],float)-St; ds=np.asarray(im["Detection Status"],object)
    m=min(len(it),len(ds)); it,ds=it[:m],ds[:m]
    sel=(it>=tt[0])&(it<=tt[-1])
    alt=np.interp(it[sel],tt,h); band=alt>=1.0
    dk=100.0*np.mean(ds[sel][band]=='ok') if band.sum()>3 else float('nan')
    print("%.3f %.3f %.1f"%(h.min(),lat[-1],dk))
except Exception:
    print("NA NA NA")
PY
)"
    fi
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$arm" "$rep" "${xy:-NA}" "${cls:-NA}" "$mh" "$le" "$dk" "$rs" "$rec" >> "$RES"
    echo "[spanrescue] $TAG arm=$arm rep=$rep xy=${xy:-NA} cls=${cls:-NA} min_h=$mh detOK=$dk rescue(consulted|rescued|ovf_cons|ovf_resc)=$rs"
  done
done
ko; echo "SPANRESCUE_DONE" >> "$RES"
