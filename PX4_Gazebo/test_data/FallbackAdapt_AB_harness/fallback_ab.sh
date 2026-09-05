#!/usr/bin/env bash
# SITL flight test for CROSS_FALLBACK_ADAPT_GATE=1 (landed 1c0fdeb6, DEFAULT OFF).
# Innovation-gated adapt fallback: on a primary detection miss, retry with the
# CLAHE+adaptiveThreshold gate and accept the result only if it agrees with a
# held-last-good prediction (Kalman innovation test). Verified offline via the
# real process_frame() end-to-end (col: detOK 79.0%->82.2%, rover_IC2/IC4 mildly
# positive) -- this is the first LIVE flight test.
#
# WORLD is a parameter -- must-not-regress on the ORDINARY scene matters more
# here than for the confirm stages (ring/balance/ensemble), since this fix is a
# real candidate for eventual default-on, not just an opt-in experiment:
#   WORLD=cross_marker  IC=2.0,2.0,5.0 N=5 bash test_data/FallbackAdapt_AB_harness/fallback_ab.sh
#   WORLD=cm_col        IC=2.0,2.0,5.0 N=5 bash test_data/FallbackAdapt_AB_harness/fallback_ab.sh
#
# Arms INTERLEAVED per rep. Metrics set copied from CmInv_AB_harness/cminv_ab.sh
# (max_lat included -- revealed the stuck-vs-lost distinction there).
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
SCENE="${WORLD:-cross_marker}"
IC="${IC:-2.0,2.0,5.0}"; N="${N:-5}"; TAG="${TAG:-$SCENE}"
LAUNCH="scripts/run_landing_retry.sh"; SURF=0.0
WORLD_ENV="WORLD=$SCENE MARKER_TYPE=cross"     # HARD RULE: never bare ArUco
# ⛔⛔ PER-REP FOREIGN-SESSION CHECK -- see CmInv_AB_harness/cminv_ab.sh for the
# full incident/rationale. Checked ONCE before the loop is not enough; ko()
# re-verifies after every cleanup and aborts (rather than re-killing) if SITL
# survives it -- that can only mean something else is live.
if [ "${FORCE:-0}" != "1" ]; then
  _busy=$(ps -eo args | grep -aE 'px4_sitl_default/bin/px4|gz-sim|MicroXRCEAgent|landing_test\.py' | grep -av grep | head -3)
  if [ -n "$_busy" ]; then
    echo "REFUSING TO START: SITL is already running (another session?):" >&2
    echo "$_busy" | cut -c1-100 >&2
    echo "Wait for it to finish, or re-run with FORCE=1 if you are certain it is yours." >&2
    exit 3
  fi
fi
RES="$SD/run_logs/fallback_${TAG}.tsv"
printf "arm\trep\txy_err\tclass\tmin_h\tdetOK\tmax_lat\trec\n" > "$RES"
SITL_PAT='px4_sitl_default/bin/px4|gz sim|gz-sim|ign gazebo|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'
foreign_check(){
  local ctx="$1"
  local p=$(ps -eo args | grep -aE "$SITL_PAT" | grep -av grep | head -3)
  if [ -n "$p" ]; then
    echo "REFUSING TO CONTINUE ($ctx): SITL processes survived our own cleanup --" >&2
    echo "this can only mean something else is running them, not us:" >&2
    echo "$p" | cut -c1-100 >&2
    echo "Stopping rather than risk killing a live session. No further ko()/launch will run." >&2
    exit 4
  fi
}
ko(){
  for r in 1 2 3; do
    pids=$(ps -eo pid,args|grep -aE "$SITL_PAT"|grep -av grep|awk '{print $1}')
    [ -z "$pids" ]&&break; for p in $pids; do kill -9 "$p" 2>/dev/null; done; sleep 2
  done
  for pat in gz-sim-server gz-sim-gui; do
    gp=$(ps -eo pid,args|grep -a "$pat"|grep -av grep|awk '{print $1}'); for p in $gp; do kill -9 "$p" 2>/dev/null; done
  done
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.* 2>/dev/null
  for w in 1 2 3 4 5 6 7 8; do gz topic -l 2>/dev/null | grep -q '/clock' || break; sleep 2; done
  for w in 1 2 3 4 5; do
    hp=$(ss -ulpn 2>/dev/null | awk '/:8888 /{print}')
    [ -z "$hp" ] && break
    fuser -k 8888/udp 2>/dev/null; sleep 2
  done
  sleep 3
  foreign_check "post-cleanup, before next launch"
}
for rep in $(seq 1 "$N"); do
  for arm in off on; do
    FB=0; [ "$arm" = on ] && FB=1
    ko
    L0=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    env LD_PRELOAD="${QT_PRELOAD:-/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5}" \
        HEADLESS=1 $WORLD_ENV LANDING_AUTOSAVE=1 MAX_ATTEMPTS=4 \
        INITIAL_DRONE_ENU="$IC" CROSS_FALLBACK_ADAPT_GATE="$FB" \
        timeout 260 bash "$SD/$LAUNCH" \
        > "$SD/run_logs/fallback_${TAG}_${arm}_${rep}.out" 2>&1
    LG="$SD/run_logs/fallback_${TAG}_${arm}_${rep}.out"
    xy=$(grep -aoE 'xy_err=[0-9.]+' "$LG"|tail -1|cut -d= -f2)
    cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$LG"|tail -1|awk '{print $3}')
    L=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    if [ "$L" = "$L0" ]; then rec="NO_NEW_REC"; mh="NA"; dk="NA"; ml="NA"; else
      rec="$(basename "$L")"
      read -r mh dk ml <<<"$("$HOME/ws/scripts/env2025/bin/python3" - "$L" "$SURF" <<'PY'
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
    sel=(it>=tt[0])&(it<=tt[-1]); alt=np.interp(it[sel],tt,h); band=alt>=1.0
    dk=100.0*np.mean(ds[sel][band]=='ok') if band.sum()>3 else float('nan')
    ml=float(lat.max())
    print("%.3f %.1f %.2f"%(h.min(),dk,ml))
except Exception:
    print("NA NA NA")
PY
)"
    fi
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$arm" "$rep" "${xy:-NA}" "${cls:-NA}" "$mh" "$dk" "$ml" "$rec" >> "$RES"
    echo "[fallback:$SCENE] arm=$arm rep=$rep xy=${xy:-NA} cls=${cls:-NA} min_h=$mh detOK=$dk max_lat=$ml"
  done
done
ko; echo "FALLBACK_DONE" >> "$RES"
