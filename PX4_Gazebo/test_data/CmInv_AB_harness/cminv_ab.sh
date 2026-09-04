#!/usr/bin/env bash
# SITL flight test: does ens_ring (CROSS_GATE_MODE=ensemble CROSS_RING_CONFIRM=1)
# behave more SAFELY than baseline on cm_inv (polarity-flipped marker) in
# PERCEPTION mode (detector actually gates the flight -- unlike RobustnessFrameset,
# which was recorded under PLASMC_GT_FEEDBACK=1 and never let the detector fly).
#
# Offline: baseline is confidently WRONG on inv 99.1% of the time (locked onto a
# plate corner, ~110px off) with 0% usable. ens_ring converts detOK 99.1->42.2%,
# i.e. it should REFUSE most of those instead of confidently reporting them.
# QUESTION THIS ASKS: does baseline fly to a wrong position / misbehave on bad
# detections, and does ens_ring fail more SAFELY (TARGET_LOST / abort / hover)
# instead? This is a capability test, not a regression gate.
#
# Flight envelope: apps/landing_test.py FENCE = +/-5m xy, +/-5m z (hard abort),
# MARKER_LOSS_GRACE backstop. Standard bounded SITL flight, same envelope as
# every other landing test in this project.
#
# Arms INTERLEAVED per rep. Shape copied from EnsRing_AB_harness/ensring_ab.sh.
#   IC=2.0,2.0,5.0 N=5 bash test_data/CmInv_AB_harness/cminv_ab.sh
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
IC="${IC:-2.0,2.0,5.0}"; N="${N:-5}"; TAG="${TAG:-ic2}"
LAUNCH="scripts/run_landing_retry.sh"; SURF=0.0
WORLD_ENV="WORLD=cm_inv MARKER_TYPE=cross"    # polarity-flipped world; still cross marker type
# ⛔ CONCURRENT-SESSION GUARD -- see EnsRing_AB_harness/ensring_ab.sh for the full note.
if [ "${FORCE:-0}" != "1" ]; then
  _busy=$(ps -eo args | grep -aE 'px4_sitl_default/bin/px4|gz-sim|MicroXRCEAgent|landing_test\.py' | grep -av grep | head -3)
  if [ -n "$_busy" ]; then
    echo "REFUSING TO START: SITL is already running (another session?):" >&2
    echo "$_busy" | cut -c1-100 >&2
    echo "Wait for it to finish, or re-run with FORCE=1 if you are certain it is yours." >&2
    exit 3
  fi
fi
RES="$SD/run_logs/cminv_${TAG}.tsv"
printf "arm\trep\txy_err\tclass\tmin_h\tdetOK\tmax_lat\trec\n" > "$RES"
SITL_PAT='px4_sitl_default/bin/px4|gz sim|gz-sim|ign gazebo|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'
# ⛔⛔ PER-REP FOREIGN-SESSION CHECK (added 2026-09-04, after a user challenge on the
# 2026-09-03 cm_inv run's failure cause). The ORIGINAL guard at the top of this file
# only runs ONCE, before the loop starts -- ko() itself, called before EVERY rep, did
# an unconditional kill -9 with no re-check. If another session started SITL any time
# after the initial guard passed, ko() would have killed it SILENTLY, with nothing
# left behind to prove it happened either way (checked after the fact: inconclusive --
# PX4 log-boot counts on 2026-09-03 were consistent with this script's OWN retry
# logic, but could not rule out a second session, since a second session's default
# launch uses the same instance-0 ports mine does). Whichever it was, the DESIGN was
# unsafe regardless, and this fix stands on its own regardless of what actually
# happened that day.
#
# PRINCIPLE: ko()'s job is to make the process table clean BY KILLING WHAT IT CAN
# SEE. If, after its own best-effort kill, SITL processes are STILL present, that is
# NOT something ko() itself spawned (it just killed everything it could see) -- it
# means something else is populating the table IN REAL TIME. That is exactly the
# foreign-live-session signal, and the correct response is to STOP, not kill again
# and launch on top of it.
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
  # PORT-8888 HARD BACKSTOP (2026-09-04): the /clock check above doesn't verify
  # MicroXRCEAgent's UDP port is actually released. fuser-kill anything still bound
  # and wait for it to clear.
  for w in 1 2 3 4 5; do
    hp=$(ss -ulpn 2>/dev/null | awk '/:8888 /{print}')
    [ -z "$hp" ] && break
    fuser -k 8888/udp 2>/dev/null; sleep 2
  done
  sleep 3
  # After our own best effort, anything still alive is NOT ours -- see foreign_check.
  foreign_check "post-cleanup, before next launch"
}
for rep in $(seq 1 "$N"); do
  for arm in off on; do
    GM="legacy"; RC=0; [ "$arm" = on ] && GM="ensemble" && RC=1
    ko
    L0=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    env LD_PRELOAD="${QT_PRELOAD:-/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5}" \
        HEADLESS=1 $WORLD_ENV LANDING_AUTOSAVE=1 MAX_ATTEMPTS=4 \
        INITIAL_DRONE_ENU="$IC" CROSS_GATE_MODE="$GM" CROSS_RING_CONFIRM="$RC" \
        timeout 260 bash "$SD/$LAUNCH" \
        > "$SD/run_logs/cminv_${TAG}_${arm}_${rep}.out" 2>&1
    LG="$SD/run_logs/cminv_${TAG}_${arm}_${rep}.out"
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
    # max lateral excursion -- the safety metric: did it fly toward the WRONG
    # point (confidently-wrong corner detection) or stay bounded/hover/abort?
    ml=float(lat.max())
    print("%.3f %.1f %.2f"%(h.min(),dk,ml))
except Exception:
    print("NA NA NA")
PY
)"
    fi
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$arm" "$rep" "${xy:-NA}" "${cls:-NA}" "$mh" "$dk" "$ml" "$rec" >> "$RES"
    echo "[cminv] $TAG arm=$arm rep=$rep xy=${xy:-NA} cls=${cls:-NA} min_h=$mh detOK=$dk max_lat=$ml"
  done
done
ko; echo "CMINV_DONE" >> "$RES"
