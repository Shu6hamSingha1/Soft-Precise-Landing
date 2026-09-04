#!/usr/bin/env bash
# A/B gate for CROSS_LOOM_R_SCHEDULE (landed 06b28ebc, DEFAULT OFF) on flat
# cross_marker, PERCEPTION mode. The schedule widens the hw-KF's r[2] (loom row
# only) as MARKER_EXTENT_PX moves away from its measured ~68 px sweet spot.
# Offline (3093 frames/11 runs): 1.3-2.0 m band r +0.309 -> +0.405, std(err)
# -7.1%, neutral elsewhere. This asks whether that converts into landings, and
# -- the real risk -- whether trusting the terminal loom LESS makes the drone
# UNDER-BRAKE near ground ([[feedback_pinv_tol_loom_scaling]] measured exactly
# that for the decoupled loom: terminal vz 0.74 -> 1.06 m/s). So the primary
# read is TERMINAL VZ and BALLOON, not median xy.
# Arms INTERLEAVED per rep so SITL temporal drift is shared, not confounded.
# Shape copied from spanrescue_ab.sh (docs/SH_REFERENCE.md §10).
#   IC=2.0,2.0,5.0 N=5 bash test_data/LoomR_AB_harness/loomr_ab.sh
SD=/home/shubham/Soft-Precise-Landing/PX4_Gazebo; cd "$SD"
IC="${IC:-2.0,2.0,5.0}"; N="${N:-5}"; TAG="${TAG:-ic2}"
LAUNCH="scripts/run_aruco_landing_retry.sh"; SURF=0.0
WORLD_ENV="WORLD=cross_marker MARKER_TYPE=cross"     # HARD RULE: never bare ArUco
# ⛔ CONCURRENT-SESSION GUARD (added 2026-09-03 after a live near-miss: another chat
# had SITL up on cross_marker while this harness was in the working set). ko() below
# does an unconditional kill -9 on every px4/gz/MicroXRCEAgent process, so running
# this while someone else is flying KILLS THEIR RUN and silently corrupts both
# datasets (shared port 8888 + CPU contention skew perception timing).
# docs/SH_REFERENCE.md §8 pitfall 10. Override only if you are certain: FORCE=1.
if [ "${FORCE:-0}" != "1" ]; then
  _busy=$(ps -eo args | grep -aE 'px4_sitl_default/bin/px4|gz-sim|MicroXRCEAgent|landing_test\.py' | grep -av grep | head -3)
  if [ -n "$_busy" ]; then
    echo "REFUSING TO START: SITL is already running (another session?):" >&2
    echo "$_busy" | cut -c1-100 >&2
    echo "Wait for it to finish, or re-run with FORCE=1 if you are certain it is yours." >&2
    exit 3
  fi
fi
RES="$SD/run_logs/loomr_${TAG}.tsv"
printf "arm\trep\txy_err\tclass\tmin_h\tvz_term\tballoon\tdetOK\tloommult\trec\n" > "$RES"
SITL_PAT='px4_sitl_default/bin/px4|gz sim|gz-sim|ign gazebo|MicroXRCEAgent|parameter_bridge|mavsdk_server|QGroundControl|/opt/ros/humble|landing_test'
# ⛔⛔ PER-REP FOREIGN-SESSION CHECK (added 2026-09-04, ported from CmInv_AB_harness
# after a user challenge on the cm_inv run's failure cause -- see that harness's
# comment for the full incident). The guard at the top of this file only runs ONCE,
# before the loop starts -- ko() itself, called before EVERY rep, did an
# unconditional kill -9 with no re-check. If another session started SITL any time
# after the initial guard passed, ko() would have killed it SILENTLY, with nothing
# left behind afterward to prove it either way. That is unsafe regardless of
# whether it ever actually fired here.
#
# PRINCIPLE: ko()'s job is to make the process table clean BY KILLING WHAT IT CAN
# SEE. If, after its own best-effort kill, SITL processes are STILL present, that is
# NOT something ko() itself spawned -- it means something else is populating the
# table IN REAL TIME. That is the foreign-live-session signal, and the correct
# response is to STOP, not kill again and launch on top of it.
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
  sleep 3
  # After our own best effort, anything still alive is NOT ours -- see foreign_check.
  foreign_check "post-cleanup, before next launch"
}
for rep in $(seq 1 "$N"); do
  for arm in off on; do
    v=0; [ "$arm" = on ] && v=1
    ko
    L0=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    # Qt5 WORKAROUND (2026-09-03) -- see spanrescue_ab.sh for the full note.
    env LD_PRELOAD="${QT_PRELOAD:-/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5}" \
        HEADLESS=1 $WORLD_ENV LANDING_AUTOSAVE=1 MAX_ATTEMPTS=4 \
        INITIAL_DRONE_ENU="$IC" CROSS_LOOM_R_SCHEDULE="$v" \
        timeout 260 bash "$SD/$LAUNCH" \
        > "$SD/run_logs/loomr_${TAG}_${arm}_${rep}.out" 2>&1
    LG="$SD/run_logs/loomr_${TAG}_${arm}_${rep}.out"
    xy=$(grep -aoE 'xy_err=[0-9.]+' "$LG"|tail -1|cut -d= -f2)
    cls=$(grep -aoE 'Landing classification: [A-Z_+]+' "$LG"|tail -1|awk '{print $3}')
    L=$(ls -td "$SD/test_data/Landing_Test/"*/ 2>/dev/null|head -1)
    if [ "$L" = "$L0" ]; then rec="NO_NEW_REC"; mh="NA"; vz="NA"; bl="NA"; dk="NA"; lm="NA"; else
      rec="$(basename "$L")"
      read -r mh vz bl dk lm <<<"$("$HOME/ws/scripts/env2025/bin/python3" - "$L" "$SURF" <<'PY'
import sys,os,numpy as np,warnings
warnings.filterwarnings('ignore')
d=sys.argv[1]; SURF=float(sys.argv[2])
try:
    g=np.load(os.path.join(d,"Ground_Truth.npy"),allow_pickle=True).item()
    U=g["UAV Pose"];T=g["Target Pose"];n=min(len(U),len(T))
    h=np.array([U[i].position.z-T[i].position.z-SURF for i in range(n)])
    tt=np.asarray(g["Time"],float)[:n]; St=g["Start Time"]
    # terminal vz = descent rate over the last 0.5 m of travel above the surface
    vz=float('nan'); term=(h<=1.0)&(h>=0.05)
    if term.sum()>4:
        dh=np.gradient(h[term],tt[term]); vz=float(np.median(np.abs(dh)))
    # balloon = largest UPWARD excursion after first dropping below 1.0 m
    bl=float('nan')
    idx=np.argmax(h<=1.0) if (h<=1.0).any() else -1
    if idx>0: bl=float(max(0.0,(h[idx:]-np.minimum.accumulate(h[idx:])).max()))
    im=np.load(os.path.join(d,"Img_Data.npy"),allow_pickle=True).item()
    it=np.asarray(im["Time"],float)-St; ds=np.asarray(im["Detection Status"],object)
    m=min(len(it),len(ds)); it,ds=it[:m],ds[:m]
    sel=(it>=tt[0])&(it<=tt[-1]); alt=np.interp(it[sel],tt,h); band=alt>=1.0
    dk=100.0*np.mean(ds[sel][band]=='ok') if band.sum()>3 else float('nan')
    # ENGAGEMENT CHECK: did the flag actually take effect this run? (median applied
    # r[2] multiplier; 1.0 => schedule inert, i.e. the arm did NOT differ)
    lm=float('nan')
    if "Loom R Mult" in im:
        lr=np.asarray(im["Loom R Mult"],float)
        if lr.size: lm=float(np.nanmedian(lr))
    print("%.3f %.3f %.3f %.1f %.2f"%(h.min(),vz,bl,dk,lm))
except Exception:
    print("NA NA NA NA NA")
PY
)"
    fi
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "$arm" "$rep" "${xy:-NA}" "${cls:-NA}" "$mh" "$vz" "$bl" "$dk" "$lm" "$rec" >> "$RES"
    echo "[loomr] $TAG arm=$arm rep=$rep xy=${xy:-NA} cls=${cls:-NA} min_h=$mh vz=$vz balloon=$bl detOK=$dk loommult=$lm"
  done
done
ko; echo "LOOMR_DONE" >> "$RES"
