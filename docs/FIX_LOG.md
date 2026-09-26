# FIX LOG — bugs, fixes, and how to confirm each fix

Purpose: work across several Claude Code sessions (Windows / Ubuntu / Pi) without losing what a fix was
supposed to do or how to prove it. **Every bug gets an entry while it is being worked on** (rule from the user,
2026-09-26). Newest first. Do not delete entries; change Status and fill Result.

Status values: `open` (found, not fixed) -> `in-repo` (committed, not deployed/flown) -> `deployed` (on Pi/PX4, unflown)
-> `verified` (confirmed with the data below) or `failed` (checks did not pass; say why).

Entry template:

```
### FIX-NNN <short title>  [status] (opened YYYY-MM-DD)
- Symptom / evidence:  what was observed, with data paths / run ids / numbers
- Root cause:          file:function, why it happens (mark "hypothesis" if unverified)
- Fix:                 what changes, files, commit, env knob + default
- Confirm with:        data to collect (sim IC set / flights / recording), command or script, expected numbers (PASS = ...)
- Must not regress:    other checks that must stay unchanged
- Result:              filled when checked (date, numbers, verdict)
```

---

### FIX-001 Cross detector rejects an off-frame junction (partial visibility)  [open] (opened 2026-09-26)
- Symptom / evidence: Gazebo GT scoring (`PX4_Gazebo/tools/score_partial_visibility.py test_data 0.5`, 74 consistent reps,
  alt > 0.3 m). Centre in frame: 95% detected, 2.9 px median error. Centre outside frame by 0-20 / 20-40 / 40-80 / >80 px:
  42 / 49 / 50 / 31% "ok" but centre error 170 / 176 / 203 / 347 px (far more than the distance outside = wrong lock).
  Requirement: any two partially visible lines must be enough.
- Root cause: `cross_stroke_detector.py` `side_stats()` returns `bal=None` when J is at/outside the frame edge and the pair loop
  does `if ba is None and bb is None: continue`; `cross_marker_perception.py` never reads `det.in_fov`.
  (Details + line refs: `PX4_Gazebo/docs/HANDOFF_partial_visibility_offframe_junction.md`.)
- Fix: (planned) accept off-frame J with >=2 angled lines each verified on the visible side, capped extrapolation, `in_fov=False`;
  perception uses `in_fov` with inflated KF noise; env knob, default ON only after the gate passes.
- Confirm with: (1) `tools/score_partial_visibility.py test_data 0.5` before/after. PASS = off-frame rows' centre error within
  ~2x the in-frame error + small extrapolation error (not 170-350 px), off-frame detected rate up, in-frame row unchanged
  (>=95% detected, <=3 px median). (2) `tools/validate_detector_gt.py --set test_data/PerceptionEvalSet --variant stroke`:
  accuracy/POISON no worse. (3) `scripts/run_ic_validation.sh` IC2-5 (only when the user asks, HEADLESS=1) still passes.
- Must not regress: in-frame precision; no two border lines meeting at a frame corner accepted as an X; the peer session's baked defaults
  (stroke, S_LOSS_FADE, TD_SETTLE) and its n=5 pure-perception IC1-5 gate (see handover, Coordination). Note `ln["width"]` is ridge sigma, not pixel
  width; peer's ring-radius change (`CROSS_STROKE_RING_K` 2.5->4) touches the same function and is not applied yet.
- Result: pending.

### FIX-002 HW_POS_FEEDBACK reference marker point does not match the real marker  [open] (opened 2026-09-26, hypothesis)
- Symptom / evidence: 2026-09-25 run `Fri Sep 25 10-32-10` (video `10-31-48`): the pixel projection of the analytic reference s
  sits 60-80 px from the real cross and moves the opposite way to it later in the descent (ref y 208 -> 500 while the real cross
  moves up in the image). Controller steers to the snapshotted point, not the physical marker.
- Root cause: unknown (candidates: marker snapshot taken from an off-centre pose, EKF drift, geometry/sign error in
  `Hardware/scripts/hw_pos_feedback.py` V-frame/lever arm, Control_Data index misalignment). NOT investigated.
- Fix: none yet.
- Confirm with: mocap or a hand-measured marker position vs the snapshot NED point; replay: project the reference into the
  video frames for several runs (`Hardware/scripts/perception_hw_common.py` `ref_pixel`, `depth_yaw`) and check it stays on the
  cross once the offset is fixed. PASS = projected point within ~10 px of the cross over the descent in >= 80% of runs.
- Must not regress: touchdown logic and the other 2026-09-25 fixes (`FLIGHT_ANALYSIS_2026-09-25.md`).
- Result: pending.

### FIX-003 Cross perception (s, alpha, h, w) not running on the Pi  [open] (opened 2026-09-26)
- Symptom / evidence: `Img_Data.npy` for 09-24 (39 runs) and 09-25 (47 runs) is 100% `coast`; no online cross h/w/alpha.
  Offline reprocessing of the videos: stroke detector 60% miss on visible frames (weak > 2.5 m and < 1.2 m), legacy 55% wrong-place.
- Root cause: `cross_marker_perception` not ported (`Hardware/docs/CROSS_MARKER_PORT_PLAN.md` S4); detectors tuned at f=135,
  Pi is fx~513 / 35 deg hfov, thin strokes at 320x240.
- Fix: port perception + log raw detector output in `Img_Data`; tune the detector on real footage after the user's mocap recording.
- Confirm with: mocap-GT recording (heights, offsets, yaw, lighting) then `Hardware/scripts/hw_perception_quality.py`,
  `oracle_scan.py`, `score_cross_perception.py`; corr/nRMSE of s, alpha, h, w vs mocap GT. PASS thresholds to be set with the user
  (Gazebo reference: s corr 0.97-0.99 > 1.5 m; alpha 10-15 deg circ-RMS; h_z corr 0.72-0.88).
- Result: pending.


### FIX-004 a_u -> I_a uses the full body DCM: lateral command leaks into thrust  [in-repo] (opened 2026-09-26)
- Symptom / evidence: 2026-09-25 (47 controller flights): 37/47 ended by PILOT throttle-stick takeover (roll/pitch/yaw sticks never moved),
  at median 0.36 m height, 1.2 m/s down (1.6-2.6 for stick-up cases), tilt 22-37 deg; commanded thrust fell to 0.03-0.29 vs ~0.45-0.52
  needed. Peak raw `I_a_z` median +53 m/s^2 in takeover flights (-9.2 otherwise). Analysis: `Hardware/docs/FLIGHT_ANALYSIS_2026-09-25.md`.
- Root cause: `Hardware/scripts/controller.py::_attCtrl`: `I_a_raw = R_body @ a_u - g e3` with R_body the full body DCM
  (`PLASMC_AU_ROTZ_ONLY=0`), but `a_u` is in the gravity-levelled V frame; tilt leaks uncapped `a_u_xy` (100-9900 m/s^2) into `I_a_z`.
  Replay with the Pi quaternion reproduces the logged `I_a_raw_z` to 0.01 m/s^2 (n=1198). Same code in `PX4_Gazebo/src/controller.py` (FIX-016).
- Fix: `_levelled_basis(R)` + env `PLASMC_AU_FRAME` = `vframe` (DEFAULT: `R @ _levelled_basis(R)`, the frame perception uses) | `rotz`
  (also `PLASMC_AU_ROTZ_ONLY=1`) | `body` (legacy, A/B only). Not deployed, not flown.
- Confirm with: next Pi flight session, `Hardware/Test_Data/analysis_2026-09-25/` scripts (dates edited). PASS = (1) vertical leak
  `(I_a_raw_z+9.81)-a_u_z` |p99| < 0.5 m/s^2 in Control_Data even when `a_u_xy` is large; (2) flights with any `I_a_raw_z > -5`: ~0 (was 33/47);
  (3) `.ulg` offboard exits by pilot takeover (nav_state 14 -> 2) a small minority (was 37/47), script land (nav 18) dominant;
  (4) thrust in last 1 s before handover >= 0.8x hover/cos(tilt) (was collapsing to 0.03-0.29); (5) vertical re-ascent after first <1 m
  (was > 0.3 m in 12/47) and handover vertical speed (was median 1.2 m/s).
- Must not regress: lateral tracking (xy offset at kill was median 0.53 m), yaw hold, touchdown detection (FIX-008).
- Result: offline only (2026-09-26): all 47 flights / 31,315 ticks replayed with the new frame: max |I_a_z| 2746 -> 12.8, I_a_z>-5 samples 1496 -> 0,
  flights affected 33 -> 0 (open loop). Flight: pending. If it fails: `PLASMC_AU_FRAME=rotz` isolates frame vs heading; `body` = legacy.

### FIX-005 Thrust law has no tilt compensation  [in-repo] (opened 2026-09-26)
- Symptom / evidence: legacy `B_T = m(I_a_z+g)/(cos phi cos theta)` gives T = m g at I_a_z = -g for any tilt, so vertical lift falls by g(1-cos):
  1.3 m/s^2 at 30 deg, 2 m/s^2 at 37 deg (thrust median 0.38 vs 0.42 needed in the last 1 s before takeovers). Same formula in PX4_Gazebo.
- Root cause: `controller.py::_attCtrl` B_T line.
- Fix: `B_T = m g + m I_a_z / max(cc, 0.5)`, `cc = cos(roll)cos(pitch)` (measured); `PLASMC_THRUST_TILT_COMP=0` restores legacy.
- Confirm with: ulg `vehicle_thrust_setpoint` vs `hover_thrust_estimate / cos(tilt)` while tilt > 15 deg (PASS ratio ~1; was ~0.5-0.6);
  thrust > 0.9 fraction stays < 1 % and `actuator_motors` max < 0.95; vertical speed gain in the last second < 0.5 m/s.
- Must not regress: hover thrust (+-0.01 of before; near-hover replay change was 0.005), voltage-corrected hover table (FIX-008 area / 09-24 fix).
- Result: offline (2026-09-26) full-pipeline replay: last-1 s thrust median 0.38 -> 0.42 (p10 0.18 -> 0.33), thrust>0.9 0.96 % -> 0.07 %. Flight: pending.

### FIX-006 Uncapped lateral a_u command (PLASMC_AU_MAX_XY = 0)  [in-repo] (opened 2026-09-26)
- Symptom / evidence: normal flight |a_u_xy| p50 0.7 / p99 5.3 / p99.9 17.8 m/s^2; terminal 100-9900 in 31/47 flights (last 0.2-0.9 s, 0.3-0.7 m).
- Root cause: no bound on `a_u[:2]` (cap knob existed, default off). This is a limiter; the terminal blow-up cause is FIX-011.
- Fix: `PLASMC_AU_MAX_XY` default 10 m/s^2 (clips 0.23 % of normal samples); `=0` disables.
- Confirm with: terminal tilt > 25 deg flights (was 14/47), pilot takeovers, lateral offset at handover/kill (was 0.41 / 0.53 m median).
  PASS = takeover/tilt counts fall and lateral offset not worse. Watch the opposite failure: authority too low to brake (raise to 15).
- Must not regress: precision; earlier history says capping can hurt braking ("lateral wall = commanded-but-not-delivered").
- Result: pending.

### FIX-007 CBF maps inertial<->image with ZYX yaw, perception uses the body-y x gravity frame  [in-repo] (opened 2026-09-26)
- Symptom / evidence: heading of the perception/analytic V frame relative to `yaw_c`: 0 deg single-axis tilt, 7 (20/20), 16 (30/30), 24 (37/37).
- Root cause: `controller.py` passes `yaw_c` to `cbf_visibility.cbf2_filter` (Rz(+-yaw)).
- Fix: pass the V-frame heading `atan2((R @ _levelled_basis(R))[1,0], [0,0])`; `PLASMC_CBF_VYAW=0` restores `yaw_c`.
- Confirm with: perception-feedback flight (CBF is largely bypassed under HW_POS_FEEDBACK): `theta_cone`/`rho_fov`, corner count, no new
  `CBF_CORNERS_STALE` aborts vs 09-25. Untestable on HW_POS flights.
- Result: unit-checked only (basis math). Pending.

### FIX-008 Touchdown trigger: EKF depth and marker scale rejected -> IMU contact jerk  [in-repo; older version deployed] (opened 2026-09-26)
- Symptom / evidence: `MARKER_EXTENT_PX == 0` in all 47 runs under `PLASMC_HW_POS_FEEDBACK` (scale path dead); EKF-depth trigger rejected by the
  user; |a| magnitude unusable (soft contacts read 11-13 m/s^2; a 50 m/s^2 threshold misses ~31/47).
- Root cause: design (depth/scale signals); Gazebo's `_impactDetector` (|a|>50) is tuned to 500-900 m/s^2 sim contacts.
- Fix: `flight_controller.py::_imuTouchdownStep` (from `_getAcc`, sensor timestamps): 3-sample-mean `|d a_z/dt| > FC_IMU_TD_JERK` (800 m/s^3),
  persist 1, dwell 0.5 s, armed via `controller._td_armed` -> `fc.IMU_TD_ARM`; sets `LANDED` (-> existing `action.land()`). Controller depth/scale
  latches opt-in (`PLASMC_TD_USE_GT_DEPTH`, `PLASMC_TD_SCALE`, default 0). `FC_IMU_TD=0` disables.
- Confirm with: console `[FC] IMU contact detected (...)` per flight; compare with EKF height (`vehicle_local_position`) at that time.
  PASS = fired in >= ~80 % of flights, height at trigger <= 0.25 m (median ~0), no trigger > 0.6 m; PLASMC now flies to contact (new below 0.25 m):
  check touchdown speed, bounce, lateral slide.
- Must not regress: PX4 ON_GROUND backstop (`_getLandedState`), post-LANDED `action.land()`.
- Result: offline (real method on logged 188 Hz IMU streams, 47 flights): fired 40, 38 within 0.25 m (median at contact), 2 early (0.51, 0.66 m), 7 missed.
  Flight: pending. Known limit ~15 % missed (soft settles).

### FIX-009 Run-start (arming) failures after landings  [in-repo; OLDER version deployed - re-deploy]  (opened 2026-09-26)
- Symptom / evidence: 09-25 10:50 (`arm() COMMAND_DENIED: Resolve system health failures first`, is_armable had gone true 13.7 s earlier), 10:55
  (60 s hard timeout), 10:56 (user Ctrl-C during the same wait); readiness flickers 1-46 s ("height estimate not stable", "GPS ... Drift too high",
  kill switch / termination latch).
- Root cause: `flight_controller.py::arm_and_takeoff` armed on one transient `is_armable` sample and gave up after 60 s. Why PX4 flickers is unknown (FIX-015).
- Fix: readiness must hold `ARM_STABLE_S` (2 s) continuously (timer POLLED by a background health pump, not tied to new samples); denied `arm()`
  retried until `ARM_WAIT_TIMEOUT_S` (120 s). NOTE: the Pi has the earlier version whose timer only ran on new samples - re-deploy.
- Confirm with: console `is_armable + position stable for 2.0s after X s`, `arm() denied (attempt n)`; count run-start failures (was 3 of 50 attempts).
- Result: offline mock vehicle: steady-then-silent armed 1.2 s; flicker 2.1 s; two denials then ok 5.4 s / 3 calls; never-ready -> timeout at deadline. Flight: pending.

### FIX-010 Console flood from TD_DEBUG  [deployed] (opened 2026-09-26)
- Symptom / evidence: 10,509 of ~13,100 lines of the 09-25 transcript were `[TD_DEBUG]`; the transcript started at 10:44 instead of 10:20.
- Fix: `controller.py::_td_dbg` prints every `TD_DEBUG_PERIOD_S` (0.5 s) (every tick when depth <= 0.5 m or hold running).
- Confirm with: next transcript keeps the whole session. Result: pending.

### FIX-011 Terminal lateral blow-up / funnel vs achievable lateral precision  [open] (opened 2026-09-26)
- Symptom / evidence: 31/47 flights `a_u_xy >= 100` in the last 0.2-0.9 s at 0.3-0.7 m; kappa_xy to 30 (3 flights); SEN funnel p_s shrinks 1.2 -> 0.35
  on a time schedule so it needs lateral error <= 0.35 x height, but achieved (vs arm point, EKF) 0.22 m at 3 m, peak 0.50 m at 1-1.5 m, 0.23 m at
  0.3-0.6 m: inside the funnel 97 % / 44 % / 26 % of the time; s_e_n first > 1 at 1.3-3 m height in most flights.
  Caveat: measured relative to the snapshotted arm point (see FIX-002; may differ from the physical marker).
- Root cause: partly FIX-004 (leak couples lateral spikes into altitude); remaining: hypothesis - funnel schedule / gains tuned for sim precision.
- Fix: none yet - re-evaluate after FIX-004..006 fly.
- Confirm with: timeline of s, p_s, s_e_n, kappa_xy, a_u_xy vs height (analysis dir `lat`-style), fraction of time s_e_n > 1, terminal a_u_xy, pilot takeovers.
- Result: pending.

### FIX-012 Integral windup (izeta at its clamp 5.0 in 9/47 flights)  [open] (opened 2026-09-26)
- Symptom / evidence: `izeta_max = 5.00` in 10 of the 09-25 flights (blown runs); clamp exists (`controller.py` `_izeta_clamp`), freeze-when-unfresh only.
- Root cause: hypothesis - integrates through funnel breach / saturated commands. Fix: none yet (candidate: conditional integration).
- Confirm with: izeta trace vs s_e_n > 1 periods after FIX-004..006. Result: pending.

### FIX-013 Yaw loop only partly converging  [open] (opened 2026-09-26)
- Symptom / evidence: `|e_a|` ended smaller than at start in 24/47 flights (end median 4 deg); 7 flights grew > 0.1 rad (worst -0.57 rad).
- Root cause: not investigated (the 09-24 alpha-sign fix removed the 31/33 divergence). Confirm with: e_a(t), u_a(t) vs measured yaw rate after FIX-004. Result: pending.

### FIX-014 Takeoff overshoot to ~6 m in 2 flights; stick-down takeovers at altitude  [open] (opened 2026-09-26)
- Symptom / evidence: 09-25 flights `10_49_44` (peak 5.9 m) and one more (3.5-3.9 m) ended by pilot stick-DOWN takeover at 3.5-5.9 m.
- Root cause: not investigated. Confirm with: ulg takeoff phase altitude vs setpoint and hover throttle at arm. Result: pending.

### FIX-015 PX4 preflight flicker and kill-switch latch after each landing  [open] (opened 2026-09-26)
- Symptom / evidence: see FIX-009; 7 low-battery failsafes on 09-25; each flight ends with the pilot's kill switch which latches "Kill switch engaged /
  Flight termination active" until released. Root cause unknown (no pre-arm logs exist). Confirm with: battery V at arm and EKF status while disarmed. Result: pending.

### FIX-016 Port FIX-004..007 to PX4_Gazebo  [open] (opened 2026-09-26)
- Evidence: `PX4_Gazebo/src/controller.py` has identical `R_au`, `B_T`, `PLASMC_AU_MAX_XY=0`, CBF yaw code; sim Final VISTA-GT results unaffected
  (a_u_xy <= 2 m/s^2); blown YawRateLaw sims show the same leak. Handover: `Hardware/docs/HANDOVER_AU_FRAME_FIX_UBUNTU.md`.
- Confirm with: `scripts/run_rotz_ic1_ab.sh` extended with a `vframe` arm; `scripts/run_ic_validation.sh` (only when the user asks, HEADLESS=1). Result: pending.
