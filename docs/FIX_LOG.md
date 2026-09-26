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
- Interaction with FIX-006 (a_u_xy cap 10 m/s^2): a wrong-lock centre (170-350 px error) becomes a large fake lateral error; the cap bounds the
  resulting a_u but does not detect the wrong lock. FIX-001 remains the actual fix for that failure.
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
- Update 2026-09-26 (hardware analysis, evidence narrowing the hypothesis list): the analytic `s` in `hw_pos_feedback.py` is IMPLEMENTED FAITHFULLY.
  Independent recomputation from `Telemetry_Data` (Position Body + Quaternion, marker = median pose before takeoff, the model camera offset
  [0,0,0.15], Z_REG 0.2, V frame) reproduces the logged `s(t)` to rms 0.002-0.012 (corr 1.00/1.00) on 8 09-25 runs (with NO camera offset rms rises to
  0.03-0.09). So the mismatch is in the INPUTS, not the s math. Leading candidate: camera lever arm - `_CAM_OFF_FRD` is the Gazebo value (purely
  vertical 0.15 m) while `img_geometry.py` documents a real non-zero Pi camera lever arm (~0.19-0.23 m systematic bearing error, user-confirmed
  2026-07-27, unmeasured). A fixed 0.2 m lateral offset at Z = 1.5 m is 0.13 in s = ~65 px at fx ~513 (matches the 60-80 px), and grows as 1/Z
  as the drone descends (matches the reference moving away from the real cross). Other candidates: snapshot pose vs the physical marker position,
  EKF drift. Next check: measure the camera position relative to the FC in x/y/z and the marker position relative to the drone at arming, set
  `PLASMC_GT_CAM_DZ` plus a lateral offset accordingly, then re-project the reference into the videos (PASS criterion already in this entry).
- Caveat on the original 60-80 px / 208->500 px numbers (added by the Windows session, 2026-09-26): they projected the logged s, which is the REGULARISED bearing x/(z+0.2),
  as if it were the true bearing x/z. Below ~1.5 m that inflates the apparent offset (13 % at 1.5 m, 40 % at 0.5 m), so part of the late divergence is this, not the lever arm.
  When re-projecting, use the true bearing (s*(z+0.2)/z) or compare in regularised form on both sides.
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
- Interaction with the 2026-09-26 controller fixes (FIX-004..008): every 09-25/09-24 flight is HW_POS_FEEDBACK with dead perception (Img_Data 100% coast, so
  `MARKER_EXTENT_PX` == 0 always), so those flights say nothing about perception-mode behaviour. When perception is ported, re-verify FIX-004 (frame),
  FIX-006 (a_u_xy cap vs noisy real s / wrong locks, see FIX-001), FIX-007 (CBF heading, first time it is really exercised) and FIX-008 arming
  (`h_z` would then be perception-derived) in a perception-feedback flight.
- Fix: port perception + log raw detector output in `Img_Data`; tune the detector on real footage after the user's mocap recording.
- Confirm with: mocap-GT recording (heights, offsets, yaw, lighting) then `Hardware/scripts/hw_perception_quality.py`,
  `oracle_scan.py`, `score_cross_perception.py`; corr/nRMSE of s, alpha, h, w vs mocap GT. PASS thresholds to be set with the user
  (Gazebo reference: s corr 0.97-0.99 > 1.5 m; alpha 10-15 deg circ-RMS; h_z corr 0.72-0.88).
- Result: pending.


### FIX-004 a_u -> I_a uses the full body DCM: lateral command leaks into thrust  [deployed] (opened 2026-09-26)
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
- Metric caveat (FIX-002): lateral offset "vs arm point" is offset from the controller's reference, not necessarily from the physical marker
  (unmodelled camera lever arm, possible snapshot offset). Use video / mocap for the true landing offset when judging precision.
- Result: offline only (2026-09-26): all 47 flights / 31,315 ticks replayed with the new frame: max |I_a_z| 2746 -> 12.8, I_a_z>-5 samples 1496 -> 0,
  flights affected 33 -> 0 (open loop). Flight: pending. If it fails: `PLASMC_AU_FRAME=rotz` isolates frame vs heading; `body` = legacy.
- Second data set (2026-09-24, 39 flights, same pipeline `Hardware/Test_Data/analysis_2026-09-25/day_compare.py`): identical signature - 31/39 offboard
  phases ended by pilot takeover, `a_u_xy >= 100` in 22 (>= 1000 in 11), vertical leak > 5 m/s^2 in 22 and `I_a_raw_z > -5` in 22 flights. The bug predates
  09-25; it was masked because every flight was also being disrupted by other faults (09-24 yaw positive feedback, see FIX-013).
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-005 Thrust law has no tilt compensation  [deployed] (opened 2026-09-26)
- Symptom / evidence: legacy `B_T = m(I_a_z+g)/(cos phi cos theta)` gives T = m g at I_a_z = -g for any tilt, so vertical lift falls by g(1-cos):
  1.3 m/s^2 at 30 deg, 2 m/s^2 at 37 deg (thrust median 0.38 vs 0.42 needed in the last 1 s before takeovers). Same formula in PX4_Gazebo.
- Root cause: `controller.py::_attCtrl` B_T line.
- Fix: `B_T = m g + m I_a_z / max(cc, 0.5)`, `cc = cos(roll)cos(pitch)` (measured); `PLASMC_THRUST_TILT_COMP=0` restores legacy.
- Confirm with: ulg `vehicle_thrust_setpoint` vs `hover_thrust_estimate / cos(tilt)` while tilt > 15 deg (PASS ratio ~1; was ~0.5-0.6);
  thrust > 0.9 fraction stays < 1 % and `actuator_motors` max < 0.95; vertical speed gain in the last second < 0.5 m/s.
- Must not regress: hover thrust (+-0.01 of before; near-hover replay change was 0.005), voltage-corrected hover table (FIX-008 area / 09-24 fix).
- Result: offline (2026-09-26) full-pipeline replay: last-1 s thrust median 0.38 -> 0.42 (p10 0.18 -> 0.33), thrust>0.9 0.96 % -> 0.07 %. Flight: pending.
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-006 Uncapped lateral a_u command (PLASMC_AU_MAX_XY = 0)  [deployed] (opened 2026-09-26)
- Symptom / evidence: normal flight |a_u_xy| p50 0.7 / p99 5.3 / p99.9 17.8 m/s^2; terminal 100-9900 in 31/47 flights (last 0.2-0.9 s, 0.3-0.7 m).
- Root cause: no bound on `a_u[:2]` (cap knob existed, default off). This is a limiter; the terminal blow-up cause is FIX-011.
- Fix: `PLASMC_AU_MAX_XY` default 10 m/s^2 (clips 0.23 % of normal samples); `=0` disables.
- Confirm with: terminal tilt > 25 deg flights (was 14/47), pilot takeovers, lateral offset at handover/kill (was 0.41 / 0.53 m median).
  PASS = takeover/tilt counts fall and lateral offset not worse. Watch the opposite failure: authority too low to brake (raise to 15).
- Must not regress: precision; earlier history says capping can hurt braking ("lateral wall = commanded-but-not-delivered").
- Metric caveat (FIX-002): lateral offset "vs arm point" is offset from the controller's reference, not necessarily from the physical marker
  (unmodelled camera lever arm, possible snapshot offset). Use video / mocap for the true landing offset when judging precision.
- Result: pending.
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-007 CBF maps inertial<->image with ZYX yaw, perception uses the body-y x gravity frame  [deployed, unverifiable until perception runs] (opened 2026-09-26)
- Symptom / evidence: heading of the perception/analytic V frame relative to `yaw_c`: 0 deg single-axis tilt, 7 (20/20), 16 (30/30), 24 (37/37).
- Root cause: `controller.py` passes `yaw_c` to `cbf_visibility.cbf2_filter` (Rz(+-yaw)).
- Fix: pass the V-frame heading `atan2((R @ _levelled_basis(R))[1,0], [0,0])`; `PLASMC_CBF_VYAW=0` restores `yaw_c`.
- Confirm with: perception-feedback flight (CBF is largely bypassed under HW_POS_FEEDBACK): `theta_cone`/`rho_fov`, corner count, no new
  `CBF_CORNERS_STALE` aborts vs 09-25. Untestable on HW_POS flights.
- Revised 2026-09-26 with FIX-003: no cross perception runs on the Pi yet, so this cannot be confirmed until perception is ported and flown
  in perception-feedback mode. Until then its status stays in-repo / unverifiable; the V frame it uses is the same one `img_geometry._rp_basis` builds.
- Result: unit-checked only (basis math). Pending.
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-008 Touchdown trigger: EKF depth and marker scale rejected -> IMU contact jerk  [deployed] (opened 2026-09-26)
- Symptom / evidence: `MARKER_EXTENT_PX == 0` in all 47 runs (cross perception not ported / 100% coast, FIX-003; not caused by HW_POS itself) so a scale path could never fire; EKF-depth trigger rejected by the
  user; |a| magnitude unusable (soft contacts read 11-13 m/s^2; a 50 m/s^2 threshold misses ~31/47).
- Root cause: design (depth/scale signals); Gazebo's `_impactDetector` (|a|>50) is tuned to 500-900 m/s^2 sim contacts.
- Fix: `flight_controller.py::_imuTouchdownStep` (from `_getAcc`, sensor timestamps): 3-sample-mean `|d a_z/dt| > FC_IMU_TD_JERK` (800 m/s^3),
  persist 1, dwell 0.5 s, armed via `controller._td_armed` -> `fc.IMU_TD_ARM`; sets `LANDED` (-> existing `action.land()`). Controller depth/scale
  latches opt-in (`PLASMC_TD_USE_GT_DEPTH`, `PLASMC_TD_SCALE`, default 0). `FC_IMU_TD=0` disables.
- Revised 2026-09-26 (FIX-003): arming no longer relies only on the controller's `_td_armed` (h_z-based; EKF-derived under HW_POS but perception-derived once
  perception is ported and could then never arm). `hardware_landing.py` now arms the IMU detector when `_td_armed` OR after `FC_IMU_TD_ARM_FALLBACK_S` (4.0 s,
  0 disables) of controlled flight. Replay with time-only arming (flag never set): identical to before - fired 40/47, 38 within 0.25 m, same 2 early + 7 missed.
- Confirm with: console `[FC] IMU contact detected (...)` per flight; compare with EKF height (`vehicle_local_position`) at that time.
  PASS = fired in >= ~80 % of flights, height at trigger <= 0.25 m (median ~0), no trigger > 0.6 m; PLASMC now flies to contact (new below 0.25 m):
  check touchdown speed, bounce, lateral slide.
- Must not regress: PX4 ON_GROUND backstop (`_getLandedState`), post-LANDED `action.land()`.
- Result: offline (real method on logged 188 Hz IMU streams, 47 flights): fired 40, 38 within 0.25 m (median at contact), 2 early (0.51, 0.66 m), 7 missed.
  Flight: pending. Known limit ~15 % missed (soft settles).
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-009 Run-start (arming) failures after landings  [deployed]  (opened 2026-09-26)
- Symptom / evidence: 09-25 10:50 (`arm() COMMAND_DENIED: Resolve system health failures first`, is_armable had gone true 13.7 s earlier), 10:55
  (60 s hard timeout), 10:56 (user Ctrl-C during the same wait); readiness flickers 1-46 s ("height estimate not stable", "GPS ... Drift too high",
  kill switch / termination latch).
- Root cause: `flight_controller.py::arm_and_takeoff` armed on one transient `is_armable` sample and gave up after 60 s. Why PX4 flickers is unknown (FIX-015).
- Fix: readiness must hold `ARM_STABLE_S` (2 s) continuously (timer POLLED by a background health pump, not tied to new samples); denied `arm()`
  retried until `ARM_WAIT_TIMEOUT_S` (120 s). (The earlier Pi version, whose timer only ran on new samples, was replaced on 2026-09-26.)
- Confirm with: console `is_armable + position stable for 2.0s after X s`, `arm() denied (attempt n)`; count run-start failures (was 3 of 50 attempts).
- Result: offline mock vehicle: steady-then-silent armed 1.2 s; flicker 2.1 s; two denials then ok 5.4 s / 3 calls; never-ready -> timeout at deadline. Flight: pending.
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-010 Console flood from TD_DEBUG  [deployed] (opened 2026-09-26)
- Symptom / evidence: 10,509 of ~13,100 lines of the 09-25 transcript were `[TD_DEBUG]`; the transcript started at 10:44 instead of 10:20.
- Fix: `controller.py::_td_dbg` prints every `TD_DEBUG_PERIOD_S` (0.5 s) (every tick when depth <= 0.5 m or hold running).
- Confirm with: next transcript keeps the whole session. Result: pending.
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-011 Terminal lateral blow-up / funnel vs achievable lateral precision  [open] (opened 2026-09-26)
- Symptom / evidence: 31/47 flights `a_u_xy >= 100` in the last 0.2-0.9 s at 0.3-0.7 m; kappa_xy to 30 (3 flights); SEN funnel p_s shrinks 1.2 -> 0.35
  on a time schedule so it needs lateral error <= 0.35 x height, but achieved (vs arm point, EKF) 0.22 m at 3 m, peak 0.50 m at 1-1.5 m, 0.23 m at
  0.3-0.6 m: inside the funnel 97 % / 44 % / 26 % of the time; s_e_n first > 1 at 1.3-3 m height in most flights.
  Caveat (FIX-002): measured relative to the snapshotted arm point = the controller's own reference (the analytic s was verified to be computed faithfully, rms <= 0.012),
  so the funnel-vs-precision comparison is valid for the controller; the offset to the PHYSICAL marker may differ (unmodelled camera lever arm).
- Root cause: partly FIX-004 (leak couples lateral spikes into altitude); remaining: hypothesis - funnel schedule / gains tuned for sim precision.
- Fix: none yet - re-evaluate after FIX-004..006 fly.
- Confirm with: timeline of s, p_s, s_e_n, kappa_xy, a_u_xy vs height (analysis dir `lat`-style), fraction of time s_e_n > 1, terminal a_u_xy, pilot takeovers.
- Result: pending.

### FIX-012 Integral windup (izeta at its clamp 5.0 in 9/47 flights)  [open] (opened 2026-09-26)
- Symptom / evidence: `izeta_max = 5.00` in 10 of the 09-25 flights (blown runs); clamp exists (`controller.py` `_izeta_clamp`), freeze-when-unfresh only.
- Root cause (revised 2026-09-26): mostly INERT state. In the combined-barrier config only `izeta_z` enters sigma (`_sig[2] += Omega_z*izeta_z`, max
  0.1*5 = 0.5); `izeta_xy` is only used after the terminal commit (`_tc_integral and _committed`), otherwise it is logging-only. The clamp is hit on the
  X component in 8 of 9 flights (harmless) and on Z in 3 flights (10-34-55, 10-53-06, 10-58-07). It winds up over 3-12 s of funnel breach (s_e_n > 1 for
  35-100 % of the time). izeta was frozen at exactly 0 on 09-24 (camera gating, fixed that evening), so 09-25 is the first data with active integrators.
  Fix: none needed for x/y; for z consider conditional integration (freeze while |s_e_n| > 1) only if z windup shows up again after FIX-004..006.
- Confirm with: izeta trace vs s_e_n > 1 periods after FIX-004..006. Result: pending.

### FIX-013 Yaw loop only partly converging  [open] (opened 2026-09-26)
- Symptom / evidence: `|e_a|` ended smaller than at start in 24/47 flights (end median 4 deg); 7 flights grew > 0.1 rad (worst -0.57 rad).
- Root cause: not investigated (the 09-24 alpha-sign fix removed the 31/33 divergence). Confirm with: e_a(t), u_a(t) vs measured yaw rate after FIX-004. Result: pending.
- Update 2026-09-26 (both days, FC heading as independent reference, `analysis_2026-09-25/yaw_check.py`):
  09-24 the loop was in POSITIVE feedback: controller `e_a` had slope +0.98 vs the true heading change (09-25: -1.00), the commanded yaw rate reduced the true
  error 0 % of the time, heading drifted a median -28.5 deg (max 75 deg), max |u_a| 1.5 rad/s. The 09-24 alpha-sign fix (deployed for 09-25) works: 09-25 max heading
  error median 11 deg (p90 23), net drift median 1.9 deg (max 39).
  Residual on 09-25: the command reduces the true error only 50 % of the time (chance level), corr(u_a, gyro_z) 0.19, corr(w_u[2], gyro_z) 0.39; std(gyro_z)
  0.094 vs std(u_a) 0.060 rad/s - yaw command is small versus disturbance/noise, so yaw is effectively uncontrolled but stays within ~11 deg (little effect on
  lateral landing because analytic s does not use alpha). Low priority; no code defect found. Confirm after FIX-004 that heading error stays <= 25 deg.

### FIX-014 Takeoff overshoot to ~6 m in 2 flights; stick-down takeovers at altitude  [open] (opened 2026-09-26)
- Symptom / evidence: 09-25 flights `10_49_44` (peak 5.9 m) and one more (3.5-3.9 m) ended by pilot stick-DOWN takeover at 3.5-5.9 m.
- Root cause: not investigated. Confirm with: ulg takeoff phase altitude vs setpoint and hover throttle at arm. Result: pending.
- Update 2026-09-26: takeoff itself is fine on both days (height at offboard start median 3.06 m 09-24 / 3.13 m 09-25, p10-p90 3.0-3.3). Outliers are few:
  09-25 10-33-24 climbed from 3.15 to 5.85 m during control (V 20.7, low-battery failsafe active), 10-34-55 to 3.75 m, 10-54-47 started at 3.71 m
  (blind-land failsafe), 11-14-50 started at 3.60 m; 09-24 15-15-12 (3.59 -> 4.49 m, low battery), 15-21-19 (4.11 m, blind land), 15-25-24 (2.53 m). Most coincide
  with PX4 failsafes (FIX-015), so treat as a consequence, not a separate takeoff bug.

### FIX-015 PX4 failsafes during flight (low battery) + preflight flicker / kill-switch latch  [deployed (battery guard); flicker cause open] (opened 2026-09-26)
- Symptom / evidence: see FIX-009; 7 low-battery failsafes on 09-25; each flight ends with the pilot's kill switch which latches "Kill switch engaged /
  Flight termination active" until released. Root cause unknown (no pre-arm logs exist). Confirm with: battery V at arm and EKF status while disarmed. Result: pending.
- Update 2026-09-26 (both days): a PX4 failsafe fired DURING offboard control in 12/39 flights on 09-24 (9 low battery, 3 'invalid setpoints / blind land') and 8/47 on
  09-25 (7 low battery). All 16 low-battery flights engaged at 20.5-21.4 V (median 21.1) under hover load; the other 70 flights engaged at 21.3-24.1 V (median 22.2);
  40 % of flights engaged at <= 21.9 V hit the failsafe. Min voltage in flight 20.1-23.8 V, peak current 35 A. Those flights are compromised (FC overrides the controller).
- Fix (guard): `hardware_landing.py` after takeoff: hover voltage < `HW_MIN_FLIGHT_V` (default 21.5 V; 0 disables) -> RuntimeError -> existing abort path (PX4 land) instead
  of descending. Operating rule: swap the pack when the hover-load voltage is < 21.8 V.
- Confirm with: console `Battery ... is below HW_MIN_FLIGHT_V` lines on low packs; zero low-battery failsafe messages in `.ulg` for flown flights. The 3 09-24 'invalid setpoints'
  events predate the 09-24 fixes (check they do not recur). Flicker/kill-latch cause still unknown.
- Deployed 2026-09-26 14:44 to the Pi (192.168.1.110) with `Hardware/deploy_to_pi.sh`; md5 controller 99a48e58..., flight_controller d7a03add..., hardware_landing d8301f1c...; backups `*.bak_before_fixdeploy_20260926_144435`; import check OK. Unflown until the next session.

### FIX-016 Port FIX-004..007 to PX4_Gazebo  [open] (opened 2026-09-26)
- Evidence: `PX4_Gazebo/src/controller.py` has identical `R_au`, `B_T`, `PLASMC_AU_MAX_XY=0`, CBF yaw code; sim Final VISTA-GT results unaffected
  (a_u_xy <= 2 m/s^2); blown YawRateLaw sims show the same leak. Handover: `Hardware/docs/HANDOVER_AU_FRAME_FIX_UBUNTU.md`.
- Confirm with: `scripts/run_rotz_ic1_ab.sh` extended with a `vframe` arm; `scripts/run_ic_validation.sh` (only when the user asks, HEADLESS=1). Result: pending.

### FIX-017 RATE_CORRECTION over-corrects roll/pitch (achieved rate ~0.8x intended)  [open] (opened 2026-09-26)
- Symptom / evidence: `hardware_landing.py` multiplies the controller body-rate command by RATE_CORRECTION = (0.758, 0.739, 0.665) from the input calibration (which assumed
  achieved = commanded/0.76). In flight PX4 tracks its setpoint at about unity: achieved/FC-setpoint gain 1.08 roll/pitch on 09-25 (0.97/0.99 on 09-24) with 50-60 ms lag, so
  achieved/intended = 0.82 roll, 0.80 pitch (09-25) / 0.79, 0.75 (09-24). Command mapping itself is faithful (FC setpoint = 0.75 x w_u, corr 1.00; B_T -> thrust setpoint slope
  -0.0296 vs -0.0313 expected, corr -0.98). Yaw is inconsistent between days (achieved/intended 1.32 on 09-25 vs 0.70 on 09-24), see FIX-013. Scripts:
  `Hardware/Test_Data/analysis_2026-09-25/{map_check,rate_gain}.py`.
- Root cause: input-cal gain not representative of flight (different frequency/amplitude or PX4 rate gains changed since the cal); hypothesis.
- Fix: none yet, and NOT a clear bug - raising the roll/pitch factors to ~0.92 would raise loop gain ~15-20 % with ~50 ms lag and all gains were tuned with today's effective
  0.8x. Decide after FIX-004..006 are flown; if lateral response is still slow, test `RATE_CORRECTION_WX/WY=0.92` as a separate, explicit A/B.
- Confirm with: achieved/intended gain from `rate_gain.py` on the next session (PASS = ~1.0 if the factors are changed; unchanged = 0.8 expected).
- Result: pending.

### FIX-018 Hardware-default gains reset per GROUP instead of per axis when one env var is set  [in-repo] (opened 2026-09-26)
- Symptom / evidence: 09-25 recorded `Control_Params.npy` show the gain tests were not single-variable: `PLASMC_GAMMA_Z=1.5` ran with Gamma = (2.0, 2.0, 1.5) (xy 8x the hardware 0.25);
  `PLASMC_GAMMA_X/Y=1.0` (5 flights) ran Gamma = (1, 1, 1.0) and `=1.2` (4 flights) ran (1.2, 1.2, 1.0), both with z 1.0 instead of the hardware 0.75. Effective configs flown (n): default 19,
  Gamma(1,1,1) 5, P_xy 2.5 4, Gamma(1.2,1.2,1) 4, K_R 1.0 3, kappa_max 10 3, K_R 2.0 2, K_R 3.0 2, P_z 2.5 2, N_z 0.2 2, Gamma(2,2,1.5) 1.
- Root cause: `controller.py` combined-barrier auto-align applied the hardware defaults for GAMMA, KAPPA0, E and XI2 only `if not any(PLASMC_<K>_{X,Y,Z} in os.environ)`; setting any one axis
  reset the unset axes to the `pa()` code defaults (GAMMA 2,2,1; KAPPA0/E/XI2 similarly). (P2INF and the per-axis `pa()` gains such as P, N, OMEGA, KAPPA_MAX were already per axis.)
- Fix: `_hw_axes(key, hw, cur)` applies each hardware default per axis (only axes without an env override). Unit-checked: GAMMA_Z=1.5 -> (0.25, 0.25, 1.5); GAMMA_X/Y=1.2 -> (1.2, 1.2, 0.75);
  old logic reproduces the recorded 09-25 values. Repo only - NOT deployed (the 2026-09-26 flight plan sets none of these variables, so its behaviour is unchanged either way).
- Confirm with: next session that sets a GAMMA/KAPPA0/E/XI2 env var: `Control_Params.npy` shows only the overridden axis changed and the others equal the hardware defaults
  (Gamma 0.25/0.25/0.75, kappa_0 0.5/0.5/0.25, E 1/1/0.5, Xi2 0.7/0.7/1.0). Must not regress: default (no env) config identical to before.
- Result: unit-checked only. Consequence: all 09-25 GAMMA-based conclusions (FIX-011 sweep tables, Gamma 1.2 / revert / GAMMA_Z tests) are void.
