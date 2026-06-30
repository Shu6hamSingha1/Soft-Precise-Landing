---
name: project_moving_target_prep
description: "HANDOFF for the moving-target (rover) landing phase: carried-over baked state (W_U_MAX=2.0, VDS_KF_Q=1, Z_REG=0.2; 6/9 SP stationary base), controller readiness (gt_feedback is moving-target-ready; YAW CAL is the first gap), the prep plan/work-items, and the residual-cycle findings that interact with target-tracking velocity."
metadata:
  node_type: memory
  type: project
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**2026-06-30 HANDOFF — switching from STATIONARY ArUco to MOVING-target (rover) landing.**
Written at the end of the stationary residual-cycle session so a fresh chat can pick up the
moving-target phase. (Rover = `rover` world, airframe 4022 per CLAUDE.md; stationary = `aruco`,
4014. Manual rover launch in tips.txt; survey the exact infra fresh — see "Infra" below.)

## Carried-over BAKED state (from this session — all GT-FB, stationary)
- **W_U_MAX 1.0→2.0** (controller.py:2188+1781) — the 1.0 clamp's discontinuity SEEDED the terminal
  limit cycle; 2.0 (above where the cmd lands) removes the seeding. [[project_residual_cycle_wumax_bake]]
- **VDS_KF_Q 10→1** (controller.py:405) — smooths `s_dot_meas` (the drift's bearing-rate input, the
  dominant a_u oscillation driver) → drift-osc 4.7→0.9, SP 3→6/9.
- **Z_REG=0.2** (gt_feedback.py:127) — the gear-floor fix (1/z bounded). [[feedback_zreg_gear_floor_artifact]]
- Base config (IC2/4/5 n=3, GT-FB): **6/9 SP, 0 fly**. P2INF_xy=1.5 is an OPEN env candidate (stronger
  ζ_h damping but soft↔precise tradeoff; NOT baked — runs were env P2INF_X/Y=1.5).
- DEAD-ENDS this session (don't re-try): DHD smoothing (DERIV_KF_Q down→fly-aways, up→more c-term osc;
  reference-lag), flow-`h` smoothing (GT_FB_TAU up→regress; `h` is the primary damping/loom signal,
  lags). Only `s_dot_meas` (VDS) was cleanly smoothable. E_xy 0.7/0.5 ≈ neutral (re-test of the OLD
  "κ hurts X/Y" finding, now un-pinned — see skill).

## gt_feedback IS moving-target-ready (verified)
`gt_feedback.py` computes the RELATIVE pose/velocity from BOTH poses: `W_x_tu = tpp − up`
(target−UAV, line 100) and `W_v_tu = LS-slope of W_x_tu` (line 146). So the flow `h = V_v/(z+Z_REG)`
NATURALLY includes the target's motion, and driving `h_e→0` means the UAV TRACKS the target (IBVS's
structural strength for moving targets). No gt_feedback change needed for translation.

## Controller readiness — the GAPS for a moving target
1. **YAW CAL = the first work item.** `alpha`/`s[3]` is UNCALIBRATED (`cal_s[3]=1.0`) and INERT for a
   stationary square marker — but a moving/turning target makes the relative yaw ACTIVE → the yaw
   feature must be calibrated. The yaw SMC path (moment-alpha κ_a, [[feedback_moment_yaw_canonical]])
   is tuned; the CAL is pending ([[project_yaw_calibration_pending]]). Do this FIRST.
2. **CTRL_ZERO_WXY=1** zeros the target-relative ω_xy — fine for a TRANSLATING rover; revisit if the
   rover ROTATES (relative ω_xy then non-zero). ([[feedback_wxy_unobservable_imu_fusion]] — w_xy
   observable but uncalibrated → kept zeroed for stationary.)
3. **Commit/touchdown must stay CLOSED-LOOP until contact.** The open-loop LANDING_COMMIT is INVALID
   for a moving target (already default-off, rejected). TERMINAL_COMMIT (s_e_n→0 ramp, baked ON) is
   OK if closed-loop ("moving-target-OK, closed-loop until contact" per [[feedback_terminal_kick_commit_vs_live]]).
4. **Eval works unchanged:** `SoftPrecise` already uses RELATIVE xy/rel_vel (UAV−target), so rel_vel→0
   = matched to the rover. No eval change.

## The NEW control challenge (and why the session's findings matter MORE)
Lateral TRACKING (keep pace with the rover while descending) adds a **baseline lateral velocity = the
target speed**. This stacks on the residual-cycle work: the "entry velocity" (the deterministic IC5r2
driver — high entry → un-deliverable κ_eq spike → terminal cycle) now INCLUDES the rover speed. So the
faster the rover, the more the terminal cycle / κ-deliverability is stressed. The session's levers
(W_U_MAX=2.0, q=1, arrest-v_lat-early / descent-pacing, the entry-velocity sensitivity) carry straight
over and likely bind harder. Expect the cycle to be the binding issue at rover speed too.

## Infra (SURVEYED 2026-06-30 — concrete)
- **No rover launch script exists** — fork `scripts/run_aruco_landing.sh` (it's stationary-only,
  hard-codes `world/aruco`, single PX4 instance at `:101-104,129-135`). The rover procedure is a
  manual multi-terminal one in `tips.txt:60-105`: **TWO PX4 instances** — rover target
  (`PX4_SYS_AUTOSTART=4022 PX4_SIM_MODEL=rover_aruco PX4_GZ_WORLD=rover -i 1`) + UAV
  (`4014 x500_mono_cam_down PX4_GZ_WORLD=rover -i 0`).
- **World `rover.sdf` is empty** (ground+sun); both vehicles spawned by PX4. Airframe 4022 =
  `4022_gz_rover_aruco` (Ackermann rover). Model `rover_aruco` = `rover_ackermann` + `arucotag`
  marker mounted 0.5 m above the rover (`~/.gazebo/models/rover_aruco/model.sdf`).
- **Pose bridge DIFFERS**: rover bridges `/world/rover/dynamic_pose/info` (moving models only)
  vs the stationary `/world/aruco/pose/info` → **different PoseArray membership/ordering**.
- **⚠️ LANDMINE — `gz_subscriber.py:84-85` HARD-CODES pose indices** `UAV=poses[2]`, `target=poses[1]`
  for the aruco PoseArray. The rover `dynamic_pose` has different ordering → these WILL be wrong →
  silently corrupts BOTH perception-NED AND GT-feedback (`controller.py:973-975` feeds them to
  gt_feedback). **Fix these indices FIRST for the rover world.**
- **Rover motion**: it's a full PX4 Ackermann rover on its own SITL instance (`-i 1`), driven by
  gz joint-controller plugins. **NO trajectory plugin, NO speed knob today** — motion must be
  commanded externally (QGC mission / MAVLink offboard / manual) on `-i 1`. Building a guidance/
  speed source is part of the test harness.
- **`PLASMC_HD_FUNNEL_REF` (controller.py:884-890) is the labeled "moving-target candidate"** knob
  (h_d x/y rate = funnel ref, un-degenerates ζ_h; default-OFF, currently un-baked to the stationary
  `s_dot_meas` config). Revisit for moving.
- **Yaw caveats confirmed**: `gt_feedback.py:153` `w_z=−alpha_dot` is a STATIONARY-target sign
  assumption (breaks if the rover YAWS/turns); the alpha filters (`controller.py:343-364`,
  YAW_ALPHA_FILT, **max-rate cap 0.30 rad/s**) could reject a fast-turning rover as "corruption".
  Straight-driving rover = fine; turning rover = needs these revisited.
- No moving-target test code or recordings exist anywhere.

## Prep plan / first steps for the new chat
2. **Build a moving-target IC/landing test** (analogue of `run_ic_validation.sh` for the rover world;
   the rover moving; pick a speed range). SoftPrecise eval is already relative.
3. **Yaw calibration** for the moving/turning target (the moment-alpha cal; cal_s[3]).
4. **Run the carried-over baked config** (W_U_MAX=2.0, q=1, Z_REG=0.2) on the rover as the baseline,
   then tune the tracking + terminal cycle at rover speed.
5. Keep GT-FB for control-isolation first (gt_feedback already handles the moving target), then
   perception (mind the flow under-report 0.79×→0.48× speed-dependent = dynamic-range, NOT recalibratable).
