---
name: project_20260922_ackermann_rover_loops_not_tracking
description: "2026-09-22: at slow position setpoints (~0.15 m/s Lissajous) the Ackermann rover does NOT track -- it parks ~0.5 m off then drives re-approach loops at 1.5-2.2 m/s. All 11 Lissajous_final reps have GT target peaks 1.6-2.8 m/s. The k=0.1 'SOFT+PRECISE' (23-16-02) landed on a rover parked for its final 6.5 s. Invalidates the Lissajous speed-retune conclusions."
metadata:
  node_type: memory
  type: project
  originSessionId: eb3af863-38fc-4d7b-86dd-b7c9bc16b44d
  modified: 2026-09-22T17:56:52.860Z
---

**Finding (2026-09-22, from GT `Target Pose` + the rover's PX4 ulog `rootfs/1/log/2026-09-22/17_36_42.ulg`).**
The Ackermann rover under `rover_drive.py` position setpoints (`set_position_ned`, 5 Hz in the
ulog) cannot follow a setpoint moving at ~0.15 m/s. It stays in offboard (nav_state 14, no
failsafe) but alternates: **parked ~0.46 m from the setpoint → once error grows, a full loop at
1.5-2.2 m/s → parked again**. It already does this every ~6 s during the pre-gate "holding start
pos" phase (the hold point is a fixed setpoint it still cannot settle on).

**Evidence, rep `test_data/RecordGTFB_dev/Lissajous_final/Tue Sep 22 23-07-55 2026`**
(= `Test_Videos/chase_2026-09-22_23-06-44.mp4`, identical md5): commanded 0.150 m/s median;
GT target 0.00 m/s for t=0-3.8 s, then a U-turn (heading 165°→0°, radius ~1 m) peaking **2.17 m/s
at t=5.3 s**; rover EKF speed matches GT (max 2.26). Drone failed (xy 2.65 m, rel_vel 4.3 m/s).
All 11 Lissajous_final reps (k=0.4/0.2/0.1): GT peak 1.6-2.8 m/s.

**Consequences:**
- The k=0.4→0.2→0.1 "speed retunes" in [[project_20260922_lissajous_cbf_and_divergence_mechanism]]
  changed the COMMAND only; the drone never saw a slow Lissajous target.
- The k=0.1 SOFT+PRECISE rep (`23-16-02`, xy=0.013 m) — the rover looped at up to 1.6 m/s for
  t=0-3.5 s, then was **parked for the final 6.5 s** until touchdown. That is effectively a static
  landing, not "Lissajous solved".
- Likely also the "unresolved ~3x Lissajous speed-tracking gap" noted in
  [[project_20260922_rover_drive_wallclock_pacing_bug]] — a different mechanism from the clock bug.
  Other slow profiles (CircularYaw ~0.26 m/s, etc.) may be affected too — not yet checked.

**How to apply:** never quote a rover-profile speed from `rover_trajectory.py`/`ROVER_SPEED_MULT`;
compute it from each rep's `Ground_Truth.npy` `Target Pose` (dedupe stale samples, 0.2-0.5 s
window). Candidate fix (untested): velocity/feedforward setpoints in `rover_drive.py`, or keep
commanded speed above the rover's minimum controllable speed. pyulog isn't in env2025 — use a
scratch venv to read the rover ulog.

**Root cause (verified in PX4 source + rover ulog `rootfs/1/log/2026-09-22/17_44_49.ulg`, rep 23-16-02).**
`rover_drive.py` sends `set_position_ned` only → PX4 `AckermannPosVelControl::offboardPositionMode()`
(`~/PX4-Autopilot/src/modules/rover_ackermann/AckermannPosVelControl/AckermannPosVelControl.cpp`):
- `distance_to_target <= NAV_ACC_RAD` (=0.5 m) → speed setpoint **0** (the park). The trajectory's
  velocity and yaw fields are ignored in this mode (it treats every setpoint as an arrival point).
- beyond 0.5 m → speed = `computeMaxSpeedFromDistance(RO_JERK_LIM=15, RO_DECEL_LIM=6, d)` (≈0.59 m/s
  at 0.5 m, 1.12 at 1.0, 1.42 at 1.3), heading = pure-pursuit toward the point.
- By the time a 0.15 m/s setpoint leaves the 0.5 m circle it is beside/behind the parked rover
  (relative bearing −110° to +150°); min turn radius = 0.321/tan(30°) = 0.556 m, so the rover must
  circle, distance grows while it turns, speed rises (d=1.3 m → 1.56 m/s) → the ~2 m/s loop.
- 23-16-02: stopped t=3.5 s when d fell to 0.48; the setpoint then drifted THROUGH the 0.5 m circle
  (d 0.50→0.16→0.48, bearing −64°→+145°), a ~1 m chord at 0.15 m/s ≈ 7 s parked, covering touchdown
  (~t=10.0); exited at t≈10.3 and was already doing 1.03 m/s at t=10.5.
**Fix direction:** stop sending bare position points — velocity setpoints with feedforward +
P-correction on position error (rover_drive has telemetry), or a lead/carrot point kept >0.5 m ahead
along the path. Just shrinking NAV_ACC_RAD won't stop the loops: a sideways point still forces a turn.

**FIX IMPLEMENTED 2026-09-23 — `apps/rover_drive.py` `ROVER_CTRL=vel` (default for Lissajous only; other
profiles still `pos` so peers' experiments aren't silently changed).** Offboard VELOCITY setpoints =
profile velocity (feedforward) + bounded position correction (`_track_cmd`: along-track speed
clamped to [0.5|v_ff|, ROVER_VEL_MAX=0.4], cross-track ≤ 0.5× along-track → never reverses heading,
so no Ackermann loops). Pre-gate hold = zero velocity (rover stays at spawn); at the gate the path is
anchored at the rover's position and rotated so its start tangent matches the rover heading
(`ROVER_VEL_ALIGN=1`; shape unchanged). First live rep (GT-FB, IC2,
`test_data/RecordGTFB_dev/Lissajous_velctrl/Wed Sep 23 00-05-47 2026`): GT target speed median
0.135, max 0.285 m/s (was 2.2), tracking error ≤0.13 m; landing xy=0.018 m, rel_vel=0.20 m/s
(precise; soft only under the harness's relaxed 0.5 m/s). Residual speed surging 0.05-0.27 m/s is
PX4's RO_SPEED_TH=0.1 m/s measurement deadband at these low speeds. n=1 — not yet repeated.
Caveat: at k=0.1 the ~10 s descent covers only ~1.5 m of a ~132 s-period curve, so on video the
target path looks nearly straight.

**FOLLOW-UP 2026-09-23 — `ROVER_SPEED_MULT=2` clears the low-speed creep-then-lurch residual.**
The first `vel`-mode rep (above) had GT target speed dipping to 0.045-0.08 m/s (below PX4's
`RO_SPEED_TH=0.1 m/s` rover-EKF deadband), causing a visible creep/catch-up cycle around t=5-7s
(tracking error briefly 0.30 m, drone yaw overshot to -23 deg) even though the loop mechanism
itself was fixed. At `ROVER_SPEED_MULT=2` (commanded ~0.3-0.6 m/s): GT target speed never drops
below 0.146 m/s (median 0.284, max 0.404); `rover_drive.py` tracking error stays <=0.11 m
throughout (vs 0.30 m at 1x); landing xy=0.015 m, rel_vel=0.22 m/s (precise, same as 1x) — visibly
smoother continuous motion in the chase video, no creep-then-lurch.
(`test_data/RecordGTFB_dev/Lissajous_velctrl_2x/Wed Sep 23 00-23-52 2026/`,
`Test_Videos/chase_2026-09-23_00-22-46.mp4`). n=1 for both configs — not yet repeated at either.
Recommend `ROVER_SPEED_MULT=2` (or higher) as the new Lissajous default over speed_mult=1 once
repeated; 1x is usable but has this residual creep artifact at the low end of the Lissajous cycle.

**FOLLOW-UP 2026-09-23 (same session) — net-speed 10x crashed the DRONE, not the rover; fixed
by widening the Lissajous turning radius, not by slowing down.** `ROVER_SPEED_MULT=10` (GT
target median 1.2 m/s, max ~1.7 m/s) gave clean rover tracking (err <=0.08 m, matching the
offline bicycle-model prediction) but the DRONE tumbled: xy_err=3.79 m, min_alt=0.08 m
(effectively a crash). ⚠ Root-cause correction: my first read used a raw-quaternion tilt metric
(2*acos(|w|)) that conflates YAW with actual tip-over; recomputed properly (angle of the body
z-axis from world vertical) the real tilt still clearly diverges (0.7 deg at t=0 -> 33 deg and
climbing at t=5s, when it hit the ground) -- a genuine, still-unresolved-at-1x-radius mechanism,
just not as extreme as the flawed 155 deg first reported. See `diagnose-flight-data` skill:
compute signals properly, don't reuse a metric that conflates two rotations.

**Fix (user-directed): widen the turning radius at fixed net speed, `ROVER_LISS_RADIUS_MULT`**
(new `rover_trajectory.py` knob, Lissajous only): scales amplitude A,B by m and the base
w1,w2 by 1/m before speed_mult is applied. Curvature is a pure function of A,B (independent
of w -- see the "curvature is pure geometry" note already on file), so this leaves NET SPEED
UNCHANGED (v=A*w invariant) while multiplying the minimum curvature radius by m: verified
offline, radius_mult=1/2/3/5 -> min radius (t<15s window) 0.85/1.70/2.55/4.35 m at essentially
the same commanded speed (median ~1.2, max ~1.53 m/s throughout).

`ROVER_SPEED_MULT=10 ROVER_LISS_RADIUS_MULT=3 ROVER_VEL_KP=3.0 ROVER_VEL_MAX=2.0` — LIVE
RESULT (GT-FB, IC2, `test_data/RecordGTFB_dev/Lissajous_velctrl_10x_r3/Wed Sep 23 00-43-14
2026/`, `Test_Videos/chase_2026-09-23_00-42-10.mp4`): true tilt stays 0.7-6.6 deg (vs 33 deg+
climbing at radius_mult=1), rover tracking error 0.02-0.05 m (even tighter than at radius_mult=1,
despite the ~8x bigger absolute excursion), landing PRECISE (xy=0.146 m) but not soft
(rel_vel=0.894 m/s vs 0.2 m/s target) -- touchdown speed is still high at 10x net speed, expected
and separate from the tilt/crash mechanism this fixes. n=1, not yet repeated; radius_mult=1/2
not tested live (only offline) so the exact margin needed is unbisected -- 3 was chosen directly
per user request for "a bigger radius", not as a minimal fix.

**Read as one finding, not two competing explanations:** curvature radius does NOT change with
speed_mult alone (verified: it's a fixed 0.85 m at 1x/2x/10x, a pure function of A,B), so "the
turn was too sharp for the speed" was never literally about the OLD radius scaling WITH speed --
but the sustained lateral ACCELERATION (v^2/R) at a fixed 0.85 m radius does scale as speed^2,
and that is what the wider radius directly relieves. Consistent with, and does not contradict,
the earlier-documented I_a_z/I_a_xy cannibalization mechanism -- it just shows lateral
acceleration (not raw speed alone) is a real lever on it, worth testing on other trajectories
(Circular/EightShape) that hit the same family of failures.

**FOLLOW-UP 2026-09-23 (same session) — chase-camera framing: `ROVER_VEL_ROT_DEG` knob.**
User noticed `chase_2026-09-23_00-42-10.mp4` (the radius_mult=3 fix above) shows the target
moving TOWARD the fixed chase camera (closing distance) rather than laterally, so it balloons
in apparent size and gets clipped near touchdown. Diagnosed by tracking the chase video's pixel
centroid/bbox (OpenCV threshold+contour) and fitting it against GT `Target Pose` world position:
`cx ~ 61.2*East -20.9*North + c`, `size ~ 26.0*East -85.5*North + c` (this run's ENU-ish
`Target Pose` fields; camera is `<pose>8 -8 3.5 0 0.250 2.216</pose>` in
`~/PX4-Autopilot/Tools/simulation/gz/worlds/rover_cross.sdf`, fixed, aimed at ~(2.7,2.5)). Net
motion that run was NED heading ~119 deg (mostly size growth, weak cx change); solving for the
direction with zero size-growth (pure lateral, constant camera distance) gives NED heading ~73
deg -- a -45.5 deg rotation offset from what VEL_ALIGN alone produces.

Added `ROVER_VEL_ROT_DEG` to `rover_drive.py` (additive on top of `ROVER_VEL_ALIGN`'s
heading-match rotation; 0 = old behavior). Live-tested at
`ROVER_VEL_ROT_DEG=-45.5` (same speed/radius config as above,
`test_data/RecordGTFB_dev/Lissajous_velctrl_10x_r3_rot/Wed Sep 23 00-52-50 2026/`,
`Test_Videos/chase_2026-09-23_00-51-46.mp4`): bbox area at touchdown ~3x its value 5s earlier
(was ~50-60x before this fix); pixel-tracked: NOT clipped by the right frame edge until the very
last 2 of 204 frames (~0.1s, at touchdown itself) -- vs clipped/oversized for several seconds
before. Landing unaffected (xy=0.159 m vs 0.146 m before, both precise-class, not soft --
control physics is orientation-invariant as expected, this knob only changes framing).
⚠ This rotation is specific to the CURRENT chase camera pose + this Lissajous shape/anchor;
re-derive with the same pixel-tracking method (see this entry) if either changes. n=1.
