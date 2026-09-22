---
name: project_20260922_ackermann_rover_loops_not_tracking
description: "⭐ FULL ARC 2026-09-22/23, read top-to-bottom (chronological, each entry supersedes/extends the last). ROOT CAUSE: PX4 Ackermann offboard POSITION mode parks within NAV_ACC_RAD=0.5m and loops at 1.5-2.2 m/s -- invalidated all prior Lissajous speed-retune conclusions (target speed != commanded speed). FIX: rover_drive.py ROVER_CTRL=vel (velocity-tracking, tracking err <=0.05m). Then found+fixed: (1) net-speed-x10 crashed the DRONE via lateral accel, fixed with ROVER_LISS_RADIUS_MULT (wider turns, same speed); (2) chase-camera framing (target closing on camera -> clipped) fixed with ROVER_VEL_ROT_DEG; (3) the Lissajous w1:w2 ratio was ~1:1 (an ELLIPSE, not a real figure) -- corrected to 2:3; (4) a v^2/R sweet spot (0.204, ROVER_SPEED_MULT=7/RADIUS_MULT=5) gives BOTH a precise landing (xy=0.111m) AND a visibly curving path -- promoted to test_data/Final/VISTA-GT/Lissajous/. ⚠ Two of my own hand-derived-formula bugs are documented and corrected inline (a period-formula 3x error, and a false 'exact cusp caused the crash' claim -- the cusp is real but never occurred within any tested descent window) -- read those corrections, don't stop at the first draft of either claim."
metadata:
  node_type: memory
  type: project
  originSessionId: eb3af863-38fc-4d7b-86dd-b7c9bc16b44d
  modified: 2026-09-22T23:04:08.472Z
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

**FOLLOW-UP 2026-09-23 (same session) — shape correction: w1:w2 ratio is ~1:1, so the "Lissajous"
profile traces an ELLIPSE, not a multi-lobed figure.** User watched `chase_2026-09-23_00-51-46.mp4`
and correctly flagged it doesn't look like a Lissajous figure. Confirmed mathematically: current
`ROVER_LISS_W1/W2` defaults (-0.05/0.0475, unchanged since the 2026-09-22 cusp/speed-retune
thread) give ratio -1.053:1 -- essentially 1:1. A 1:1-ratio Lissajous curve is a tilted ELLIPSE,
not the classic crossing/multi-lobed pattern (needs a small-integer ratio like 1:2 or 2:3 for
that). The manuscript's original spec (w1=-0.5,w2=0.85, ratio -0.59:1) was closer to that
character; the ratio drifted to ~1:1 as an unflagged side effect of the 09-22 retuning (phase/freq
changes targeted the cusp and speed, not the ratio). On top of that, any single ~10-15s descent
covers most but not all of one period (13.2s at speed_mult=10, 132s at speed_mult=1), so the video
only ever shows an ARC of that ellipse, never the closed oval -- a single smooth one-direction
sweep, exactly what was observed. NOT caused by today's RADIUS_MULT/ROT_DEG/SPEED_MULT work --
those only rescale amplitude/rate together or add a rigid rotation, neither touches the w1:w2
ratio. Unresolved: if a genuine multi-lobed Lissajous LOOK is wanted, the ratio needs to change
(e.g. toward 1:2), which reopens the cusp/curvature-margin analysis for the new ratio -- not done.

**FOLLOW-UP 2026-09-23 (same session) — genuine multi-lobed Lissajous, re-recorded.** Fixed the
w1:w2 ratio (see prior entry) to -2:3 (base `ROVER_LISS_W1=-0.095170 ROVER_LISS_W2=0.142755`,
`ROVER_LISS_PHI_DEG=30`, `ROVER_LISS_B=0.96` for B/A=0.6 -- phase and B/A amplitude ratio
grid-searched for the gentlest curvature at this frequency ratio, minimizing worst-case v^2/R
over one period). Confirmed genuinely non-elliptical: one full period is a self-crossing curve
(teardrop with a crossing point); a 10-15s descent shows a visible direction-reversal "S-hook",
not a monotonic one-direction sweep.

⚠ FIRST ATTEMPT CRASHED despite a LOWER worst-case v^2/R (0.695) than the proven-safe ellipse
case (~0.92): `ROVER_SPEED_MULT=10 ROVER_LISS_RADIUS_MULT=3` gave real tilt climbing past 28 deg
by t=6s (same signature as the earlier crash), rover tracking itself fine (<=0.14m). So v^2/R
(peak lateral acceleration) alone does NOT fully predict controller safety for this trajectory
family -- the DIRECTION-REVERSAL itself (this shape's yaw swings ~50->176->-141 deg over 8s, vs
the ellipse-arc's gentler ~66 deg over similar time) appears to be an independent stressor, not
captured by the curvature-radius metric alone. Worth flagging alongside the existing I_a_z/I_a_xy
cannibalization mechanism finding -- possibly the RATE of lateral direction change, not just its
peak magnitude.

**WORKING, SAFE config** (`ROVER_SPEED_MULT=6 ROVER_LISS_RADIUS_MULT=6 ROVER_VEL_KP=3.0
ROVER_VEL_MAX=2.0 ROVER_VEL_ROT_DEG=-45.5`, same B/A/W1/W2/PHI as above):
`test_data/RecordGTFB_dev/Lissajous_23ratio_slow/Wed Sep 23 01-07-45 2026/`,
`Test_Videos/chase_2026-09-23_01-06-39.mp4`. True tilt stayed <=3.3 deg the whole descent;
landing PRECISE (xy=0.072 m), not soft (rel_vel=0.805 m/s). Chase framing: reused the ellipse
case's `ROVER_VEL_ROT_DEG=-45.5` unchanged (camera-fit coefficients are pose-only, not
trajectory-shape-dependent) -- held up fine, ZERO clipped frames, bbox grew only modestly
(~19% over the last 3s) despite not being re-derived for this shape. n=1.

**Trade-off, same as noted for the ellipse case:** slowing down (both mults raised) to reach
safety also lengthens the period (T=39.6s -> reduces to a smaller fraction of one period per
descent -- this run's 10.5s window shows visibly LESS of the loop/hook than the failed faster
attempt would have). The recorded video shows a real, unambiguous direction-reversal, but not
(within one descent) the full closed self-crossing curve -- that would need either a longer
controlled descent or a faster/less-safe profile, which isn't available without a controller-side
fix (see the I_a_z/I_a_xy mechanism entry).

**FOLLOW-UP 2026-09-23 (same session, 2nd re-record) — user correctly flagged the first
"corrected" re-record still didn't look Lissajous.** Root cause: the safe config
(`speed_mult=6/radius_mult=6`) only showed 24% of the actual period (T=44s) in a ~10.5s
descent -- mostly a near-straight diagonal with one subtle bend, easy to mistake for non-
Lissajous motion. Also found along the way: **phi=30 deg (the phase used for this -2:3 ratio) is
an EXACT geometric cusp** (vx=vy=0 simultaneously; analytically, cusps recur at phi in
{30,90,150} deg mod 180 for THIS frequency ratio, independent of amplitude/aspect -- confirmed
both by solving the vx=vy=0 condition and by a high-resolution (n=40000) numerical curvature
scan; my EARLIER coarse phase search (2-3 deg steps, n=4000) under-resolved this exact point and
reported a falsely large Rmin there, which is how phi=30 got picked as "best" in the first place
-- same category of bug the original 2026-09-22 cusp thread already flagged once for the 1:1 case.
Despite that, the phi=30 config still worked live at low speed (tilt<=3.3-4.3 deg in both the
"safe" and "medium" reps below) -- so the cusp, while a real geometric defect, wasn't fatal at
these speeds; **overall speed/lateral-accel (v^2/R) remains the dominant live-tested driver**,
not exclusively the cusp. A cleaner phase (avoiding {30,90,150}) was NOT used in the end (time
budget) -- worth doing before this profile is used for anything beyond a demo video.

The FIRST FAILED live attempt at this ratio (`speed_mult=10/radius_mult=3`, v^2/R=0.695, real
tilt to 28+ deg, see the entry above) sits BETWEEN two configs that both later worked fine
(v^2/R=0.33 and v^2/R=0.88) -- so v^2/R alone is not a clean monotonic predictor either; n=3
live points is too few to fit a real threshold, treat any single v^2/R cutoff quoted here as
indicative only.

**Final config used for the re-record** (`ROVER_SPEED_MULT=8 ROVER_LISS_RADIUS_MULT=4`, same
B/A=0.6/W1/W2/PHI=30/KP=3.0/ROT_DEG=-45.5 as before, `ROVER_VEL_MAX=2.2`): v^2/R=0.88 (close to
the known-safe ellipse reference), period=22s, 10.5s descent covers ~48% of it.
`test_data/RecordGTFB_dev/Lissajous_23ratio_med/Wed Sep 23 01-16-51 2026/`,
`Test_Videos/chase_2026-09-23_01-15-46.mp4`. Result: real tilt stayed <=4.3 deg (no crash,
min_alt=0.51m normal touchdown) but landing classification FAIL (xy=0.160 m, just over the 0.15m
threshold; rel_vel=1.178 m/s, well over the 0.2 soft target) -- a genuine precision/speed cost
for showing more of the figure. Recorded GT path now shows a clear MULTI-LOBE S-wave (two visible
inflections), a qualitative improvement over the "safe" rep's single subtle bend -- confirmed
both in the GT plot and visually in the chase video (platform orientation visibly curves and
re-curves). Zero clipped frames (the -45.5 deg rotation still holds for this shape too).

**Net summary of the 3-rep progression this sub-thread:**
| config | v^2/R | period | %shown/10.5s | tilt | landing |
|---|---|---|---|---|---|
| sm=10,rm=3 | 0.695 | 13.2s | 80% | 28+deg, CRASHED | FAIL, min_alt=0.18 |
| sm=6,rm=6 | 0.330 | 44.0s | 24% | <=3.3deg | PRECISE (xy=0.072) |
| sm=8,rm=4 | 0.880 | 22.0s | 48% | <=4.3deg | FAIL (xy=0.160), not soft |

No single config is best on all axes; sm=8/rm=4 was kept as the final re-record because the user's
ask was specifically about LOOKING Lissajous, and the landing, while not precise, was NOT a crash
(min_alt=0.51m normal touchdown, just imprecise+fast). If future work needs BOTH a good landing
AND a clearly-Lissajous-looking path, the cusp-free-phase fix above is the next thing to try
before further mult tuning.

**CORRECTION 2026-09-23 (same session) -- the earlier "phi=30 is an exact cusp, likely the
crash cause" claim above is WRONG in its causal part (the cusp itself is real, confirmed
below, but it never occurred during any live test).** Two errors compounded:
1. **Unit/formula bug in the period math**: hand-derived `T=2*pi/k` with `k=2*pi/T_target`
   repeatedly conflated `w2` with `k` (since `w2=3k`), giving T values 3x too SMALL every time
   it was used (claimed T=44s/22s for the safe/medium configs; the CORRECT, `eval_traj`-verified
   formula is `T(rm,sm) = 132 * rm/sm` seconds, giving T=132s/66s respectively -- so "% of period
   shown in one ~10.5s descent" was also wrong throughout: actually ~8%/~16%, not ~24%/~48%).
   **Do not hand-derive `w1,w2,T` from `RADIUS_MULT`/`speed_mult` again -- extract them from
   `rover_trajectory.eval_traj` directly (position/velocity match) and verify numerically
   (measure the period from when the path returns to its start) before trusting any formula.**
2. **The v^2/R "medium config" value quoted to the user as 0.88 was also wrong** (same
   hand-derivation bug); the correct, `eval_traj`-cross-checked value is 0.333.

**The cusp is real but irrelevant to what happened.** Solving `vx=vy=0` exactly (analytically,
then numerically confirmed to 1e-6) for `phi=30 deg` at the rm=4/sm=8 operating point: BOTH
components hit exactly zero at t=27.5s and t=60.5s within the 66s period -- genuine cusps, exist
at phi in {30,90,150} mod 180 for this -2:3 ratio, independent of rm/sm (confirmed by the
analytic vx=vy=0 solve, matching the very first derivation). But every live descent in this
session only ran the first ~10.5s -- nowhere near t=27.5s. **The cusp never fired in any test
run to date; it did not cause the rm=3/sm=10 crash (that crash happened at t~6-7s).**

**Corrected, reliable picture (analytic curvature, `eval_traj`-verified, phi=30 unchanged
throughout -- no phase fix was actually needed):**
| config | v^2/R (correct) | outcome |
|---|---|---|
| rm=3,sm=10 | 0.694 | CRASHED (tilt 28+ deg) |
| rm=4,sm=8  | 0.333 | landed, NOT precise (xy=0.160m), fast (rel_vel=1.18) |
| **rm=5,sm=7** | **0.204** | **PRECISE (xy=0.111m), tilt<=3.5 deg, 0 clipped frames** |
| rm=6,sm=6 | 0.125 | PRECISE (xy=0.072m), least visible curve |

**Best config found: `ROVER_SPEED_MULT=7 ROVER_LISS_RADIUS_MULT=5`** (same
B=0.96/W1=-0.095170/W2=0.142755/PHI=30/KP=3.0/ROT_DEG=-45.5, VEL_MAX=2.0).
`test_data/RecordGTFB_dev/Lissajous_23ratio_v2r02/Wed Sep 23 04-07-09 2026/`,
`Test_Videos/chase_2026-09-23_04-06-05.mp4`. Precise landing, safe tilt, AND the recorded GT
path still shows a clear S-curve (comparable visual quality to the rm=4/sm=8 rep, better than
rm=6/sm=6's near-straight arc). This is the config to use going forward for this profile, not
the earlier "safe" (rm=6/sm=6) or "medium" (rm=4/sm=8) reps.

**CLOSING 2026-09-23 (same session) — promoted to `test_data/Final/VISTA-GT/Lissajous/`.**
The old rep there (`Tue Sep 22 23-16-02 2026`, xy=0.0128m, from the very first entry above) was
replaced -- it's the exact "parked rover" rep this whole thread started by diagnosing, so it was
never a valid Lissajous demo to begin with. Old set backed up to
`Obsolete/test_data/Lissajous_VISTA-GT_v1_parked_rover_20260923/` (not git-tracked, local only).
New promoted rep: `RecordGTFB_dev/Lissajous_23ratio_v2r02/Wed Sep 23 04-07-09 2026` (the
`v^2/R=0.204` config from the entry above). Regenerated `Lissajous_overlay_s_alpha.mp4`,
`Lissajous_overlay_h.mp4` (`tools/overlay_image_features.py --split`, note: `--out` MUST include
an extension, e.g. `Lissajous_overlay.mp4` -- passing an extension-less stem makes cv2.VideoWriter
silently fail via a `CvVideoWriter_Images` fallback, discovered live) and `Lissajous_montage.mp4`
(`tools/make_landing_montage.py --drone <s_alpha> --drone2 <h> --chase --run --out`); verified the
montage's 3D target-path plot shows a genuinely curving line. `test_data/Final/MANIFEST.md` updated
with a dated note. Committed: `627584d2`.

**Where things stand for future work:**
- Live-validated Lissajous config: `ROVER_CTRL=vel ROVER_SPEED_MULT=7 ROVER_LISS_RADIUS_MULT=5
  ROVER_LISS_B=0.96 ROVER_LISS_W1=-0.095170 ROVER_LISS_W2=0.142755 ROVER_LISS_PHI_DEG=30
  ROVER_VEL_KP=3.0 ROVER_VEL_MAX=2.0 ROVER_VEL_ROT_DEG=-45.5`. n=1, precise not soft
  (rel_vel=0.821 vs 0.2 target).
- `ROVER_CTRL=vel` and its knobs (`ROVER_VEL_KP/MAX/MIN_FRAC/LAT_FRAC/ALIGN/ROT_DEG`) default to
  legacy `pos` mode for every OTHER trajectory (`Static/Linear/Circular/EightShape/Sinusoidal/
  CircularYaw`) -- none of them have been re-tested under `vel` mode; the original park/loop bug
  this thread found is generic to ALL of them (any slow-enough moving profile), not Lissajous-
  specific, so the same fix likely helps there too if/when they're revisited.
  `ROVER_LISS_RADIUS_MULT`/`ROVER_LISS_W1/W2/PHI_DEG`/`ROVER_VEL_ROT_DEG` are Lissajous-shape-
  specific and don't carry over as-is.
- The exact cusps at phi in {30,90,150} deg (mod 180, for the -2:3 ratio) are UNFIXED -- the live
  config above uses phi=30 and is only safe because no tested descent reaches t=27.5s/60.5s where
  they occur. A LONGER descent (or a different IC/altitude giving more descent time) at this exact
  config WOULD eventually hit one. If Lissajous is ever flown for longer than ~20s, pick a
  cusp-free phase first (see the analytic vx=vy=0 method above) or re-verify the cusp timing is
  still safely out of range.
