#!/usr/bin/env python3
"""
Rover target trajectory generator — planar (x, y, yaw) port of the MATLAB
`MATLAB/Common/traj_Gen.m` used to drive the moving landing target.

The MATLAB generator produces full 6-DOF ship-deck motion (surge/sway + heave
z + roll/pitch/yaw). A ground rover (Ackermann) is planar and nonholonomic, so
only the horizontal position p_xy(t) and the heading are realizable; the
z-heave and roll/pitch oscillations (which model a ship deck for the MATLAB
sim) are dropped here.

Frame: NED (x = North, y = East), matching the MAVSDK offboard PositionNedYaw /
VelocityNedYaw setpoints. p[0] <- MATLAB p_x, p[1] <- MATLAB p_y.

Constants are copied verbatim from traj_Gen.m (as of 2026-07-01) so the SITL
rover follows the same paths the MATLAB campaign used. `speed_mult` scales speed
exactly as the MATLAB `speed_mult` argument.

Yaw convention: for trajectory types whose MATLAB spec pins yaw (Circular,
CircularYaw) we reproduce that yaw. For the others (Linear, EightShape,
Sinusoidal, Lissajous) the MATLAB target keeps yaw = 0, but an Ackermann car
must physically point along its motion, so we command the path-tangent heading
atan2(v_E, v_N). Use `yaw_mode` to override.
"""

import math
import os
from dataclasses import dataclass

TRAJECTORY_TYPES = (
    "Static", "Linear", "Circular", "EightShape",
    "Sinusoidal", "Lissajous", "CircularYaw",
)


@dataclass
class TrajState:
    """Planar target state at a time t (NED)."""
    x: float          # North position [m]
    y: float          # East position [m]
    vx: float         # North velocity [m/s]
    vy: float         # East velocity [m/s]
    yaw: float        # heading [rad], NED (0 = North, +CW toward East)
    yaw_rate: float   # heading rate [rad/s]

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


def _tangent_yaw(vx, vy, prev_yaw):
    """Path-tangent heading; hold previous heading when nearly stopped."""
    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        return prev_yaw
    return math.atan2(vy, vx)


def eval_traj(t, traj_type="Circular", speed_mult=1.0, yaw_mode="spec",
              prev_yaw=0.0):
    """
    Evaluate the planar target trajectory at time t.

    yaw_mode:
      "spec"    -> MATLAB-specified yaw where defined, else path-tangent.
      "tangent" -> always path-tangent heading (natural for a car).
      "zero"    -> always 0 (target faces North).
    """
    if traj_type not in TRAJECTORY_TYPES:
        raise ValueError(f"Unknown trajectory type: {traj_type!r}; "
                         f"choose from {TRAJECTORY_TYPES}")

    spec_yaw = None          # None => use tangent/zero fallback
    spec_yaw_rate = None

    if traj_type == "Static":
        x = y = vx = vy = 0.0

    elif traj_type == "Linear":
        # Ship deck moving at fixed forward speed (x=y ramp). Heave dropped.
        s = 1.1 * speed_mult
        x, y = s * t, s * t
        vx, vy = s, s
        spec_yaw, spec_yaw_rate = 0.0, 0.0   # MATLAB psi = 0

    elif traj_type == "Circular":
        # r=10 m (2026-07-03, user): the small-radius circle (0.8 m, ~Ackermann min
        # turn radius) made the TARGET motion jerky (steering saturated) — not a fair
        # moving-target test. A large radius gives smooth target motion. The tangential
        # speed is PRESERVED (v = r*wz held at the old 0.384*speed_mult) by scaling wz
        # down, so the speed envelope is unchanged; only the path curvature drops ~12x.
        # Both r and tangential speed are env-tunable.
        # REDUCED 10->3 m (2026-09-22, user request, for a visibly tighter curve on video):
        # still well clear of the rejected r=0.8m jerky zone (Ackermann min turn radius
        # ~0.56m). SITL-validated at r=3: landed SoftPrecise, xy=0.097m/vel=0.380m/s
        # (test_data/Final/Circular). REDUCED FURTHER 3->1.5 m (same day, user request):
        # still ~1.9x the rejected r=0.8m radius; v_tan unchanged so wz scales up
        # correspondingly (path curvature ~6.7x tighter than r=10). NOT yet SITL-validated
        # at r=1.5.
        r = float(os.environ.get("ROVER_CIRCLE_R", "1.5"))
        v_tan = float(os.environ.get("ROVER_CIRCLE_VTAN", "0.384")) * speed_mult
        wz = v_tan / r
        x = -r * (math.cos(wz * t) - 1.0)
        y = r * math.sin(wz * t)
        vx = r * wz * math.sin(wz * t)
        vy = r * wz * math.cos(wz * t)
        spec_yaw, spec_yaw_rate = wz * t, wz   # MATLAB psi = wz*t

    elif traj_type == "EightShape":
        # LOOP RADIUS a=5 (2026-07-03, user): at the native a=1/w0=0.3 the figure-eight's
        # velocity REVERSES mid-descent (~t=2.6 s y, 5.2 s x); the drone velocity-matches with
        # lag, overshoots the reversal, and the lateral loop runs away (28-33 m miss). A LARGER
        # loop radius makes the turns gentler AND (with the tangential speed preserved by
        # w0=v_tan/a) pushes the reversals LATER — a>=~5 moves them past the ~9 s descent so it
        # stays trackable. Same fix as the Circular r bump. Both env-tunable.
        a = float(os.environ.get("ROVER_EIGHT_A", "5.0"))
        v_tan = float(os.environ.get("ROVER_EIGHT_VTAN", "0.4")) * speed_mult
        w0 = v_tan / a
        x = a * math.sin(w0 * t)
        y = a * math.sin(w0 * t) * math.cos(w0 * t)
        vx = a * w0 * math.cos(w0 * t)
        vy = a * w0 * math.cos(2.0 * w0 * t)

    elif traj_type == "Sinusoidal":
        A = 0.5
        w0 = 0.8 * speed_mult
        v0 = 0.5 * speed_mult
        x = A * math.sin(w0 * t)
        y = v0 * t
        vx = A * w0 * math.cos(w0 * t)
        vy = v0

    elif traj_type == "Lissajous":
        # RE-PARAMETERIZED for Ackermann feasibility (2026-09-22, same spirit as the
        # EightShape/Circular radius bumps above). The MATLAB spec (A=0.4,B=0.8,w1=-0.5,
        # w2=0.85, no phase) has NO fundamental-limitation fix via amplitude or uniform-speed
        # scaling: for a bare x=A sin(w1 t), y=B sin(w2 t) curve, curvature radius is a PURE
        # GEOMETRY property (invariant to scaling w1,w2 by a common factor -- that only changes
        # how fast the same shape is traced) and the un-phased curve has moments where vx and
        # vy hit zero near-simultaneously -- a near-cusp with curvature radius ~0.0006 m,
        # independent of amplitude, confirmed by direct numerical curvature search (see
        # Memory/px4/... 2026-09-22 Lissajous investigation). A relative PHASE OFFSET between
        # the two axes removes that coincidence (breaks the simultaneous-zero-velocity
        # symmetry); w2 was also re-picked (0.85->0.475) for a wider curvature margin. With
        # amplitude then scaled 4x (0.4/0.8 -> 1.6/3.2) the minimum curvature radius over a
        # realistic ~15-20s descent window is ~0.85 m, a 1.5x margin over the Ackermann rover's
        # ~0.56m physical minimum turn radius (vs ~0.0006m / 900x infeasible before). This is a
        # SHAPE deviation from the manuscript's literal Lissajous (larger, phase-correlated,
        # rounder) -- not a drop-in match -- by user decision (chose "scale amplitude,
        # same spirit as EightShape/Circular" once told plain amplitude scaling alone doesn't
        # work; phase+frequency change was the actual fix, amplitude tops up the margin).
        #
        # SPEED RE-TUNE (2026-09-22, same day, follow-up): the cusp fix above is geometrically
        # sound (curvature radius IS scale-invariant under uniformly rescaling w1/w2 -- verified
        # numerically, matches the comment above) but the w1=-0.5/w2=0.475 defaults left the
        # commanded speed at median 1.21 m/s (max 1.54) -- 2-3x the ~0.4-0.6 m/s envelope that is
        # the only regime anything has actually landed in this session (Circular/EightShape/
        # Sinusoidal all succeed or get close there; Linear fails outright at 1.6-2.2 m/s in a
        # dead-straight line, no curvature involved at all -- speed alone is sufficient to fail
        # this controller). Live reps at the untuned speed (test_data/RecordGTFB_dev/
        # Lissajous_slow) failed 7/7, one with a_u_xy hitting 1e5 (terminal blow-up). Rescaling
        # BOTH w1 and w2 by a common factor k retraces the IDENTICAL path (same curvature radius
        # everywhere, same 0.85 m margin) just slower -- pure speed control, zero effect on the
        # cusp fix.
        #
        # FURTHER SLOWED (2026-09-22, same day, 2nd follow-up): k=0.4 (median 0.52 m/s) was
        # SITL-tested (4 reps, test_data/RecordGTFB_dev/Lissajous_final) -- 3/4 failed badly
        # (xy 2.4-6.0m), 1/4 landed close (xy=0.36m). Checked what actually differed between
        # the good and bad reps: NOT target excursion (the successful rep had the LARGEST
        # excursion, 2.91m, of all four -- rules out "sweep too wide" as the driver) but
        # `vis_active` (visibility-CBF engagement fraction): 63%/51%/27% on the three failures
        # vs 3.4% on the success, with a_u_xy blowing up hardest (90-94k) whenever vis_active
        # was high. GT-FB does NOT feed the CBF (marker_center_px still comes from live
        # perception, confirmed in controller.py/visibility_projection.py) -- so target speed
        # still matters here via HOW OFTEN the marker nears the FoV edge and triggers the CBF,
        # which then appears to fight the lateral authority needed to keep tracking. k=0.4 is a
        # marginal/borderline speed, not a working one. k=0.2 (w1=-0.1, w2=0.095) brings median
        # speed to ~0.26 m/s, max ~0.31 m/s -- well under the borderline point, closer to
        # CircularYaw's ~0.26-0.28 m/s (the slowest profile tested this session).
        #
        # FURTHER SLOWED AGAIN (2026-09-22, user request): k=0.2 was SITL-tested (4 reps,
        # test_data/RecordGTFB_dev/Lissajous_final reps 5-8) -- still 0/4 SP, xy=[1.90,1.71,
        # 0.21,3.45]m, not a clean improvement over k=0.4's [6.0,2.4,0.36,2.8]m on 3/4 runs --
        # consistent with the vis_active/CBF finding above (speed isn't the whole story), but
        # user asked for w1/w2 reduced further regardless. k=0.1 (w1=-0.05, w2=0.0475) brings
        # median speed to ~0.14 m/s, max ~0.15 m/s.
        #
        # RESOLVED (2026-09-22, same day): k=0.1 SITL-validated -- genuine SOFT+PRECISE,
        # xy=0.0128m, rel_vel=0.120m/s (clears even the manuscript's strict 0.08m/0.2m/s
        # thresholds, not just this harness's relaxed gate). Promoted to
        # test_data/Final/Lissajous/. Going slow enough converts to a clean landing, even
        # though the vis_active/CBF correlation and the t~2.7s thrust-cannibalization
        # divergence trigger found at k=0.2 (Memory/px4/project_20260922_lissajous_cbf_and_
        # divergence_mechanism.md) are both likely still-real mechanisms that this speed
        # simply never triggers. Don't read k=0.1's success as proof that mechanism is fixed
        # -- it's a per-profile speed workaround, not a controller-side fix. All four knobs
        # are env-tunable for a future re-tune.
        #
        # ⚠ CORRECTION -- COMMANDED SPEED != TARGET SPEED (2026-09-22, GT + rover ulog): the
        # Ackermann rover does NOT track these slow position setpoints. It parks ~0.5 m off the
        # setpoint, then when the error grows drives a full re-approach LOOP at 1.5-2.2 m/s
        # (stays in offboard, no failsafe). Every Lissajous_final rep (k=0.4/0.2/0.1, 11 reps)
        # has GT target peaks of 1.6-2.8 m/s; at k=0.1 the setpoint moves 0.15 m/s but the GT
        # target is either parked (0 m/s) or looping at ~2 m/s. So the speed conclusions above
        # are about the COMMAND, not what the drone saw, and the "RESOLVED" k=0.1 rep (23-16-02,
        # xy=0.013 m) landed on a rover PARKED for its final 6.5 s (it looped at up to 1.6 m/s
        # only in t=0-3.5 s) -- effectively a static landing, not Lissajous tracking. Lowering
        # w1/w2 does not slow the target, it only changes how often the loops happen. Fix is on
        # the rover-drive side (velocity/feedforward setpoints, or speeds above the rover's
        # minimum controllable speed); always verify with each rep's Ground_Truth.npy target
        # speed. (Memory/px4/project_20260922_ackermann_rover_loops_not_tracking.md)
        #
        # SPEED FIX 2026-09-23: rover_drive.py's ROVER_CTRL=vel mode now tracks the target
        # velocity directly (see its own docstring), so speed_mult finally controls what the
        # drone actually sees, not just the setpoint. That unblocked a genuine 10x net-speed
        # request (ROVER_SPEED_MULT=10 -> GT target median 1.2 m/s, max ~1.7 m/s) -- but the
        # DRONE now tumbles (tilt 10deg->155deg over t=0-5s, xy_err=3.79m) even though rover
        # tracking itself stays tight (<=0.08m). Curvature radius here is a PURE GEOMETRY
        # property of A,B alone (see the note above) -- it does NOT change with speed_mult,
        # so the ~0.85m radius was already fixed at every speed tested (1x-10x); this rules out
        # "the turn is too sharp for this speed" as the geometric mechanism (radius unaffected
        # by speed), and is consistent with the drone-side mechanism already on file
        # (Memory/px4/project_20260922_lissajous_cbf_and_divergence_mechanism.md: I_a_z
        # cannibalized by I_a_xy, driven by SUSTAINED lateral speed, not by curvature alone).
        # RADIUS_MULT nonetheless eases lateral ACCELERATION (v^2/R) at fixed speed and is
        # cheap to try (user request, 2026-09-23): scaling A,B by m and w1,w2 (pre-speed_mult)
        # by 1/m retraces a GEOMETRICALLY LARGER path (radius x m) at the SAME net speed (v =
        # A*w is invariant under this joint rescaling -- verified: kappa ~ B/A^2 depends only
        # on the A,B ratio, cancels w entirely, matching the "curvature is pure geometry"
        # derivation above). Default 1.0 = unchanged behavior.
        RADIUS_MULT = float(os.environ.get("ROVER_LISS_RADIUS_MULT", "1.0"))
        A = float(os.environ.get("ROVER_LISS_A", "1.6")) * RADIUS_MULT
        B = float(os.environ.get("ROVER_LISS_B", "3.2")) * RADIUS_MULT
        w1 = float(os.environ.get("ROVER_LISS_W1", "-0.05")) / RADIUS_MULT * speed_mult
        w2 = float(os.environ.get("ROVER_LISS_W2", "0.0475")) / RADIUS_MULT * speed_mult
        phi = math.radians(float(os.environ.get("ROVER_LISS_PHI_DEG", "104.0")))
        x = A * math.sin(w1 * t + phi)
        y = B * math.sin(w2 * t)
        vx = A * w1 * math.cos(w1 * t + phi)
        vy = B * w2 * math.cos(w2 * t)

    elif traj_type == "CircularYaw":
        r = 1.0
        w_tr = 0.2
        w_yaw = 0.4
        x = r * (math.cos(w_tr * t) - 1.0)
        y = r * math.sin(w_tr * t)
        vx = -r * w_tr * math.sin(w_tr * t)
        vy = r * w_tr * math.cos(w_tr * t)
        spec_yaw, spec_yaw_rate = w_yaw * t, w_yaw   # independent yaw spin

    # Resolve yaw per mode.
    if yaw_mode == "zero":
        yaw, yaw_rate = 0.0, 0.0
    elif yaw_mode == "tangent":
        yaw, yaw_rate = _tangent_yaw(vx, vy, prev_yaw), 0.0
    else:  # "spec"
        if spec_yaw is not None:
            yaw, yaw_rate = spec_yaw, spec_yaw_rate
        else:
            yaw, yaw_rate = _tangent_yaw(vx, vy, prev_yaw), 0.0

    return TrajState(x=x, y=y, vx=vx, vy=vy, yaw=yaw, yaw_rate=yaw_rate)


def deck_state(t, traj_type):
    """Ship-deck platform motion (manuscript Cases 2 & 5; MATLAB traj_Gen Linear/Circular):
    heave A_z sin(w_z t) m, roll 15deg sin(0.9 t), pitch 8deg sin(0.6 t + pi/3).
    Returns (heave_m, roll_rad, pitch_rad); zeros for the other trajectories.

    HEAVE FREQUENCY REDUCED (2026-09-22, user): manuscript spec w_z=0.5 rad/s (period ~12.6s)
    -> 0.25 rad/s tried first, user said still too fast -> 0.125 rad/s (period ~50s, quarter
    of spec). Amplitude UNCHANGED at 0.2m (user clarified the problem was frequency, not
    amplitude, after an initial amplitude-reduction ask was superseded). Both env-tunable.
    """
    if traj_type not in ("Linear", "Circular"):
        return 0.0, 0.0, 0.0
    A_z = float(os.environ.get("DECK_HEAVE_AMP", "0.2"))
    w_z = float(os.environ.get("DECK_HEAVE_W", "0.125"))
    return (A_z * math.sin(w_z * t),
            math.radians(15.0) * math.sin(0.9 * t),
            math.radians(8.0) * math.sin(0.6 * t + math.pi / 3.0))


if __name__ == "__main__":
    # Quick self-check: print a few samples of each trajectory.
    for tt in TRAJECTORY_TYPES:
        s0 = eval_traj(0.0, tt)
        s5 = eval_traj(5.0, tt)
        print(f"{tt:12s} t=0: ({s0.x:+.3f},{s0.y:+.3f}) v={s0.speed:.3f} "
              f"yaw={math.degrees(s0.yaw):+.1f}deg | "
              f"t=5: ({s5.x:+.3f},{s5.y:+.3f}) v={s5.speed:.3f}")
