#!/usr/bin/env python3
"""
Drive the SITL rover target along a MATLAB `traj_Gen.m` trajectory via MAVSDK
offboard POSITION setpoints, so the landing test has a repeatable moving target.

The rover is a PX4 Ackermann vehicle running on its own SITL instance (-i 1).
There is no built-in trajectory/speed source, so this streams offboard local-NED
position setpoints computed from `src/rover_trajectory.eval_traj`; PX4's
rover_ackermann position controller (pure-pursuit) handles the nonholonomic
steering toward each setpoint.

Run standalone (for bring-up / verification), with the two-instance rover stack
already up (see scripts/run_rover_landing.sh):

    ~/ws/scripts/env2025/bin/python3 apps/rover_drive.py

Or wire into the launcher via its ROVER_DRIVE hook.

Env config:
  ROVER_MAV_URL    MAVSDK connect URL for the rover instance (default udp://:14541).
                   PX4 SITL instance 1's offboard/GCS UDP. VERIFY per your build:
                   if connect times out, try udp://:14550 / the port printed by
                   the -i 1 PX4 console ("mavlink ... remote port").
  ROVER_TRAJ       Trajectory type (default Circular). One of: Static, Linear,
                   Circular, EightShape, Sinusoidal, Lissajous, CircularYaw.
  ROVER_SPEED_MULT Speed multiplier (default 1.0). Start slow — the rover's
                   baseline speed adds lateral velocity that stresses the
                   terminal-cycle / kappa-deliverability (see moving-target memo).
  ROVER_YAW_MODE   spec | tangent | zero (default spec).
  ROVER_RATE_HZ    Setpoint stream rate (default 20).
  ROVER_MAX_T      Optional stop time [s]; unset = run until killed.
  ROVER_GATE_FILE  If set, HOLD the start position (offboard active) until this
                   flag file appears, then begin the trajectory with t=0 at the
                   gate moment. The controller touches the same flag at
                   descent-start (CHASE_GATE_FILE mechanism), so the rover starts
                   moving exactly when the landing starts — the drone's arm/
                   takeoff/IC sequence (~60 s) happens over a stationary rover
                   (the IC rig doesn't have to chase a moving target).
  ROVER_GATE_TIMEOUT  Max seconds to wait for the gate (default 180).

CLOCK SOURCE (fixed 2026-09-22, project_20260922_rover_drive_wallclock_pacing_bug):
  The trajectory reference `t` fed to eval_traj() is now read from Gazebo's own
  simulated /clock (via gz_subscriber.Clock_Node, the SAME clock Ground_Truth.npy's
  "Time" field uses), NOT accumulated from asyncio.sleep()'s wall-clock ticks. The
  old wall-clock accumulation raced ahead of / lagged the true simulated world
  whenever Gazebo's real-time factor was not exactly 1.0 (headless / multi-SITL /
  loaded machine), so GT-measured rover speed diverged from the commanded
  ROVER_SPEED_MULT/formula value by up to ~3.5x in some profiles -- every rover
  speed number logged before this fix should be treated as uncontrolled (check the
  rep's own GT, don't trust the env vars). asyncio.sleep(dt) still paces the
  setpoint SEND rate (independent of which t value is evaluated).
"""

import os
import sys
import asyncio
import math
import subprocess

import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

from rover_trajectory import eval_traj, deck_state, TRAJECTORY_TYPES
from gz_subscriber import Clock_Node, GZ_Subscriber

MAV_URL = os.environ.get("ROVER_MAV_URL", "udp://:14541")
TRAJ = os.environ.get("ROVER_TRAJ", "Circular")
SPEED_MULT = float(os.environ.get("ROVER_SPEED_MULT", "1.0"))
YAW_MODE = os.environ.get("ROVER_YAW_MODE", "spec")
RATE_HZ = float(os.environ.get("ROVER_RATE_HZ", "20"))
MAX_T = os.environ.get("ROVER_MAX_T")
MAX_T = float(MAX_T) if MAX_T else None
# Ship-deck heave/roll/pitch (manuscript Cases 2/5): apps/deck_publisher.py, a subprocess
# publishing to the JOINTED rover_cross deck's JointPositionController topics (REAL physics the
# whole time). A teleported/kinematic standalone platform (tried 2026-09-22) never achieves
# genuine sustained collision contact in this Gazebo/ODE build -- confirmed by a bare-world
# falling-ball test (a static OR kinematic body launched a resting dynamic ball away on its first
# teleport, every time) -- so that approach was abandoned in favor of this one. Needs
# sim_models/rover_cross_deck_model.sdf installed as the live rover_cross model (see
# scripts/run_rover_landing.sh's DECK_MOTION handling). Default OFF; only started when
# deck_state(t, TRAJ) is actually nonzero for this trajectory (Linear/Circular).
DECK = os.environ.get("ROVER_DECK", "1") == "1"
GATE_FILE = os.environ.get("ROVER_GATE_FILE", "")
GATE_TIMEOUT = float(os.environ.get("ROVER_GATE_TIMEOUT", "180"))

# ROVER_CTRL (2026-09-23, Memory/px4/project_20260922_ackermann_rover_loops_not_tracking.md):
#   "pos" = stream the trajectory POSITION as offboard position setpoints (legacy). PX4's
#           Ackermann offboardPositionMode() treats each point as an ARRIVAL point: inside
#           NAV_ACC_RAD (0.5 m) it commands speed 0, outside it the speed is set from distance
#           only and the setpoint velocity is ignored. At slow profile speeds the rover parks,
#           the point drifts beside/behind it, and it drives a ~2 m/s re-approach loop.
#   "vel" = stream offboard VELOCITY setpoints = the profile's own velocity (feedforward) plus a
#           bounded position correction toward the reference, so the rover follows the path at
#           the profile's speed (see _track_cmd). The path is anchored at the rover's position at
#           the gate (the pre-gate hold is zero velocity, so it never drives to a start point).
# Default "vel" for Lissajous only; other profiles stay "pos" so their existing experiments
# are not silently changed -- set ROVER_CTRL=vel to opt in.
CTRL = os.environ.get("ROVER_CTRL", "vel" if TRAJ == "Lissajous" else "pos")
VEL_KP = float(os.environ.get("ROVER_VEL_KP", "0.5"))              # position-error gain [1/s]
VEL_MAX = float(os.environ.get("ROVER_VEL_MAX", "0.4"))            # hard speed cap [m/s]
VEL_MIN_FRAC = float(os.environ.get("ROVER_VEL_MIN_FRAC", "0.5"))  # along-track floor, x |v_ff|
VEL_LAT_FRAC = float(os.environ.get("ROVER_VEL_LAT_FRAC", "0.5"))  # |cross-track| <= frac x along
# vel mode: rotate the whole path (shape unchanged) about its start so its initial tangent matches
# the rover's heading at the gate. The rover spawns facing North while e.g. Lissajous starts
# heading ~83 deg East; without this the first ~15 s (the whole descent) are a turn-in transient
# of up to ~0.5 m instead of the profile. 0 = keep the profile's own world orientation.
VEL_ALIGN = os.environ.get("ROVER_VEL_ALIGN", "1") == "1"


async def _wait_connected(rover):
    print(f"[rover_drive] connecting to {MAV_URL} ...", flush=True)
    await rover.connect(system_address=MAV_URL)
    async for state in rover.core.connection_state():
        if state.is_connected:
            print("[rover_drive] rover connected.", flush=True)
            return
    # generator ends only on disconnect
    raise RuntimeError("rover connection state generator ended without connect")


async def _arm(rover):
    # A rover is armable quickly in SITL; poll health briefly then arm.
    print("[rover_drive] waiting for armable ...", flush=True)
    for _ in range(120):
        async for h in rover.telemetry.health():
            if h.is_armable:
                break
            else:
                await asyncio.sleep(0.5)
                break
        else:
            await asyncio.sleep(0.5)
            continue
        try:
            await rover.action.arm()
            print("[rover_drive] armed.", flush=True)
            return
        except Exception as e:
            print(f"[rover_drive] arm() retry ({e})", flush=True)
            await asyncio.sleep(0.5)
    raise RuntimeError("rover did not arm within timeout")


class _Ref:
    """Reference state after anchoring: p = anchor + R(rot) (p_traj(t) - p_traj(0)),
    v = R(rot) v_traj(t)."""
    def __init__(self, s, s0, anchor, rot):
        c, n = math.cos(rot), math.sin(rot)
        dx, dy = s.x - s0.x, s.y - s0.y
        self.x = anchor[0] + c * dx - n * dy
        self.y = anchor[1] + n * dx + c * dy
        self.vx = c * s.vx - n * s.vy
        self.vy = n * s.vx + c * s.vy
        self.speed = s.speed


def _track_cmd(s, pos):
    """Velocity command following reference s (+ anchor offset) from current NED position.

    Split into along-track / cross-track w.r.t. the reference velocity so the command never
    points backwards (a heading reversal is what forces an Ackermann loop): along-track speed is
    |v_ff| + KP*e_along, clamped to [VEL_MIN_FRAC*|v_ff|, VEL_MAX]; the cross-track correction
    is clamped to VEL_LAT_FRAC x the along-track speed (heading within ~27 deg of the tangent at
    the default 0.5). Returns (vn, ve)."""
    vff = math.hypot(s.vx, s.vy)
    if vff < 1e-6:
        return 0.0, 0.0
    tx, ty = s.vx / vff, s.vy / vff
    ex, ey = s.x - pos[0], s.y - pos[1]
    e_t = ex * tx + ey * ty
    e_n = -ex * ty + ey * tx
    u_t = min(max(vff + VEL_KP * e_t, VEL_MIN_FRAC * vff), VEL_MAX)
    lat = VEL_LAT_FRAC * u_t
    u_n = min(max(VEL_KP * e_n, -lat), lat)
    k = min(1.0, VEL_MAX / math.hypot(u_t, u_n))   # VEL_MAX caps the TOTAL speed
    return k * (u_t * tx - u_n * ty), k * (u_t * ty + u_n * tx)


async def _wait_for_sim_clock(time_node, timeout_s=10.0):
    """Block (async-friendly) until the first /clock message arrives, mirroring
    landing_test.py's own Clock_Node bring-up wait."""
    start = asyncio.get_event_loop().time()
    while time_node.perf_counter() is None:
        if asyncio.get_event_loop().time() - start > timeout_s:
            raise RuntimeError("rover_drive: unable to get simulation time from /clock")
        await asyncio.sleep(0.05)


async def run():
    if TRAJ not in TRAJECTORY_TYPES:
        raise SystemExit(f"ROVER_TRAJ={TRAJ!r} invalid; choose {TRAJECTORY_TYPES}")
    print(f"[rover_drive] traj={TRAJ} speed_mult={SPEED_MULT} yaw={YAW_MODE} "
          f"rate={RATE_HZ}Hz max_t={MAX_T}", flush=True)

    # Sim-clock source (fixed 2026-09-22): read Gazebo's /clock the same way
    # landing_test.py does, so the trajectory's `t` tracks true simulated elapsed
    # time instead of racing ahead of / lagging behind it via asyncio.sleep().
    rclpy.init()
    time_node = Clock_Node()
    clock_sub = GZ_Subscriber(time_node)
    await _wait_for_sim_clock(time_node)
    print("[rover_drive] sim clock acquired.", flush=True)

    try:
        await _run_with_clock(time_node)
    finally:
        clock_sub.close()
        if rclpy.ok():
            rclpy.shutdown()


async def _run_with_clock(time_node):
    # Dedicated mavsdk_server gRPC port: the default System() port is 50051,
    # which landing_test's FC also uses (its own embedded server). Sharing the
    # port makes the FC connect to THIS rover server (bound to udp 14541) ->
    # landing_test hangs waiting for the UAV on 14540. 50052 keeps them separate.
    rover = System(port=int(os.environ.get("ROVER_MAVSDK_PORT", "50052")))
    await _wait_connected(rover)
    await _arm(rover)

    # Seed an initial offboard setpoint at the current (start) pose before
    # starting offboard, as PX4 requires.
    s0 = eval_traj(0.0, TRAJ, SPEED_MULT, YAW_MODE)
    pos = [None]   # latest NED (north, east) from telemetry, vel mode only
    hdg = [0.0]    # latest heading [rad, NED]
    if CTRL == "vel":
        async def _pos_feed():
            async for pv in rover.telemetry.position_velocity_ned():
                pos[0] = (pv.position.north_m, pv.position.east_m)

        async def _hdg_feed():
            async for h in rover.telemetry.heading():
                hdg[0] = math.radians(h.heading_deg)
        hdg_task = asyncio.ensure_future(_hdg_feed())
        try:
            await rover.telemetry.set_rate_position_velocity_ned(RATE_HZ)
        except Exception as e:
            print(f"[rover_drive] set_rate_position_velocity_ned failed ({e}); using default rate",
                  flush=True)
        pos_task = asyncio.ensure_future(_pos_feed())
        while pos[0] is None:
            await asyncio.sleep(0.05)
        print(f"[rover_drive] ctrl=vel kp={VEL_KP} vmax={VEL_MAX} "
              f"start pos=({pos[0][0]:+.2f},{pos[0][1]:+.2f})", flush=True)

    async def _hold():
        if CTRL == "vel":
            await rover.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, math.degrees(s0.yaw)))
        else:
            await rover.offboard.set_position_ned(
                PositionNedYaw(s0.x, s0.y, 0.0, math.degrees(s0.yaw)))

    await _hold()
    try:
        await rover.offboard.start()
        print("[rover_drive] offboard started.", flush=True)
    except OffboardError as e:
        print(f"[rover_drive] offboard start failed: {e._result.result}", flush=True)
        raise

    dt = 1.0 / RATE_HZ

    # Gate: hold the start position (keep streaming setpoints so offboard stays
    # alive) until the descent-start flag appears; trajectory t=0 = gate moment
    # (measured on the SIM clock, not wall-clock -- see module docstring).
    if GATE_FILE:
        print(f"[rover_drive] holding start pos; waiting for gate {GATE_FILE} "
              f"(timeout {GATE_TIMEOUT:.0f}s)", flush=True)
        import time as _time
        _w0 = _time.time()
        while not os.path.exists(GATE_FILE):
            if _time.time() - _w0 > GATE_TIMEOUT:
                print("[rover_drive] gate never opened — exiting (rover stays put).",
                      flush=True)
                try:
                    await rover.offboard.stop()
                except Exception:
                    pass
                return
            await _hold()
            await asyncio.sleep(dt)
        print("[rover_drive] gate open — starting trajectory.", flush=True)

    # t0_sim anchors the trajectory's t=0 to the CURRENT simulated time (gate
    # moment, or now if ungated); every subsequent t is read back off the sim
    # clock rather than accumulated from asyncio.sleep(), so it tracks true
    # elapsed simulated time regardless of Gazebo's real-time factor.
    t0_sim = time_node.perf_counter()
    t = 0.0
    # vel mode: anchor the path at where the rover actually is at the gate (and, with
    # VEL_ALIGN, rotate it so it starts along the rover's current heading).
    if CTRL == "vel":
        anchor = pos[0]
        rot = 0.0
        if VEL_ALIGN and s0.speed > 1e-6:
            rot = hdg[0] - math.atan2(s0.vy, s0.vx)
        print(f"[rover_drive] path anchored at ({anchor[0]:+.2f},{anchor[1]:+.2f}), "
              f"rotated {math.degrees(rot):+.1f} deg (rover heading {math.degrees(hdg[0]):+.1f})",
              flush=True)
    prev_yaw = s0.yaw

    deck = None
    if DECK and deck_state(1.0, TRAJ) != (0.0, 0.0, 0.0):
        deck = subprocess.Popen(
            ["/usr/bin/python3", os.path.join(os.path.dirname(os.path.abspath(__file__)), "deck_publisher.py")],
            stdin=subprocess.PIPE, text=True, bufsize=1,
            env={**os.environ, "PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION": "python"})
        print("[rover_drive] deck motion ON (heave/roll/pitch, real physics joints)", flush=True)

    try:
        while MAX_T is None or t <= MAX_T:
            now_sim = time_node.perf_counter()
            if now_sim is not None:
                t = now_sim - t0_sim
            # else: clock momentarily unavailable -- hold the last known t
            # rather than falling back to wall-clock pacing.
            s = eval_traj(t, TRAJ, SPEED_MULT, YAW_MODE, prev_yaw=prev_yaw)
            prev_yaw = s.yaw
            if deck is not None:
                try:
                    deck.stdin.write("%.5f %.5f %.5f\n" % deck_state(t, TRAJ))
                except BrokenPipeError:
                    pass
            if CTRL == "vel":
                s = _Ref(s, s0, anchor, rot)
                vn, ve = _track_cmd(s, pos[0])
                await rover.offboard.set_velocity_ned(
                    VelocityNedYaw(vn, ve, 0.0, math.degrees(math.atan2(ve, vn))))
            else:
                await rover.offboard.set_position_ned(
                    PositionNedYaw(s.x, s.y, 0.0, math.degrees(s.yaw)))
            if abs((t / dt) % (RATE_HZ * 2) ) < 1:  # ~ every 2 s
                msg = (f"[rover_drive] t={t:5.1f} p=({s.x:+.2f},{s.y:+.2f}) "
                       f"v={s.speed:.2f} yaw={math.degrees(math.atan2(s.vy, s.vx)):+.0f}")
                if CTRL == "vel":
                    err = math.hypot(s.x - pos[0][0], s.y - pos[0][1])
                    msg += (f" | pos=({pos[0][0]:+.2f},{pos[0][1]:+.2f}) err={err:.2f} "
                            f"cmd={math.hypot(vn, ve):.2f}")
                print(msg, flush=True)
            await asyncio.sleep(dt)
    finally:
        if deck is not None:
            deck.terminate()
        print("[rover_drive] stopping offboard.", flush=True)
        try:
            await rover.offboard.stop()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("[rover_drive] interrupted.", flush=True)
