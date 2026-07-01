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
"""

import os
import sys
import asyncio
import math

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

from rover_trajectory import eval_traj, TRAJECTORY_TYPES

MAV_URL = os.environ.get("ROVER_MAV_URL", "udp://:14541")
TRAJ = os.environ.get("ROVER_TRAJ", "Circular")
SPEED_MULT = float(os.environ.get("ROVER_SPEED_MULT", "1.0"))
YAW_MODE = os.environ.get("ROVER_YAW_MODE", "spec")
RATE_HZ = float(os.environ.get("ROVER_RATE_HZ", "20"))
MAX_T = os.environ.get("ROVER_MAX_T")
MAX_T = float(MAX_T) if MAX_T else None


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


async def run():
    if TRAJ not in TRAJECTORY_TYPES:
        raise SystemExit(f"ROVER_TRAJ={TRAJ!r} invalid; choose {TRAJECTORY_TYPES}")
    print(f"[rover_drive] traj={TRAJ} speed_mult={SPEED_MULT} yaw={YAW_MODE} "
          f"rate={RATE_HZ}Hz max_t={MAX_T}", flush=True)

    rover = System()
    await _wait_connected(rover)
    await _arm(rover)

    # Seed an initial offboard setpoint at the current (start) pose before
    # starting offboard, as PX4 requires.
    s0 = eval_traj(0.0, TRAJ, SPEED_MULT, YAW_MODE)
    await rover.offboard.set_position_ned(
        PositionNedYaw(s0.x, s0.y, 0.0, math.degrees(s0.yaw)))
    try:
        await rover.offboard.start()
        print("[rover_drive] offboard started.", flush=True)
    except OffboardError as e:
        print(f"[rover_drive] offboard start failed: {e._result.result}", flush=True)
        raise

    dt = 1.0 / RATE_HZ
    t = 0.0
    prev_yaw = s0.yaw
    try:
        while MAX_T is None or t <= MAX_T:
            s = eval_traj(t, TRAJ, SPEED_MULT, YAW_MODE, prev_yaw=prev_yaw)
            prev_yaw = s.yaw
            await rover.offboard.set_position_ned(
                PositionNedYaw(s.x, s.y, 0.0, math.degrees(s.yaw)))
            if abs((t / dt) % (RATE_HZ * 2) ) < 1:  # ~ every 2 s
                print(f"[rover_drive] t={t:5.1f} p=({s.x:+.2f},{s.y:+.2f}) "
                      f"v={s.speed:.2f} yaw={math.degrees(s.yaw):+.0f}", flush=True)
            await asyncio.sleep(dt)
            t += dt
    finally:
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
