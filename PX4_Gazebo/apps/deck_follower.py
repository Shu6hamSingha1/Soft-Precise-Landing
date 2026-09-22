#!/usr/bin/python3
"""Kinematically drive the standalone `deck_platform` model to ride the rover while playing
the manuscript ship-deck heave/roll/pitch (Cases 2/5), with ZERO physical coupling to the rover
chassis.

Background: an earlier version (2026-09-22) jointed the platform directly onto the rover's
base_link (prismatic heave + 2 revolute tilt joints, position-controlled). The joint reaction
torques disturbed the rover chassis enough to stall its PX4 EKF/steering a few seconds into
every run, independent of speed -- see Memory/px4/project_20260922_*deck* notes. This version
instead keeps `deck_platform` as a SEPARATE, <static>true</static> top-level model (its own
entry in /pose, no joint to the rover at all) and teleports it every tick via the gz-sim
UserCommands `/world/<world>/set_pose` service to (rover's live pose) (+) (local heave/roll/
pitch offset from src/rover_trajectory.deck_state). A static model has no physics response, so
nothing it does can ever push back on the rover -- full decoupling by construction, verified in
a bare-world smoke test (repeated set_pose calls held exactly, no drift back under gravity).

Run as SYSTEM python3 (the mavsdk venv has no gz bindings); rover_drive.py spawns it as a
subprocess, matching the deck_publisher.py pattern, with
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python (gz's generated pb2 clashes with the newer
protobuf in ~/.local).

Env:
  DECK_WORLD        Gazebo world name (default rover_cross).
  DECK_ROVER_MODEL  name of the rover's top-level pose entry to follow, as it appears in
                     /world/<world>/pose/info (default rover_ackermann_1 -- VERIFY per launch;
                     PX4_SIM_MODEL names the spawned model, gz suffixes _<n>).
  DECK_PLATFORM     name of the deck_platform include in the world (default deck_platform_1).
  DECK_TRAJ         trajectory type, passed to rover_trajectory.deck_state (default Linear).
  DECK_RATE_HZ      follower tick rate (default 20).
  DECK_GATE_FILE    if set, hold at t=0 (platform sits on the rover with zero offset) until
                     this file appears -- same gate rover_drive.py/record_chase.py use, so deck
                     motion starts exactly when the trajectory does.
  DECK_MOUNT_Z      fixed vertical mount offset above the rover's own body origin, meters
                     (default 0.30, matching the original jointed rover_cross model's platform
                     pose). REQUIRED to clear the bare rover_ackermann chassis: without it the
                     deck_platform collision box sits at the same height as the chassis and
                     constantly interpenetrates it, and the resulting one-way static-into-
                     dynamic contact force chaotically rotates the rover, worst right as the
                     drone's landing weight presses down (2026-09-22, first observed as the
                     rover roll running away to +-180 deg near touchdown -- confirmed root
                     cause via missing offset, not a quaternion-composition bug).
"""
import os
import sys
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

from rover_trajectory import deck_state

WORLD = os.environ.get("DECK_WORLD", "rover_cross")
ROVER_MODEL = os.environ.get("DECK_ROVER_MODEL", "rover_ackermann_1")
PLATFORM = os.environ.get("DECK_PLATFORM", "deck_platform_1")
TRAJ = os.environ.get("DECK_TRAJ", "Linear")
RATE_HZ = float(os.environ.get("DECK_RATE_HZ", "20"))
GATE_FILE = os.environ.get("DECK_GATE_FILE", "")
MOUNT_Z = float(os.environ.get("DECK_MOUNT_Z", "0.30"))

_rover_pose = {"pos": (0.0, 0.0, 0.0), "quat": (0.0, 0.0, 0.0, 1.0)}  # (x,y,z,w)


def _pose_cb(msg):
    for p in msg.pose:
        if p.name == ROVER_MODEL:
            _rover_pose["pos"] = (p.position.x, p.position.y, p.position.z)
            _rover_pose["quat"] = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            return


def _quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy); ty = 2.0 * (z * vx - x * vz); tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _rpy_quat(roll, pitch):
    """roll about local X then pitch about local Y (intrinsic), yaw=0 (deck doesn't add yaw)."""
    cr, sr = __import__("math").cos(roll / 2), __import__("math").sin(roll / 2)
    cp, sp = __import__("math").cos(pitch / 2), __import__("math").sin(pitch / 2)
    q_roll = (sr, 0.0, 0.0, cr)
    q_pitch = (0.0, sp, 0.0, cp)
    return _quat_mul(q_pitch, q_roll)   # pitch after roll, both about the parent (rover) frame


def main():
    node = Node()
    ok = node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", _pose_cb)
    if not ok:
        print(f"[deck_follower] FAILED to subscribe /world/{WORLD}/pose/info", flush=True)
        sys.exit(1)
    print(f"[deck_follower] world={WORLD} rover={ROVER_MODEL} platform={PLATFORM} "
          f"traj={TRAJ} rate={RATE_HZ}Hz", flush=True)

    dt = 1.0 / RATE_HZ
    if GATE_FILE:
        print(f"[deck_follower] waiting for gate {GATE_FILE}", flush=True)
        while not os.path.exists(GATE_FILE):
            time.sleep(0.05)
        print("[deck_follower] gate open -- deck motion starting.", flush=True)
    t0 = time.time()

    svc = f"/world/{WORLD}/set_pose"
    _last_dbg = 0.0
    while True:
        t = time.time() - t0
        if os.environ.get("DECK_DEBUG", "0") == "1" and t - _last_dbg > 2.0:
            print(f"[deck_follower] t={t:.1f} rover_pose={_rover_pose}", flush=True)
            _last_dbg = t
        heave, roll, pitch = deck_state(t, TRAJ)
        rx, ry, rz = _rover_pose["pos"]
        rq = _rover_pose["quat"]
        local_off = _rotate(rq, (0.0, 0.0, MOUNT_Z + heave))  # fixed mount height + heave, both along the rover's own +z
        q_local = _rpy_quat(roll, pitch)
        q_world = _quat_mul(rq, q_local)

        req = Pose()
        req.name = PLATFORM
        req.position.x = rx + local_off[0]
        req.position.y = ry + local_off[1]
        req.position.z = rz + local_off[2]
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = q_world
        node.request(svc, req, Pose, Boolean, 100)
        time.sleep(dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
