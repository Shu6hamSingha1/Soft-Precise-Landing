#!/usr/bin/python3
"""Joint-command publisher for the JOINTED rover_cross deck (real physics, not teleportation --
see the 2026-09-22 collision investigation: a teleported/kinematic platform never achieves
genuine sustained contact with a landing drone in this Gazebo/ODE build, confirmed by a bare-world
falling-ball test). This drives the three JointPositionController topics
(/deck/deck_{heave,roll,pitch}/cmd_pos) that sim_models/rover_cross_deck_model.sdf's
prismatic+revolute+revolute chain listens on, so the platform moves via REAL physics the whole
time -- correct, persistent contact by construction.

Run as SYSTEM python3 (the mavsdk venv has no gz bindings, and gz's generated pb2 clashes with
the newer protobuf in ~/.local -- PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python works around it).
Reads lines "heave roll pitch" (m, rad, rad) on stdin, one per tick; apps/rover_drive.py feeds it
from src/rover_trajectory.deck_state(t, traj_type).
"""
import os
import sys

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

node = Node()
pubs = [node.advertise(f"/deck/deck_{j}/cmd_pos", Double) for j in ("heave", "roll", "pitch")]
for line in sys.stdin:
    try:
        vals = [float(x) for x in line.split()]
    except ValueError:
        continue
    if len(vals) != 3:
        continue
    for p, v in zip(pubs, vals):
        m = Double()
        m.data = v
        p.publish(m)
