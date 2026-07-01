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
        r = 0.5
        wz = 0.48 * speed_mult
        x = -r * (math.cos(wz * t) - 1.0)
        y = r * math.sin(wz * t)
        vx = r * wz * math.sin(wz * t)
        vy = r * wz * math.cos(wz * t)
        spec_yaw, spec_yaw_rate = wz * t, wz   # MATLAB psi = wz*t

    elif traj_type == "EightShape":
        a, w0 = 1.0, 0.3
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
        A, B = 0.4, 0.8
        w1 = -0.5 * speed_mult
        w2 = 0.85 * speed_mult
        x = A * math.sin(w1 * t)
        y = B * math.sin(w2 * t)
        vx = A * w1 * math.cos(w1 * t)
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


if __name__ == "__main__":
    # Quick self-check: print a few samples of each trajectory.
    for tt in TRAJECTORY_TYPES:
        s0 = eval_traj(0.0, tt)
        s5 = eval_traj(5.0, tt)
        print(f"{tt:12s} t=0: ({s0.x:+.3f},{s0.y:+.3f}) v={s0.speed:.3f} "
              f"yaw={math.degrees(s0.yaw):+.1f}deg | "
              f"t=5: ({s5.x:+.3f},{s5.y:+.3f}) v={s5.speed:.3f}")
