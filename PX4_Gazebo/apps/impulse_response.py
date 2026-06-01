"""Impulse-response test for the PX4 rate loop.

Takes off, hovers at 3 m, then injects a sequence of brief step commands
in body-rate (w_u) on each axis. Logs the commanded vs measured ω at
high sample rate so we can fit a first-order response and extract:

  - pure delay (deadtime t_d) — time between command sent and ω starting to rise
  - time constant (τ) — exponential rise time of the response
  - effective total lag — t_d + 3τ (95% settle)

These give a cleaner lag estimate than the convergent-flight cross-
correlation in analyze_loop_latency.py, since hover removes the flight-
dynamics confound.

Output: $HOME/ws/Test_Data/ImpulseResponse/<timestamp>/impulse_log.npy

Run via:
    PY_SCRIPT=impulse_response.py bash run_aruco_landing.sh
or via the wrapper:
    bash run_impulse_response.sh
"""

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio
import os
import time
import numpy as np

from flight_controller import FC as FlightController


# Impulse sequence: (axis_index, magnitude_rad_s, duration_s)
# axis: 0=roll, 1=pitch, 2=yaw
# 3 impulses per axis, alternating sign — 18 events total
SEQUENCE = []
for axis in (0, 1, 2):
    for mag in (+0.5, -0.5, +0.5):
        SEQUENCE.append((axis, mag, 0.3))

SETTLE_S = 1.0          # zero-rate hold between impulses
STABILIZE_S = 3.0       # hover stabilization before first impulse
TAKEOFF_ALT_M = 3.0     # AGL
HOVER_THRUST = 0.71     # x500 normalized hover thrust (set via experiment)
LOOP_HZ = 200
LOOP_DT = 1.0 / LOOP_HZ

OUTDIR = os.path.expanduser(
    f"~/ws/Test_Data/ImpulseResponse/{time.strftime('%Y%m%d-%H%M%S')}"
)
os.makedirs(OUTDIR, exist_ok=True)


async def main():
    FC = FlightController()
    await FC.start()

    print(f"[impulse] arm + takeoff to {TAKEOFF_ALT_M} m")
    await FC.arm_and_takeoff(TAKEOFF_ALT_M)

    # Hold altitude via position-NED for a couple of seconds so we enter
    # OFFBOARD cleanly with a non-rate setpoint (PX4 prefers a setpoint
    # stream before switching modes).
    pos = FC.getPosBody()
    n0, e0, d0 = pos.x_m, pos.y_m, pos.z_m
    print(f"[impulse] hover at NED ({n0:.2f}, {e0:.2f}, {d0:.2f}) for stabilize")
    t_end = time.perf_counter() + STABILIZE_S
    while time.perf_counter() < t_end:
        await FC.send_position_ned(n0, e0, d0, 0.0)
        await asyncio.sleep(0.02)

    log_t = []
    log_cmd = []   # (3,) commanded body rate [rad/s]
    log_thr = []   # commanded thrust [0..1]
    log_meas = []  # (3,) measured ω [rad/s]
    log_event = []   # event id (impulse_index or -1 for settle/stabilize)

    async def send_loop(rate_cmd_rad_s, thrust, dur_s, event_id):
        t_start = time.perf_counter()
        while True:
            t = time.perf_counter()
            if t - t_start >= dur_s:
                break
            # MAVSDK AttitudeRate takes DEGREES/s.  Convert.
            r_dps, p_dps, y_dps = np.degrees(rate_cmd_rad_s)
            await FC.send_attitude_rate(r_dps, p_dps, y_dps, thrust)
            # Capture measured ω at the same instant.
            # IMU frame is FRD (forward/right/down), matching send_attitude_rate
            # body-rate axes — so we can compare directly without rotation.
            try:
                av = FC.getAngVelIMU()       # AngularVelocityFrd object
                av_arr = np.array([av.forward_rad_s, av.right_rad_s, av.down_rad_s])
            except (IndexError, AttributeError):
                av_arr = np.array([np.nan, np.nan, np.nan])
            log_t.append(t)
            log_cmd.append(np.array(rate_cmd_rad_s, dtype=float))
            log_thr.append(thrust)
            log_meas.append(av_arr)
            log_event.append(event_id)
            await asyncio.sleep(LOOP_DT)

    print("[impulse] starting impulse sequence")
    for i, (axis, mag, dur) in enumerate(SEQUENCE):
        cmd = [0.0, 0.0, 0.0]
        cmd[axis] = mag
        print(f"  impulse {i+1}/{len(SEQUENCE)}: axis={['roll','pitch','yaw'][axis]} "
              f"mag={mag:+.2f} rad/s for {dur:.2f}s")
        await send_loop(cmd, HOVER_THRUST, dur, event_id=i)
        # Settle: zero rate
        await send_loop([0.0, 0.0, 0.0], HOVER_THRUST, SETTLE_S, event_id=-1)

    print("[impulse] sequence complete — landing")
    # Switch back to position setpoint, then hand off to PX4 LAND mode
    pos2 = FC.getPosBody()
    for _ in range(10):
        await FC.send_position_ned(pos2.x_m, pos2.y_m, d0, 0.0)
        await asyncio.sleep(0.05)
    try:
        await FC.vehicle.action.land()
    except Exception as e:
        print(f"[impulse] action.land failed: {e}; disarming instead")
        try:
            await FC.vehicle.action.disarm()
        except Exception:
            pass

    # Save
    out_path = os.path.join(OUTDIR, "impulse_log.npy")
    np.save(out_path, {
        "t":     np.array(log_t),
        "cmd":   np.array(log_cmd),         # (N, 3) commanded ω rad/s
        "thr":   np.array(log_thr),
        "meas":  np.array(log_meas),        # (N, 3) measured ω rad/s
        "event": np.array(log_event),
        "sequence":      SEQUENCE,
        "stabilize_s":   STABILIZE_S,
        "settle_s":      SETTLE_S,
        "loop_hz":       LOOP_HZ,
        "hover_thrust":  HOVER_THRUST,
        "takeoff_alt_m": TAKEOFF_ALT_M,
    }, allow_pickle=True)
    print(f"[impulse] saved {out_path}  (samples={len(log_t)})")

    await FC.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[impulse] aborted by user")
