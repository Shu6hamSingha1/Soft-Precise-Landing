# CLIMB-DIRECTION thrust-map validation flight.
#
# Small variant of apps/record_input_validation.py's 'landing' profile (same
# convert_2_sys_cmd / send_attitude_rate path), but walking thrust_norm UP
# from hover toward 1.0 instead of down toward THR_MIN. Purpose: the existing
# input-cal (record_input_calibration.py, +-5N sinusoid around hover) and the
# 'landing' validation staircase (0.74 -> 0.55) both stay in a narrow band
# around hover (thrust_norm in [0.55, 0.86]) -- neither exercises anything
# close to full throttle, so the "T_max ~ 31.8 N" figure used for the planned
# az visibility-CBF ceiling is a pure extrapolation of that small-signal fit,
# never validated against real SITL behavior near saturation. This script
# gets that real number: command thrust_norm as an ASCENDING staircase from
# just above hover to (near) 1.0, record achieved vertical accel at each
# step, and look for where the accel-vs-thrust_norm curve goes flat
# (saturation) instead of continuing the hover-region slope.
#
# SAFETY (climbing is open-ended, unlike the landing profile's floor-bounded
# descent): starts LOW (default 5 m) for headroom, aborts the staircase and
# transitions to a hover-and-land sequence if GT altitude exceeds
# VAL_CLIMB_MAX_ALT or climb rate exceeds VAL_CLIMB_MAX_VZ, and always
# inserts a hover-settle hold before landing so it never calls
# vehicle.action.land() while still climbing hard.
#
# MEASURE/BRAKE SCHEDULE (2026-08-22, first live run): a plain monotonic
# staircase (every level held above hover, nothing ever commanded below it)
# makes climb velocity accumulate continuously across the WHOLE ramp, not
# per-level -- the first live run hit MAX_VZ=8m/s after only 6.9s at
# thrust_norm=0.85, nowhere near the 1.0 needed to see saturation. Fixed by
# bracketing each measurement dwell with a BRAKE dwell at the mirror-image
# thrust (2*HOVER_THRUST - thr, same duration) that approximately cancels
# the velocity just gained under local linearity -- exact only away from
# the saturation region we're actually probing, which is fine: MAX_VZ/
# MAX_ALT stay armed as the real backstop throughout, this is just to reach
# high thrust levels at all before the ceiling fires.
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
import asyncio, time
import numpy as np
import rclpy

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node

SLEEP_TIME = 1/200
SEND_TIMEOUT_S = 0.5

TAKEOFF_H     = float(os.environ.get("VAL_CLIMB_TAKEOFF_HEIGHT", "5.0"))    # low start -> headroom to climb
HOVER_THRUST  = float(os.environ.get("VAL_CLIMB_HOVER_THRUST",   "0.738"))  # matches landing_test.py's hover point
THR0          = float(os.environ.get("VAL_CLIMB_THRUST_START",   "0.75"))   # first measured level, just above hover
DTHR          = float(os.environ.get("VAL_CLIMB_THRUST_STEP",    "0.02"))   # increase per level
THR_MAX       = float(os.environ.get("VAL_CLIMB_THRUST_MAX",     "1.00"))   # walk all the way to full throttle
MEASURE_S     = float(os.environ.get("VAL_CLIMB_MEASURE_S",      "0.6"))    # dwell at the test level (accel readout)
BRAKE_S       = float(os.environ.get("VAL_CLIMB_BRAKE_S",        str(0.6)))  # dwell at the mirror (velocity-cancelling) level
MAX_ALT       = float(os.environ.get("VAL_CLIMB_MAX_ALT",        "30.0"))   # GT-altitude ceiling -> abort schedule
MAX_VZ        = float(os.environ.get("VAL_CLIMB_MAX_VZ",         "8.0"))    # m/s climb-rate ceiling -> abort schedule
SETTLE_S      = float(os.environ.get("VAL_CLIMB_SETTLE_S",       "3.0"))    # post-schedule hover hold before landing
MAX_DUR_S     = float(os.environ.get("VAL_CLIMB_MAX_S",          "60.0"))   # hard cap regardless of schedule progress

CONTROLLER_READY = False
telemetry_data = gt_data = None


def yaw_from_quaternion(q):
    return np.degrees(np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)))


def build_schedule():
    """[(thrust_norm, duration_s), ...]: for each level THR0..THR_MAX (step DTHR),
    a MEASURE_S dwell at the test thrust followed by a BRAKE_S dwell at the
    mirror-image thrust (2*HOVER_THRUST - thr, clipped to [0,1]) that
    approximately cancels the velocity just gained -- see module docstring."""
    n_steps = int(round((THR_MAX - THR0) / DTHR)) + 1
    sched = []
    for i in range(n_steps):
        thr = min(THR_MAX, THR0 + i * DTHR)
        brake = float(np.clip(2.0 * HOVER_THRUST - thr, 0.0, 1.0))
        sched.append((thr, MEASURE_S))
        sched.append((brake, BRAKE_S))
    return sched


async def main():
    global telemetry_data, gt_data, CONTROLLER_READY
    pose_subscriber = time_subscriber = FC_node = None
    UAV_pose = []; target_pose = []; ac_cmd = []; t_c = []; start_pose = None; start_time = 0.0
    abort_reason = None

    print(f"[val:climb] {THR0:.2f} -> {THR_MAX:.2f} step {DTHR:.2f}, "
          f"measure/brake {MEASURE_S:.1f}s/{BRAKE_S:.1f}s, "
          f"ceiling alt={MAX_ALT:.0f}m vz={MAX_VZ:.1f}m/s, max {MAX_DUR_S:.0f}s")
    try:
        rclpy.init()
        pose_node = Pose_Node(); pose_subscriber = GZ_Subscriber(pose_node)
        time_node = Clock_Node(); time_subscriber = GZ_Subscriber(time_node)

        t0 = time.perf_counter()
        while time_node.perf_counter() is None:
            if time.perf_counter() - t0 > 20: raise Exception("Unable to get simulation time.")

        FC_node = FC(time_node); await FC_node.start()
        t0 = time.perf_counter()
        while not FC_node.has_quat():
            if time.perf_counter() - t0 > 20: raise Exception("Unable to get data from FC.")
            time.sleep(0.05)
        while start_pose is None: start_pose = pose_node.getPose().UAV

        await FC_node.arm_and_takeoff(TAKEOFF_H)
        yaw = yaw_from_quaternion(FC_node.getQuat())
        await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, yaw)
        await asyncio.sleep(1.0)

        schedule = build_schedule()
        # Cumulative end-time of each (thrust, duration) entry, so tau maps to a
        # phase by simple threshold lookup instead of re-deriving it from thr.
        phase_ends = np.cumsum([d for _, d in schedule])
        schedule_end_s = float(phase_ends[-1])
        abort_tau = None   # tau at which the safety abort latched, for the settle-hold timer
        last_phase_idx = -1

        start_time = time_node.perf_counter()
        for _k in range(int(MAX_DUR_S / SLEEP_TIME)):
            if FC_node.LANDED:
                break
            if CONTROLLER_READY:
                t_c.append(time_node.perf_counter() - start_time)
            else:
                start_time = time_node.perf_counter(); CONTROLLER_READY = True; t_c = [0.0]
            tau = time_node.perf_counter() - start_time
            pose = pose_node.getPose()
            gt_alt = abs(pose.UAV.position.z - pose.target.position.z)
            vel = FC_node.getVelBody()
            vz_up = -vel.z_m_s   # body-FRD down -> up-positive for readable comparisons

            # SAFETY ABORT: stop the schedule (freeze at hover) the instant either
            # ceiling is breached -- don't wait for the current dwell to finish.
            if abort_reason is None and (gt_alt > MAX_ALT or abs(vz_up) > MAX_VZ):
                abort_reason = f"alt={gt_alt:.1f}m vz={vz_up:.1f}m/s"
                abort_tau = tau
                print(f"  [val:climb] SAFETY ABORT ({abort_reason}) -> freezing at hover thrust.")

            if abort_reason is not None:
                thr = HOVER_THRUST
            else:
                phase_idx = int(np.searchsorted(phase_ends, tau, side='right'))
                phase_idx = min(phase_idx, len(schedule) - 1)
                thr = schedule[phase_idx][0]
                if phase_idx != last_phase_idx:
                    label = "measure" if phase_idx % 2 == 0 else "brake"
                    print(f"  [val:climb] level {phase_idx//2 + 1}/{len(schedule)//2} "
                          f"({label}) thr={thr:.3f} alt={gt_alt:.1f}m vz={vz_up:.1f}m/s")
                    last_phase_idx = phase_idx

            try:
                await asyncio.wait_for(FC_node.send_attitude_rate(0.0, 0.0, 0.0, thr),
                                       timeout=SEND_TIMEOUT_S)
            except asyncio.TimeoutError:
                print("  [bail] send_attitude_rate timed out -- PX4 likely dropped OFFBOARD.")
                break
            cmd = np.array([0.0, 0.0, 0.0, thr])   # record commanded thrust_norm (direct, no offset map)

            await asyncio.sleep(SLEEP_TIME)
            UAV_pose.append(pose.UAV)
            target_pose.append(pose.target)
            ac_cmd.append(cmd)

            # Move on to the settle/land phase once either the full schedule has
            # completed, or (post-abort) enough time has passed at hover to
            # arrest any residual climb rate.
            if abort_reason is not None:
                if tau - abort_tau >= MEASURE_S + BRAKE_S:
                    break
            elif tau >= schedule_end_s:
                print(f"  [val:climb] schedule complete, alt={gt_alt:.1f}m -> settling.")
                break

        # Settle at hover before landing -- never call land() while still climbing hard.
        print(f"  [val:climb] hover-settle {SETTLE_S:.1f}s before landing.")
        settle_t0 = time_node.perf_counter()
        while time_node.perf_counter() - settle_t0 < SETTLE_S:
            try:
                await asyncio.wait_for(FC_node.send_attitude_rate(0.0, 0.0, 0.0, HOVER_THRUST),
                                       timeout=SEND_TIMEOUT_S)
            except asyncio.TimeoutError:
                break
            await asyncio.sleep(SLEEP_TIME)

        yaw = yaw_from_quaternion(FC_node.getQuat())
        await FC_node.send_position_ned(0.0, 0.0, FC_node.getPosBody().z_m, yaw)
        await FC_node.vehicle.action.land()

    except Exception as e:
        print(f"[val:climb] error: {e}")

    finally:
        if pose_subscriber and FC_node:
            telemetry_data = FC_node.getLogData()
            gt_data = {"Start Time": start_time, "Time": t_c, "Start Pose": start_pose,
                       "UAV Pose": UAV_pose, "Target Pose": target_pose, "Command": ac_cmd,
                       "Abort Reason": abort_reason}
        for node in (pose_subscriber, time_subscriber):
            if node and node.is_alive():
                node.close(); node.join()
        if FC_node:
            await FC_node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")

    if CONTROLLER_READY:
        dir_name = os.environ.get("VAL_CLIMB_OUT_DIR")
        if not dir_name:
            base = os.environ.get(
                "VAL_CLIMB_OUT_BASE",
                "/home/shubham/Soft-Precise-Landing/PX4_Gazebo/validation_data/input_climb")
            dir_name = f"{base}/{time.ctime().replace(':', '-')}"
        os.makedirs(dir_name, exist_ok=True)
        np.save(f'{dir_name}/Telemetry_Data', telemetry_data)
        np.save(f'{dir_name}/Ground_Truth', gt_data)
        print(f"[val:climb] validation data saved -> {dir_name}")
