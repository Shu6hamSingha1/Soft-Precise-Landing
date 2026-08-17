# *****************************************************************
# Used simulator time instead of system time
# Removed arm_and_takeoff() from init_setup()
# Added takeoff_hgt as argument to arm_and_takeoff()
# Added Actuator Command Functions
# *****************************************************************
#!/usr/bin/env python3

# Imports for communication with flight controller
import grpc
import asyncio
import os
import subprocess
import time
import numpy as np
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed, AccelerationNed, Attitude, AttitudeRate, ActuatorControl, ActuatorControlGroup)
from mavsdk.action import ActionError
from mavsdk.telemetry import LandedState


def _set_cpu_governor(mode):
    """Best-effort CPU governor switch (performance/ondemand). Non-fatal on
    failure (e.g. not running on the Pi, no passwordless sudo) — a flight
    must never abort over a governor write. 'performance' pins all cores at
    max clock to avoid ondemand's reactive ramp-up lag right at arm/takeoff;
    reverted to 'ondemand' on close() to keep idle power/thermal draw low
    between flights."""
    try:
        subprocess.run(
            f"echo {mode} | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null",
            shell=True, check=True, timeout=5,
        )
        print(f"-- CPU governor set to {mode}")
    except Exception as e:
        print(f"WARNING: could not set CPU governor to {mode}: {e}")


class FC():
    def __init__(self, time_keeper=time):
        self._time = time_keeper
        # 2026-06-02 — LAG FIX: transport for the high-rate body-rate setpoints.
        #   'mavsdk' (default): offboard.set_attitude_rate over UDP (~30 ms).
        #   'dds': direct uXRCE-DDS publish to /fmu/in/vehicle_rates_setpoint
        #          (cuts the MAVSDK/MAVLink/UDP hops; ~38 ms -> ~10 ms target).
        # Cold path (arm/takeoff/offboard-entry/land) stays MAVSDK regardless.
        self._cmd_transport = os.environ.get("CMD_TRANSPORT", "mavsdk")
        self._dds = None
        # Flags
        self.CONNECTED = False
        self._OFFBOARD = False
        self.LANDED = True
        # BUG FIX (2026-08-04, see project_pi_landed_flag_race_2026_08_04 memory):
        # arm_and_takeoff() resets LANDED=False once, right after the AUTO_TAKEOFF
        # wait window -- but if AUTO_TAKEOFF plateaued without leaving the ground
        # (a documented "consistent" PX4 quirk, see the AUTO_TAKEOFF_WINDOW_S
        # comment below), PX4's own landed_state() telemetry is STILL reporting
        # ON_GROUND at that exact moment, and _getLandedState() immediately
        # re-latches LANDED=True -- with nothing left to ever clear it again, so
        # a genuinely successful OFFBOARD climb afterward starts the landing loop
        # with a stale LANDED=True and exits instantly, 0 control ticks logged.
        # _landed_reset_allowed gates BOTH halves of the fix: (a) arm_and_takeoff
        # does an explicit second `self.LANDED = False` the moment real altitude
        # is confirmed (AUTO_TAKEOFF success OR the OFFBOARD climb fallback), and
        # (b) _getLandedState() also clears LANDED=False on IN_AIR/TAKING_OFF
        # telemetry as a general safety net for the same race -- but ONLY while
        # this flag is True. Both halves flip it False the moment real altitude
        # is confirmed, so once genuine flight begins this reverts to a true
        # one-way latch again: a real touchdown (PX4 ON_GROUND or the accel-spike
        # _impactDetector) can never be silently undone by a late/transient
        # IN_AIR sample (e.g. during a hard-landing bounce before landed_state
        # settles) -- IN_AIR-based resets are structurally impossible once
        # _landed_reset_allowed is False.
        self._landed_reset_allowed = True
        self._STAY_OPEN  = True
        self._governor_boosted = False   # only close() reverts if arm_and_takeoff set it

        # Initialize variables       
        # States
        self._pos_body = []
        self._quat = []
        self._vel_body = []
        self._ang_vel_body = []
        self._pos_ned = []
        self._vel_ned = []
        self._att = []
        self._acc_frd = []
        self._ang_vel_frd = []

        # timestamps
        self._odo_ts = []
        self._pos_vel_ts = []
        self._alt_ts = []
        self._att_ts = []
        self._imu_ts = []

        self._alt = []
        self._flight_state = [LandedState.ON_GROUND]

        # Downward rangefinder (2026-08-17, for hw_pos_feedback.py's opt-in
        # PLASMC_HW_USE_RANGEFINDER_Z path -- see that file's HWPoseNode docstring).
        # Confirmed present on this airframe by the user 2026-08-17. Not consumed
        # by anything unless explicitly opted into; harmless if the sensor is
        # absent (the async generator simply never yields, list stays empty,
        # getDistanceSensor() returns None).
        self._dist_sensor = []
        self._dist_sensor_ts = []

        # Instantiate a System object, that will serve as a proxy to all the MAVSDK plugins.
        self.vehicle = System()

    # Calling destructor
    def __del__(self):
        print("Flight Controller is disconnected...")

    #-- Define the function for initial system setup
    async def start(self):
        try:
            # print("Here")
            # Setup connection with Flight Controller
            await self.vehicle.connect(system_address="serial:///dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00:57600")
            # await self.vehicle.connect(system_address="udp://:14540")

            print("Waiting for Flight Controller to connect...")
            async for state in self.vehicle.core.connection_state():
                if state.is_connected:
                    print(f"-- Connected to Flight Controller!")
                    self.CONNECTED = True
                    break

            # Start the tasks
            self.tasks = [
                asyncio.create_task(self._getOdometry()),
                asyncio.create_task(self._getAcc()),
                asyncio.create_task(self._getLandedState()),
                asyncio.create_task(self._getDistanceSensor()),
                asyncio.create_task(self.print_status_text())
            ]
            # _impactDetector is Gazebo-tuned (IMPACT_THRESH=50 m/s^2 assumes
            # Gazebo's 500-900 m/s^2 contact-physics spikes) and is DISABLED
            # by default on real hardware, where touchdown/turbulence/maneuver
            # accel spikes are a different order of magnitude and could
            # false-trigger LANDED=True mid-flight. _getLandedState() (PX4's
            # own onboard-fused landing detection) is the hardware-safe
            # signal. Opt in only after deriving a real threshold from actual
            # flight accel data: FC_IMPACT_DETECTOR=1.
            if os.environ.get("FC_IMPACT_DETECTOR", "0") == "1":
                self.tasks.append(asyncio.create_task(self._impactDetector()))

            # Telemetry rate bumped 60 → 200 Hz (2026-05-22).  60 Hz adds ~16 ms
            # quantization to logged ω, inflating Phase 2 cross-correlation lag
            # by ±8 ms.  Doesn't change control-path latency (we don't use ω as
            # feedback), but cleans up diagnostics and any future feedback use.
            await self.vehicle.telemetry.set_rate_odometry(200)
            await self.vehicle.telemetry.set_rate_imu(200)

            # 2026-05-23 — Rate-gain tuning hook.  Impulse-response test gave
            # pitch τ = 8 ms at default MC_*RATE_P; tightening P (and adding
            # some D) should shrink τ further.  Env knob is a uniform scale
            # over the X500 defaults so we can A/B without per-axis fiddling.
            # Eagerly create the DDS rate sender here (single-threaded setup
            # moment) rather than lazily on the first in-flight send_attitude_rate
            # — lazy creation from the asyncio control loop, with the
            # gz_subscriber executors already spinning, raced rclpy context state
            # ("publisher's context is invalid"). Its own isolated context shares
            # no locks with the subscribers.
            if self._cmd_transport == "dds" and self._dds is None:
                from dds_setpoint import DDSRateSender
                self._dds = DDSRateSender(time_keeper=self._time)

            await self.vehicle.action.hold()
                
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Flight Controller Thread\n")

        except RuntimeError as e:
            print("RuntimeError: Flight Controller Thread: {e}\n")

        except SyntaxError as e:
            print("SyntaxError: Flight Controller Thread: {e}\n")

        except Exception as e:
            print(f"Unexpected error: Flight Controller Thread: {e}\n")

    async def set_px4_param_int(self, name, value, verify=True):
        """Set (and optionally read back) a PX4 int parameter via MAVSDK.
        Used by COMPASS_FREE_VALIDATE to cut EKF2 mag fusion at controller
        engage (EKF2_MAG_TYPE=5). Returns the read-back value (or None)."""
        await self.vehicle.param.set_param_int(name, int(value))
        if verify:
            rb = await self.vehicle.param.get_param_int(name)
            print(f"[FC] PX4 param {name} = {rb} (requested {value})")
            return rb
        return None

    #-- Close connection
    async def close(self):
        """
        Gracefully close the flight controller connection.
        """
        if not self.CONNECTED:
            return

        # Revert CPU to power-saving ondemand scaling now that the flight is
        # done — 'performance' mode should only be pinned during arm/flight.
        # Only if THIS session actually boosted it: scripts that never arm
        # (e.g. output_calibration.py, hand-move takes) still call close()
        # for cleanup, and must not clobber a governor setting they never
        # touched (this reverted an externally-set performance mode once).
        if self._governor_boosted:
            _set_cpu_governor("ondemand")
            self._governor_boosted = False

        self.CONNECTED = False
        self._STAY_OPEN  = False

        try:
            # Cancel all tasks
            for task in self.tasks:
                task.cancel()    

            # Wait for tasks to exit
            await asyncio.gather(*self.tasks, return_exceptions=True)
            await asyncio.sleep(0)   # ⭐ important

        except grpc.RpcError as e:
            print(f"gRPC error during close: {e.details()}")

        except asyncio.CancelledError:
            print(f"CancelledError")

        except Exception as e:
            print(f"Unexpected error during close: {e}")
        finally:        
            # Ensure the System object is deleted
            del self.vehicle     

    def has_quat(self):
        return len(self. _quat) > 0

    #-- Define the function for arm and takeoff
    async def arm_and_takeoff(self, takeoff_hgt=None):
        """
        Arms vehicle.
        """
        # 2026-07-11: refuse to arm while a MAVSDK log download is in
        # progress (download_flight_logs.py writes/removes this lock file) --
        # a stuck/running log download is documented to interfere with a new
        # flight starting. If a download crashed hard enough to leave this
        # behind, remove it manually after confirming no download is running:
        # rm Test_Data/.log_download_active
        LOG_DOWNLOAD_LOCK_FILE = os.environ.get(
            "LOG_DOWNLOAD_LOCK_FILE", "Test_Data/.log_download_active")
        if os.path.exists(LOG_DOWNLOAD_LOCK_FILE):
            raise RuntimeError(
                f"Refusing to arm: {LOG_DOWNLOAD_LOCK_FILE} exists, meaning a "
                f"MAVSDK log download is (or was) in progress -- this can "
                f"interfere with a new flight. If no download is actually "
                f"running, remove the stale lock file first."
            )

        # Pin CPU to max clock before arming — avoids ondemand's reactive
        # ramp-up lag right as FC+camera+compute load spikes together.
        _set_cpu_governor("performance")
        self._governor_boosted = True

        #  Arm.
        if takeoff_hgt is None:
            arm_res = input('Do you want to arm? (y/n): ')
        else:
            arm_res = 'y'
        if arm_res != 'n':
            # PX4 prints "Ready for takeoff" before all preflight checks settle.
            # arm() can still return COMMAND_DENIED for 5-30 s afterwards because
            # of the PX4-gz_bridge lockstep race: gz_bridge subscribes to IMU
            # before the lockstep scheduler has synced with Gazebo's clock,
            # producing IMU timestamps=0 which the EKF rejects, which gates the
            # is_armable health flag.
            #
            # Instead of blind retries, poll telemetry.health() until
            # is_armable goes true (EKF has accepted valid IMU samples). This
            # is adaptive: typical wait is 0-10 s on a healthy SITL boot,
            # 20-40 s when lockstep is racing. After is_armable, arm() should
            # succeed first try; if it doesn't we raise to trigger an outer
            # retry of the whole stack.
            # BUG FIX (2026-08-04, fix (a) -- see project_pi_command_denied_takeoff_2026_08_04
            # memory): waiting on is_armable alone isn't sufficient. Root-caused from 3
            # real COMMAND_DENIED-on-takeoff() failures on 2026-08-03: is_armable reflects
            # baseline EKF/IMU readiness for ARMING, but PX4 requires a separately-settled
            # GPS/position estimate to accept a subsequent TAKEOFF mode-switch -- 2 of the 3
            # failures showed explicit "GPS PDOP too high"/"GPS ... Drift too high"/"height
            # estimate not stable" CRITICAL warnings in the same window right before
            # is_armable went true, meaning arm() succeeded on a still-settling position
            # estimate that PX4 then refused to fly on. Now also wait for
            # is_global_position_ok and is_home_position_ok, not just is_armable.
            print("-- Waiting for is_armable + global/home position ready (EKF / lockstep ready)...")
            armable_timeout = 60.0
            async def _wait_armable():
                async for health in self.vehicle.telemetry.health():
                    if (health.is_armable and health.is_global_position_ok
                            and health.is_home_position_ok):
                        return
            t0 = time.monotonic()
            try:
                await asyncio.wait_for(_wait_armable(), timeout=armable_timeout)
            except asyncio.TimeoutError:
                raise RuntimeError(
                    f"is_armable + global/home position did not go True within "
                    f"{armable_timeout}s — PX4 lockstep race is not recovering."
                )
            print(f"  is_armable + position ready after {time.monotonic() - t0:.1f}s")

            print("-- Arming")
            try:
                await self.vehicle.action.arm()
            except Exception as e:
                # If arm() fails even when is_armable=True, something else is
                # wrong (e.g. preflight check beyond EKF) — signal outer retry.
                raise RuntimeError(f"arm() failed even after is_armable: {e}")
            print('Armed!')
        else:
            raise Exception("Request for arming rejected.")

        # Takeoff.
        if takeoff_hgt is None:
            while self._STAY_OPEN :
                x = input('Enter takeoff height (in m): ')
                try:
                    takeoff_hgt = float(x)
                    break
                except:
                    print("Wrong input. Enter a non-positive value to terminate takeoff operation.")
        if takeoff_hgt > 0:
            print("-- Taking off")
            await self.vehicle.action.set_takeoff_altitude(1.2*takeoff_hgt)
            # BUG FIX (2026-08-04, fix (b) -- see project_pi_command_denied_takeoff_2026_08_04
            # memory): fix (a) above closes the main gap, but retry here too as a safety net
            # for any remaining transient PX4-side rejection, mirroring the existing
            # offboard.start() retry pattern elsewhere in this same method. Unlike
            # offboard.start(), takeoff() doesn't need a setpoint stream pumped between
            # attempts -- it's a discrete action command, not an offboard mode -- so a
            # short backoff sleep is enough.
            TAKEOFF_RETRY_ATTEMPTS = 3
            TAKEOFF_RETRY_DELAY_S = 2.0
            last_err = None
            for attempt in range(TAKEOFF_RETRY_ATTEMPTS):
                try:
                    await self.vehicle.action.takeoff()
                    last_err = None
                    break
                except ActionError as error:
                    last_err = error
                    print(f"  takeoff() attempt {attempt + 1}/{TAKEOFF_RETRY_ATTEMPTS} "
                          f"failed: {error._result.result} — retrying after "
                          f"{TAKEOFF_RETRY_DELAY_S:.0f}s...")
                    await asyncio.sleep(TAKEOFF_RETRY_DELAY_S)
            if last_err is not None:
                raise RuntimeError(
                    f"takeoff() failed after {TAKEOFF_RETRY_ATTEMPTS} attempts: "
                    f"{last_err._result.result}. See project_pi_command_denied_takeoff_"
                    f"2026_08_04 memory for the known root cause (GPS/position estimate "
                    f"not settled) — if this keeps recurring even after fix (a)'s "
                    f"stronger pre-arm wait, the position estimate may need longer to "
                    f"settle than the health flags alone indicate."
                )
        #-- wait to reach the target altitude via PX4's own AUTO_TAKEOFF
            # 2026-07-11: root-caused via .ulg analysis (see
            # project_flight_anomaly_investigation_2026_07_10 memory) -- PX4's
            # AUTO_TAKEOFF does NOT reliably climb all the way to the commanded
            # altitude before exiting to Hold/LOITER. Across multiple flights it
            # consistently exited TAKEOFF mode after ~6-9s regardless of altitude
            # actually reached (0.3m, 1.0m, 3.2m all exited around the same
            # elapsed time) -- actuator output during the climb was essentially
            # IDENTICAL in every case (~1500 of ~2000 max), ruling out a
            # thrust/battery-authority limit. This is a fixed-duration
            # open-loop-ish climb profile, not a closed-loop "climb until
            # target reached" controller. So: give AUTO_TAKEOFF a SHORT window
            # to do its thing, but don't treat falling short as fatal here --
            # fall through to our own closed-loop OFFBOARD position climb
            # below, which is a much more reliable way to actually reach the
            # requested altitude.
            AUTO_TAKEOFF_WINDOW_S = 12.0
            t0_takeoff = time.monotonic()
            _auto_takeoff_reached_alt = False
            while self._STAY_OPEN :
                await asyncio.sleep(0.5)
                alt_now = -self._pos_body[-1].z_m
                if alt_now >= 0.9*takeoff_hgt:
                    print("Altitude reached via AUTO_TAKEOFF")
                    print(alt_now, takeoff_hgt)
                    _auto_takeoff_reached_alt = True
                    break
                if (time.monotonic() - t0_takeoff) > AUTO_TAKEOFF_WINDOW_S:
                    print(f"AUTO_TAKEOFF plateaued at alt={alt_now:.2f}m (target "
                          f"{0.9*takeoff_hgt:.2f}m) after {AUTO_TAKEOFF_WINDOW_S:.0f}s -- "
                          f"this is expected PX4 behavior, not an error. Will finish "
                          f"the climb via our own OFFBOARD position setpoint next.")
                    break
            self.LANDED = False
            # BUG FIX (2026-08-04) fix (a) part 1 -- see _landed_reset_allowed
            # comment in __init__. Only close the gate here if AUTO_TAKEOFF
            # actually reached real altitude: in the plateau/timeout branch the
            # drone is still genuinely on the ground, so PX4's landed_state()
            # can legitimately re-latch LANDED=True moments after this reset --
            # leave the gate OPEN so fix (b)'s IN_AIR/TAKING_OFF safety net can
            # still clear it once the OFFBOARD climb below actually leaves the
            # ground (which also closes the gate itself, at that point).
            if _auto_takeoff_reached_alt:
                self._landed_reset_allowed = False
        else:
            raise Exception("Request for takeoff rejected.")

        print("Switching to OFFBOARD mode!")

        # Bombard PX4 with stationary setpoints BEFORE calling offboard.start().
        # MAVSDK requires a valid setpoint stream to be flowing; a single one
        # (the original code) can race with PX4's TAKEOFF->HOLD transition and
        # cause offboard.start() to silently fail. The script then proceeds with
        # offboard NOT active, and PX4 ignores subsequent setpoints (stays in
        # HOLD, drifts).
        for _ in range(20):                    # ~0.5 s at 40 Hz of setpoints
            await self.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.025)

        # Retry offboard.start() a few times — first attempt sometimes fails
        # because PX4 is still in the TAKEOFF->HOLD transition.
        last_err = None
        for attempt in range(5):
            try:
                print(f"-- Starting offboard (attempt {attempt + 1}/5)")
                await self.vehicle.offboard.start()
                last_err = None
                break
            except OffboardError as error:
                last_err = error
                print(f"  offboard.start failed: {error._result.result} "
                      f"— continuing to pump setpoints and retrying...")
                for _ in range(20):
                    await self.send_velocity_body(0.0, 0.0, 0.0, 0.0)
                    await asyncio.sleep(0.025)
        if last_err is not None:
            # Don't silently continue with offboard NOT active — caller needs to know.
            raise RuntimeError(
                f"offboard.start() failed after 5 attempts: {last_err._result.result}. "
                f"Subsequent send_position_ned / send_attitude_rate calls won't take effect."
            )

        # Verify mode actually flipped to OFFBOARD before we hand control back.
        try:
            mode = await asyncio.wait_for(self.vehicle.telemetry.flight_mode().__aiter__().__anext__(),
                                          timeout=1.0)
            print(f"  PX4 reported flight_mode = {mode}")
        except Exception as e:
            print(f"  (couldn't read flight_mode: {e})")

        # 2026-07-11: if AUTO_TAKEOFF plateaued short of the requested altitude
        # (see note above the AUTO_TAKEOFF_WINDOW_S loop), finish the climb
        # ourselves now that OFFBOARD is confirmed active -- a closed-loop
        # position setpoint is a far more reliable way to actually reach
        # takeoff_hgt than trusting PX4's own AUTO_TAKEOFF completion logic.
        alt_now = -self._pos_body[-1].z_m
        if alt_now < 0.9 * takeoff_hgt:
            print(f"-- Finishing climb via OFFBOARD position setpoint "
                  f"(currently {alt_now:.2f}m, target {takeoff_hgt:.2f}m)")
            p_now = self.getPosBody()
            q_now = self.getQuat()
            yaw_now = float(np.degrees(np.arctan2(
                2.0 * (q_now.w * q_now.z + q_now.x * q_now.y),
                1.0 - 2.0 * (q_now.y * q_now.y + q_now.z * q_now.z))))
            OFFBOARD_CLIMB_TIMEOUT_S = 15.0
            t0_climb = time.monotonic()
            while self._STAY_OPEN:
                await self.send_position_ned(p_now.x_m, p_now.y_m, -takeoff_hgt, yaw_now)
                await asyncio.sleep(0.1)
                alt_now = -self._pos_body[-1].z_m
                if alt_now >= 0.9 * takeoff_hgt:
                    print(f"Altitude reached via OFFBOARD climb: {alt_now:.2f}m")
                    # BUG FIX (2026-08-04) fix (a) part 2 -- see _landed_reset_allowed
                    # comment in __init__. Real altitude is now confirmed (this is
                    # exactly the branch that races against a stale ON_GROUND sample
                    # from the AUTO_TAKEOFF-plateau case above), so explicitly clear
                    # any stale LANDED=True one more time here and close the gate for
                    # good -- from this point on LANDED is a genuine one-way latch
                    # again, same as before this fix.
                    self.LANDED = False
                    self._landed_reset_allowed = False
                    break
                if (time.monotonic() - t0_climb) > OFFBOARD_CLIMB_TIMEOUT_S:
                    raise RuntimeError(
                        f"OFFBOARD climb did not reach {0.9*takeoff_hgt:.2f}m within "
                        f"{OFFBOARD_CLIMB_TIMEOUT_S:.0f}s (current alt={alt_now:.2f}m) -- "
                        f"this is a closed-loop position command, so falling short here "
                        f"is a genuine problem (thrust authority, EKF, or battery), not "
                        f"just AUTO_TAKEOFF's known fixed-duration quirk. Investigate "
                        f"before retrying."
                    )

        print(self.getPosBody())
    
    async def send_position_ned(self, n = 0.0, e = 0.0, d = 0.0, yaw = 0.0):
        """ Sends the target position commands in NED format.

        'n', 'e', and 'd' are position in m in NED format.

        """     
        await self.vehicle.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))
    
    async def send_velocity_body(self, vx = 0.0, vy = 0.0, vz = 0.0, wz = 0.0):
        """ Sends the target velocity commands in body-fixed frame.

        'vx', 'vy', and 'vz' are velocities in m/s.

        """     
        await self.vehicle.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, wz))
    
    async def send_attitude_rate(self, roll_rate = 0.0, pitch_rate = 0.0, yaw_rate = 0.0, thrust = 0.0):
        """ Sends the target attitude rate commands.

        'roll_rate', 'pitch_rate', and 'yaw_rate' are angular rates in deg/s.

        CMD_TRANSPORT=dds routes this through the low-latency uXRCE-DDS path
        (DDSRateSender, lazily created) instead of MAVSDK. PX4 stays in the
        OFFBOARD mode entered by MAVSDK at takeoff; the DDS body_rate stream
        seamlessly takes over (same uORB topics, no failsafe gap).
        """
        if self._cmd_transport == "dds":
            if self._dds is None:   # fallback (start() normally creates it eagerly)
                from dds_setpoint import DDSRateSender
                self._dds = DDSRateSender(time_keeper=self._time)
            self._dds.send_rates(roll_rate, pitch_rate, yaw_rate, thrust)
            return
        await self.vehicle.offboard.set_attitude_rate(
                AttitudeRate(roll_rate, pitch_rate, yaw_rate, thrust))

    async def send_attitude(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, thrust=0.0):
        """Send a target ATTITUDE (Euler deg, body->NED) + normalized thrust [0,1].

        PX4's default attitude/rate PID tracks the setpoint. Used by the isolated
        CBF test (apps/cbf_isolation_test.py) to command theta_safe as a tilt while
        the rest of the stack stays stock PX4. Always goes via MAVSDK offboard
        (no DDS path — DDS only carries body rates).
        """
        await self.vehicle.offboard.set_attitude(
                Attitude(float(roll_deg), float(pitch_deg), float(yaw_deg), float(thrust)))

    async def _getOdometry(self):
        try:
            async for odometry in self.vehicle.telemetry.odometry():
                self._odo_ts.append(self._time.perf_counter())
                self._pos_body.append(odometry.position_body)
                self. _quat.append(odometry.q)
                self._vel_body.append(odometry.velocity_body)
                self._ang_vel_body.append(odometry.angular_velocity_body)
                # print("0")
                if not self._STAY_OPEN:
                    break
                # await asyncio.sleep(0.01)
        except grpc.RpcError as e:
            print(f"gRPC error during odometry retrieval: {e.details()}")
        except Exception as e:
            print(f"Unexpected error during odometry retrieval: {e}")

    async def _getAcc(self):
        try:
            async for imu in self.vehicle.telemetry.imu():
                self._acc_frd.append(imu.acceleration_frd)
                self._ang_vel_frd.append(imu.angular_velocity_frd)
                self._imu_ts.append(self._time.perf_counter())
                # print("6")
                if not self._STAY_OPEN:
                    break
                # await asyncio.sleep(0.01)
        except grpc.RpcError as e:
            print(f"gRPC error during acceleration retrieval: {e.details()}")
        except Exception as e:
            print(f"Unexpected error during acceleration retrieval: {e}")

    async def _getDistanceSensor(self):
        """Subscribe to the downward rangefinder. See __init__'s comment --
        currently only consumed by hw_pos_feedback.py's opt-in
        PLASMC_HW_USE_RANGEFINDER_Z path."""
        try:
            async for ds in self.vehicle.telemetry.distance_sensor():
                self._dist_sensor.append(ds)
                self._dist_sensor_ts.append(self._time.perf_counter())
                if not self._STAY_OPEN:
                    break
        except grpc.RpcError as e:
            print(f"gRPC error during distance sensor retrieval: {e.details()}")
        except Exception as e:
            print(f"Unexpected error during distance sensor retrieval: {e}")

    async def _getLandedState(self):
        """Subscribe to PX4's LandedState — onboard-only landing detection.
        PX4 fuses accel/baro/gyro/EKF to publish one of:
          IN_AIR, LANDING, ON_GROUND, TAKING_OFF, UNKNOWN
        Updates self.LANDED on transitions so the main loop can exit cleanly
        without relying on Gazebo ground truth.
        """
        try:
            async for state in self.vehicle.telemetry.landed_state():
                self._flight_state.append(state)
                # Only flip LANDED → True if we previously took off. Initial
                # ON_GROUND-before-takeoff is normal and arm_and_takeoff sets
                # LANDED=False explicitly after climbing.
                if state == LandedState.ON_GROUND and not self.LANDED:
                    print("[FC] PX4 reports ON_GROUND — LANDED=True")
                    self.LANDED = True
                # BUG FIX (2026-08-04, fix (b) -- see the _landed_reset_allowed
                # comment in __init__ for the full race-condition writeup): a
                # late/stale ON_GROUND sample arriving right after
                # arm_and_takeoff's LANDED=False reset (still-genuinely-grounded
                # AUTO_TAKEOFF-plateau case) can re-latch LANDED=True with nothing
                # left to clear it once the OFFBOARD climb afterward actually
                # succeeds. This general safety net clears it on IN_AIR/
                # TAKING_OFF telemetry too -- but ONLY while _landed_reset_allowed
                # is still True (i.e. before real altitude has been confirmed).
                # Gated so it can NEVER undo a genuine post-landing latch (PX4
                # ON_GROUND or the accel-spike _impactDetector) from a late/
                # transient IN_AIR sample during a hard-landing bounce.
                elif (state in (LandedState.IN_AIR, LandedState.TAKING_OFF)
                        and self.LANDED and self._landed_reset_allowed):
                    print(f"[FC] PX4 reports {state} during takeoff -- clearing stale LANDED=True")
                    self.LANDED = False
                if not self._STAY_OPEN:
                    break
        except grpc.RpcError as e:
            print(f"gRPC error during landed_state retrieval: {e.details()}")
        except Exception as e:
            print(f"Unexpected error during landed_state retrieval: {e}")

    async def _impactDetector(self):
        """Fast touchdown detection via accelerometer spike.
        PX4's LandedState typically takes 1–3 s of stillness to fire; in our
        IC 5 test the gap was 3.3 s, during which the drone slid 0.9 m on its
        gear. The physical impact shows up as a clear spike in body-frame
        accel magnitude (~15.8 m/s² at touchdown vs ~9.81 baseline gravity).
        We watch for |a| above IMPACT_THRESH and flip LANDED immediately so
        the main loop can disarm before slide accumulates. Once airborne
        (LANDED=False), we wait for at least one descent-detection cycle to
        avoid triggering on the brief tilt transients of takeoff.
        """
        # Threshold tuned 2026-05-18 after multi-IC sweep: true touchdowns
        # spike to 500–900 m/s² in Gazebo (the physics engine produces large
        # contact impulses); false positives during descent maneuvers cluster
        # at 13–14 m/s². 50 m/s² (~5 g) gives 10× margin under real impacts
        # and 4× over the SMC's descent transients.
        IMPACT_THRESH = 50.0
        try:
            while self._STAY_OPEN:
                await asyncio.sleep(0.005)        # 200 Hz check
                if self.LANDED or not self._acc_frd:
                    continue
                a = self._acc_frd[-1]
                a_mag = (a.forward_m_s2**2 + a.right_m_s2**2 + a.down_m_s2**2) ** 0.5
                if a_mag > IMPACT_THRESH:
                    print(f"[FC] Impact detected (|a|={a_mag:.1f} m/s² > "
                          f"{IMPACT_THRESH}) — LANDED=True")
                    self.LANDED = True
        except Exception as e:
            print(f"Impact detector error: {e}")
                           
    async def print_status_text(self):
        try:
            async for status_text in self.vehicle.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
                # print("7")
                if not self._STAY_OPEN:
                    break
                # await asyncio.sleep(0.01)
        except grpc.RpcError as e:
            print(f"gRPC error during status text retrieval: {e.details()}")
        except Exception as e:
            print(f"Unexpected error during status text retrieval: {e}")
        
    def getPosBody(self):
        return self._pos_body[-1]
    
    def getQuat(self):
        return self._quat[-1]

    def getVelBody(self):
        return self._vel_body[-1]

    def getAngVelIMU(self):
        # None until the IMU task posts its first sample (can lag the odometry/
        # quat task at boot); callers (e.g. Image_Node angvel deque) must handle None.
        return self._ang_vel_frd[-1] if self._ang_vel_frd else None

    def getAccIMU(self):
        # None until the IMU task posts its first sample - same race as
        # getAngVelIMU() (both come from _getAcc()); callers must handle None.
        return self._acc_frd[-1] if self._acc_frd else None

    def getDistanceSensor(self):
        # None until the rangefinder task posts its first sample, OR if no
        # rangefinder is actually publishing telemetry - callers must handle None.
        return self._dist_sensor[-1] if self._dist_sensor else None

    def getLogData(self):
        return {
            "Position Body": self._pos_body,
            "Quaternion": self. _quat,
            "Velocity Body": self._vel_body,
            "Angular Velocity Body": self._ang_vel_body,
            "Odometry Timestamp": self._odo_ts,
            "Acceleration": self._acc_frd,
            "Angular Velocity FRD": self._ang_vel_frd,
            "IMU Timestamp": self._imu_ts,
            "Distance Sensor": self._dist_sensor,
            "Distance Sensor Timestamp": self._dist_sensor_ts,
        }