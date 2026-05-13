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
import time 
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed, AccelerationNed, AttitudeRate, ActuatorControl, ActuatorControlGroup)
from mavsdk.telemetry import LandedState
    
class FC():
    def __init__(self, time_keeper=time):
        self._time = time_keeper
        # Flags
        self.CONNECTED = False
        self._OFFBOARD = False
        self.LANDED = True
        self._STAY_OPEN  = True

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
            await self.vehicle.connect(system_address="udp://:14540")
            # await self.vehicle.connect(system_address="serial:///dev/ttyACM0:57600")

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
                asyncio.create_task(self.print_status_text())
            ]

            await self.vehicle.telemetry.set_rate_odometry(60)
            await self.vehicle.telemetry.set_rate_imu(60)

            await self.vehicle.action.hold()
                
        except KeyboardInterrupt:
            print("KeyboardInterrupt: Flight Controller Thread\n")

        except RuntimeError as e:
            print("RuntimeError: Flight Controller Thread: {e}\n")

        except SyntaxError as e:
            print("SyntaxError: Flight Controller Thread: {e}\n")

        except Exception as e:
            print(f"Unexpected error: Flight Controller Thread: {e}\n")
    
    #-- Close connection
    async def close(self):
        """
        Gracefully close the flight controller connection.
        """
        if not self.CONNECTED:
            return
        
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
        #  Arm.
        if takeoff_hgt is None:
            arm_res = input('Do you want to arm? (y/n): ')
        else:
            arm_res = 'y'
        if arm_res != 'n':
            # PX4 prints "Ready for takeoff" before all preflight checks settle.
            # arm() can still return COMMAND_DENIED for ~5-15 s after that
            # (heading estimate / EKF still converging). Retry instead of crashing.
            print("-- Arming")
            arm_err = None
            for attempt in range(10):                  # ~10 * 2s = 20 s budget
                try:
                    await self.vehicle.action.arm()
                    arm_err = None
                    break
                except Exception as e:
                    arm_err = e
                    print(f"  arm attempt {attempt + 1}/10 denied: {e} — waiting 2s")
                    await asyncio.sleep(2.0)
            if arm_err is not None:
                raise RuntimeError(f"arm() failed after 10 retries: {arm_err}")
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
            await self.vehicle.action.takeoff()
        #-- wait to reach the target altitude
            while self._STAY_OPEN :
                await asyncio.sleep(0.5)
                if -self._pos_body[-1].z_m >= 0.9*takeoff_hgt:
                    print("Altitude reached")
                    print(self._pos_body[-1].z_m, takeoff_hgt)
                    break
            self.LANDED = False
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

        """
        await self.vehicle.offboard.set_attitude_rate(
                AttitudeRate(roll_rate, pitch_rate, yaw_rate, thrust))

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
        return self._ang_vel_frd[-1]

    def getAccIMU(self):
        return self._acc_frd[-1]
    
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
        }