# **************************************************************************
# Basic goto python script
# **************************************************************************
import asyncio
import numpy as np
import time

from flight_controller import FC
from ahrs import RAD2DEG

TAKEOFF_HEIGHT = 1.0        # in metres

FC_node = FC()

def is_at_position(x, y, z, offset):
    """offset: meters"""
    desired = np.array((x, y, z))
    current = FC_node.getPosBody()
    pos = np.array((current.x_m, current.y_m, current.z_m))
    return np.linalg.norm(desired - pos) < offset

async def reach_position(x, y, z, yaw, timeout):
    """timeout(int): seconds"""
    # set a position setpoint
    start_time = time.perf_counter()
    while (time.perf_counter() - start_time) < timeout:
        await FC_node.send_position_ned(x, y, z, yaw)
        if is_at_position(x, y, z, 0.01):
            print(f"Position reached: ({x}, {y}, {z}) in {time.perf_counter() - start_time} seconds")
            return True
        await asyncio.sleep(0.1)
    print(f"Failed to reach position: ({x}, {y}, {z}) within {timeout} seconds")
    return False

def yaw_from_quaternion(q):
    """
    Compute yaw angle (rad) from quaternion.
    Quaternion format: [w, x, y, z]
    """
    yaw = np.arctan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )
    return RAD2DEG*yaw

async def main(res = 'n'):
    global FC_node
    try:
        await FC_node.start()
        await FC_node.arm_and_takeoff(TAKEOFF_HEIGHT)
       
        yaw = yaw_from_quaternion(FC_node.getQuat())      
        await reach_position(0.0, 0.0, FC_node.getPosBody().z_m, yaw, 20)

        await FC_node.vehicle.action.land()
        await FC_node.close()
            
    except KeyboardInterrupt:
        if FC_node:
            await FC_node.vehicle.action.land()
        print("KeyboardInterrupt: Main Thread\n")

    except RuntimeError as e:
        print(f"RuntimeError: Main Thread: {e}\n")

    except SyntaxError:
        print("SyntaxError: Main Thread\n")

    except Exception as e:
        print(f"Unexpected error: Main Thread: {e}\n")

    finally:
        # Close flight controller
        if FC_node:
            await FC_node.close()

if __name__ == "__main__":
    # res = input("Do you want to record? (y/n)")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")