# **************************************************************************
# Basic goto python script
# **************************************************************************
import asyncio
import numpy as np
import rclpy # Python library for ROS 2
import time

from flight_controller import FC
from gz_subscriber import GZ_Subscriber, Pose_Node, Clock_Node
import img_data as ID

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
RESOLUTION = (320, 240)

# Flight Controller Details:
TAKEOFF_HEIGHT = 5.0        # in metres

async def main(res = 'n'):
    time_subscriber = None
    pose_subscriber = None
    img_node = None
    try:
        rclpy.init()
        pose_node = Pose_Node()
        pose_subscriber = GZ_Subscriber(pose_node)
        time_node = Clock_Node()
        time_subscriber = GZ_Subscriber(time_node)

        start_time = time.perf_counter()
        while time_node.perf_counter() is None:
            if (time.perf_counter() - start_time) > 10:
                raise Exception("Unable to get simulation time.")

        FC_node = FC(time_node)
        await FC_node.start()
        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, time_keeper=time_node, controller=FC_node) # start the thread for onboard camera flow streaming
        img_node.RECORD = res
        await FC_node.arm_and_takeoff(TAKEOFF_HEIGHT)
         
        while img_node.is_alive() and not FC_node.LANDED:
            # find target pose
            target_pose = pose_node.getPose().target
            uav_pose = pose_node.getPose().UAV
            x = FC_node.getPosBody().x_m + uav_pose.position.y - target_pose.position.y
            y = FC_node.getPosBody().y_m + uav_pose.position.x - target_pose.position.x
            z = FC_node.getPosBody().z_m
            yaw = target_pose.orientation.z
            await FC_node.send_position_ned(x, y, z+0.3, yaw)
            alt = - FC_node.getPosBody().z_m - uav_pose.position.z + target_pose.position.z
            if alt < 0.3:
                break
            await asyncio.sleep(0.01)

        await FC_node.vehicle.action.land()

        img_node.close()
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
        # Close img_node thread
        if img_node and img_node.is_alive():
            img_node.close()
            img_node.join()

        # Close pose_subscriber thread
        if pose_subscriber and pose_subscriber.is_alive():
            pose_subscriber.close()
            pose_subscriber.join()

        # Close time_subscriber thread
        if time_subscriber and time_subscriber.is_alive():
            time_subscriber.close()
            time_subscriber.join()

        # Close flight controller
        if FC_node:
            await FC_node.close()

        # Shutdown ROS client library if not already shut down
        if rclpy.ok():  # Check if the context is still active
            rclpy.shutdown()

if __name__ == "__main__":
    res = input("Do you want to record? (y/n)")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")