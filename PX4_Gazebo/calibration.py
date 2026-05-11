# **************************************************************************
# Based on adaptive_controller.py
# Include initial conditions: non-zero velocity
# **************************************************************************
import os
import asyncio
import time
import pandas as pd
import numpy as np

from flight_controller import FC
# import flowstreamer as fs
import img_data as ID
from numerical_methods import RK5
from controller import Controller

# Setup variables
FENCE = (-5.0, 5.0, -5.0, 5.0, -5.0, 5.0) # xmin, xmax, ymin, ymax, zmin, zmax -- bounding box all values in m

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
RESOLUTION = (640, 480)
SLEEP_TIME = 1/60
REF_RAD_OPT_FLOW = -0.30
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# Flight Controller Details:
TAKEOFF_HEIGHT = 0.5        # in metres
INITIAL_HEIGHT = 1.5        # in metres
INITIAL_VELOCITY = 0.2      # in m/s
LANDING_HEIGHT = 0.5       # in metres
LANDING_VELOCITY = 0.20     # in m/s
HOME_LOCATION = (13.017442, 77.565477, 955.0) #Lab GPS location and sea level altitude

# Flags
CONTROLLER_READY = False

async def maneuver():
    # print("-- Go 0m North, 0m East, -5m Down \
    #         within local coordinate system")
    await int_controller.send_local_position(0.0, 0.0, -3.0)
    await asyncio.sleep(0.1)

    # print("-- Go 5m North, 0m East, -5m Down \
    #         within local coordinate system, turn to face East")
    await int_controller.send_local_position(1.0, 0.0, -3.0)
    await asyncio.sleep(0.1)

    # print("-- Go 5m North, 10m East, -5m Down \
    #         within local coordinate system")
    await int_controller.send_local_position(1.0, 2.0, -3.0)
    await asyncio.sleep(0.1)

    # print("-- Go 0m North, 10m East, 0m Down \
    #         within local coordinate system, turn to face South")
    await int_controller.send_local_position(0.0, 2.0, -3.0)
    await asyncio.sleep(0.1)

if __name__ == '__main__':
    try:
        img_data = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION) # start the thread for onboard camera flow streaming 
        ext_controller = Controller(REF_RAD_OPT_FLOW, DES_IMG_FEATURE_PARAM)

        int_controller = FC(bounds=FENCE, origin = HOME_LOCATION)
        
        # Run the asyncio loop
        loop = asyncio.get_event_loop()
        loop.run_until_complete(int_controller.init_setup())

        # # Wait for sensor data
        # start_time = time.perf_counter()
        # while img_data.getImgFeatureParam() is None:
        #     if (time.perf_counter() - start_time) > 20:
        #         raise Exception("Unable to get data from Companion Computer.")
        #     time.sleep(0.5)
            
        # if int_controller.CONNECTED and int_controller.getFlightStatus() != 1:                
        #     img_data.res = input("Do you want to record? (y/n)")

        CONTROLLER_READY = True
        start_time = time.perf_counter()
        # while not int_controller.LANDED:
        while img_data.is_alive() and not int_controller.LANDED:
            t_c = time.perf_counter() - start_time
            loop.run_until_complete(maneuver())

            # if img_data.FEATURE_IS_VISIBLE:
            #     feature_param = img_data.getImgFeatureParam()
            #     opt_flow_ang_vel = img_data.getOptFlowAngVel()
            #     a_u = ext_controller.PLASMC(feature_param, opt_flow_ang_vel, t_c)
            #     # print(a_u)

            # else:
            #     loop.run_until_complete(int_controller.send_local_position(0.0, 0.0, int_controller.getPos().down_m))
            #     # start_time = time.perf_counter()
            #     # print(f"N:{int_controller.getPos().north_m}, E:{int_controller.getPos().east_m}, D:{int_controller.getPos().down_m}")
            #     # loop.run_until_complete(int_controller.send_local_velocity(0.0, 0.0, 0.0))
            # time.sleep(SLEEP_TIME)

            
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Main Thread\n")

    except RuntimeError as e:
        print(f"RuntimeError: Main Thread: {e}\n")

    except SyntaxError:
        print("SyntaxError: Main Thread\n")

    except Exception as e:
        print(f"Unexpected error: Main Thread: {e}\n")

    finally:
        # Save data
        x = input('Do you want to save the dataset? (y/n)')
        if x != 'n' and CONTROLLER_READY:
            # record timestamp
            timestamp = time.ctime().replace(':', '-')

            # Create a directory named based on timestamp
            dir_name = f"/home/shubham/ws/Test_Data/Calibration/{timestamp}"
            os.makedirs(dir_name)

            # Save files inside the folder 
            np.save(f'{dir_name}/Img_Data', img_data.getLogData())
            np.save(f'{dir_name}/Telemetry_Data', int_controller.getLogData())
            np.save(f'{dir_name}/Control_Data', ext_controller.getLogData())
            np.save(f'{dir_name}/Control_Params', ext_controller.getParams())
            f = open(f'{dir_name}/Img_Params.txt', 'w')
            f.write(img_data.getParams())
            f.close()

        # Closing all the active threads
        if int_controller.CONNECTED:
            loop.run_until_complete(int_controller.close())
        
        del int_controller
        if img_data.is_alive():
            img_data.close()
        
        # Ensure all threads are properly shut down
        time.sleep(0.1)
        loop.close()

        img_data.join()