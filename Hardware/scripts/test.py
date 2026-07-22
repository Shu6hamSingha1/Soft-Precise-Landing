# # **************************************************************************
# # Test Image Feed
# # **************************************************************************
# import asyncio
# import time
# import numpy as np
# import img_data_bare as ID
# from flight_controller import FC

# SLEEP_TIME = 1/30

# # Companion Computer Details:
# CAPTURE_RATE = 30 # Capture Rate = {90, 120, 200}
# # RESOLUTION = (1280, 960)
# # RESOLUTION = (640, 480)
# RESOLUTION = (320, 240)
# REF_RAD_OPT_FLOW = -0.30
# DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])

# # Flags
# CONTROLLER_READY = False

# if __name__ == '__main__':
#     # Initialize variables to None to avoid NameError in the `finally` block
#     img_node = None
        
#     try:
#         img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION) # start the thread for onboard camera flow streaming
        
#         while img_node.is_alive():
#             if CONTROLLER_READY:
#                 t_c.append(time.perf_counter() - start_time)

#             else:
#                 start_time = time.perf_counter()
#                 CONTROLLER_READY = True
#                 t_c = [0.0]

#             if time.perf_counter() - start_time > 100.0:
#                 break

#             time.sleep(SLEEP_TIME)

#     except KeyboardInterrupt:
#         print("KeyboardInterrupt: Main Thread\n")        

#     except RuntimeError as e:
#         print(f"RuntimeError: Main Thread: {e}\n")

#     except SyntaxError:
#         print("SyntaxError: Main Thread\n")

#     except Exception as e:
#         print(f"Unexpected error: Main Thread: {e}\n")

#     finally:
#         # Close mocap_node thread
#         if img_node is not None and img_node.is_alive():
#             img_node.close()
#             img_node.join()



# **************************************************************************
# Test Image + FC Feed
# **************************************************************************
import asyncio
import time
import numpy as np
import img_data as ID
from flight_controller import FC

SLEEP_TIME = 1/30

# Companion Computer Details:
CAPTURE_RATE = 60 # Capture Rate = {90, 120, 200}
# RESOLUTION = (1280, 960)
# RESOLUTION = (640, 480)
RESOLUTION = (320, 240)
REF_RAD_OPT_FLOW = -0.30
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0, 0.0])
# Flags
CONTROLLER_READY = False

async def main():
    # Global Logging variable
    global CONTROLLER_READY

    # Initialize variables to None to avoid NameError in the `finally` block
    img_node = None
    FC_node = None

    try:
        FC_node = FC()
        await FC_node.start()

        start_time = time.perf_counter()
        while not FC_node.has_quat():
            if (time.perf_counter() - start_time) > 20:
                raise Exception("Unable to get data from Flight Controller.")
            await asyncio.sleep(0.05)

        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, controller=FC_node) # start the thread for onboard camera flow streaming
        print("Started")
        while img_node.is_alive():
            if CONTROLLER_READY:
                t_c.append(time.perf_counter() - start_time)

            else:
                start_time = time.perf_counter()
                CONTROLLER_READY = True
                t_c = [0.0]

            # if time.perf_counter() - start_time > 100.0:
            #     break

            await asyncio.sleep(SLEEP_TIME)

            if img_node.FEATURE_IS_VISIBLE:
                print(img_node.metrics())
            
        await FC_node.close()

    except KeyboardInterrupt:
        print("KeyboardInterrupt: Main Thread\n")

    except RuntimeError as e:
        print(f"RuntimeError: Main Thread: {e}\n")

    except SyntaxError:
        print("SyntaxError: Main Thread\n")

    except Exception as e:
        print(f"Unexpected error: Main Thread: {e}\n")

    finally:
        # Close mocap_node thread
        if img_node and img_node.is_alive():
            img_node.close()
            img_node.join()

        # Close flight controller
        if FC_node:
            await FC_node.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Clean exit on Ctrl+C")



# # **************************************************************************
# # Test Mocap System
# # **************************************************************************
# import time
# from mocaptools import QTMWrapper

# # # Setup variables
# QTM_IP = '192.168.0.111'

# SLEEP_TIME = 1/30

# # Flags
# CONTROLLER_READY = False

# if __name__ == '__main__':
#     # Initialize variables to None to avoid NameError in the `finally` block
#     mocap_node = None
#     UAV_pose = []
#     target_pose = []

#     try:
#         mocap_node = QTMWrapper(QTM_IP)

#         start_time = time.perf_counter()
#         while not mocap_node.getPose():
#             time.sleep(1)
#             if (time.perf_counter() - start_time) > 20:
#                 raise Exception("Unable to connect to MoCap System.")
#             # else:
#             #     print("Waiting for connection setup with the Mocap System...")
#         print(mocap_node.getPose())
        
#         while mocap_node.is_alive():
#             if CONTROLLER_READY:
#                 t_c.append(time.perf_counter() - start_time)

#             else:
#                 start_time = time.perf_counter()
#                 CONTROLLER_READY = True
#                 t_c = [0.0]
#                 restart_time = start_time

#             UAV_pose.append(mocap_node.getPose()["UAV"])
#             target_pose.append(mocap_node.getPose()["target"])
#             print(f"Target | UAV = {UAV_pose[-1]} | {target_pose[-1]}")

#             time.sleep(SLEEP_TIME)
#             if time.perf_counter() - restart_time > 1.0:
#                 # print(f"Commanded Profile | Position: {pos_cmd} | {UAV_pose[-1]}")
#                 restart_time = time.perf_counter()
#                 break

#     except KeyboardInterrupt:
#         print("KeyboardInterrupt: Main Thread\n")        

#     except RuntimeError as e:
#         print(f"RuntimeError: Main Thread: {e}\n")

#     except SyntaxError:
#         print("SyntaxError: Main Thread\n")

#     except Exception as e:
#         print(f"Unexpected error: Main Thread: {e}\n")

#     finally:
#         # Close mocap_node thread
#         if mocap_node is not None and mocap_node.is_alive():
#             mocap_node.close()
#             mocap_node.join()