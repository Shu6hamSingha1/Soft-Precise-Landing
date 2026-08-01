"""Standalone bench test for CBF_CORNERS_STALE (controller.py property, added
2026-07-30) -- NO MAVSDK arming, NO motors. Motivation: hardware_landing.py's
own CBF_CORNERS_STALE check only runs inside the post-arm control loop (see
`feature_fresh` at hardware_landing.py ~line 196), so a no-arm bench hold
with hardware_landing.py itself can NEVER exercise it (confirmed 2026-07-30 --
four consecutive no-arm runs all hung at is_armable and never reached that
code path). This script instantiates the same Controller directly (same
constructor call as HardwareLandingSystem.initialize()) and polls
CBF_CORNERS_STALE on its own, with no flight controller command path at all.

Still connects to the FC via MAVSDK (Controller's constructor needs an FC
instance for quat/angvel feed, same as hardware_landing.py) but NEVER calls
arm_and_takeoff() or sends any actuator command -- motors stay off throughout.

**BUG FOUND AND FIXED 2026-07-30 (same day, after 3 runs all failed to ever
trigger CBF_CORNERS_STALE even across a confirmed unbroken 6.6s marker-loss
window on video):** the `_cbf_corners_none_streak` counter that
CBF_CORNERS_STALE reads is only updated inside Controller._attCtrl()
(controller.py:1242), called from the Controller thread's own run() loop --
but that loop only executes its per-frame body when self._CONTROLLER_READY
is True (controller.py:1132), which only becomes True via the public
startController() method (controller.py:2655-2657) -- normally called by
hardware_landing.py AFTER takeoff, right before the Landing Control Loop.
This script never called it, so the streak-counting code never ran at all,
no matter how long the marker was covered. Fixed by calling
controller.startController() below. startController() itself only sets an
internal readiness flag + warmup counter on the Controller object -- it does
NOT touch MAVSDK, arm, or send any actuator command, so this stays fully
safe/no-arm.

Usage: hold/point the camera at the marker, then deliberately cover it for
several seconds (>1s, i.e. >CBF_CORNERS_STALE_FRAMES frames at the loop's
typical rate) to try to trigger the stale condition. Watch for the
"CBF_CORNERS_STALE FIRED" transition line below.

Ctrl+C to stop.
"""
import os
import sys
import time

sys.path.insert(0, ".")

from flight_controller import FC
from controller import Controller

REF_RAD_OPT_FLOW = float(os.environ.get("LANDING_REF_RAD_OPT_FLOW", "-0.30"))
import numpy as np
DES_IMG_FEATURE_PARAM = np.array([0.0, 0.0, 1.0,
                                   np.deg2rad(float(os.environ.get("DES_ALPHA_DEG", "0.0")))])
POLL_INTERVAL_S = 0.05   # 20 Hz -- plenty to catch a 30-frame-streak transition


def main():
    print("Connecting to flight controller via MAVSDK (no arming)...")
    fc = FC()
    import asyncio
    asyncio.get_event_loop().run_until_complete(fc.start())
    start = time.perf_counter()
    while not fc.has_quat():
        if time.perf_counter() - start > 20:
            raise RuntimeError("Unable to get data from Flight Controller.")
        time.sleep(0.05)
    print("Flight controller connected, quaternion feed live")

    print("Starting Controller (owns its own IMG_PROCESSOR/camera)...")
    controller = Controller(REF_RAD_OPT_FLOW, DES_IMG_FEATURE_PARAM, time, fc, pose_node=None)
    # REQUIRED for the streak-counting code to run at all -- see module
    # docstring's "BUG FOUND AND FIXED" note. Does not arm or touch MAVSDK.
    controller.startController(warmup_steps=0)
    print("Controller thread started AND startController() called (readiness "
          "flag only -- motors are NOT armed and never will be by this "
          "script).\nCover the marker for a few seconds to try to trigger "
          "CBF_CORNERS_STALE.\nPress Ctrl+C to stop.\n")

    was_stale = False
    try:
        while True:
            is_stale = controller.CBF_CORNERS_STALE
            if is_stale and not was_stale:
                print(">>> CBF_CORNERS_STALE FIRED (transitioned False -> True) <<<")
            elif was_stale and not is_stale:
                print(">>> CBF_CORNERS_STALE cleared (transitioned True -> False) <<<")
            was_stale = is_stale
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        print("Cleaning up...")
        if controller.is_alive():
            controller.close()
            controller.join(timeout=3)
        asyncio.get_event_loop().run_until_complete(fc.close())
        print("Cleanup complete")


if __name__ == "__main__":
    main()
