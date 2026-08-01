"""Record a short live run (real IMG_PROCESSOR + FC, camera pointed at the
marker with some yaw motion) and save Img_Data.npy + Telemetry_Data.npy in
the exact format derive_pi_cal.py::check_mount_rotation expects (no mocap
needed - that function only reads these two files), so the ALREADY-VALIDATED
offline diagnostic can run against it directly instead of re-deriving the
math standalone again.

ALSO records the camera feed for the SAME run via IMG_PROCESSOR's OWN built-in
RECORD flag (img_data.py:512-525) - already correctly debayers and uses the
real negotiated resolution, no separate video-capture code needed. Saves to
Test_Data/Calibration/Test_Videos/<timestamp>.mp4 (that module's own path).

Throwaway diagnostic - not wired into anything, discard after use.
"""
import os
os.environ.setdefault("FLOW_DH_MAX", "0")
os.environ.setdefault("FLOW_DS_MAX", "0")
os.environ.setdefault("FLOW_LOOM_DECOUPLE", "0")
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")
# WIRED 2026-08-01 to match hardware_landing.py's now-validated real-flight
# defaults (FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13) - this script
# previously left CAPTURE_RATE/CAM_EXPOSURE_US/CAM_AUTO_GAIN at old defaults.
os.environ.setdefault("CAPTURE_RATE_HZ", "30")
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

import asyncio
import time
import traceback
import numpy as np
from flight_controller import FC
import img_data as ID

# No live cv2.imshow preview - it collapsed the processing loop to ~1.8Hz in
# an earlier test (56Hz -> 1.8Hz). img_node.RECORD (set below, after
# IMG_PROCESSOR construction) gives the same after-the-fact review via a
# saved video file instead, at no such cost.
ID.VIDEO = False

CAPTURE_RATE = int(os.environ.get("CAPTURE_RATE_HZ", "60"))
RESOLUTION = (640, 480)
RUN_SECONDS = 20
HOLD_TIMEOUT_S = 60.0
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "Test_Data", "RotationCheck",
                        time.strftime("%a %b %d %H-%M-%S %Y"))


def _save(img_node, fc):
    """Save whatever was captured, whether the run completed normally or was
    interrupted - a partial recording (even just the hold-for-marker phase)
    still has real Img_Data.npy/Telemetry_Data.npy content worth keeping,
    same 'don't throw away partial data' principle as output_calibration.py's
    own finally block."""
    img_log = img_node.getLogData()
    fc_log = fc.getLogData()
    os.makedirs(OUT_DIR, exist_ok=True)
    np.save(os.path.join(OUT_DIR, "Img_Data.npy"), img_log, allow_pickle=True)
    np.save(os.path.join(OUT_DIR, "Telemetry_Data.npy"), fc_log, allow_pickle=True)
    print(f"\nSaved to: {OUT_DIR}")
    print(f"  Img_Data.npy: {len(img_log['Time'])} corner-decoded frames, "
          f"{len(img_log['Ring Time'])} total processed frames")
    print(f"  Telemetry_Data.npy: {len(fc_log['IMU Timestamp'])} IMU samples, "
          f"{len(fc_log['Odometry Timestamp'])} odometry samples")
    print(f"  Camera feed: Test_Data/Calibration/Test_Videos/ "
          f"(IMG_PROCESSOR's own RECORD path, timestamped separately)")


async def main():
    img_node = None
    fc = None
    try:
        fc = FC()
        await fc.start()
        t0 = time.perf_counter()
        while not fc.has_quat() and (time.perf_counter() - t0) < 20:
            await asyncio.sleep(0.05)
        print(f"FC quat available: {fc.has_quat()}")

        img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, controller=fc)
        img_node.RECORD = True   # built-in video recording, see module docstring above

        print(f"Point the camera at the marker - waiting up to {HOLD_TIMEOUT_S:.0f}s for it "
              f"to become visible (Ctrl+C any time to stop early and save what's captured "
              f"so far)... [recording camera feed the whole time via img_node.RECORD]")
        t0 = time.perf_counter()
        last_print = 0.0
        while not img_node.FEATURE_IS_VISIBLE and (time.perf_counter() - t0) < HOLD_TIMEOUT_S:
            if time.perf_counter() - last_print > 3.0:
                print(f"  ...still waiting ({time.perf_counter()-t0:.0f}s elapsed)")
                last_print = time.perf_counter()
            await asyncio.sleep(0.1)
        print(f"marker visible: {img_node.FEATURE_IS_VISIBLE} after {time.perf_counter()-t0:.1f}s")

        print(f"Recording for {RUN_SECONDS}s - please sweep with YAW ROTATION over the marker "
              f"(rotation-axis check needs real yaw-rate excitation)...")
        await asyncio.sleep(RUN_SECONDS)

    except asyncio.CancelledError:
        # Ctrl+C during asyncio.run() raises CancelledError here, not
        # KeyboardInterrupt directly - same pattern as output_calibration.py.
        print("\nStopped (Ctrl+C) - saving whatever was captured so far...")
    except Exception as e:
        print(f"\nUnexpected error: {e}\n")
        traceback.print_exc()
    finally:
        if img_node is not None and fc is not None:
            try:
                _save(img_node, fc)
            except Exception as e:
                print(f"Failed to save: {e}")
        if img_node is not None:
            img_node.close()   # also releases the RECORD VideoWriter (img_data.py:548-549)
        if fc is not None:
            await fc.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # A second-layer catch: if Ctrl+C lands OUTSIDE the running task (rare,
        # e.g. during asyncio.run()'s own teardown), main()'s try/finally above
        # already handled cleanup/save - this is just to avoid an ugly top-level
        # traceback on the way out.
        print("\nStopped.")
