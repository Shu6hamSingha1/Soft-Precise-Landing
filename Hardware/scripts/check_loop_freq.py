"""Confirm the ACTUAL loop frequencies with this session's img_data.py/
imgstreamer.py/output_calibration.py changes live, using the real production
IMG_PROCESSOR class (not a standalone imgstream benchmark) - so gyro comp,
ring-flow-every-frame logging, checkposts (disabled, matching output_
calibration.py's calibration-recording env defaults) etc. are all exercised
exactly as a real recording would use them.

Key metric: "Ring Time" (img_data.py's _ring_time_log) now logs UNCONDITIONALLY
every processed frame (this session's decoupling fix) - so diff(Ring Time) IS
the true img_data.py processing-loop rate, independent of whether the corner
marker was ever decoded. "Time" (_time_log) stays corner-success-gated as
before, so comparing the two shows detection yield on top of the raw loop rate.

Throwaway diagnostic - not wired into anything, discard after use.
"""
import os
os.environ.setdefault("FLOW_DH_MAX", "0")
os.environ.setdefault("FLOW_DS_MAX", "0")
os.environ.setdefault("FLOW_LOOM_DECOUPLE", "0")
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")   # matches output_calibration.py's default

# WIRED 2026-08-01 to match hardware_landing.py's now-validated real-flight defaults
# (see FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13) -- this script previously
# hardcoded CAPTURE_RATE=60 and left CAM_EXPOSURE_US/CAM_ANALOGUE_GAIN/CAM_AUTO_GAIN at
# imgstreamer.py's own old defaults (3000us/8.0/off), so a loop-frequency check run here
# was silently measuring a DIFFERENT camera configuration than what real flights now
# actually use. Override on the command line to check other configs (e.g.
# CAPTURE_RATE_HZ=60 CAM_AUTO_GAIN=0 python3 check_loop_freq.py for the old baseline).
os.environ.setdefault("CAPTURE_RATE_HZ", "30")
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")
# UPDATED 2026-07-28: the 2026-07-26 rationale for forcing these to 0 was ITSELF
# reversed the next day (output_calibration.py's 2026-07-27 comment) - the map's
# override never touches the flow-calibration path (only centroid), and disabling
# it truncates the excitation tails right where the calibration signal is largest.
# output_calibration.py now leaves both at img_data.py's own default ("1", on), so
# this script matches that by NOT overriding them (img_data.py defaults apply).
# Override PLANAR_MAP_SHADOW=0 / PLASMC_PLANAR_MAP_PRIMARY=0 in the environment if
# you specifically want to A/B the map's frame-rate cost (~9-13ms/frame, stage
# "1b_planar_map_shadow") in isolation - that tradeoff is real, just not the
# default anymore.

import asyncio
import time
import numpy as np
from flight_controller import FC
import img_data as ID

CAPTURE_RATE = int(os.environ.get("CAPTURE_RATE_HZ", "60"))
# Was a hardcoded 60 -- but IMG_PROCESSOR is called below with capRate=CAPTURE_RATE
# explicitly, which would override img_data.py's own CAPTURE_RATE_HZ-driven default
# (an explicit kwarg always wins over a function default) and silently ignore the
# os.environ.setdefault("CAPTURE_RATE_HZ", "30") above. Reading the same env var here
# keeps this script's explicit pass-through consistent with that default instead of
# fighting it.
RESOLUTION = (640, 480)
RUN_SECONDS = 20


async def main():
    fc = FC()
    await fc.start()
    t0 = time.perf_counter()
    while not fc.has_quat() and (time.perf_counter() - t0) < 20:
        await asyncio.sleep(0.05)
    print(f"FC quat available: {fc.has_quat()}")

    img_node = ID.IMG_PROCESSOR(capRate=CAPTURE_RATE, resolution=RESOLUTION, controller=fc)
    print(f"IMG_PROCESSOR running for {RUN_SECONDS}s - point the camera at the marker "
          f"if you want a corner-detection-rate comparison too (optional for this check)...")
    for i in range(RUN_SECONDS):
        await asyncio.sleep(1.0)
        if img_node.FEATURE_IS_VISIBLE:
            m = img_node.metrics()
            print(f"  t={i+1}s  cam_fps={img_node._fps:.1f}  " +
                  ", ".join(f"{k}={v:.1f}" for k, v in m.items() if isinstance(v, (int, float))))
        else:
            print(f"  t={i+1}s  cam_fps={img_node._fps:.1f}  (marker not visible)")

    log = img_node.getLogData()
    img_node.close()
    await fc.close()

    def rate_stats(label, t_arr):
        t_arr = np.asarray(t_arr, float)
        if len(t_arr) < 3:
            print(f"{label}: too few samples ({len(t_arr)})")
            return
        dt = np.diff(t_arr)
        span = t_arr[-1] - t_arr[0]
        print(f"{label}: n={len(t_arr)}  span={span:.1f}s  overall_avg={len(t_arr)/span:.1f}Hz  "
              f"median_dt={np.median(dt)*1000:.1f}ms (={1.0/np.median(dt):.1f}Hz)  "
              f"p95_dt={np.percentile(dt,95)*1000:.1f}ms  max_gap={dt.max():.2f}s")

    print("\n=== Loop-frequency results ===")
    rate_stats("Ring Time (TRUE processing-loop rate, every frame, this session's fix)", log["Ring Time"])
    rate_stats("Time (corner-success-gated, as before)", log["Time"])


if __name__ == "__main__":
    asyncio.run(main())
