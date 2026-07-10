#!/usr/bin/env python3
"""
Basic takeoff / hover / land test. No camera, no controller logic -
just verifies arm -> takeoff -> offboard hold -> land works end to end
on this airframe before moving on to hover-throttle finding and
input/output calibration.

Usage:
  /home/doctor/denv/bin/python3 basic_flight_test.py
  LANDING_TAKEOFF_HEIGHT_M=1.0 HOVER_HOLD_S=10 python3 basic_flight_test.py
"""
import os
import sys
import asyncio
import time

sys.path.insert(0, ".")

try:
    from flight_controller import FC
    print("Modules imported successfully")
except ImportError as e:
    print(f"Import failed: {e}")
    sys.exit(1)

TAKEOFF_HEIGHT = float(os.environ.get("LANDING_TAKEOFF_HEIGHT_M", "1.5"))
HOVER_HOLD_S = float(os.environ.get("HOVER_HOLD_S", "8.0"))

# Abort hover-hold and land immediately if altitude drifts beyond this
# margin from the takeoff height.
ALT_MARGIN_M = float(os.environ.get("HOVER_ALT_MARGIN_M", "0.75"))


async def main():
    fc = None
    try:
        print("=" * 60)
        print("Basic Takeoff / Hover / Land Test")
        print("=" * 60)
        print(f"Takeoff height: {TAKEOFF_HEIGHT}m, hover hold: {HOVER_HOLD_S}s")
        print(f"Safety: abort hover and land if altitude drifts >{ALT_MARGIN_M}m\n")

        fc = FC()
        await fc.start()
        t0 = time.perf_counter()
        while not fc.has_quat():
            if (time.perf_counter() - t0) > 20:
                raise RuntimeError("Unable to get data from Flight Controller.")
            await asyncio.sleep(0.05)

        await fc.arm_and_takeoff(takeoff_hgt=TAKEOFF_HEIGHT)
        await asyncio.sleep(1.0)  # settle

        p0 = fc.getPosBody()
        takeoff_alt = -p0.z_m
        print(f"Takeoff settled at alt={takeoff_alt:.2f}m")

        print(f"\n-- Holding position for {HOVER_HOLD_S:.1f}s --")
        q0 = fc.getQuat()
        import numpy as np
        yaw0 = float(np.degrees(np.arctan2(
            2.0 * (q0.w * q0.z + q0.x * q0.y),
            1.0 - 2.0 * (q0.y * q0.y + q0.z * q0.z))))

        hold_t0 = time.perf_counter()
        aborted = False
        while (time.perf_counter() - hold_t0) < HOVER_HOLD_S:
            await fc.send_position_ned(p0.x_m, p0.y_m, p0.z_m, yaw0)
            pos = fc.getPosBody()
            alt = -pos.z_m if pos else takeoff_alt
            if abs(alt - takeoff_alt) > ALT_MARGIN_M:
                print(f"  [ABORT] altitude {alt:.2f}m drifted >{ALT_MARGIN_M}m "
                      f"from takeoff height {takeoff_alt:.2f}m")
                aborted = True
                break
            await asyncio.sleep(0.05)

        if not aborted:
            print(f"Hover hold complete. Final alt={(-fc.getPosBody().z_m):.2f}m")

        print("\n-- Landing --")
        await fc.vehicle.action.land()
        print("Land command sent.")

    except KeyboardInterrupt:
        print("\nInterrupted - landing")
        if fc:
            try:
                await fc.vehicle.action.land()
            except Exception:
                pass
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        if fc:
            try:
                await fc.vehicle.action.land()
            except Exception:
                pass
    finally:
        if fc:
            await fc.close()


if __name__ == "__main__":
    asyncio.run(main())
