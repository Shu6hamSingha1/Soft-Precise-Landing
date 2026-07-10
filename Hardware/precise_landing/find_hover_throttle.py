#!/usr/bin/env python3
"""
Manual hover-throttle finder. Takes off to a safe height, then holds a
sequence of fixed throttle values (zero body rate, level attitude) for a few
seconds each, logging the altitude climb/descent rate at each value. The
throttle where climb rate is closest to zero is the real hover throttle for
THIS airframe - use that as HW_HOVER_THROTTLE_NORM for input calibration /
hardware_landing.py.

Does NOT rely on HOVER_THROTTLE_NORM at all - that's exactly the unverified
placeholder this script exists to replace. Takeoff/landing use PX4's own
position-hold (arm_and_takeoff, then vehicle.action.land()), not the
placeholder throttle.

2026-07-09 rewrite (post-incident: throttle=0.55 produced a +1.18 m/s climb
that kept climbing for ~1s after the abort was detected, because the old
code always ran a 20-iteration/~1s "return to neutral hold" step BEFORE
calling land() -- even on abort. Also: nothing was logged to disk, only
printed. Fixes:
  1. On abort, skip the neutral-hold recenter and call land() immediately.
  2. Continuously log full telemetry (fc.getLogData(): position, quat,
     velocity, ang vel, accel, timestamps) to a .npz file on every exit path
     (normal completion, abort, exception, KeyboardInterrupt), and keep
     logging through the landing descent until fc.LANDED so the crash/abort
     tail is captured too.
  3. Default THROTTLE_LIST narrowed around the manually-observed ~0.5 hover
     point instead of the old 0.35-0.75 guess range (0.35 risks a
     power-insufficient crash; the old sweep's own first point, 0.55,
     already climbed).

2026-07-10: added battery voltage logging to enable voltage→hover-throttle
modeling across battery states. Reads battery voltage at takeoff-settle and
stores it with the results for each throttle test, so single-point checks
per-session can build a calibration curve over time.

Usage:
  /home/doctor/denv/bin/python3 find_hover_throttle.py
  THROTTLE_LIST="0.46,0.48,0.50,0.52,0.54" python3 find_hover_throttle.py
"""
import os
import sys
import asyncio
import time
import numpy as np

sys.path.insert(0, ".")

try:
    from flight_controller import FC
    print("Modules imported successfully")
except ImportError as e:
    print(f"Import failed: {e}")
    sys.exit(1)

TAKEOFF_HEIGHT = float(os.environ.get("LANDING_TAKEOFF_HEIGHT_M", "1.5"))
HOLD_S = float(os.environ.get("THROTTLE_HOLD_S", "3.0"))
THROTTLE_LIST = [float(v) for v in
                  os.environ.get("THROTTLE_LIST", "0.46,0.48,0.50,0.52,0.54").split(",")]

# Safety bounds - abort the sweep and land immediately if altitude leaves
# this band around the takeoff height (climbing away or dropping too fast).
ALT_MARGIN_M = float(os.environ.get("THROTTLE_ALT_MARGIN_M", "0.5"))
MAX_CLIMB_RATE_ABORT = float(os.environ.get("THROTTLE_MAX_CLIMB_RATE", "1.0"))  # m/s

LOG_DIR = os.environ.get("THROTTLE_LOG_DIR", "Test_Data/HoverThrottle")


def _save_log(fc, results, tag):
    """Dump fc's full internal telemetry buffers to disk. Safe to call from
    any exit path (including exceptions) - fc buffers continuously in the
    background regardless of what this script does with it."""
    if fc is None:
        return
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(LOG_DIR, f"hover_throttle_{tag}_{ts}.npz")
        log = fc.getLogData()

        def _arr(v):
            try:
                return np.array(v, dtype=object)
            except Exception:
                return np.array([], dtype=object)

        np.savez(
            path,
            results=np.array(results, dtype=float) if results else np.zeros((0, 3)),
            **{k: _arr(v) for k, v in log.items()},
        )
        print(f"[LOG] Saved full telemetry to {path}")
    except Exception as e:
        print(f"[LOG] Failed to save telemetry log: {e}")


async def _wait_landed(fc, timeout=15.0):
    """Keep polling (and letting fc's background tasks keep logging) until
    PX4 reports LANDED, so the log captures the full descent tail too."""
    t0 = time.perf_counter()
    while not fc.LANDED:
        if (time.perf_counter() - t0) > timeout:
            print("[LAND] Timed out waiting for LANDED state (logging stops here).")
            break
        await asyncio.sleep(0.1)


async def main():
    fc = None
    results = []
    battery_voltage = None
    try:
        print("=" * 60)
        print("Hover-Throttle Finder")
        print("=" * 60)
        print(f"Throttle values to test: {THROTTLE_LIST}")
        print(f"Hold per value: {HOLD_S}s, takeoff height: {TAKEOFF_HEIGHT}m")
        print(f"Safety: abort if altitude drifts >{ALT_MARGIN_M}m from takeoff "
              f"height, or climb rate exceeds {MAX_CLIMB_RATE_ABORT} m/s\n")

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

        # Read battery voltage at takeoff for voltage→hover-throttle modeling
        try:
            battery = await fc.vehicle.telemetry.battery().__aiter__().__anext__()
            battery_voltage = float(battery.voltage_v)
            print(f"Battery voltage: {battery_voltage:.2f}V\n")
        except Exception as e:
            print(f"[WARN] Could not read battery voltage: {e}\n")
            battery_voltage = None

        sweep_aborted = False
        for throttle in THROTTLE_LIST:
            print(f"--- Testing throttle={throttle:.3f} ---")
            samples = []  # (t, alt)
            step_t0 = time.perf_counter()
            aborted = False
            while (time.perf_counter() - step_t0) < HOLD_S:
                await fc.send_attitude_rate(0.0, 0.0, 0.0, throttle)
                pos = fc.getPosBody()
                alt = -pos.z_m if pos else takeoff_alt
                now = time.perf_counter() - step_t0
                samples.append((now, alt))

                if abs(alt - takeoff_alt) > ALT_MARGIN_M:
                    print(f"  [ABORT] altitude {alt:.2f}m drifted >{ALT_MARGIN_M}m "
                          f"from takeoff height {takeoff_alt:.2f}m")
                    aborted = True
                    break

                await asyncio.sleep(0.05)

            if len(samples) >= 2:
                t_arr = np.array([s[0] for s in samples])
                a_arr = np.array([s[1] for s in samples])
                climb_rate = float(np.polyfit(t_arr, a_arr, 1)[0])  # m/s, linear fit slope
                print(f"  throttle={throttle:.3f} -> climb_rate={climb_rate:+.3f} m/s "
                      f"(alt {a_arr[0]:.2f} -> {a_arr[-1]:.2f}m)")
                # Store (throttle, climb_rate, battery_voltage)
                results.append((throttle, climb_rate, battery_voltage if battery_voltage else 0.0))

                if abs(climb_rate) > MAX_CLIMB_RATE_ABORT:
                    print(f"  [ABORT] climb rate {climb_rate:+.2f} m/s exceeds safety "
                          f"limit {MAX_CLIMB_RATE_ABORT} m/s")
                    aborted = True

            if aborted:
                # Do NOT run the neutral-hold recenter step here - that's what
                # kept the vehicle climbing for ~1s after abort in the
                # previous version. Land immediately.
                sweep_aborted = True
                print("Aborting sweep, landing immediately.")
                break

            # Only reached on a clean (non-aborted) step: return to a neutral
            # hold between test points using position-hold, not the tested
            # throttle (which may not be near hover).
            p_now = fc.getPosBody()
            q_now = fc.getQuat()
            yaw_now = float(np.degrees(np.arctan2(
                2.0 * (q_now.w * q_now.z + q_now.x * q_now.y),
                1.0 - 2.0 * (q_now.y * q_now.y + q_now.z * q_now.z))))
            for _ in range(20):
                await fc.send_position_ned(p_now.x_m, p_now.y_m, p0.z_m, yaw_now)
                await asyncio.sleep(0.05)

        print("\n" + "=" * 60)
        print("RESULTS (throttle -> climb rate -> battery_V)")
        print("=" * 60)
        for throttle, rate, batt_v in results:
            marker = "  <-- closest to hover" if results and \
                throttle == min(results, key=lambda r: abs(r[1]))[0] else ""
            print(f"  {throttle:.3f}  ->  {rate:+.3f} m/s  (V={batt_v:.2f}){marker}")
        if results:
            best = min(results, key=lambda r: abs(r[1]))
            print(f"\nBest estimate: throttle={best[0]:.3f} (climb rate {best[1]:+.3f} m/s, "
                  f"battery {best[2]:.2f}V)")
            print("Refine by narrowing THROTTLE_LIST around this value and re-running, "
                  "e.g. THROTTLE_LIST=\"{:.3f},{:.3f},{:.3f}\"".format(
                  best[0]-0.02, best[0], best[0]+0.02))
        print("=" * 60)

        print("\nLanding...")
        await fc.vehicle.action.land()
        await _wait_landed(fc)
        _save_log(fc, results, "aborted" if sweep_aborted else "complete")

    except KeyboardInterrupt:
        print("\nInterrupted - landing")
        if fc:
            try:
                await fc.vehicle.action.land()
                await _wait_landed(fc)
            except Exception:
                pass
            _save_log(fc, results, "interrupted")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        if fc:
            try:
                await fc.vehicle.action.land()
                await _wait_landed(fc)
            except Exception:
                pass
            _save_log(fc, results, "error")
    finally:
        if fc:
            await fc.close()


if __name__ == "__main__":
    asyncio.run(main())
