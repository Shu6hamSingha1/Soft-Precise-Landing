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

2026-07-10: default THROTTLE_LIST re-narrowed to 0.38-0.41. The only
FULL-BATTERY data point so far is 0.418 (climbs, +0.458 m/s) -- everything
below that (0.400-0.415, all sinking) was measured on a DEPLETED battery and
does not transfer (same 0.418 command sank at -0.488 m/s on depleted vs
climbed at +0.458 m/s on full charge). This range probes below 0.418 on a
confirmed full charge without assuming the old depleted-battery bracket.

2026-07-10 (later same day): first clean full-sweep completion (all 4 points,
no abort) at battery ~23.0V: 0.380 sinks -0.185, 0.390 climbs +0.033, 0.400
climbs +0.095, 0.410 climbs +0.023. Crossover (hover) sits between 0.380 and
0.390. THROTTLE_LIST re-narrowed to bisect that crossover.

2026-07-10 (two more sweeps, battery draining 22.66V then 22.40V): every value
0.386-0.392 landed within +/-0.2 m/s of zero across all three sweeps (23.0V,
22.66V, 22.40V), with the sign flipping inconsistently between runs at the
same throttle -- this is measurement noise (short 1s hold + wind), not a real
non-monotonic thrust curve. Converged estimate: hover throttle ~0.387-0.389
across this battery range, further bisection not worthwhile below the noise
floor. Default THROTTLE_LIST narrowed to a single confirmation point.

2026-07-10: added THROTTLE_MODE=sine, a continuous sinusoidal throttle sweep
(zero body rate throughout) as an alternative to the discrete THROTTLE_LIST
steps. Motivation: discrete step spacing narrow enough to bisect hover finely
is too narrow to also derive a good thrust slope (see
project_hover_throttle_search_2026_07_09 memory -- the hover-bisection data
gave a wildly inconsistent 6-45 N/unit slope estimate), and each discrete
point only gets one ~1-3s noisy segment. A continuous sine sweep instead
samples the whole throttle range across multiple cycles in one flight,
giving both the hover crossing AND the thrust slope from a single linear fit
of accelerometer specific-force (a_down) vs commanded throttle -- the same
accelerometer-based method already validated for the input-cal thrust-slope
derivation, just applied here directly instead of the position-based
climb_rate proxy. List mode (THROTTLE_MODE=list, default) is unchanged.

Usage:
  /home/doctor/denv/bin/python3 find_hover_throttle.py
  THROTTLE_LIST="0.46,0.48,0.50,0.52,0.54" python3 find_hover_throttle.py
  THROTTLE_MODE=sine SINE_CENTER=0.388 SINE_AMP=0.03 SINE_PERIOD_S=4.0 SINE_CYCLES=3 python3 find_hover_throttle.py
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

TAKEOFF_HEIGHT = float(os.environ.get("LANDING_TAKEOFF_HEIGHT_M", "3.0"))
HOLD_S = float(os.environ.get("THROTTLE_HOLD_S", "1.0"))
THROTTLE_LIST = [float(v) for v in
                  os.environ.get("THROTTLE_LIST", "0.388").split(",")]

# THROTTLE_MODE: 'list' (default, discrete bisection above) | 'sine'
# (continuous sweep, see module docstring).
THROTTLE_MODE = os.environ.get("THROTTLE_MODE", "list")
SINE_CENTER = float(os.environ.get("SINE_CENTER", "0.388"))
SINE_AMP = float(os.environ.get("SINE_AMP", "0.03"))
SINE_PERIOD_S = float(os.environ.get("SINE_PERIOD_S", "4.0"))
SINE_CYCLES = float(os.environ.get("SINE_CYCLES", "3.0"))
G_STD = 9.81  # standard gravity, m/s^2 -- target a_down at true level hover

# Safety bounds, two-tier (2026-07-10 split -- see below):
#   ALT_MARGIN_M      - SOFT cutoff. Ends the current throttle's hold early
#                        (partial samples still used for its climb-rate
#                        estimate) then continues the sweep to the next
#                        throttle. A real nonzero climb/sink rate over the
#                        full HOLD_S will drift past this within the hold on
#                        real hardware (unlike SITL), so this must NOT end
#                        the whole sweep or every run only ever tests its
#                        first throttle value (the exact gap seen 2026-07-09/10:
#                        both 0.380 runs sank -0.2ish m/s and hit this margin
#                        before HOLD_S elapsed, aborting the sweep outright).
#   ALT_HARD_MARGIN_M - HARD abort. Genuinely aborts the sweep and lands
#                        immediately, skipping the neutral-hold recenter.
#   MAX_CLIMB_RATE_ABORT - also a HARD abort (dangerous velocity regardless
#                        of displacement so far).
ALT_MARGIN_M = float(os.environ.get("THROTTLE_ALT_MARGIN_M", "0.5"))
ALT_HARD_MARGIN_M = float(os.environ.get("THROTTLE_ALT_HARD_MARGIN_M", "1.0"))
MAX_CLIMB_RATE_ABORT = float(os.environ.get("THROTTLE_MAX_CLIMB_RATE", "1.0"))  # m/s

LOG_DIR = os.environ.get("THROTTLE_LOG_DIR", "Test_Data/HoverThrottle")


def _save_log(fc, results, tag, extra=None):
    """Dump fc's full internal telemetry buffers to disk. Safe to call from
    any exit path (including exceptions) - fc buffers continuously in the
    background regardless of what this script does with it. `extra` (dict)
    adds additional arrays to the npz -- used by sine mode to save the
    commanded (time, throttle) timeline alongside the usual telemetry."""
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
            **({k: _arr(v) for k, v in extra.items()} if extra else {}),
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


async def _run_sine_sweep(fc, takeoff_alt):
    """Continuous sinusoidal throttle sweep (zero body rate throughout).
    Returns (sine_time, sine_throttle, hard_abort) -- the commanded timeline,
    saved alongside the usual telemetry for later alignment against
    fc.getLogData()'s Acceleration / IMU Timestamp."""
    duration = SINE_PERIOD_S * SINE_CYCLES
    print(f"Sine sweep: center={SINE_CENTER}, amp={SINE_AMP}, period={SINE_PERIOD_S}s, "
          f"cycles={SINE_CYCLES} (duration={duration:.1f}s)")
    sine_time = []
    sine_throttle = []
    hard_abort = False
    t0 = time.perf_counter()
    last_notice = -1.0
    while (time.perf_counter() - t0) < duration:
        t = time.perf_counter() - t0
        throttle = SINE_CENTER + SINE_AMP * np.sin(2 * np.pi * t / SINE_PERIOD_S)
        await fc.send_attitude_rate(0.0, 0.0, 0.0, throttle)
        sine_time.append(t)
        sine_throttle.append(throttle)

        pos = fc.getPosBody()
        alt = -pos.z_m if pos else takeoff_alt
        drift = abs(alt - takeoff_alt)
        if drift > ALT_HARD_MARGIN_M:
            print(f"  [HARD ABORT] altitude {alt:.2f}m drifted >{ALT_HARD_MARGIN_M}m "
                  f"from takeoff height {takeoff_alt:.2f}m")
            hard_abort = True
            break
        # No soft-cutoff tier for sine mode -- there's no discrete "step" to
        # end early; a brief mid-sweep excursion past ALT_MARGIN_M doesn't
        # invalidate the regression the way a full step-abort would in list
        # mode, so just notice it (throttled to ~1/s) and keep sweeping.
        if drift > ALT_MARGIN_M and (t - last_notice) > 1.0:
            print(f"  [notice] altitude {alt:.2f}m drifted >{ALT_MARGIN_M}m (sweep continues)")
            last_notice = t

        await asyncio.sleep(0.02)

    return np.array(sine_time), np.array(sine_throttle), hard_abort


def _fit_sine_sweep(fc, sine_time, sine_throttle):
    """Align commanded throttle onto the IMU timeline and linear-fit IMU
    accelerometer specific force (a_down) vs commanded throttle. Reports the
    throttle at which the fit crosses a_down = -G_STD (the expected specific
    force at true level hover, thrust exactly supporting weight) as the
    hover-throttle estimate, and the raw slope (m/s^2 per throttle unit,
    convertible to N/unit via SLOPE_true = mass * slope) for cross-checking
    against THRUST_SLOPE_N_PER_UNIT."""
    log = fc.getLogData()
    imu_ts = np.array(log.get("IMU Timestamp", []))
    accel = log.get("Acceleration", [])
    if len(imu_ts) == 0 or len(accel) == 0 or len(sine_time) < 2:
        print("[sine fit] insufficient data to fit.")
        return None
    a_down = np.array([a.down_m_s2 for a in accel])
    # sine_time is relative to the sweep's own start; align IMU timestamps
    # (relative to FC start) onto that same relative timeline via the
    # sweep's own first/last bounds.
    imu_rel = imu_ts - imu_ts[0]
    mask = (imu_rel >= sine_time[0]) & (imu_rel <= sine_time[-1])
    imu_rel, a_down = imu_rel[mask], a_down[mask]
    if len(imu_rel) < 2:
        print("[sine fit] insufficient IMU samples within sweep window.")
        return None

    throttle_on_imu = np.interp(imu_rel, sine_time, sine_throttle)
    A = np.polyfit(throttle_on_imu, a_down, 1)
    slope_a_per_throttle, intercept = float(A[0]), float(A[1])
    hover_throttle_est = float((-G_STD - intercept) / slope_a_per_throttle) if slope_a_per_throttle != 0 else float('nan')

    print(f"\n[sine fit] a_down = {slope_a_per_throttle:.2f}*throttle + {intercept:.2f}  (n={len(imu_rel)})")
    print(f"[sine fit] estimated hover throttle (a_down=-{G_STD}) = {hover_throttle_est:.4f}")
    print("[sine fit] NOTE: slope here is in (m/s^2)/throttle -- convert to "
          "N/unit via SLOPE_true = mass * slope_a_per_throttle if cross-checking "
          "against THRUST_SLOPE_N_PER_UNIT.")
    return dict(slope_a_per_throttle=slope_a_per_throttle, intercept=intercept,
                hover_throttle_est=hover_throttle_est)


async def main():
    fc = None
    results = []
    battery_voltage = None
    try:
        print("=" * 60)
        print("Hover-Throttle Finder")
        print("=" * 60)
        print(f"Mode: {THROTTLE_MODE}")
        if THROTTLE_MODE == "sine":
            print(f"Sine sweep: center={SINE_CENTER}, amp={SINE_AMP}, "
                  f"period={SINE_PERIOD_S}s, cycles={SINE_CYCLES}, "
                  f"takeoff height: {TAKEOFF_HEIGHT}m")
        else:
            print(f"Throttle values to test: {THROTTLE_LIST}")
            print(f"Hold per value: {HOLD_S}s, takeoff height: {TAKEOFF_HEIGHT}m")
        print(f"Safety: cut this hold short if altitude drifts >{ALT_MARGIN_M}m "
              f"(sweep continues); hard-abort + land if drift >{ALT_HARD_MARGIN_M}m "
              f"or climb rate exceeds {MAX_CLIMB_RATE_ABORT} m/s\n")

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
        sine_time = sine_throttle = None
        sine_fit = None

        if THROTTLE_MODE == "sine":
            sine_time, sine_throttle, sine_hard_abort = await _run_sine_sweep(fc, takeoff_alt)
            sweep_aborted = sine_hard_abort
            if not sine_hard_abort:
                sine_fit = _fit_sine_sweep(fc, sine_time, sine_throttle)

            print("\nLanding...")
            await fc.vehicle.action.land()
            await _wait_landed(fc)
            _save_log(fc, results, "aborted" if sweep_aborted else "complete",
                      extra={"sine_time": sine_time, "sine_throttle": sine_throttle})
            return

        for throttle in THROTTLE_LIST:
            print(f"--- Testing throttle={throttle:.3f} ---")
            samples = []  # (t, alt)
            step_t0 = time.perf_counter()
            step_cutoff = False   # soft: end this hold early, sweep continues
            hard_abort = False    # hard: end the whole sweep, land now
            while (time.perf_counter() - step_t0) < HOLD_S:
                await fc.send_attitude_rate(0.0, 0.0, 0.0, throttle)
                pos = fc.getPosBody()
                alt = -pos.z_m if pos else takeoff_alt
                now = time.perf_counter() - step_t0
                samples.append((now, alt))

                drift = abs(alt - takeoff_alt)
                if drift > ALT_HARD_MARGIN_M:
                    print(f"  [HARD ABORT] altitude {alt:.2f}m drifted >{ALT_HARD_MARGIN_M}m "
                          f"from takeoff height {takeoff_alt:.2f}m")
                    hard_abort = True
                    break
                if drift > ALT_MARGIN_M:
                    print(f"  [cutoff] altitude {alt:.2f}m drifted >{ALT_MARGIN_M}m - "
                          f"ending this hold early, continuing sweep")
                    step_cutoff = True
                    break

                await asyncio.sleep(0.05)

            if len(samples) >= 2:
                t_arr = np.array([s[0] for s in samples])
                a_arr = np.array([s[1] for s in samples])
                climb_rate = float(np.polyfit(t_arr, a_arr, 1)[0])  # m/s, linear fit slope
                print(f"  throttle={throttle:.3f} -> climb_rate={climb_rate:+.3f} m/s "
                      f"(alt {a_arr[0]:.2f} -> {a_arr[-1]:.2f}m)"
                      + (" [early-cutoff estimate]" if step_cutoff else ""))
                # Store (throttle, climb_rate, battery_voltage)
                results.append((throttle, climb_rate, battery_voltage if battery_voltage else 0.0))

                if abs(climb_rate) > MAX_CLIMB_RATE_ABORT:
                    print(f"  [HARD ABORT] climb rate {climb_rate:+.2f} m/s exceeds safety "
                          f"limit {MAX_CLIMB_RATE_ABORT} m/s")
                    hard_abort = True

            if hard_abort:
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

    except (KeyboardInterrupt, asyncio.CancelledError):
        # 2026-07-10: asyncio.CancelledError is a BaseException (not Exception)
        # since Python 3.8, so it was previously falling through both this
        # handler and `except Exception` uncaught -- a Ctrl-C during an `await`
        # (e.g. the hung takeoff-altitude wait) surfaces as CancelledError
        # first, and _save_log() never ran, silently losing whatever telemetry
        # had already been captured. Catching both here fixes that.
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
