#!/usr/bin/env python3
"""
Bench validation for CAM_AUTO_EXPOSURE (imgstreamer.py, 2026-08-16).

Purpose: confirm the new gain-pinned -> step-exposure logic actually behaves
sensibly on real hardware before it's ever flown, the same discipline
CAM_AUTO_GAIN itself went through (FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog
#13: a 60s bench recording spanning a real lighting change, not a synthetic
test). Point the camera at the marker and walk it through a real brightness
transition DURING the recording -- sun/shade, indoor/outdoor through a
doorway, a lamp toggled, whatever produces a genuine sustained over- or
under-exposure the CAM_AUTO_GAIN loop alone can't correct. A static scene
tells you nothing here; the whole point is the transition.

Logs per-frame: elapsed time, mean brightness, live gain, live exposure,
ArUco decode success -- to Test_Data/Calibration/AutoExposure_Bench/<ts>/,
as both a CSV (for the existing plotter_*.ipynb pattern / ad-hoc analysis)
and a printed summary broken down by exposure band, so you can see directly
whether decode rate held up across each step CAM_AUTO_EXPOSURE took.

Usage:
    CAM_EXPOSURE_US=2000 python3 bench_auto_exposure.py --duration 90
    (CAM_MANUAL_EXPOSURE / CAM_AUTO_GAIN / CAM_AUTO_EXPOSURE default ON below
    -- override on the command line to test a different starting point.)
"""

import sys
import os
import csv
import time
from datetime import datetime
from collections import Counter, deque

FPS_TARGET_HZ = float(os.environ.get("FPS_TARGET_HZ", "30.0"))
FPS_WINDOW_FRAMES = 30   # ~1s of frames at the 30Hz target -- windowed/instantaneous FPS

sys.path.insert(0, ".")

# Defaults for THIS bench test specifically -- CAM_AUTO_EXPOSURE is what's
# under test, so it (and its CAM_AUTO_GAIN dependency) default ON here, unlike
# hardware_landing.py/record_test_feed.py which default it OFF pending this
# validation. CAM_EXPOSURE_US is the STARTING point the loop adjusts away
# from -- pick a deliberately wrong one (e.g. 2000 outdoors, 20000 indoors) to
# also exercise the pinned-detection path from a cold start, not just steady
# state.
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")
os.environ.setdefault("CAM_AUTO_GAIN", "1")
os.environ.setdefault("CAM_AUTO_EXPOSURE", "1")
os.environ.setdefault("CAM_EXPOSURE_US", "2000")
os.environ.setdefault("CAPTURE_RATE_HZ", "30")

from imgstreamer import imgstream
from img_data import build_aruco_detector


def run_bench(duration=90, output_dir="Test_Data/Calibration/AutoExposure_Bench"):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(output_dir, timestamp)
    os.makedirs(run_dir, exist_ok=True)
    csv_path = os.path.join(run_dir, "log.csv")

    print(f"CAM_MANUAL_EXPOSURE={os.environ.get('CAM_MANUAL_EXPOSURE')} "
          f"CAM_AUTO_GAIN={os.environ.get('CAM_AUTO_GAIN')} "
          f"CAM_AUTO_EXPOSURE={os.environ.get('CAM_AUTO_EXPOSURE')} "
          f"starting CAM_EXPOSURE_US={os.environ.get('CAM_EXPOSURE_US')}")
    if os.environ.get("CAM_AUTO_EXPOSURE") != "1":
        print("WARNING: CAM_AUTO_EXPOSURE is not '1' -- this run will not "
              "exercise the thing it's meant to validate.")

    stream = imgstream(resolution=(640, 480), capRate=int(os.environ.get("CAPTURE_RATE_HZ", "30")))
    time.sleep(2.0)  # let camera/AEC/manual-exposure controls settle, matches record_test_feed.py

    _arucoDict, _arucoParams, detector = build_aruco_detector()

    print(f"Recording {duration}s to {run_dir}/  "
          f"-- walk the camera through a real brightness transition NOW.")
    print()

    rows = []
    exposure_transitions = []   # (elapsed_s, old_us, new_us)
    last_exposure = stream.getCurrentExposureUs()

    # NEW-FRAME DEDUP: getImages()/getCaptureStamps() are polled from this
    # tight loop, but the capture thread only appends a new entry when a
    # frame is actually captured -- naively re-reading imgs[-1] every 1ms
    # poll (as record_test_feed.py's own loop does) reprocesses the SAME
    # frame many times between real captures, so frame_count/elapsed would
    # measure POLLING rate, not camera throughput. getCaptureStamps()'s
    # pulled_at_perf_counter changes only on a genuine new capture (set
    # inside imgstreamer.py's run(), same thread/timing as self._count) --
    # use that to gate counting, so FPS here reflects reality.
    last_stamp_perf = None
    arrival_times = deque(maxlen=FPS_WINDOW_FRAMES)   # perf_counter() at each real new-frame arrival
    fps_min_windowed = None

    start_time = time.time()
    frame_count = 0
    last_progress_t = 0.0

    while time.time() - start_time < duration:
        stamps = stream.getCaptureStamps()
        cur_stamp = stamps[-1] if stamps else None
        cur_perf = cur_stamp.get("pulled_at_perf_counter") if cur_stamp else None
        if cur_perf is None or cur_perf == last_stamp_perf:
            time.sleep(0.001)
            continue
        last_stamp_perf = cur_perf

        imgs = stream.getImages()
        frame = imgs[-1] if imgs and len(imgs) > 0 else None
        if frame is None:
            continue

        elapsed = time.time() - start_time
        arrival_times.append(cur_perf)
        fps_inst = ((len(arrival_times) - 1) / (arrival_times[-1] - arrival_times[0])
                    if len(arrival_times) >= 2 and arrival_times[-1] > arrival_times[0] else 0.0)
        if len(arrival_times) == FPS_WINDOW_FRAMES:  # only judge min FPS once the window is full
            fps_min_windowed = fps_inst if fps_min_windowed is None else min(fps_min_windowed, fps_inst)

        corners, ids, _ = detector.detectMarkers(frame)
        decoded = ids is not None and len(ids) > 0
        gain = stream.getCurrentGain()
        exposure_us = stream.getCurrentExposureUs()
        mean_brightness = float(frame.mean())

        rows.append((frame_count, round(elapsed, 3), round(mean_brightness, 2),
                     round(gain, 3), exposure_us, int(decoded), round(fps_inst, 1)))
        frame_count += 1

        if exposure_us != last_exposure:
            exposure_transitions.append((round(elapsed, 1), last_exposure, exposure_us))
            print(f"  [{elapsed:6.1f}s] exposure {last_exposure}us -> {exposure_us}us "
                  f"(gain={gain:.2f}, brightness={mean_brightness:.1f}, fps~{fps_inst:.1f})")
            last_exposure = exposure_us

        if elapsed - last_progress_t >= 5.0:
            last_progress_t = elapsed
            recent = rows[-150:]  # last ~5s at 30Hz
            recent_rate = sum(r[5] for r in recent) / len(recent) * 100
            print(f"  {elapsed:5.1f}s / {duration}s  frames={frame_count:5d}  "
                  f"exp={exposure_us:6d}us  gain={gain:5.2f}  "
                  f"brightness={mean_brightness:5.1f}  fps~{fps_inst:5.1f}  "
                  f"decode(last~5s)={recent_rate:5.1f}%")

    stream.close()

    total_elapsed = time.time() - start_time
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "elapsed_s", "mean_brightness", "gain", "exposure_us",
                    "decoded", "fps_inst"])
        w.writerows(rows)

    print()
    print(f"Recorded {frame_count} frames over {total_elapsed:.1f}s -> {csv_path}")
    print()
    print_summary(rows, exposure_transitions, total_elapsed, fps_min_windowed)


def print_summary(rows, exposure_transitions, total_elapsed, fps_min_windowed):
    if not rows:
        print("No frames captured -- nothing to summarize.")
        return

    overall_rate = sum(r[5] for r in rows) / len(rows) * 100
    overall_fps = len(rows) / total_elapsed if total_elapsed > 0 else 0.0
    print("=== overall ===")
    print(f"  frames: {len(rows)}   decode rate: {overall_rate:.1f}%")
    if fps_min_windowed is not None:
        print(f"  average FPS: {overall_fps:.1f}Hz   "
              f"min windowed FPS (~{FPS_WINDOW_FRAMES}-frame window): {fps_min_windowed:.1f}Hz")
    else:
        print(f"  average FPS: {overall_fps:.1f}Hz   min windowed FPS: n/a (recording too short)")
    if overall_fps >= FPS_TARGET_HZ:
        print(f"  -> PASS: average FPS ({overall_fps:.1f}Hz) >= target ({FPS_TARGET_HZ:.0f}Hz)")
    else:
        print(f"  -> FAIL: average FPS ({overall_fps:.1f}Hz) BELOW target ({FPS_TARGET_HZ:.0f}Hz). "
              f"Note the project's documented raw-Bayer ceiling on this hardware is ~25-31Hz "
              f"regardless of what's requested (img_process_freq_optimization.md) -- if this run "
              f"lands in that range, confirm whether CAM_AUTO_EXPOSURE's extra set_controls() "
              f"calls made it WORSE than the pre-existing ceiling (compare against a "
              f"CAM_AUTO_EXPOSURE=0 baseline run) before assuming this change is the cause.")
    print(f"  exposure transitions: {len(exposure_transitions)}")
    for t, old, new in exposure_transitions:
        # FPS in the ~1s window straddling each transition -- checks whether
        # the set_controls() call itself caused a capture stall, not just
        # whether the STEADY-STATE FPS after settling looks fine.
        near = [r[6] for r in rows if abs(r[1] - t) <= 1.0]
        near_fps = f"{min(near):.1f}-{max(near):.1f}Hz" if near else "n/a"
        print(f"    t={t:6.1f}s  {old}us -> {new}us   fps around transition: {near_fps}")

    print()
    print("=== decode rate by exposure band actually visited ===")
    by_exposure = {}
    for r in rows:
        by_exposure.setdefault(r[4], []).append(r[5])
    for exp_us in sorted(by_exposure):
        vals = by_exposure[exp_us]
        rate = sum(vals) / len(vals) * 100
        print(f"  {exp_us:6d}us   n={len(vals):5d}   decode={rate:5.1f}%")

    print()
    print("=== sanity checks ===")
    brightness_vals = [r[2] for r in rows]
    mean_b = sum(brightness_vals) / len(brightness_vals)
    lo = sum(1 for b in brightness_vals if b < 30)
    hi = sum(1 for b in brightness_vals if b > 220)
    print(f"  mean brightness across recording: {mean_b:.1f}/255")
    print(f"  frames near-black (<30):  {lo:5d} ({lo/len(rows)*100:.1f}%)")
    print(f"  frames near-white (>220): {hi:5d} ({hi/len(rows)*100:.1f}%)")
    if lo / len(rows) > 0.2 or hi / len(rows) > 0.2:
        print("  -> a large chunk of the recording sat near a clipping extreme "
              "AFTER auto-exposure had time to react. Either the transition "
              "was too brief for CAM_EXPOSURE_PIN_TRIGGER to fire, or the "
              "exposure step/bounds need retuning (CAM_EXPOSURE_STEP_FACTOR, "
              "CAM_EXPOSURE_MIN_US/MAX_US) -- don't fly this config yet.")
    else:
        print("  -> brightness stayed off both clipping extremes for most of "
              "the recording. Compare per-band decode rates above against "
              "the pre-CAM_AUTO_EXPOSURE baseline for the same lighting "
              "before trusting this for flight.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Bench-validate CAM_AUTO_EXPOSURE against real lighting transitions")
    parser.add_argument("--duration", type=int, default=90,
                         help="Recording duration in seconds (default 90 -- long enough to walk "
                              "through a real transition and let the loop settle on both sides)")
    parser.add_argument("--output", default="Test_Data/Calibration/AutoExposure_Bench",
                         help="Output directory")
    args = parser.parse_args()
    run_bench(duration=args.duration, output_dir=args.output)
