#!/usr/bin/env python3
"""
Record camera feed while running tests
Captures video + metadata for diagnostics
"""

import sys
import os
import time
import cv2
import subprocess
import threading
from datetime import datetime

sys.path.insert(0, ".")

# WIRED 2026-08-01 to match hardware_landing.py/check_loop_freq.py/
# live_preview.py's now-validated real-flight defaults (see
# FLIGHT_TEST_ANALYSIS_PROCEDURE.md catalog #12/#13) - test recordings made
# with this script were previously run with these passed explicitly on the
# command line every time; making them the default here means a bare
# `python3 record_test_feed.py` now records under the same regime the drone
# actually flies with. Override on the command line to test a different one.
os.environ.setdefault("CAM_MANUAL_EXPOSURE", "1")
os.environ.setdefault("CAM_EXPOSURE_US", "20000")
os.environ.setdefault("CAM_AUTO_GAIN", "1")

from imgstreamer import imgstream

# CAPTURE_RATE_HZ (2026-08-01): env-overridable capRate, default 60 (unchanged
# behavior). Added to test the exposure/framerate headroom argument: the
# project's raw-Bayer 640x480 mode has an empirical ~30fps ceiling regardless
# of what's requested (confirmed via img_process_freq_optimization.md and this
# session's own measurement, ~25-31Hz effective vs a 60Hz request) - so
# requesting 60Hz costs nothing we're not already losing, but ALSO means
# ExposureTime is currently capped at the 60Hz frame period (~16667us) by
# FrameDurationLimits, even though the achieved rate never gets there anyway.
# Explicitly requesting CAPTURE_RATE_HZ=30 (matching the REAL achieved rate)
# raises that ceiling to ~33333us, giving legal headroom to test much longer
# exposures for poor-lighting decode without asking the camera for a rate it
# wasn't delivering in the first place. Leave some margin under the ceiling
# when picking CAM_EXPOSURE_US (jitter/scheduling variance), don't max it out.
CAPTURE_RATE_HZ = int(os.environ.get("CAPTURE_RATE_HZ", "30"))

# DISPLAY_PREVIEW (2026-08-01): optional live preview WHILE recording, so you
# can confirm the marker is actually in FoV during the recording instead of
# discovering after the fact that it drifted out. Mirrors live_preview.py's
# proven display pattern exactly (resize then imshow, NO cv2.namedWindow --
# see that file's 2026-07-31 comment for why namedWindow breaks the scaling)
# throttled to ~4Hz (cheap enough to stay responsive, plenty for human
# aiming) while the underlying capture/recording loop still runs at full
# rate underneath, same decoupling live_preview.py uses. Off by default (a
# small extra cost, and some sessions run headless over SSH with no display).
DISPLAY_PREVIEW = os.environ.get("DISPLAY_PREVIEW", "0") == "1"
DISPLAY_SCALE = 2.0
DISPLAY_INTERVAL_S = 0.25
if DISPLAY_PREVIEW:
    from img_data import build_aruco_detector
    _arucoDict, _arucoParams, _detector = build_aruco_detector()


def record_camera_feed(duration=60, output_dir="Test_Data/Calibration/Test_Videos"):
    """Record camera feed to MP4 with timestamp"""

    os.makedirs(output_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = os.path.join(output_dir, f"test_feed_{timestamp}.mp4")

    # Initialize stream
    # BUGFIX 2026-07-23: was (960, 720), which isn't a native IMX219 raw mode -
    # the sensor silently snapped to the nearest one, 1640x1232 (a genuinely
    # different sensor readout/binning path, not just a crop/scale of the
    # calibrated mode). Confirmed live: under the SAME manual-exposure
    # settings that give a well-exposed (mean 105/255, 90% ArUco decode) image
    # at 640x480, the 1640x1232 mode came out at mean 22/255 - essentially
    # black, 0% decode. 640x480 is also the ONLY resolution the project's
    # camera intrinsics (img_geometry.py CALIB_CX/CY/fx/fy) were ever measured
    # at - using anything else silently uses the wrong focal length too, a
    # second, independent reason this must match project convention exactly.
    stream = imgstream(resolution=(640, 480), capRate=CAPTURE_RATE_HZ)
    time.sleep(2.0)  # let the camera/AEC/manual-exposure controls settle before recording

    # ACTUAL negotiated raw size, not the requested tuple - imgstreamer.py's
    # own docstring warns the sensor may snap to a different native mode
    # ("downstream code MUST use getResolution()"). Using the wrong size here
    # would make every VideoWriter.write() silently fail on a shape mismatch,
    # on top of the channel-count bug below.
    actual_res = stream.getResolution()

    # TIGHTENED 2026-08-16 (was a hardcoded 30.0, same "assume the request"
    # bug class as the frame-dedup fix above -- CAPTURE_RATE_HZ is a REQUEST,
    # not a guarantee; this project's raw-Bayer mode has an empirical ~25-31Hz
    # ceiling regardless of what's requested). getFPS() is imgstreamer's own
    # cumulative average since stream start (self._count/elapsed, measured
    # inside the real capture thread -- same source img_data.py's production
    # loop already trusts, see its own `self._fps = self._image_node.getFPS()`
    # line), so by this point (after the 2s settle sleep above, which is
    # itself several dozen frames of real capture) it reflects the actual
    # achieved rate, not the requested one. A VideoWriter opened at the wrong
    # fixed fps plays back at the wrong apparent speed even with the frame
    # dedup fix in place (dedup fixes DUPLICATE frames; this fixes the
    # container's own timeline being scaled wrong). Clamped to a sane range
    # so a near-zero reading this early (e.g. AEC/AGC still converging, very
    # few frames counted yet) can't produce a broken/unplayable file.
    measured_fps = stream.getFPS()
    video_fps = max(1.0, min(60.0, measured_fps)) if measured_fps > 0 else 30.0

    print(f"Recording camera feed to: {output_file}")
    print(f"Duration: {duration}s")
    print(f"Resolution: {actual_res[0]}x{actual_res[1]} (negotiated)")
    print(f"FPS: requested {CAPTURE_RATE_HZ}Hz, measured {measured_fps:.1f}Hz "
          f"-> VideoWriter opened at {video_fps:.1f}fps")
    print()

    # Setup video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_file, fourcc, video_fps, actual_res)

    if DISPLAY_PREVIEW:
        print("Live preview ON (~4Hz) -- press ESC in the preview window to stop early.")

    # Record
    start_time = time.time()
    frame_count = 0
    last_display_t = 0.0

    # NEW-FRAME DEDUP (2026-08-16 fix, see bench_auto_exposure.py's identical
    # pattern): this loop used to poll getImages()[-1] every 1ms with no check
    # for whether it was actually a NEW camera frame - the capture thread only
    # appends a fresh frame when one is really captured, so a fast poll
    # re-reads (and here, re-WRITES to the video + counts toward FPS) the SAME
    # frame many times between real captures. Two consequences, both silent:
    # (1) `fps = frame_count / elapsed` measured POLLING rate, not camera
    # throughput - could read well above the real ~25-31Hz ceiling
    # (img_process_freq_optimization.md) while the camera itself did far
    # less. (2) the recorded MP4 has DUPLICATE frames baked into its 30fps
    # VideoWriter timeline, distorting apparent motion on playback - a
    # calibration hand-sweep recording reviewed for motion blur would show a
    # false stutter/hold pattern that isn't what the camera actually saw.
    # getCaptureStamps()'s pulled_at_perf_counter changes only on a genuine
    # new capture (set inside imgstreamer.py's own capture thread, same
    # timing source as its internal frame counter) - gate on that instead.
    last_stamp_perf = None

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

        # BUGFIX 2026-07-23: getImages() returns a single-channel
        # frame - writing that straight to a VideoWriter opened with
        # the default isColor=True (3-channel) silently rejects EVERY
        # frame ("expected 3 channels but got 1"), so the file was
        # never actually getting any frames despite frame_count
        # climbing.
        # UPDATED 2026-07-30: getImages() now returns the ISP-scaled
        # working stream (grayscale Y-plane), not raw Bayer data -
        # imgstreamer.py no longer exposes the raw stream at all (see
        # its module docstring). Bayer-demosaicing this would produce
        # garbage (there's no real Bayer mosaic in ISP luma output) -
        # a plain grayscale->BGR conversion is the correct match now,
        # same as every other display/recording path in this codebase.
        bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        out.write(bgr)
        frame_count += 1

        # Progress
        elapsed = time.time() - start_time
        if frame_count % 30 == 0:
            print(f"  {elapsed:.1f}s / {duration}s — {frame_count} frames")

        if DISPLAY_PREVIEW and (elapsed - last_display_t) >= DISPLAY_INTERVAL_S:
            last_display_t = elapsed
            corners, ids, _ = _detector.detectMarkers(frame)
            decoded = ids is not None and len(ids) > 0
            vis = bgr.copy()
            if decoded:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            label = f"DECODED (id={ids.flatten().tolist()})" if decoded else "not decoded"
            cv2.putText(vis, label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0) if decoded else (0, 0, 255), 1)
            vis = cv2.resize(vis, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                              interpolation=cv2.INTER_AREA)
            cv2.imshow("Recording preview (~4Hz)", vis)
            if cv2.waitKey(1) == 27:   # ESC -- stop recording early
                break

    out.release()
    stream.close()
    if DISPLAY_PREVIEW:
        cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0.0
    file_size = os.path.getsize(output_file) / (1024 * 1024)
    
    print()
    print("✓ Recording complete")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Frames: {frame_count}")
    print(f"  FPS: {fps:.1f} (whole-recording average; video written at {video_fps:.1f}fps)")
    print(f"  File size: {file_size:.1f} MB")
    print(f"  Path: {output_file}")
    # Sanity check: video_fps was fixed from a ~2s sample BEFORE recording
    # started, so it can't reflect anything that changed mid-recording (an
    # exposure-transition stall, a lighting change altering achieved rate,
    # etc). If the true whole-recording average drifted meaningfully from
    # what the file was opened at, the saved MP4's playback speed is off by
    # that ratio -- flag it rather than let it pass silently.
    if fps > 0 and abs(fps - video_fps) / video_fps > 0.10:
        print(f"  WARNING: measured whole-recording FPS ({fps:.1f}) differs from the "
              f"video's opened fps ({video_fps:.1f}) by >10% -- playback speed in the "
              f"saved file will be off by roughly that ratio. The rate likely drifted "
              f"during recording (e.g. a lighting change or CAM_AUTO_EXPOSURE step); "
              f"don't trust the video's apparent motion/timing if you need it exact --"
              f" re-record, or use bench_auto_exposure.py instead, which logs true "
              f"per-frame timing to CSV rather than baking a single fixed fps into a video.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Record camera feed for testing")
    parser.add_argument("--duration", type=int, default=60, help="Recording duration in seconds")
    parser.add_argument("--output", default="Test_Data/Calibration/Test_Videos", help="Output directory")
    
    args = parser.parse_args()
    
    record_camera_feed(duration=args.duration, output_dir=args.output)

