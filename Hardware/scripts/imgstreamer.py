"""
Picamera2 streaming for Raspberry Pi OS - single working resolution for
callers, backed internally by a dual-stream capture.

REWORKED 2026-07-30: img_data.py previously juggled two named streams
("raw" 640x480 / "main" 320x240, with a self._aruco_scale conversion
between them) - confusing, and getMainImages()/getMainResolution() were a
second API callers had to know about. That concept is gone. getImages()/
getResolution() now return the ISP-scaled working stream (size set by
MAIN_STREAM_SIZE, default 320x240) directly - the ONE resolution img_data.py
sees or does geometry in. img_geometry.py's CALIB_CX/CY/fx/fy were rescaled
to match (see its own 2026-07-30 comment).

Internally, a raw stream is still configured (NOT surfaced to callers) for
two hardware-forced reasons, both true regardless of what callers need:
1. AEC/AGC: the IPA's auto-exposure control loop needs a raw stream present
   in the configuration or frames come back severely underexposed.
2. Sensor-mode pinning: including `raw={"size": resolution}` in the
   Picamera2 config forces the sensor into a specific fixed native mode
   (this project uses 640x480, the fastest/smallest available - the IMX219
   has no smaller native raw mode) rather than letting it default to
   something else. The ISP's "main" output has no such limit; it can scale
   to arbitrary sizes regardless of which raw mode the sensor is in.
Because nothing consumes the raw pixels anymore, `run()` no longer calls
`request.make_array("raw")` or unpacks RAW10 - that per-frame CPU cost is
gone along with the naming confusion.
"""

from picamera2 import Picamera2
from threading import Thread
import os
import time
from collections import deque
import numpy as np

# ArUco-detection "main" stream size - env-configurable for A/B testing
# against the raw stream's native resolution without editing this file.
MAIN_STREAM_SIZE = tuple(
    int(x) for x in os.environ.get("MAIN_STREAM_SIZE", "320,240").split(","))

# Manual exposure/gain override, OFF by default (auto AEC/AGC is fine for
# normal flight/landing use). Diagnosed 2026-07-22: output-calibration hand
# sweeps were losing 60-80% of each recording to multi-second ArUco
# detection dropouts, and the frame immediately before every dropout had the
# marker well-centered (not near the FOV edge) - ruling out the marker
# swinging out of frame. With auto-exposure picking a several-ms-to-tens-of-
# ms exposure under typical indoor light, a vigorous hand sweep blurs the
# marker's bit pattern past decodability while it's still square in frame.
# A short fixed exposure (compensated by higher gain, since a wall-clock-
# independent shutter needs more light per frame) removes that blur at the
# cost of more sensor noise - acceptable for a marker-decode use case, unlike
# a photographic one. Tune CAM_EXPOSURE_US/CAM_ANALOGUE_GAIN per lighting
# conditions; output_calibration.py enables this by default (see its own
# os.environ.setdefault block) since it's specifically for vigorous sweeps.
CAM_MANUAL_EXPOSURE = os.environ.get("CAM_MANUAL_EXPOSURE", "0") == "1"
CAM_EXPOSURE_US = int(os.environ.get("CAM_EXPOSURE_US", "3000"))
CAM_ANALOGUE_GAIN = float(os.environ.get("CAM_ANALOGUE_GAIN", "8.0"))


class imgstream(Thread):
    def __init__(self, resolution=(640, 480), capRate=30):
        Thread.__init__(self)
        self.daemon = True

        self._camera = Picamera2()
        config = self._camera.create_video_configuration(
            main={"format": "YUV420", "size": MAIN_STREAM_SIZE},
            raw={"size": resolution},
            display=None
        )
        self._camera.configure(config)
        self._camera.start()

        # WIRED 2026-07-23: `capRate` used to be accepted, stored in
        # self._capRate, and reported by getParams()/getFPS() metadata -
        # but never actually applied to the camera, so a caller requesting
        # capRate=60 (e.g. output_calibration.py's CAPTURE_RATE) silently
        # got whatever frame duration the sensor mode defaulted to. Set it
        # explicitly via FrameDurationLimits (microseconds, fixed min=max so
        # it's a target rather than just a ceiling). Still bounded by the
        # sensor's real per-mode minimum frame duration - the project's
        # raw-Bayer 640x480 mode has an empirical ~30fps ceiling regardless
        # of what's requested here (CLAUDE.md), so this makes the request
        # honest rather than guaranteeing a specific achieved rate; check
        # getFPS() / this class's actual measured rate, not the argument.
        controls = {}
        if capRate and capRate > 0:
            frame_us = int(round(1e6 / capRate))
            controls["FrameDurationLimits"] = (frame_us, frame_us)
        if CAM_MANUAL_EXPOSURE:
            controls["AeEnable"] = False
            controls["ExposureTime"] = CAM_EXPOSURE_US
            controls["AnalogueGain"] = CAM_ANALOGUE_GAIN
        if controls:
            self._camera.set_controls(controls)
            print(f"imgstream: controls set -> {controls}"
                  + (f" (manual exposure ON, AEC/AGC disabled)" if CAM_MANUAL_EXPOSURE else ""))

        # Sanity-check the sensor actually landed on the requested raw mode
        # (internal only - this is about pinning the sensor's native mode
        # for AEC/AGC + speed, not about what callers see; see module
        # docstring). Callers never see this "raw" size at all now.
        _raw_size = tuple(self._camera.camera_configuration()["raw"]["size"])
        if _raw_size != tuple(resolution):
            print(f"WARNING: imgstream requested raw sensor mode={resolution} but "
                  f"the sensor negotiated {_raw_size} (nearest native mode). This "
                  f"only affects AEC/AGC + speed pinning, not the working "
                  f"resolution callers use (getResolution()).")

        # The working stream callers see IS the ISP-scaled 'main' stream -
        # this is the one and only resolution img_data.py does geometry in.
        self._resolution = tuple(self._camera.camera_configuration()["main"]["size"])
        self._width = self._resolution[0]
        self._capRate = capRate
        self._break_flag = False
        self._img_deque = deque([None, None], maxlen=2)
        # Hardware capture timestamps, parallel to the image deques above -
        # ported from a PX4_Gazebo fix (bba5c33) for the SAME symptom we hit
        # on the Pi (inconsistent GT-vs-raw time-lag scan, no stable peak):
        # SITL found it was a CLOCK-SOURCE mismatch, not fundamentally noisy
        # data - the flow signal's timestamp and GT's timestamp were on
        # different clocks, so ANY fixed-lag alignment was chasing a
        # jittery, not-actually-constant offset. Our current "Time" log
        # (self._time.perf_counter() in img_data.py) is stamped when the
        # flow-processing thread gets around to it, not when the frame was
        # actually captured by the camera hardware - picamera2's raw-stream
        # capture happens on this background thread with its own
        # buffering/scheduling latency, so that gap is likely real and
        # possibly variable frame-to-frame. Logging BOTH the hardware
        # SensorTimestamp/FrameWallClock (this request's metadata) and our
        # existing perf_counter() processing-time log lets a fresh
        # recording show which clock GT actually aligns to - log-only for
        # now, no control-path change, mirroring SITL's own first step.
        self._cap_stamp_deque = deque([None, None], maxlen=2)
        self._meanTimePerImage = 1e-06
        self._count = 0
        self._start_time = time.perf_counter()

        self.start()

    def run(self):
        try:
            while not self._break_flag:
                # A raw stream is still requested in the Picamera2 config (see
                # module docstring: AEC/AGC + sensor-mode pinning), but its
                # array is never pulled here - nothing consumes raw pixels
                # anymore, so skip the make_array("raw")+unpack cost entirely.
                request = self._camera.capture_request()
                try:
                    main_yuv = request.make_array("main")
                    md = request.get_metadata()
                    cap_stamp = {
                        "sensor_timestamp_ns": md.get("SensorTimestamp"),
                        "frame_wall_clock": md.get("FrameWallClock"),
                        "pulled_at_perf_counter": time.perf_counter(),
                    }
                finally:
                    request.release()
                if main_yuv is not None:
                    # YUV420 planar array: Y (luma) plane is the first
                    # height rows - directly usable as grayscale, no
                    # separate colour conversion needed for ArUco/detection.
                    frame = main_yuv[:self._resolution[1], :self._width]
                    self._img_deque.append(frame)
                    self._cap_stamp_deque.append(cap_stamp)
                    self._count += 1
                    self._meanTimePerImage = (time.perf_counter() - self._start_time) / self._count
        except Exception as e:
            print(f"Error in imgstream: {e}")
        finally:
            self._camera.stop()

    def close(self):
        self._break_flag = True

    def getImages(self):
        """Grayscale (Y-plane) frames from the ISP-scaled working stream -
        the single resolution img_data.py does all geometry/detection in."""
        return self._img_deque

    def getCaptureStamps(self):
        """Hardware capture timestamps (see __init__ comment) parallel to
        getImages() - dicts with sensor_timestamp_ns, frame_wall_clock, and
        pulled_at_perf_counter (when the capture thread actually received
        this request, distinct from when img_data.py's processing thread
        later logs its own perf_counter() for the SAME frame)."""
        return self._cap_stamp_deque

    def getResolution(self):
        """Actual negotiated (width, height) of the working stream - use
        this, not the requested tuple."""
        return self._resolution

    def getFPS(self):
        if self._meanTimePerImage > 0:
            return 1.0 / self._meanTimePerImage
        return 0
