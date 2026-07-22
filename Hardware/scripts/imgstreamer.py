"""
Picamera2 streaming for Raspberry Pi OS - RAW BAYER capture for 30+ FPS

Dual-stream config: a small "main" YUV420 stream keeps the IPA's AEC/AGC
control loop running (raw-only capture disables it, leaving frames
severely underexposed) AND is exposed to callers via getMainImages() as a
genuinely smaller ISP-scaled grayscale image (its Y-plane) - unlike the
"raw" stream, the ISP CAN scale to arbitrary sizes (the sensor's native
raw-mode limits don't apply here), so this is the correct path for anyone
wanting a smaller frame for cheaper CPU-bound processing (e.g. ArUco
detectMarkers - see img_data.py._detect_markers). Both streams are pulled
from the SAME captured request each loop iteration (capture_request(),
not two separate capture_array() calls) so they stay frame-synced.

The raw stream is SBGGR10_CSI2P: MIPI RAW10 packed format, 4 pixels packed
into 5 bytes (captured shape is (H, W*10/8), NOT (H, W)). _unpack_raw10()
unpacks this into a true (H, W) 8-bit Bayer image (taking the 8 MSBs of
each 10-bit sample; the 2 packed LSBs are discarded - not needed for
ArUco/optical-flow use).

IMPORTANT: the IMX219 only has a fixed set of native raw sensor modes.
Requesting a size that isn't one of them makes Picamera2 silently snap to
the nearest native mode WITHOUT raising an error - so the requested
`resolution` passed to __init__ may not be what the sensor actually
delivers. We query the real negotiated size from camera_configuration()
after start() and use THAT everywhere (unpacking width, getResolution()),
never the raw constructor argument. The "main" stream has no such
limitation - it negotiates to the exact requested size via ISP scaling.
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


def _unpack_raw10(packed, width):
    """MIPI RAW10 packed -> 8-bit Bayer, shape (H, W*10/8) -> (H, W)."""
    h = packed.shape[0]
    groups = width // 4
    reshaped = packed[:, :groups * 5].reshape(h, groups, 5)
    return reshaped[:, :, :4].reshape(h, groups * 4)


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
        if CAM_MANUAL_EXPOSURE:
            self._camera.set_controls({
                "AeEnable": False,
                "ExposureTime": CAM_EXPOSURE_US,
                "AnalogueGain": CAM_ANALOGUE_GAIN,
            })
            print(f"imgstream: manual exposure ON - ExposureTime={CAM_EXPOSURE_US}us "
                  f"AnalogueGain={CAM_ANALOGUE_GAIN} (auto AEC/AGC disabled)")

        # Actual negotiated raw stream size - may differ from the requested
        # `resolution` if the sensor has no matching native mode.
        actual_size = tuple(self._camera.camera_configuration()["raw"]["size"])
        if actual_size != tuple(resolution):
            print(f"WARNING: imgstream requested resolution={resolution} but "
                  f"sensor negotiated raw size={actual_size} (nearest native "
                  f"mode). Downstream code MUST use getResolution(), not the "
                  f"requested tuple.")

        self._resolution = actual_size
        self._width = actual_size[0]
        self._main_resolution = tuple(self._camera.camera_configuration()["main"]["size"])
        self._main_width = self._main_resolution[0]
        self._capRate = capRate
        self._break_flag = False
        self._img_deque = deque([None, None], maxlen=2)
        self._main_img_deque = deque([None, None], maxlen=2)
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
                # Pull raw + main from the SAME request so they're frame-synced
                # (two separate capture_array() calls could each trigger a
                # fresh capture, drifting the two streams apart in time).
                request = self._camera.capture_request()
                try:
                    packed = request.make_array("raw")
                    main_yuv = request.make_array("main")
                    md = request.get_metadata()
                    cap_stamp = {
                        "sensor_timestamp_ns": md.get("SensorTimestamp"),
                        "frame_wall_clock": md.get("FrameWallClock"),
                        "pulled_at_perf_counter": time.perf_counter(),
                    }
                finally:
                    request.release()
                if packed is not None:
                    frame = _unpack_raw10(packed, self._width)
                    self._img_deque.append(frame)
                    self._cap_stamp_deque.append(cap_stamp)
                    self._count += 1
                    self._meanTimePerImage = (time.perf_counter() - self._start_time) / self._count
                if main_yuv is not None:
                    # YUV420 planar array: Y (luma) plane is the first
                    # main_height rows - directly usable as grayscale, no
                    # separate colour conversion needed for ArUco/detection.
                    self._main_img_deque.append(main_yuv[:self._main_resolution[1], :self._main_width])
        except Exception as e:
            print(f"Error in imgstream: {e}")
        finally:
            self._camera.stop()

    def close(self):
        self._break_flag = True

    def getImages(self):
        return self._img_deque

    def getMainImages(self):
        """Grayscale (Y-plane) frames from the ISP-scaled 'main' stream -
        genuinely smaller than the raw stream (see module docstring), for
        cheap CPU-bound detection work. Frame-synced with getImages()."""
        return self._main_img_deque

    def getCaptureStamps(self):
        """Hardware capture timestamps (see __init__ comment) parallel to
        getImages()/getMainImages() - dicts with sensor_timestamp_ns,
        frame_wall_clock, and pulled_at_perf_counter (when the capture
        thread actually received this request, distinct from when
        img_data.py's processing thread later logs its own perf_counter()
        for the SAME frame)."""
        return self._cap_stamp_deque

    def getResolution(self):
        """Actual negotiated (width, height) - use this, not the requested tuple."""
        return self._resolution

    def getMainResolution(self):
        """Actual negotiated (width, height) of the 'main' ISP-scaled stream."""
        return self._main_resolution

    def getFPS(self):
        if self._meanTimePerImage > 0:
            return 1.0 / self._meanTimePerImage
        return 0
