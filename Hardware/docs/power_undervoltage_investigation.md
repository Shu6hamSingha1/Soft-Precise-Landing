# Pi Undervoltage Investigation — 2026-07-10

## Symptom

`output_calibration.py` on the Pi showed `img_process_freq` ~5.6 Hz against a
camera capture rate (`fps`) of ~29.7 Hz — a ~5x processing bottleneck.

## Confirmed contributors

1. **CPU-bound floor.** Synthetic profiling on the Pi 4 (`cv2.aruco.detectMarkers`
   + ring-flow `cv2.calcOpticalFlowPyrLK`, 300 pts / 41x41 win / 3 pyramid levels):

   | Step | Time/frame | Share |
   |---|---|---|
   | ArUco `detectMarkers` (2 frames) | 91.9 ms | 76% |
   | Ring LK (300 pts) | 28.6 ms | 24% |
   | **Total floor** | **120.5 ms** | **→ 8.3 Hz** |

   ArUco detection dominates, not the ring-flow safety net (initial suspicion
   was wrong — profiled and corrected).

2. **Undervoltage/throttling events.** `vcgencmd get_throttled` returned
   `0x50000` (bit 16: under-voltage has occurred; bit 18: throttling has
   occurred). `dmesg -T` showed:
   ```
   [Fri Jul 10 05:06:32 2026] hwmon hwmon1: Undervoltage detected!
   [Fri Jul 10 05:06:52 2026] hwmon hwmon1: Voltage normalised
   [Fri Jul 10 05:07:28 2026] hwmon hwmon1: Undervoltage detected!
   [Fri Jul 10 05:07:32 2026] hwmon hwmon1: Voltage normalised
   ```
   Both events landed right before/at the start of the calibration run
   (05:08:12) — i.e. right when the FC/motors/camera/compute load stacked up.
   Observed 5.6 Hz is below even the 8.3 Hz CPU-bound synthetic floor, so
   throttling is stacking with the inherent ArUco cost.

## Power architecture — RULED OUT as root cause (user, 2026-07-10)

- Single **LiPo, 1700 mAh**, powers drone + Pi together; Pi has a **dedicated
  UBEC**.
- User confirmed this exact power setup has been used successfully before
  (pre-existing, working hardware config) and the undervoltage events /
  0x50000 throttle flag are **not** the explanation for the low
  `img_process_freq`. **The bottleneck is in the code path**, not hardware.
- The `dmesg` undervoltage events at 05:06:32-05:07:32 remain logged/true but
  are not being treated as the cause of the 5.6 Hz processing rate — do not
  re-litigate the wiring theory; investigate the code path instead.

## Next steps (code-focused)

- [ ] Profile the LIVE `_optFlowAngVel` call path frame-by-frame on the Pi
      (not just the synthetic ArUco/LK microbenchmark) to find where the real
      120ms+ is actually going — synthetic floor was 8.3 Hz but live run
      showed 5.6 Hz, so something beyond ArUco+ring-LK is adding cost (frame
      capture/debayer sync, `_getVirtualPts`/`_fill_A`/KF steps, thread
      contention with the camera-capture thread, `imgstreamer` decode cost).
- [ ] Check `cv2.aruco.DetectorParameters()` tuning
      (`adaptiveThreshWinSizeMin/Max/Step`, `minMarkerPerimeterRate`) to cut
      ArUco candidate-search cost for the known marker size/distance — this
      was the dominant synthetic cost (76%) and is a pure-code lever.
- [ ] Check for accidental serialization/blocking between the image-capture
      thread and the flow-processing thread (e.g. lock contention, unnecessary
      copies, synchronous camera reads inside the processing loop).

## Related fix (same session)

`img_data.py` on the Pi (`~/ws/scripts/precise_landing/img_data.py`) had a
separate bug: `_getVirtualPts`/`_getRealPtsFromV`/`_vframe_w` called
`quat.to_DCM()` directly on the raw MAVSDK `odometry.q` object, which has no
`.to_DCM()` method (only `.w/.x/.y/.z`). `controller.py` correctly wraps with
`ahrs.Quaternion([...]).to_DCM()`; `img_data.py` never did. Fixed by adding
`from ahrs import Quaternion` and wrapping at all three call sites (lines
721/849/865), matching the SITL (`PX4_Gazebo/src/`) pattern. Verified with
`py_compile`; confirmed working by the subsequent successful calibration run
(`Fri Jul 10 05-08-12 2026`, real flow/feature samples logged).
