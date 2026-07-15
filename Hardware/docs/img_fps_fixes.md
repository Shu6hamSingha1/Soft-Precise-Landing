# Pi Image FPS Fixes — 2026-07-09

Camera `fps` (raw capture rate, distinct from `img_process_freq` — see
`power_undervoltage_investigation.md` for the processing-side bottleneck)
was raised via changes in `imgstreamer.py` and `img_data.py`. Diffed from
the `.bak_*` snapshots pulled off the Pi into `Hardware/precise_landing/archive/`.

## 1. `imgstreamer.py` — raw Bayer capture instead of decoded preview

**Before** (`archive/imgstreamer.py.bak_before_rawbayer_20260709_003010`):
- `create_preview_configuration(main={"format": "BGR888", "size": resolution})`
- Captured `capture_array()` off the ISP-decoded BGR stream — full debayer +
  color-conversion cost every frame.
- Capture loop had `time.sleep(1.0 / self._capRate)`, artificially throttling
  capture to `capRate` regardless of how fast the pipeline could actually run.

**After** (current `imgstreamer.py`):
- `create_video_configuration()` with a **dual stream**: a small 320x240
  YUV420 "main" stream to keep the IPA's AEC/AGC control loop alive (raw-only
  capture disables auto-exposure, leaving frames severely underexposed), plus
  a full-resolution **raw `SBGGR10_CSI2P`** stream that's the one actually
  consumed.
- Capture pulls `capture_array("raw")` — packed MIPI RAW10 (4 pixels/5 bytes)
  — and unpacks it manually via a new `_unpack_raw10()` (takes the 8 MSBs of
  each 10-bit sample; the 2 packed LSBs are discarded, not needed for
  ArUco/optical-flow use). Skips the ISP debayer/color-convert pipeline
  entirely.
- Removed the `time.sleep(1.0/capRate)` throttle — capture now runs as fast
  as the sensor/pipeline allows.
- Added negotiated-resolution query: `camera_configuration()["raw"]["size"]`,
  with a loud warning if it differs from the requested `resolution`. The
  IMX219 only has fixed native raw sensor modes; requesting a non-native size
  makes Picamera2 silently snap to the nearest one. `getResolution()` added
  so downstream code uses the ACTUAL negotiated size, never the constructor
  argument.

## 2. `img_data.py` — resolution 960x720 -> 640x480

Working resolution dropped from 960x720 to 640x480 (less data per frame),
with `fx`/`fy`/principal-point constants recalibrated at 640x480 specifically,
and a loud warning if `IMG_PROCESSOR` resolution ever drifts from that
calibrated value (falls back to an uncalibrated geometric-center assumption
otherwise).

## Net effect

Raw-Bayer capture (skip ISP decode) + removed artificial `capRate` throttle +
smaller resolution together raised camera `fps` to the ~29.7 Hz seen in the
2026-07-10 calibration run (`Fri Jul 10 05-08-12 2026`) — close to the
sensor's native raw-mode ceiling at 640x480.

---

# CPU Governor — attempted fix, did NOT persist

Found in Pi bash history (2026-07-09 session):
```
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

Switches the CPU frequency governor from `ondemand` to `performance` (pins
all 4 cores at max clock, 1.8 GHz, instead of scaling down at idle and
ramping up reactively). Relevant both for `img_process_freq` throughput and
for undervoltage risk — `ondemand`'s reactive ramp can lag right at the
moment a sudden combined load (FC armed + camera + compute all spiking
together) hits, which is exactly the kind of transient that trips the
under-voltage detector (see `power_undervoltage_investigation.md`).

**Status as of 2026-07-10: NOT active.** This was a raw `sysfs` write, not a
systemd service or boot-time script — it resets on every reboot. Confirmed
via `uptime -s` (booted 2026-07-10 04:46:29) and
`cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor` (`ondemand` on
all 4 cores).

## To make it persist

Options, cheapest first:
1. Add to `/etc/rc.local` (before `exit 0`):
   ```
   echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
   ```
2. Install `cpufrequtils` and set `GOVERNOR="performance"` in
   `/etc/default/cpufrequtils` (cleaner, package-managed).
3. A small systemd oneshot service (`ExecStart=` the same `tee` command,
   `WantedBy=multi-user.target`) if `rc.local` isn't enabled on this image.

Not yet applied — pick one and re-verify with the same two commands above
after a reboot.
