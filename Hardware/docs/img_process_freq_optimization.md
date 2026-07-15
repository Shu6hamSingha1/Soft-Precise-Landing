# img_process_freq Optimization — 2026-07-10

## Starting point

`output_calibration.py`'s `img_node.metrics()` showed `img_process_freq`
~5.6 Hz against camera `fps` ~29.7 Hz — a ~5x processing bottleneck. Full
investigation trail (power/governor red herrings, ruled out) in
`power_undervoltage_investigation.md` and `img_fps_fixes.md`.

## Root cause: `cv2.aruco.detectMarkers` on the raw Bayer stream

In-code per-stage timing instrumentation (`img_data.py` `_tstage`/
`_tmark_frame_end`, gated behind `IMG_TIMING_DBG=1`) pinned the bottleneck
precisely: `detectMarkers`, called on the raw 640x480 unpacked-Bayer image,
cost **~300-340ms per frame-pair** (up to 150-300ms/call under difficult
visibility — motion blur, marker near edge, partial occlusion — since
ArUco's adaptive-threshold candidate search is heavily scene-dependent).
Everything else in `_optFlowAngVel` (ring flow, geometry, lstsq, KF,
checkposts, fusion) was consistently under 30ms combined.

An earlier `cProfile`-based hypothesis ("picamera2's capture-handling
dominates") was WRONG — cumulative time there included legitimate blocking
waits (`selectors.select`, which releases the GIL) that don't actually
compete with the processing thread. Looking at `tottime` (self-time, actual
CPU work) instead of `cumtime` corrected this: ArUco was the real cost.

## Fix 1: detect on the ISP-scaled "main" stream, not raw Bayer

`imgstreamer.py` already opened a small "main" YUV420 stream (kept alive
only for the IPA's AEC/AGC control loop) alongside the "raw" stream used for
everything else. The raw stream is locked to the IMX219's native sensor
modes (smallest: 640x480 — confirmed via `Picamera2().sensor_modes`, no
320x240 raw mode exists). The "main" stream has no such limit: the ISP can
scale to ANY requested size.

Changes:
- `imgstreamer.py`: `run()` now uses `capture_request()` once per loop
  iteration and pulls BOTH `"raw"` and `"main"` arrays from the same
  request (frame-synced), instead of a single `capture_array("raw")` call.
  Added `getMainImages()`/`getMainResolution()`. Main stream size is
  env-configurable via `MAIN_STREAM_SIZE` (default `320,240`).
- `img_data.py`: `_optFlowAngVel` now requires `main_imgs` (raises if
  missing — an unsafe silent fallback to raw-space imgs was removed, since
  it would double-apply the scale factor). `_detect_markers` runs
  `detectMarkers` on the smaller main-stream Y-plane image, then scales
  detected corners back up to the calibrated raw-resolution pixel space
  (`self._aruco_scale = raw_size / main_size`) before returning — every
  downstream consumer (geometry, calibration) is unaffected by which
  stream detection ran on.

**Result: two stacked, separately-confirmed wins:**
1. Switching FROM raw Bayer TO the ISP-processed main stream, at the SAME
   640x480 resolution: ~300-340ms -> ~8-9ms (~35x). Likely because the
   ISP's denoising/sharpening produces much cleaner edges for the
   adaptive-threshold candidate search than raw sensor noise.
2. Downsampling on top of that, 640x480 main -> 320x240 main: ~8-9ms ->
   ~2-6ms (further ~2-3x).

## Fix 2: ROI-crop fast path once locked

Once a marker is locked (`self._locked_marker_id` set), it barely moves
frame-to-frame. `_detect_markers` now tracks `self._last_locked_corners`
(raw pixel space) and, when set, first tries `detectMarkers` on a small
crop (`ARUCO_ROI_MARGIN_PX`, default 80 RAW px, converted to main-stream
px internally) around that location — searching far fewer pixels than a
full-frame scan. Falls back to full-frame search (same call) on a miss,
and drops the ROI entirely (forcing full-frame re-acquisition) after
`ARUCO_ROI_MAX_MISSES` (default 5) consecutive misses.

Validated live: `roi_hits`/`roi_misses`/`fullframe_searches` counters
(printed in the `[TIMING]` summary) confirmed the ROI path engages
correctly once locked, with hit rates 0-75%+ depending on how steady the
marker hold was.

**Known cost on a miss:** a missed ROI attempt pays for the (wasted) crop
search AND falls through to a full-frame retry in the same call — plus
misses correlate with harder-to-detect frames (the same motion/blur that
caused the miss also makes full-frame ArUco itself slower, per the
scene-dependent cost noted above). This compounds: miss-heavy windows
spiked to 27-47ms vs ~10-15ms in hit-heavy windows. Tested increasing
`ARUCO_ROI_MARGIN_PX` 80->150 to reduce misses: did NOT help net — bigger
crop costs more per attempt (even on hits) without cutting the miss rate
enough under hand-held motion to compensate. Reverted to 80 (default,
no code change needed — was only an env var test).

## Fix 3: print-statement / GIL-contention cleanup

Two per-frame `print()` calls were unconditionally spamming, competing for
the GIL with the compute-heavy flow-processing thread (which shares the
same process/interpreter as the capture thread and the main asyncio loop):
- `img_data.py`: `"No common marker in both frames..."` fired every frame
  during marker loss. Now fires once on the loss->reacquire transition
  (`self._no_common_marker_warned` flag), matching the existing pattern for
  `"LANDING PAD VISIBLE/NOT VISIBLE"`.
- `output_calibration.py`: `print(img_node.metrics())` ran unthrottled at
  the main loop's full 30 Hz (`SLEEP_TIME=1/30`) whenever the marker was
  visible - formatting full 17-digit float reprs and doing a stdout write
  syscall every tick. Throttled to 1 Hz, values rounded to 2 decimals.

## Fix 4: CPU governor `performance` mode, persisted correctly

See `img_fps_fixes.md` / `project_pi_hardware_fixes_2026_07_09` memory for
the full history (governor reset on reboot -> `rc.local` fix -> a bug where
`FC.close()` unconditionally reverted to `ondemand` even for scripts that
never armed, e.g. `output_calibration.py`'s hand-move takes -> fixed with a
`self._governor_boosted` flag so `close()` only reverts if
`arm_and_takeoff()` actually set it).

## Final result

`test.py` (FC connected, no mocap, default settings: 320x240 main stream,
80px ROI margin): `img_process_freq` **34-46 Hz**, consistently EXCEEDING
camera `fps` (~29.5 Hz) — the pipeline is now camera-fps-bound, not
compute-bound. Up from the ~5.6 Hz starting point: roughly a **6-8x overall
improvement**.

## Diagnostic tooling left in place (zero-cost when off)

- `IMG_TIMING_DBG=1` / `IMG_TIMING_EVERY=<n>` (default 30): per-stage
  timing breakdown printed every n calls to `_optFlowAngVel`, plus
  `roi_hits`/`roi_misses`/`fullframe_searches` counts. Single `if` check
  when off (`self._timing_dbg`).
- `MAIN_STREAM_SIZE=<w>,<h>` (default `320,240`): main-stream ArUco
  detection resolution, for future A/B testing (e.g. if longer-range
  marker-decode reliability becomes a concern - larger main resolution
  trades some of this speedup for more pixels-on-target at distance).
- `ARUCO_ROI_MARGIN_PX` (default 80) / `ARUCO_ROI_MAX_MISSES` (default 5).
