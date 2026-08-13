# Hardware Flight-Test Analysis Procedure

**Living document — update this every session that touches a real hardware flight test.**
Purpose: stop re-deriving the same extraction/diagnosis steps from scratch each session.
If you (Claude, future session) are about to analyze a Pi flight test, read this first.

Companion docs: `.claude/skills/pi-landing-test/SKILL.md` (pre-flight checklist, safety
mechanisms), `.claude/skills/planar-map-perception/SKILL.md` (PlanarFeatureMap internals,
portable from Gazebo to Pi), `.claude/skills/io-calibration/SKILL.md` (cal workflow),
`.claude/skills/tune-plasmc/SKILL.md` (Gazebo-side controller tuning history — most of it
doesn't port 1:1 to hardware, see "MATLAB/Gazebo gains don't port" gotcha below).

---

## 1. Extraction procedure (do this first, every time)

**Connectivity:** the Pi's IP changes with whatever Wi-Fi it's on. Known candidates (try in
order, don't assume dead if one times out): `10.196.6.17`, `192.168.0.161`, `192.168.0.86`,
`10.176.60.133`. Ask the user for the current IP if none respond — don't guess/scan.

**⚠ Never SSH/ping the Pi while a flight is in progress or about to start** — it can add
network load/contention during a live test. Only extract data after the user confirms
testing is done for the session.

**What to pull, and from where:**

| Data | Source path (on Pi) | Destination (Windows) |
|---|---|---|
| Per-run telemetry (`Control_Data.npy`, `Img_Data.npy`, `Telemetry_Data.npy`, `Img_Params.txt`) | `~/ws/scripts/precise_landing/Test_Data/Landing/<Day Mon DD HH-MM-SS YYYY>/` (Pi writes new runs flat, no date subfolder) | `Hardware/Test_Data/Landing/<YYYY-MM-DD>/<Day Mon DD HH-MM-SS YYYY>/` — **reorganized 2026-08-04 into per-test-date subfolders**; file each pull into the matching date subfolder (check for name collisions first), don't create a separate `_pulled_<date>` staging folder |
| Videos | `~/ws/scripts/precise_landing/Test_Data/Landing/Test_Videos/*.mp4` | `Hardware/Test_Data/Landing/<YYYY-MM-DD>/Test_Videos/` |
| PX4 flight logs (`.ulg`) | FC's own SD card, `/fs/microsd/log/<date>/` — **not** on the Pi filesystem, needs MAVSDK FTP | `Hardware/Test_Data/FlightLogs/<date>/` |

**Flight logs — use the existing tool, don't hand-roll FTP calls:**
```bash
ssh doctor@<pi-ip> "cd ~/ws/scripts/precise_landing && \
  LOG_REMOTE_DATE_DIR=/fs/microsd/log/<YYYY-MM-DD> /home/doctor/denv/bin/python3 -u download_flight_logs.py"
```
This downloads to `Test_Data/FlightLogs/<date>/` **on the Pi**, then still needs pulling to
Windows (it doesn't scp automatically). The script also writes/removes a lock file
(`Test_Data/.log_download_active`) that blocks arming during download — safe to run between
flights, never *during* one. It always scopes to one date dir — **never run without
`LOG_REMOTE_DATE_DIR`**, the unscoped default lists everything ever logged.

**Bandwidth discipline (explicit user preference, 2026-07-31):** don't `scp` individual
files one at a time for bulk pulls — use a single compressed tar stream over the same SSH
connection:
```bash
ssh doctor@<pi-ip> "cd ~/ws/scripts/precise_landing/Test_Data/Landing && \
  tar czf - '<Day> <Mon> <DD>'*/ Test_Videos/'<Day> <Mon> <DD>'*.mp4" 2>/dev/null \
  | tar xzf - -C "Hardware/Test_Data/Landing/<YYYY-MM-DD>"
```
Extract into the matching `Hardware/Test_Data/Landing/<YYYY-MM-DD>/` subfolder (create it if
new) — **check for run-directory name collisions first** (`ls -d "<Day Mon DD>"*`) rather
than staging into a separate `_pulled_<date>` folder (an earlier session did this once; it
was an unnecessary parallel structure and got merged back in). Scope the tar's glob to
**only the date(s) you need** — the full `Test_Data/Landing/` history goes back multiple
days and is large. Same pattern for pulling `.ulg` logs from the Pi to Windows after
`download_flight_logs.py` has staged them there.

For code edits (not data pulls): **SSH-edit in place** (heredoc/sed), never `scp` a locally
staged copy — see `feedback_pi_edit_in_place` memory. Data extraction (this section) is the
one case where bulk `scp`/tar transfer is correct; code deployment is not.

**⚠ Pi system-clock skew, 2026-08-06 through 2026-08-10 — RESOLVED 2026-08-11.** The Pi's
system clock was stuck reporting `2026-08-06` across what were actually three separate real
calendar days (08-06, 08-08, 08-10) of testing — this stuck clock drives both the console
`Started:` timestamp and the `Test_Data/Landing/<run-dir>` folder name, so all three days'
Landing run directories landed on disk literally named `Thu Aug  6 ...`. The FC's own onboard
clock (independent hardware) stayed correct, so the `.ulg` logger paths
(`/fs/microsd/log/<real-date>/...`) are the ground truth for which day a run actually happened.
The console transcript filenames were NOT renamed (`06082026.txt`, `08082026.txt`,
`10082026.txt` — the original names, reverted 2026-08-11 after a brief rename-to-match
experiment); each file's `Started:` lines still read `2026-08-06` internally regardless of
its own filename (don't trust that field for the `08082026.txt`/`10082026.txt` files), but its
`[logger] /fs/microsd/log/...` lines give the true date, which does match the filename. These
five loose console `.txt` files (`05082026.txt`, `06082026.txt`, `08082026.txt`,
`10082026.txt`, `11082026.txt`) sat directly under `Hardware/Test_Data/Landing/` until
2026-08-11, when they were moved into their matching `<YYYY-MM-DD>/` subfolder (matching this
doc's `<YYYY-MM-DD>/<DDMMYYYY>[_N].txt` convention below) — e.g. `08082026.txt` now lives at
`Hardware/Test_Data/Landing/2026-08-08/08082026.txt` even though its filename encodes the
same misleading `Started:` date issue described here, not its true date. Each
file's session forms one
contiguous chronological block, so the Landing run-directories were reconciled by extracting
each file's `Flight data saved -> Test_Data/Landing/<dir>` lines and moving those directories
out of the mixed `Hardware/Test_Data/Landing/2026-08-06/` bucket into correctly-dated
`2026-08-06/` (12 runs, true), `2026-08-08/` (42 runs), `2026-08-10/` (31 runs) subfolders —
now consistent with `Hardware/Test_Data/FlightLogs/<date>/`'s `.ulg` counts (13/39/29
respectively; the small mismatch is expected — some armed attempts produced a `.ulg` but
aborted before a Landing run directory was saved, or vice versa when a session's tail output
was lost to an SSH drop before printing `Flight data saved ->`, e.g. the `13-58-53` run).
**If a new console log's `Started:` date looks internally inconsistent with its own
`[logger]` line dates, suspect this same stuck-clock pattern before trusting either field
blindly — cross-check against the FC's `.ulg` date directories.**

**⚠ Recurrence, 2026-08-11 (intraday, no date-folder move needed).** Same stuck-clock
mechanism recurred same-day: a batch of 31 `Test_Data/Landing/` run directories and their
console `Started:` lines both read `Tue Aug 11 16:03:59`–`16:54:56`, but the matching 32
`.ulg` files (confirmed by count/session-shape correlation, not by trusting either clock) are
timestamped `17:31:42`–`18:36:46` in `/fs/microsd/log/2026-08-11/` — there are zero `.ulg`
files anywhere in the `15:xx`/`16:xx` hour range that day. User confirmed no testing happened
4–5pm; the real session was 5–6pm. Because both mislabeled artifacts stayed within the same
calendar date this time, no directory move was needed (unlike the multi-day 08-06/08/10 case
above) — this is purely a **within-file timestamp problem**: don't trust the `16:xx` label on
this specific batch for anything time-sensitive (e.g. correlating against video/other
instrumentation by wall-clock time). **Lesson: always sanity-check a session's claimed hour
against the actual `.ulg` hour-range on the SD card before assuming Pi wall-clock timestamps
are trustworthy, even within a single already-correct calendar date.**

---

## 2. Console-log triage (fast, no data pulled yet)

If you only have the terminal transcript (`Hardware/Test_Data/Landing/<YYYY-MM-DD>/<DDMMYYYY>[_N].txt`),
grep for these markers first — they answer "what happened" before any numeric analysis:

```
Started:|Test Summary|Altitude:|Duration:|RuntimeError|Error:|CRITICAL
Marker lost beyond grace|open-loop fallback|RTL handoff failed
Failsafe activated|No data collected|Landed \(
PositionBody:                          # launch-hover offset from home
cbf_corners.*none_streak               # perception staleness (needs PLANAR_MAP_DBG=1)
```

**Multiple transcript files for one session are common** (terminal restarts, SSH drops) —
check `wc -l` and the first `Started:` timestamp of each before assuming they're sequential
or duplicates. One past session had a `.txt` and a `_1.txt` that were byte-identical prefix
copies of the same session; don't double-count flights.

**`PositionBody: [x_m: .., y_m: ..]` at takeoff** — compute radial offset
`sqrt(x²+y²)`. Compare against the FoV visibility budget at the takeoff altitude (see
§4 "FoV geometry" below) before attributing a marker-loss failure to software.

---

## 3. Quantitative telemetry analysis (when console text isn't enough)

Load per-run `.npy` files directly — don't guess from console text alone once you have the
data pulled.

### `Control_Data.npy` (controller-internal state, dict of arrays over `t`)
Key fields for FoV/convergence diagnosis: `s_e_n(t)` (normalized position error — `|s_e_n|`
crossing 1.0 = FoV edge), `MARKER_EXTENT_PX(t)`, `h(t)` (flow/velocity estimate),
`kappa(t)`, `izeta(t)`, `a_u(t)`, `zeta_r(t)`, `sigma(t)`.

```python
import numpy as np
c = np.load('Control_Data.npy', allow_pickle=True).item()
t = np.asarray(c['t']); t -= t[0]
sen_mag = np.linalg.norm(c['s_e_n(t)'], axis=1)
ext = np.asarray(c['MARKER_EXTENT_PX(t)'])
```

**Staleness proxy — use `MARKER_EXTENT_PX` frozen-streak, NOT bit-identical `h`.**
`h` holds bit-identical for short bursts (~9-10 control ticks) as a *normal* artifact of the
camera frame rate being slower than the control loop — this is present in every flight,
including good ones, and is NOT evidence of a genuine coast. `MARKER_EXTENT_PX` frozen for
tens-to-hundreds of consecutive frames is the correct signal for a real, sustained
perception loss:
```python
runs = []
i = 0
while i < len(ext):
    j = i
    while j+1 < len(ext) and ext[j+1] == ext[i]:
        j += 1
    runs.append((i, j, ext[i], j - i + 1))
    i = j + 1
long_coasts = [r for r in runs if r[3] > 20]   # frames; tune threshold to loop rate
```

### `Img_Data.npy` (perception-layer, dict of arrays, lengths can differ by 1 from
`Control_Data`'s `t` — truncate to `min(len(...))` before correlating, don't assume equal length)
Key field: **`Opt Flow Estimator Tag`** — tells you which estimator actually produced `h`
each frame (`'coast'`, `'lstsq'`, `'lstsq+klt'`, `'map_flow'`, etc.). Cross-reference this
against the `MARKER_EXTENT_PX` freeze windows to find which fallback path is dominating a
failure — this is how the "coast estimator has no staleness cap" finding (§4) was made.
```python
d = np.load('Img_Data.npy', allow_pickle=True).item()
tags = np.asarray(d['Opt Flow Estimator Tag'])
import collections; collections.Counter(tags.tolist())
```
Also present: `Planar Map Confidence`, `Planar Map Center`, `S Estimator Tag`, `Image
Feature Pts`, `Virtual Feature Pts`. `Time` array can be ~1 frame shorter than the tag
arrays (append-order artifact) — always slice to matching length.

**⚠ Trap: `Image Feature Pts`/`Virtual Feature Pts` are circular for validating decode
success.** These arrays hold their last value indefinitely during any coast (by design —
see `planar-map-perception` skill's freshness section), so they only change value *when a
decode/KLT-update actually succeeds*. Checking "is the marker's logged position inside the
frame, and did decode succeed" against these arrays is checking decode success against
itself — it can't independently validate whether a *currently, genuinely* visible marker is
decodable. **The only valid test is running the detector directly on raw video pixels**
(extract real frames, run `build_aruco_detector()` on them) — verified by trying the
corner-log approach first and getting a tautological 100%/0% split that dissolved under
scrutiny (2026-07-31).

### `.ulg` PX4 flight logs (EKF/GPS/attitude ground truth, independent of our Python stack)
```bash
ulog2csv <file>.ulg -o out_dir     # then inspect estimator_status, vehicle_local_position, failsafe_flags
```
Use this when altitude/position telemetry from the console looks physically implausible
(e.g. reported altitude going deeply negative or far overshooting a commanded RTL target) —
that's a PX4 EKF/GPS-layer question, not a controller-logic one. **Per 2026-07-31 user
direction: pre-arm EKF/GPS instability findings are out of scope for controller/perception
analysis** — the controller doesn't rely on GPS. Still worth noting in a report if seen, but
don't chase it as part of a controller/perception debugging thread.

### Video (`Test_Videos/<timestamp>.mp4`)
Use `cv2` on **native Windows paths** (`C:\...`), not MSYS `/c/...` paths — `imwrite`
silently fails on the latter. `Hardware/scripts/align_video_to_tags.py` pairs a run's video
frame index to its `Img_Data.npy` row (validate with `--validate`: checks `frame_count ==
len(Time)` before trusting index alignment — this invariant was unverified as of 2026-07-31,
first confirm it before relying on frame↔tag correspondence in an analysis).

---

## 4. Known failure-mode catalog (update this table as new ones are found/fixed)

| # | Failure mode | Diagnostic signal | Status |
|---|---|---|---|
| 1 | **FoV geometry budget** — marker physically can't survive lateral drift at altitude | HFOV≈34.8°, 0.28m marker → lateral budget ≈ ±0.31·altitude(m). Compare `PositionBody` offset at takeoff against this. | Structural (hardware), not fixable in software without a wider lens/larger marker/lower altitude |
| 2 | **`CBF_CORNERS_STALE` false-abort** — fast ~30-frame kappa-freeze threshold was reused for the mission-abort decision, tripping on ordinary coast bursts (2-327 frames is normal) | `cbf_corners src=none` streak climbing while `Marker lost beyond grace → RTL` fires almost instantly | **Fixed 2026-07-31** — `CBF_CORNERS_STALE_ABORT` (350-frame threshold) added, deployed to Pi (`Hardware/scripts/`). Gazebo-side (`PX4_Gazebo/apps/landing_test.py`) still pending — task tracked, needs Ubuntu SSH |
| 3 | **`'coast'` fallback estimator has no staleness cap** — `Opt Flow Estimator Tag == 'coast'` dominates real flights (97-100% of frames in some), holds last flow value indefinitely with no decay, dead-reckons `s_e_n` monotonically past the FoV edge | `s_e_n` magnitude climbing linearly/unboundedly during a `MARKER_EXTENT_PX`-frozen window while `Opt Flow Estimator Tag=='coast'` | **Open** — not yet fixed. Distinct from the `_flowMap` duration cap (fixed 07-31) and the `izeta`/`kappa` conditional-integration freeze (fixed 07-31) — those stop the *integral* from winding up but don't stop `s_e_n` itself from drifting |
| 4 | **`chi_r`/`p_r_inf` gain choice outside the MATLAB-proven-safe region** | Deployed `chi_r=1.5`/`p_r_inf=0.8` vs proof-validated `chi_r≈0.85`/`p_r_inf≥1.0`; code comment on `controller.py` itself flags `chi_r=1.5` as "⚠️ DIVERGES from MATLAB manuscript" | **Open, untested on hardware** — recommended A/B (`PLASMC_CHI_R_X/Y=0.85`, `PLASMC_PRINF_X/Y=1.0`) not yet flown |
| 5 | **`h` (optical-flow velocity) under-reports true drift ~4-7x** | Compare logged `h` against controller's own `s_dot_meas` (KF-filtered rate of position feature) | **Open, partially explained** — attenuation bias (noisy-regressor OLS bias) accounts for ~1.1-2.8x; remainder suspected train/run mismatch in `derive_pi_cal.py` (fits raw corner-flow, not the fused runtime signal) |
| 6 | **Pre-arm EKF/GPS instability** (`height estimate not stable`, `GPS ... Drift too high`) | Repeated `Preflight Fail` cycling before `is_armable`, sometimes 3+ consecutive attempts; also seen mid-RTL as implausible altitude (-6m, +14m) | **Out of scope** (2026-07-31 user direction — controller doesn't rely on GPS). Still worth noting if seen, not worth debugging as part of controller/perception work |
| 7 | **Launch position outside FoV visibility budget** | `PositionBody` radial offset at takeoff exceeds the budget in #1 | Not a bug — a test-setup/launch-precision issue. Recommend launching directly over the marker within budget |
| 8 | **`PlanarFeatureMap` couldn't bridge >2s of marker invisibility** — `decode_staleness_max_seconds` (`PLANAR_MAP_DECODE_STALENESS_SECONDS`) hard-decays `map_confidence` to 0 over a fixed window | `map_confidence` linearly hits 0 exactly `_since_decode` seconds after the last real decode, regardless of whether tracking is actually still good | **Fixed 2026-07-31 (default raised 2.0→5.0), deployed to Pi — but UNVALIDATED.** KLT-only tracking accumulates drift the longer it runs uncorrected; raising the trust window without confirming `rigid_ok`/reprojection-error still catch a bad estimate at 5s risks confidently reporting a wrong position for longer. **Validation plan (no flight needed):** QTM mocap bench test — `output_calibration.py` already co-records QTM ground truth with camera data; cover the marker by hand for ≥5s during a GT-recorded run, replay through `img_data.py`/`planar_map.py`, compare `PlanarFeatureMap`'s `s`/`alpha`/`h` against QTM truth. |
| 9 | **ArUco decode fails even when the marker is clearly, fully visible in-frame** (single-frame flicker within otherwise-good tracking) | Real frames pulled from `Opt Flow Estimator Tag` transitions (`lstsq`↔`coast`) around a decode gap, marker visually confirmed present/well-contrasted | **Tested and ruled out as a tuning fix (2026-07-31).** Widening the adaptive-threshold window sweep (`ARUCO_THRESH_WIN_MIN/MAX/STEP`, live default effectively disables multi-scale sweep at `min=max=15`) recovers *some* individual failing frames but **also newly breaks others that the live config decoded fine** — a wash, not a net improvement, at 3.4x the per-frame CPU cost (5.75ms vs 1.67ms). **Don't re-try generic threshold-window tuning for this.** Also: these isolated 1-3 frame blips are already bridged gracefully by the existing KLT fallback (`lstsq+klt` tag appears right after a miss) and don't accumulate into the multi-second coasts that actually matter — the real "coast" problem (catalog #1/#3) is driven by genuine marker absence/edge-clipping, confirmed separately by direct frame inspection, not by decodability of a present marker. |
| 10 | **Baseline decode rate when marker is genuinely, continuously fully visible** | Direct-detector test on a real, visually-verified 26-frame span (flight `17-38-47`, video frames 400-425): 22/26 decoded (~85%) | **Measured 2026-07-31, one clean span only.** ~15% single-frame miss rate matches catalog #9 (KLT-absorbed flicker, not a real problem). Getting a larger clean sample is hard — see the `Image Feature Pts`/`Virtual Feature Pts` circularity trap above; most attempts to build a bigger "fully visible" sample from logged corner data are contaminated by held/stale positions from genuine marker loss elsewhere in the flight, not independent evidence. Trust direct-pixel tests over corner-log-based ones for this question. |
| 11 | **Image-processing rate vs control-loop rate** | `Img_Data.npy`'s `Time`-array delta → effective processed-image rate; `Control_Data.npy`'s `t`-array delta → control-loop rate | **No regression, confirmed 2026-07-31.** Measured ~25-31Hz image processing, ~60.5Hz control loop — consistent with the documented post-optimization target in `img_process_freq_optimization.md` (34-46Hz, camera-fps-bound at ~29.5Hz, not compute-bound). Camera's own `FPS` field reports higher (mean 56Hz) but drops as low as 6.3Hz at times — the `Time`-array-derived rate is the more honest "what the controller actually saw" number. |
| 12 | **Poor-lighting ArUco decode: gain/post-processing alone cannot fix it; real exposure time is the correct lever, and it's now been validated live** | Original poor-light default (`CAM_EXPOSURE_US=3000`, `CAM_ANALOGUE_GAIN=8.0`): 0% decode. Synthetic digital brightness-boost (2-8x) on that same footage: still 0-2% decode even after matching the well-lit clip's brightness — proves the problem is sensor noise floor (photon starvation), not display levels; denoising made it worse (blurs the fine edges ArUco needs, same mechanism as motion blur) | **Fixed and validated live, 2026-08-01.** Root cause: real analog/digital gain amplifies existing noise proportionally, it doesn't add missing signal — only real photon integration time (`ExposureTime`) fixes a noise-floor-limited image. Confirmed the hardware ceiling directly via Picamera2 (`camera_controls`): `AnalogueGain` range `(1.0, 16.0)`, `ExposureTime` range `(1, 66666µs)`. Also confirmed the *effective* achieved capture rate is ~25-31Hz regardless of the requested rate (see #11) — so requesting a 60Hz `FrameDurationLimits` was needlessly capping `ExposureTime` at ~16667µs; explicitly requesting `CAPTURE_RATE_HZ=30` (matching what's actually delivered) raises that ceiling to ~33333µs at no real cost. **Recommended poor-light profile: `CAM_MANUAL_EXPOSURE=1 CAPTURE_RATE_HZ=30 CAM_EXPOSURE_US=20000 CAM_ANALOGUE_GAIN=16.0`** — validated on 5 real bench recordings: 3 valid takes (2 were invalid, camera pointed at pure darkness — confirmed visually, not a settings problem) gave 7.9%/43.4%/65.4% decode rates (target brightness ~73-90, contrast~36-40 was the best-performing take), vs 0% at the old default in equivalent lighting. **Blur tolerance at 20ms also checked directly from the same footage** (no new recording needed — motion naturally varies within a hand-held clip): decode rate stays 65-72% up to a frame-to-frame motion-diff of ~10, degrades to 50% at 10-15, collapses above ~15 (thin sample, n=2). **Still open:** this is a poor-light profile, NOT a universal default — applying 20ms exposure in good lighting will overexpose (the original two-clip comparison already showed one fixed setting can't serve both regimes). A closed-loop gain controller (fixed short exposure for blur safety + auto-adjusted gain targeting the ~73-90 brightness sweet spot) is the correct long-term fix; drafted 2026-08-01, see `imgstreamer.py`. |
| 13 | **Closed-loop gain controller (`CAM_AUTO_GAIN=1`) — validated live, real result beats every static-gain test** | `imgstreamer.py`: keeps `ExposureTime` fixed (blur safety untouched), periodically measures frame brightness and nudges `AnalogueGain` (proportional, damped, clamped to the confirmed `[1.0, 16.0]` hardware range) toward `CAM_GAIN_TARGET_BRIGHTNESS` (default 85) | **Validated 2026-08-01, `CAM_MANUAL_EXPOSURE=1 CAM_AUTO_GAIN=1 CAM_EXPOSURE_US=20000 CAPTURE_RATE_HZ=30`, 60s real bench recording: 73.7% decode rate (297/403 sampled frames) — the best result of the day, beating the best static-gain test (65.4%).** Brightness-over-time trace showed textbook closed-loop behavior: cold-start dark period → proportional overshoot → settle near target → a mid-recording scene/lighting change caused a second dip → controller re-converged and held steady (~85-96 mean) for the remainder. Confirmed via direct frame extraction that the marker was in FoV throughout, including during the dark cold-start (it just wasn't bright enough to decode yet, not a framing problem). **Found + fixed same day: slow cold-start convergence** — the first ~15% of the recording (~9s) was spent dark before the damped `CAM_GAIN_STEP_FRACTION=0.3` caught up. Added `CAM_GAIN_FAST_START_UPDATES` (default 3) / `CAM_GAIN_FAST_START_STEP_FRACTION` (default 1.0, i.e. jump straight to the estimated target) for the first few update cycles only, reverting to the normal damped rate once roughly converged — avoids re-introducing steady-state oscillation while fixing the slow start. **Deployed to both the Windows repo mirror and the Pi (confirmed via `grep CAM_GAIN_FAST_START imgstreamer.py`), 2026-08-01.**

**Wired into the real flight pipeline + `check_loop_freq.py`, 2026-08-01:** `hardware_landing.py` now sets `CAPTURE_RATE_HZ=30`/`CAM_EXPOSURE_US=20000`/`CAM_AUTO_GAIN=1` as defaults (in addition to the existing `CAM_MANUAL_EXPOSURE=1`) — real flights use this configuration automatically, no env vars needed. `img_data.py`'s `IMG_PROCESSOR` capRate default changed from hardcoded `60` to `CAPTURE_RATE_HZ`-driven so the real pipeline (not just `record_test_feed.py`) can use the 30Hz/longer-exposure headroom. `check_loop_freq.py` updated to match (was hardcoding `CAPTURE_RATE=60` locally, which would have silently overridden the env default via its explicit `capRate=` kwarg — fixed to read the same env var). **Live loop-frequency check confirms no regression, actually a slight improvement**: old (60Hz/3000µs/static gain=8.0) gave Ring Time avg 26.1Hz, p95 53.0ms, max_gap 0.09s; new (30Hz/20000µs/auto-gain) gave avg **29.7Hz**, p95 **43.1ms**, max_gap **0.06s** — higher throughput AND tighter tail latency, likely because requesting the honest achievable 30Hz reduces scheduling contention the previous over-ambitious 60Hz request was causing. Concern about 20ms exposure eating into the processing budget did not materialize.

**Cold-start speedup validated WITHOUT a new recording** — calibrated the scene's brightness-per-gain relationship from the one real data point already in hand (mean=16.4 at gain=8.0, the actual start of the 192001 recording) and simulated both controllers' exact update math against it: old (damped-only) predicted ~5.5s to cross a decodable brightness floor (~11 small steps); new (fast-start) predicted ~0.5s (one jump straight to the clamped gain ceiling). Directionally matches the real recording's observed ~9s cold start under the old logic (the simulation's linear brightness∝gain assumption underestimates the absolute time somewhat — real sensor response isn't perfectly linear near the gain ceiling — but the >10x relative speedup is robust to that and doesn't depend on getting the nonlinearity exactly right). **User-accepted this analysis in place of a confirmatory live re-test** (2026-08-01) — if the cold-start still looks slow on a future real recording, revisit whether the linear-response assumption broke down enough to matter, don't assume the fix silently failed to deploy. |

**Camera settings propagated to every remaining capture script, 2026-08-01.** Beyond
`hardware_landing.py`/`img_data.py`/`check_loop_freq.py` (above), the same
`CAM_MANUAL_EXPOSURE=1 CAPTURE_RATE_HZ=30 CAM_EXPOSURE_US=20000 CAM_AUTO_GAIN=1` defaults were
wired (via `os.environ.setdefault`, still overridable on the command line) into every other
`Hardware/scripts/` file that constructs `imgstream(`/`IMG_PROCESSOR(` directly:
`live_preview.py` (also fixed hardcoded `capRate=60` literal → env-driven; superseded its old
"don't force manual exposure" rationale — that made aiming checks reflect a different regime
than what the drone actually flies with, which defeated the point of an aiming check),
`record_test_feed.py` (already had `CAPTURE_RATE_HZ`; added the exposure/gain pair, default
bumped 60→30), `capture_frames_for_review.py` (only the "manual exposure" burst — the "auto
exposure" burst deliberately still forces `CAM_MANUAL_EXPOSURE=0`, that comparison is the
point of the script), `bench_planar_map_marker.py`, `bench_planar_map.py`, `display_camera.py`
(had no env defaults at all before), `record_rotation_check.py` (had `CAM_MANUAL_EXPOSURE=1`
already; added the other three, hardcoded `CAPTURE_RATE=60`→env-driven), and `test.py` (scratch
file, wired for consistency). All deployed to the Pi (`~/ws/scripts/precise_landing/`),
timestamped `.bak_before_camwiring_20260801_204135` backups kept for the 7 that pre-existed
there, `ast.parse`-verified post-deploy.

**Correction (2026-08-01, later same day):** `output_calibration.py`'s local repo copy already
had the `CAM_EXPOSURE_US=20000`/`CAM_AUTO_GAIN=1`/`CAPTURE_RATE_HZ=30` additions from an
earlier point in this session (comments dated "MATCHED TO hardware_landing.py 2026-08-01") —
but **the Pi's live copy had never actually received that deploy**: `grep` on the Pi showed
the old hardcoded `CAPTURE_RATE = 60` and no `CAM_EXPOSURE_US`/`CAM_AUTO_GAIN` lines at all, a
silent local-repo-vs-Pi drift. Backed up (`output_calibration.py.bak_before_camwiring_<ts>`)
and deployed the correct file to the Pi, `ast.parse`+grep verified. **Lesson: a comment saying
a file was "matched"/"updated" is a claim about the local repo, not proof the Pi actually has
it — always grep the live Pi copy directly when auditing what's deployed, don't trust file
comments alone.**

`record_lever_arm.py` remains deliberately unwired directly — it imports and reuses
`output_calibration.py`'s env defaults verbatim (its own comment: "must be made under the SAME
regime as the ordinary output-cal recording it complements, or its coast/margin numbers aren't
comparable"), so it inherits the now-confirmed-deployed settings through that import instead.

---

## 5. Cross-reference — where things live

- **Pre-flight checklist / safety mechanisms:** `.claude/skills/pi-landing-test/SKILL.md`
- **PlanarFeatureMap internals** (rescue/override gates, `FEATURE_PTS_FRESH` vs
  `FEATURE_IS_STALE`, confidence blind spots): `.claude/skills/planar-map-perception/SKILL.md`
  — written for Gazebo but the module (`src/planar_map.py`) is shared/portable to the Pi.
- **Calibration workflow** (input/output, phased vs validation data separation):
  `.claude/skills/io-calibration/SKILL.md`
- **Gazebo-side controller tuning history** (gain sweeps, limit-cycle mechanisms, the
  combined-barrier surface derivation): `.claude/skills/tune-plasmc/SKILL.md` — **most
  numeric gain values do NOT port directly to hardware** (different loop lag/noise profile);
  treat as *mechanism* reference (why a limit cycle forms, what `chi_r` trades off against),
  not a source of hardware-ready numbers.
- **Sh script patterns:** `.claude/skills/sh-script-patterns/SKILL.md`
- **Design docs:** `PX4_Gazebo/docs/PLASMC_TUNING_GUIDE.md` (start here for any Gazebo-side
  tuning/diagnosis — auto-injected each Gazebo session), `PARAMETER_ANALYSIS.md`,
  `PERCEPTION_FLOW_FINDINGS.md`, `CONTROL_FRAMEWORK_REVIEW.md`, `FUNNEL_CBF_DESIGN.md`.
- **Memory:** `project_pi_izeta_kappa_ratchet_fix_2026_07_31`,
  `project_pi_video_tag_alignment_2026_07_31`, `project_pi_coast_root_cause_2026_07_27`,
  `reference_pi_input_cal_final` — check `MEMORY.md` index for the full, current list.

---

## 6. Changelog

- **2026-07-31 (created):** first version, written after a session that repeatedly
  re-derived the same extraction/diagnosis steps across ~5 rounds of hardware flight-test
  analysis in one day. Captures: extraction procedure (Pi paths, `download_flight_logs.py`,
  tar-based bulk pull), console-log triage markers, `Control_Data.npy`/`Img_Data.npy` field
  reference + the `MARKER_EXTENT_PX`-freeze-streak staleness methodology, and the failure-mode
  catalog current as of that session (`CBF_CORNERS_STALE_ABORT` fix deployed to Pi; `'coast'`
  estimator no-cap issue found but not yet fixed; `chi_r`/`p_r_inf` gain concern still
  untested).
- **2026-07-31 (same day, later):** added catalog #8/#9. `PLANAR_MAP_DECODE_STALENESS_SECONDS`
  root-caused as the reason `PlanarFeatureMap` only bridges ~2s of marker invisibility (was
  simply never configured for 5s) — raised to 5.0, deployed to Pi, but flagged unvalidated
  pending a QTM mocap bench test (no flight needed). Separately, tested and **ruled out**
  adaptive-threshold-window widening as a fix for ArUco misses on a clearly-visible marker —
  real-frame testing showed it's a wash (trades which frames succeed/fail, no net gain, 3.4x
  CPU cost) — don't re-try this lever; the real "coast" problem is FoV geometry, not detector
  tuning.
