---
name: pi-landing-test
description: Run/monitor/debug the first (or any subsequent) real-hardware PLASMC landing test on the Pi (`hardware_landing.py`). Use when the user wants to fly the actual landing test on the drone (not calibration recording), asks about landing-test safety mechanisms (marker-loss grace, watchdog, TOUCHDOWN_DETECTED), or asks whether the hardware is ready to fly. Encodes what was fixed/deployed on 2026-07-28 (perception pipeline + real output calibration), what's still open, and the pre-flight checklist.
---

# Pi hardware landing test

**Entry point:** `Hardware/scripts/hardware_landing.py` on the Pi
(`doctor@192.168.0.161:~/ws/scripts/precise_landing/hardware_landing.py`). Modeled on Gazebo's
`PX4_Gazebo/apps/landing_test.py`, driving the same `Controller` class, with the Gazebo-only
pieces removed (no ROS2 ground truth, no fly-to-ENU-IC, no MATLAB post-hoc classification).
Keeps everything safety-relevant: warmup before controller engage, marker-loss grace + open-loop
fallback descent, `TOUCHDOWN_DETECTED` handling, a hover/descent-stall watchdog, clean shutdown.

**Before invoking anything here, read** `project_pi_final_landing_prep_2026_07_28` (memory) —
it's the full day-1 session summary this skill is built on top of. Don't re-derive it; use it.
**Also read `project_pi_izeta_kappa_ratchet_fix_2026_07_31`** — first real closed-loop flights
after 07-28 (4 flown, 2 blew up: a_u to 59-180, kappa up 20-24x). Root cause + two code fixes
(izeta/is_e_n conditional integration, `_flowMap`/`_loomMapM` duration cap) are summarized in the
new checklist item 5 below; both fixes are UNTESTED LIVE.

## Pre-flight checklist (read this before the user flies)

1. **Output (image/flow) calibration — DONE 2026-07-28.** `_sensor_cal_hw`/`_sensor_cal_s`/
   `_sensor_cal_ring` in `img_data.py` are now the real `derive_pi_cal.py` aggregate (44/48 runs),
   not the old never-applied placeholder. **R² is moderate (~0.3-0.6), not high** — user made an
   informed decision to proceed anyway (dominant limiter is a marker/HFOV FoV geometric ceiling,
   not a bug). Don't re-litigate this call; know the number if asked.
2. **Input (thrust/rate) calibration — CONFIRMED CALIBRATED, 2026-07-31.** The old "placeholder"
   language above was stale — `hardware_landing.py` lines 14-18 now state the defaults ARE the
   final calibration, and `HOVER_THROTTLE_NORM=0.42` / `THRUST_SLOPE_N_PER_UNIT=31.98` exactly
   match `Test_Data/Calibration/Input_Clean/CALIBRATION_RESULT.txt`'s final adaptive-trim,
   filtered, drag-corrected, R²-weighted values (2026-07-22 derivation, 47-run dataset via
   `analyze_input_calibration.py`; implied mass -1.2% off known 1.204 kg). `RATE_CORRECTION_WX/WY/WZ
   = 0.758/0.739/0.665` also matches. The `project_hover_throttle_search_2026_07_09` memory's
   `HOVER=0.388, THRUST_SLOPE=34.8` was a superseded preliminary sine-sweep bootstrap, not a
   conflicting number. **No action needed** unless the airframe (motors/props/battery/payload) has
   changed since 2026-07-22, in which case this cal is stale and should be redone.
3. **Perception pipeline fixes (all deployed + hash-verified, all default-on except
   `PLASMC_DENSE_RECOVER`):** `h_extrap`, map-derived flow (`_flowMap`, gated on
   `_planar_map_gate_on`), `ARUCO_ROI_MARGIN_PX=80` (reverted from an unvalidated 200 bump),
   frequency ~27-42Hz depending on config. None of these change happy-path behavior — they're
   additive fallbacks for marker-loss frames, so they shouldn't introduce new failure modes, but
   if something looks wrong during the flight, the **real calibration (item 1)** is the biggest
   single behavioral change today and the first thing to suspect, not the perception fallbacks.
4. **Known-open issues (don't be surprised by these):**
   - Detection-path reliability (KLT-chain failures, high ArUco `rejected_candidates`) is
     unresolved — root-caused to either detector-param tuning or the deliberate
     `CAM_EXPOSURE_US=3000/CAM_ANALOGUE_GAIN=8.0` blur-vs-noise tradeoff, never disambiguated.
     Expect some coast/marker-loss during flight; the marker-loss grace + watchdog are the backstop.
   - Marker/HFOV geometric ceiling (small marker, ~35° HFOV) bounds how far the drone can drift
     laterally before the marker leaves frame — don't expect large lateral excursions to track
     cleanly at low altitude. **2026-07-31: video-confirmed this is a REAL failure mode, not
     theoretical** — pulled frames from two real flights' recordings during a marker-loss coast and
     the camera was genuinely looking at bare pavement, marker nowhere in frame, for ~400 control
     loop ticks straight.
5. **`izeta`/`_flowMap` kappa-ratchet fix — 2026-07-31, UNTESTED LIVE.** First 4 real closed-loop
   flights after item 1's cal landed: 2/4 blew up (`a_u` to 59-180, `kappa` up 20-24x, `sigma` up
   ~80x) during exactly this marker-loss coast. Root cause: `_flowMap` (map-derived flow fallback)
   had no staleness/duration cap (unlike `h_extrap`, which decays to 0 within 10 frames) and kept
   asserting a confident nonzero dead-reckoned velocity for the entire ~400-frame coast in ALL 4
   flights (blowup or not) — this ramped `s_e_n` into the barrier (`zeta_r` hits exactly 3.6636, the
   `S_MARGIN=0.95` ceiling) and wound `izeta` up to its hard clamp. Two fixes landed in both
   `Hardware/scripts/` and `PX4_Gazebo/src/` (`controller.py` conditional-integration freeze on
   `izeta`/`is_e_n` when `FEATURE_PTS_FRESH` is false, mirroring the existing yaw `ie_a` anti-windup
   pattern; `img_data.py` confidence-floor gate on `_flowMap`/`_loomMapM_*`, new env
   `PLANAR_MAP_FLOW_MIN_CONFIDENCE` default 0.1). Full detail + replay validation in
   `project_pi_izeta_kappa_ratchet_fix_2026_07_31`. **Also found, separately: `h` (optical-flow
   velocity) under-reports the controller's own `s_dot_meas` by ~4-7x consistently in ALL 4 flights
   — likely calibration attenuation bias (moderate R² 0.28-0.44 per `derive_pi_cal.py`'s own flag),
   not yet fixed, `derive_pi_cal.py`'s regression method (OLS vs errors-in-variables) not yet
   checked.** This is why real lateral drift wasn't arrested before marker loss in the first place —
   separate from and upstream of the izeta/kappa fix.

## Key env knobs (`hardware_landing.py`)
- `LANDING_TAKEOFF_HEIGHT_M` (default 3.0), `LANDING_REF_RAD_OPT_FLOW` (default -0.30),
  `DES_ALPHA_DEG` (default 0.0)
- `HW_HOVER_THROTTLE_NORM` / `HW_THRUST_SLOPE` — see checklist item 2, verify before flying
- `RATE_CORRECTION_ENABLED` (default on) + `RATE_CORRECTION_WX/WY/WZ` (0.758/0.739/0.665)
- `LANDING_MARKER_LOSS_GRACE` (default 1.0s) — how long a marker dropout is tolerated before
  falling back to open-loop descent
- `LANDING_FINAL_DESCENT_THROTTLE` / `LANDING_FINAL_DESCENT_TIMEOUT_S` (default 5.0s)
- `LANDING_CONTROL_TIMEOUT_S` (default 90.0s) — hard abort if the controller runs this long
- `LANDING_HOVER_STALL_S` (default 25.0s) / `LANDING_HOVER_STALL_DZ` (default 0.3m) — watchdog:
  aborts if altitude hasn't changed by more than `DZ` in `S` seconds (stuck hover/descent-stall)
- `LANDING_OUT_BASE` (default `Test_Data/Landing`) — where the recording saves

## Conventions carried over from today's session
- **User runs Gazebo/hardware flights themselves** — I don't invoke `hardware_landing.py` or any
  real camera/marker/FC test myself (`feedback_no_remote_camera_marker_tests`,
  `feedback_no_px4_file_edits` supersession notwithstanding — that's about *editing* files, not
  flying). I analyze output the user pastes back.
- **All Pi code edits: SSH-edit in place, backup first, `ast.parse` + real `import` verify, then
  `sha256sum` both sides to confirm the deploy matched** (`feedback_pi_edit_in_place`). Every
  change today followed this pattern — keep doing it for any post-flight fix.
- **Pull any new recording to the workstation before analyzing it** (`Test_Data/Landing/...` this
  time, not `Test_Data/Calibration/Output/...`) — same `scp` pattern used all session for
  calibration recordings.
- If the user asks to re-derive/re-tune anything **after** seeing landing-test results, the
  `io-calibration` skill covers the calibration side; this skill is specifically the landing-test
  entry point, safety mechanisms, and pre-flight state.
