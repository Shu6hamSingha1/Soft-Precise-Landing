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
**⭐⭐⭐ Also read `project_pi_output_recal_2026_08_01` and `project_pi_pending_code_deploy_2026_08_02`
(memory) before flying again** — a real, confirmed bug (`img_geometry.py::_rp_basis` gravity-vector
sign error, live since day 1) was found and fixed 2026-08-02, but the fix is NOT yet deployed to the
Pi (was closed when found). Do not fly on the current deployed calibration believing it reflects the
best available understanding — see the updated checklist item 1 below.

**⭐⭐⭐ If you're analyzing flight-test DATA after a session (not just preparing to fly), read
`Hardware/docs/FLIGHT_TEST_ANALYSIS_PROCEDURE.md` first** — it's the living procedure doc for
extraction (Pi paths, `download_flight_logs.py`, bandwidth-conscious tar pulls),
console-log triage markers, `Control_Data.npy`/`Img_Data.npy` analysis methodology (including the
`MARKER_EXTENT_PX`-freeze-streak staleness proxy — NOT bit-identical `h`, which freezes normally
every ~9-10 ticks), and a living known-failure-mode catalog. Keep it updated as you find/fix new
issues — don't re-derive the same steps from scratch each session.

## Pre-flight checklist (read this before the user flies)

1. **Output (image/flow) calibration — the CURRENTLY DEPLOYED cal was derived under a CONFIRMED BUG,
   fix not yet on the Pi (2026-08-02).** `img_geometry.py::_rp_basis` had the gravity-vector sign
   wrong (`g = R @ [0,0,1]` instead of `R.T @ [0,0,1]`) — the exact bug `PX4_Gazebo/src/img_data.py`
   already documented fixing on 2026-06-01, never ported to the Pi. This corrupts the V-frame
   transform (hidden near-level, amplifies error during real tilt) - live in every real flight, not
   just calibration recordings. Fixed locally (`Hardware/scripts/img_geometry.py`), verified on both
   the Aug-1 (6-run) and old Jul23-28 (44-run) archives (Hx/Hy/Wz up 4-6x, centroid `sx`/`sy` went
   from unstable/physically-impossible-negative to consistently positive, corr 0.7-0.9), **but the
   fix is NOT yet deployed to the Pi** (was closed when found - see
   `project_pi_pending_code_deploy_2026_08_02`). A new candidate `_sensor_cal_hw`/`_sensor_cal_s` was
   also derived (Aug-1-only, matching the current camera config) but is PENDING REVIEW, not deployed.
   **Do not treat the "R² moderate ~0.3-0.6, not a bug" framing from 2026-07-28 as current** — it
   predates this finding. Deploy the `_rp_basis` fix (and ideally the reviewed new cal) before the
   next flight if at all possible; if flying before then, know that the deployed cal is derived from
   admittedly-corrupted geometry.
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
   `_planar_map_gate_on`), `ARUCO_ROI_MARGIN_PX=40` (current pinned default as of 2026-08-01, NOT the
   80 this line previously said — that value itself drifted 200→80→40 across project history with no
   calibration-specific reasoning, see `project_pi_video_tag_alignment_2026_07_31`), frequency
   ~27-42Hz depending on config. **Camera config also changed 2026-08-01**: `hardware_landing.py` now
   defaults to the low-light-validated profile (`CAM_EXPOSURE_US=20000`, `CAM_AUTO_GAIN=1`,
   `CAPTURE_RATE_HZ=30`), NOT the old `CAM_EXPOSURE_US=3000/CAM_ANALOGUE_GAIN=8.0` this line
   previously described. None of these change happy-path behavior — they're additive fallbacks for
   marker-loss frames, so they shouldn't introduce new failure modes, but if something looks wrong
   during the flight, the **real calibration (item 1)** and the **camera config change** are the
   biggest behavioral changes and the first things to suspect, not the perception fallbacks.
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
   `project_pi_izeta_kappa_ratchet_fix_2026_07_31`. **CORRECTED 2026-08-01 (see
   `project_pi_output_recal_2026_08_01`): the "~4-7x h under-report" finding above does NOT
   generalize.** Re-checked `h` vs `s_dot_meas` across 47 real flights (not just these 4) and found
   median ratio 0.87 (near 1:1), scattered weak correlation elsewhere - the 4-7x pattern is isolated
   to these 4 flights' marker-loss dead-reckoning specifically (already addressed by the izeta/`_flowMap`
   fixes above), not a general calibration-magnitude problem. Don't chase "closing the 4-7x gap" as a
   calibration goal - the real, confirmed calibration issue is the `_rp_basis` bug in checklist item 1.

6. **Quantified impact of the deployed (buggy) `sx`/`sy` on real flights (2026-08-02).** Deployed
   `sx=0.17` vs the corrected `sx=0.38` (same 44-run archive, geometry fix applied) — the deployed
   cal has been under-reporting real X-axis position error to the controller by roughly **2x**, for
   every flight to date, not just the ones that blew up. `sy` was comparatively fine (deployed 0.42
   vs corrected 0.35, ~18% off, opposite direction). Ring flow (`Hx/Hy/Hz/Wz`) reproduced exactly -
   never affected, always trustworthy as the fallback signal. This gives a concrete, quantified
   mechanism for why real lateral drift (X-axis specifically) wasn't well-corrected before marker
   loss in the izeta/kappa blowup flights (item 5) - not proven as the direct cause of any specific
   flight's outcome (no clean flight-data trace exists yet), but a real, structural contributor
   layered on top of the already-known dead-reckoning mechanism.

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
