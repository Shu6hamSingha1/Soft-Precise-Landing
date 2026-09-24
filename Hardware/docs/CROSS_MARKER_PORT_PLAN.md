# Cross-marker + post-08-31 PX4 feature port → hardware (Pi)

*Created 2026-09-24. Living doc — update the status column as stages land.*

## 0. Starting state (verified 2026-09-24)

| Item | State |
|---|---|
| Pi `~/ws/scripts/precise_landing` vs repo `Hardware/scripts` | **In sync as of 2026-09-24.** Deployed 7 files that were behind: `controller.py`, `flight_controller.py`, `hardware_landing.py`, `hw_pos_feedback.py`, `img_geometry.py`, `numerical_methods.py`, `planar_map.py`. Backups on the Pi are `*.bak_before_repo_sync_<ts>`. All files parse, the six library modules import, and SHA256 matches. The rest of the shared `.py` files were already byte-identical. |
| What the sync delivered to the Pi | 08-26/27 fixes that had never been deployed. (1) GT-touchdown false-trigger fix (rate+progress guards). 48 of 56 runs on 08-26 wrongly detected touchdown at 2.4–3.1 m. (2) `PLASMC_TD_GT_SEN_MAX=40` relaxed sen gate. (3) `extrapolate()` t/y off-by-one. (4) `CMD_TRANSPORT=dds` → MAVSDK fallback. (5) `P2INF_X/Y` 1.0→2.5 (08-28 rebake). (6) `theta_ctrl(t)` log. **None of this has been flight-tested.** |
| Hardware perception | ArUco + PlanarFeatureMap (`img_data.py`), old `cbf_visibility.cbf2_filter`. |
| Hardware `controller.py` | A fork of PX4 `controller.py` from ~07-07 (`fb2ae9f`), with fixes cherry-picked in both directions since. It still differs from `fb2ae9f` by 767 lines and from `623bdc39` (08-31) by 1539 lines. |
| PX4 since 08-31 (`623bdc39..a2787b5b`) | `controller.py` +1424/−…, `cross_marker_detector.py` +1334, `cross_marker_perception.py` +1351, new `visibility_projection.py` (409), `cbf_visibility*.py` deleted. |
| Camera | **Pi: 320×240, fx=fy≈513.5 (HFOV ≈35°)** (`img_geometry.py`). **SITL: 320×240, fx=fy=135 (HFOV ≈100°).** A pixel on the Pi is ~3.8× smaller in angle, and the FoV is ~3× narrower. |

## 1. Rules for this port

* **Port the structure, re-derive the values.** No gain, threshold or calibration value moves across unchanged unless it is dimensionless *and* independent of the plant. This follows `feedback_gain_values_not_portable_either_direction` and `feedback_sitl_vs_hardware_no_blind_copy`.
* Anything measured in **pixels** in the cross pipeline was tuned at f=135 px. It either scales by 513.5/135 ≈ 3.8 (when it is really an angle) or must be re-tuned (when it is really a count, for example "≥8 inliers per arm").
* Judge perception by **correlation / nRMSE of measured vs true s, α, h, w** against QTM mocap, not by whether a landing happens (`feedback_judge_perception_not_landing`).
* New features stay **default-OFF on hardware** until each has passed its own bench or flight check. Pi edits follow the in-place pattern: backup, `ast.parse` + import, `sha256` both sides.

## 2. Feature inventory — PX4 validation status → hardware applicability

| # | Feature (PX4 commit) | PX4 validation | Applies to HW? | Blocking prerequisite |
|---|---|---|---|---|
| F1 | **Two-tier visibility QP** `visibility_projection.py` (82fa9c16, e63751e2, 827933b5 τ=0.15, 96271ba6 axis-transpose fix) | Stationary IC gates; τ baked on CBF behaviour (0 exits from the physical sensor); transpose fix on 09-17 | **Yes, marker-agnostic.** Replaces `cbf2_filter`. | `fov_limit()`/`marker_tangent()` assume the SITL `_SWAP` (post-`ROTATE_90_CW`) frame. **Re-derive this for the Pi's `R_CAM_TO_BODY`.** `CBF_BUFFER_FRAC`, `A_CAP` (thrust sphere from the real input-cal slope) |
| F2 | VDS s_dot KF inter-step glitch gate `PLASMC_VDS_KF_GATE*` (04b324b7) | Bit-identical inside the band; noise-spike guard | **Yes** | `GATE_RATE` is in normalised units/s. Check it against the Pi's clean s_dot std. |
| F3 | `CBF_SPHERE_TRUE_THRUST` deliverability-bound fix (937db5a9) | Bug fix (the old bound admitted 2.39 g) | **Yes**, and it goes into F1's QP | `A_CAP` from the Pi input cal (`THRUST_SLOPE`, mass 1.204 kg) |
| F4 | `PLASMC_AU_LEAD_QGATE` (18860629, d717d4c7) | IC1-5 18/25 vs 17/25 | Inert: `PLASMC_AU_LEAD` defaults to 0 on both PX4 and hardware. Port it as a knob. | `MARKER_EXTENT_PX / frame_min` fill: extent is cross-perception |
| F5 | `PLASMC_KAPPA_DZ_*` (532d67d1) | Default 0, inert | Port as an inert knob | — |
| F6 | `PLASMC_W_U_MAX` | Not new: the only new site is the `baselines.py` path (38dddbc). It has been baked at 2.0 since 06-30 | **N/A** | — |
| F7 | Removal of DTHETA / DGATE / RHOFOV / LFOV machinery | Replaced by F1 | **Yes**, together with F1 | Check hardware isn't relying on any of these defaults |
| F8 | **Cross detector** `cross_marker_detector.py` (+1334: span rescue a53a5f63, adapt-gate f1e93c9a, margin rescue a893a77e, `_best_pair` fix 899d8d26, fill-band ee858086) | Per-feature SITL gates (flat/clutter/rover/col) | **Yes, pure OpenCV** | Every px threshold made scale-free (R3, S3), no marker-size input; **CPU cost** (see R2) |
| F9 | **Cross perception** `cross_marker_perception.py` (+1351: geometry width bf812f1f, loom innovation gate b6a0998d/7e9843ae, CROSS_ALPHA_0 b963e207, h_z scale-rate fusion = dead end) | SITL IC gates; alpha R² 0.90-0.94 | **Yes, after heavy adaptation** | Replace the `gz_subscriber.Image_Node` source with `imgstreamer` + FC attitude/gyro; `img_data` fx/fy → `img_geometry`; `_getVirtualPts` for the Pi mount; **new output cal (`_sensor_cal_hw/_s`), CROSS_ALPHA_0, w_z scale on the Pi** |
| F10 | **Yaw-rate law** `PLASMC_YAW_RATE_LAW` (87cf0202, 5849ceaa, 17e09674, 63aa2581) | IC gate 25/25 land, 18/25 precise (cross only) | Structure yes, **values no** | Needs F9 alpha + w_z. `WZ_SCALE=2.5`, `RL_KP`, the 287 ms yaw lag and the actuation-sign inversion are all **SITL-plant-measured**, so re-measure the Pi yaw chain sign/lag first. MATLAB side found a kp slow-pole issue (`project_ic2_speed_sweep_failure_2026_09_17`). |
| F11 | **TD_V2 touchdown** `PLASMC_TD_V2`, `PLASMC_TDV2_*` (2177670b) | 09-21 fixes for 22% false-TD at altitude | Perception path: yes, after F9. **Under `HW_POS_FEEDBACK`, keep the Pi's GT-depth path as primary.** | FF thresholds now in tangent units, so they scale with f automatically; extent/sat fractions need checking at the Pi FoV |
| F12 | Moving-target lead τ·d (`condition_drift`) | Rover SITL | Only needed for a moving target; keep τ=0.15 (inert when the target is stationary) | h_xy de-rotated flow source = F9 |
| — | `baselines.py`, `rover_trajectory.py`, `gz_subscriber.py`, `dds_setpoint.py`, GT-FB `gt_feedback.py` | SITL-only | **No** | — |

## 3. Risks / open items specific to hardware

* **R1 — FoV.** A 35° HFOV vs 100° means the visibility box φ is ~3× tighter. The marker leaves the frame after much smaller lateral error, which was already the documented hardware failure mode (`project_pi_coast_root_cause_2026_07_27`, HFOV budget ±0.10-0.33 m). F1 matters *more* on hardware, but Tier-2 descent-ease will fire much more often.
* **R2 — CPU.** PX4 found today (`Memory/px4/project_20260924_terminal_perception_rate_drop.md`) that `detect()` cost scales with arm-pixel count. It goes from 5.6 ms to 28 ms per frame on a desktop CPU, and the live rate drops 49 → 12 Hz below 1 m. At fx=513 the marker fills the frame at ~3.8× the SITL altitude, and a Pi CPU is several times slower. **Profile `detect()` on the Pi with recorded frames before anything flies.** The fix PX4 suggested (subsample points before `fitLine`, keep numpy arrays) is probably mandatory here. **Measured on the Pi 2026-09-24** (synthetic 320x240 frames, `tmp/bench_cross_detect.py`), per `detect()` call with the tracked ROI, marker 60 / 120 / 200 px across: **legacy 10 / 8 / 13 ms** (78-124 Hz); **stroke 24 / 50 / 82 ms** (41 / 20 / 12 Hz), and ~150-175 ms for a full-frame (re)acquire. PX4's new `CROSS_DETECTOR=stroke` (19cdabf, the locked design, closest fit to R3) is therefore too slow on the Pi at terminal range as-is. It needs a point/pixel budget or a lower working resolution before it can fly.
* **R3 — No marker-size information (user decision 2026-09-24).** The port must not depend on the physical marker size, whether as a known metric length, a size-derived altitude band, or a `MARKER_SCALE`. Every size-dependent quantity in the detector/perception has to be scale-free: (a) express thresholds relative to the detected marker's own apparent extent (arm length, stroke width, `MARKER_EXTENT_PX`) or to the frame size, not as absolute px; (b) trigger overfill, TD_V2 and AU_LEAD fill gating on the measured extent/frame fraction (which they already are), never on a predicted altitude; (c) cap the terminal CPU cost (R2) by subsampling to a fixed point budget, not by assuming a max apparent size. Absolute-px minimums that can't be made relative (e.g. a minimum resolvable stroke width) stay as detection floors, and their effect shows up in the measured detection range on the Pi bench.
* **R4 — Suspected gyro double-count.** PX4's gyro subtraction in `_solve_jacobian` may double-count (`project_ic2_speed_sweep_failure_2026_09_17`, not yet verified). Settle this before trusting the Pi w/h calibration.
* **R5 — Untested sync.** The just-synced 08-26/27 fixes have never flown. Fly at least one ArUco session on them first, so that any regression is separated from the cross-marker work.

## 4. Stages

| Stage | Work | Validation gate (no flight unless stated) | Status |
|---|---|---|---|
| S0 | Sync Pi ← repo | sha256 + import | ✅ 2026-09-24 |
| S0b | ArUco flight session on the synced code (user flies) | TOUCHDOWN_DETECT fires at genuine contact only; no false TD at altitude | ☐ |
| S1 | **Controller rebase.** New `Hardware/scripts/controller.py` = PX4 HEAD `controller.py` + the hardware deltas re-applied (HWPosFeedback wiring, GT-depth TD path + sen-max, hardware IMG_PROCESSOR import, voltage-hover, KAPPA_MAX_XY backstop, CBF_CORNERS_STALE bypass, a_u_z clamp, s-coast/KF-coast freeze, izeta freeze, predict-gap cap, theta_ctrl log, …). Each delta gets checked: already in PX4, hardware-only, or superseded. `MARKER_TYPE` default stays `aruco`. | Offline replay of recent Pi `Control_Data`/`Img_Data`: with new features OFF, the old and new controllers give ≈ identical `a_u` | ☐ |
| S2 | F1+F3+F7 visibility QP on the ArUco path, with the Pi frame map re-derived | Unit test the frame map with synthetic tilted poses (`img_geometry`). Replay: φ on the Pi equals the physical sensor edge; vis fires where ArUco corners approach the edge | ☐ |
| S3 | Scale-free audit of `cross_marker_detector.py` / `cross_marker_perception.py`: list every absolute-px threshold. Convert each to extent-relative or frame-relative, or classify it as a resolvable-detail floor (R3) | Offline: on SITL recordings, results are bit-identical or equivalent at f=135 after the conversion (same decisions per frame) | ☐ |
| S4 | F8+F9 cross detector + perception on `imgstreamer`; profile on the Pi (R2) | Bench with QTM mocap: `output_calibration.py`-style recordings. **corr/nRMSE of s, α, h, w vs GT.** Derive `_sensor_cal_hw/_s` and `CROSS_ALPHA_0` for the Pi. process_frame rate ≥ 25 Hz across the altitude band | ☐ |
| S5 | F11 TD_V2 (perception path) + F4 AU_LEAD_QGATE + F2 VDS gate | Replay on S4 bench recordings | ☐ |
| S6 | F10 yaw-rate law: measure the Pi yaw actuation sign/lag (`impulse_response`-style), then set WZ sign/scale | Hover-yaw bench flight, `PLASMC_YAW_RATE_LAW=1` | ☐ |
| S7 | `hardware_landing.py`: mirror `landing_test.py`'s relevant changes (MARKER_TYPE plumbing, vis logs, drift-off handling) | Dry-run import | ☐ |
| S8 | Landing flights `MARKER_TYPE=cross` (user flies), features enabled one at a time | Per the flight-test analysis procedure | ☐ |
