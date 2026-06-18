---
name: PLASMC MATLAB→Python port status (PX4_Gazebo)
description: Current alignment state of PX4_Gazebo/ vs the MATLAB single-run reference, what's been verified, and what's still open
type: project
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
**State as of 2026-05-12 (latest commit on `main`):**

Headless SITL pipeline is end-to-end functional (Gazebo Harmonic + PX4 SITL + uXRCE-DDS + 3 bridges + QGC offscreen). Image feed validated and measured. Sensor calibration recomputed but **not yet applied**.

## ✅ Fully aligned (matches MATLAB `visualControl_IBVS_adaptive.m`)

- All outer-loop PID gains: `K_rp=diag(9,9)`, `K_ri=diag(0.1,0.1)`, `K_rd=diag(1.4375,1.4375)`
- Middle-loop envelope: `gamma=[0.2,0.2,0.2]`, `p_0=[25,25,4]`, `p_inf=[2.5,2.5,1.5]`
- SMC: `Omega=diag(0.05,0.05,0.025)`, `Gamma=diag(0.4375,0.5,0.75)`, `E=diag(1,1,1)`
- Adaptive κ: `N=diag(0.02,0.02,0.05)`, `P=diag(1.5,1.5,5.0)`, `kappa_0=[0.125,0.125,0.25]`
- Yaw κ_a: `Omega_a=0.5, Gamma_a=0.5, n_a=1.0, p_a=2.0, kappa_a_0=2.0, E_a=3.0`
- LPF on `I_a`: `tau_ia=0.08 s`
- Anti-windup clamps: `||iV_s_e_n||≤5`, `||izeta||≤5`
- FoV-margin cone (M6 ported): `theta_cap=60°`, `l_fov=0.1`, pixel margins scaled 2× from MATLAB for the 640×480 camera (`rho_fov_0=[290,210]`, `rho_fov_inf=[80,80]`)
- Thrust map (A1): `0.738 − cmd[3]/45` clipped to [0,1]
- W_d kinematics (A2 fixed): `_attCtrl` uses desired W_d to map Euler-rate→body-rate, not current W
- Full 3-axis FRD body rate (A3 fixed) instead of zeroed roll/pitch
- `g = 9.80` matches Gazebo `aruco.sdf`
- `REF_RAD_OPT_FLOW = -0.42` matches MATLAB `h_rd`

## 🟡 Architecturally intentional differences

- Inner attitude/rate loop is **PX4's, not MATLAB's geometric SO(3)**. We ship body-rate + thrust via MAVSDK; PX4 closes the rate loop. `K_ep=diag(5,5,5)`, `K_ei=K_ed=0` is the small PD that converts desired Euler to body-rate setpoint.
- Camera: Gazebo 640×480 @ fx=fy=270, hfov=1.74. MATLAB 320×240 @ f=135. **Same hfov** → gain math invariant. Pixel-space FoV margins scale 2×.

## ✅ Sensor-cal matrices APPLIED 2026-05-12

`img_data.py:65-66`:
```python
_sensor_cal_hw = np.diag([0.1518, 0.1777, 0.0651, 0.2083, 0.2209, 0.2435])
_sensor_cal_s  = np.diag([0.5814, 0.5809, 1.0000, 1.0000])
```
Derived from median across 5 valid output_calibration runs (`calibration_data/Tue May 12 16-{26-26, 34-24, 35-57, 37-33, 39-30} 2026/`). Per-axis std: 22–58% for `_sensor_cal_hw`, 3–4% for `_sensor_cal_s` (centroid is rock-solid). See `project_camera_calibration_status.md` for the workflow and `feedback_calibration_lessons.md` for what fails and how.

The legacy values `diag(1,1,1,1/3,1/3,1)` / `diag(1/6,1/6,1,1)` were for a 1280×960 camera; raw optical flow was ~10× over-scaled — likely root cause of earlier PLASMC `a_u` blow-up.

## ✅ Runtime savgol APPLIED 2026-05-13

`img_data.py:33-39` (now active in both `getOptFlowAngVel()` AND `getImgFeatureParam()`):
```python
FILTER_WIN = 13       # was nominally 51 but the dead-code path was inert
FILTER_POLYORDER = 1
```
Tuned via `tune_savgol.py` lag-aware sweep across 5 recordings × 8 channels. Pattern matches user's legacy `img_data*.py`: `sgf(buf[-W:], W, P, axis=0)[W/2+1]`. Mean|corr| improves from 0.329 (no filter) → 0.357. Centroid (0.80 → 0.76) takes a small hit; optical-flow / ang-vel improve by 5-20% absolute.

## ⚠️ DH_D_MAX retuned 2026-05-20 (smaller effect than first claimed)

`controller.py:444` now reads `PLASMC_DH_D_MAX` (default **50.0**, raised from 20.0). At cap=20, smooth4(dh_d) was saturating 34–50% of samples. Cap=50 reduces saturation to ~36% and is **at worst neutral** for precision, at best slightly better:
- Combined cap=50 plain landings (7 reps): mean xy 0.66m, std 0.66m
- cap=20 baseline (3 reps): mean xy 0.67m, std 0.61m
- The original sweep's "cap=50 mean 0.45m / std 0.16m" was a 3-rep small-sample fluke.

Uncapped (1e9) was definitively WORSE (mean 1.15m) — the rate limiter is load-bearing, just not at 20. **The xy_err distribution is heavy-tailed**: most reps 0.1–0.7m, ~15% > 1.5m. A single outlier blows the std budget regardless of cap value. The precision-variance source is NOT the c-term cap — it's somewhere else (lateral PID noise / LK corner noise / EKF drift remain the main suspects). Sweep data: `~/ws/Test_Data/DhDMaxSweep/20260520-155216/`. Re-test data: `~/ws/Test_Data/PlainCap50/`. Scripts: `PX4_Gazebo/run_dh_d_max_sweep.sh`, `run_plain_cap50.sh`, `run_combo_on_cap50.sh`.

**Combo (KP_X×1.5 + KP_Y×0.75 + N_Z=0.005) on cap=50 (5 reps):** trades precision for softness — 3/5 SOFT but mean xy 0.95m (worse than plain). Verdict: don't use the combo by default.

## ✅ Marker-loss grace period applied 2026-05-20

`landing_test.py:284-300` — when ArUco marker is briefly lost, hold the last good IBVS attitude-rate command for `LANDING_MARKER_LOSS_GRACE` seconds (default **1.0s**) before falling back to open-loop FINAL_DESCENT_THRUST. Previously the fallback engaged on the first lost frame, causing 5s of lateral drift at zero body rate.

**This was the dominant variance source.** Outlier analysis: PlainCap50/plain_rep2 (xy=2.19m) had 2 "NOT VISIBLE" events; first one immediately triggered fallback → 5s of momentum-driven drift.

Test results (5 reps each on IC1):
- No grace:       mean xy 0.66, max 2.19, std 0.66 (heavy tail)
- grace=0.5s:     partial — mid-flight transients fixed, end-of-flight still drifts
- grace=1.0s:     mean xy 0.49, max 0.77, std 0.24 — **heavy tail eliminated**, 5/5 landings

Best xy this session = 0.11m. Still 0 PRECISE landings (criteria xy≤0.08, rel_vel≤0.2) but variance ceiling is now ~0.8m instead of ~2.2m. Sweep data: `~/ws/Test_Data/MarkerGrace/`. Script: `PX4_Gazebo/run_marker_grace.sh`.

## ⚠️ REF_RAD_OPT_FLOW: env-overridable, default reverted to -0.42 (was briefly -0.70)

`landing_test.py:27` — `LANDING_REF_RAD_OPT_FLOW` env-overridable, default **back at MATLAB's -0.42** after IC2-5 validation.

**Story (2026-05-20):** Sweep over {-0.42, -0.55, -0.70} on **IC1 only** showed -0.70 dramatically improved precision (mean xy 0.71 → 0.28 m on IC1, achieved first PRECISE landing at xy=0.076). I bumped default to -0.70. THEN ran IC2-5 validation and found severe regression:
- IC2 (2,2,5)  mean xy 2.21
- IC3 (-2,2,5) mean xy 2.21
- IC4 (2,2,7)  mean xy 1.96
- IC5 (2,2,3)  mean xy 4.75 **+ one full crash** (xy=7.1m, vel=4.9 m/s)

Mechanism: starting 2.83 m off-center at altitude 3-7 m, fast descent (-0.70) gives only 3-5 s of flight; the drone hits low altitude before lateral can converge, then hovers low for 17 s trying to crawl in (some succeed SOFT but never PRECISE; IC5 ran out of altitude entirely).

**Lesson:** all of this session's earlier tuning (DH_D_MAX, grace=1.0s, REF_RAD, κ_0×1.25) was IC1-only. The grace and cap changes likely transfer (they're safety nets), but REF_RAD is a fundamental descent-dynamics knob and overfits hard. Future tuning must hit IC2-5 before any default change. Validation data: `~/ws/Test_Data/ICValidation/`. Script: `PX4_Gazebo/run_ic_validation.sh`.

**The -0.70 win is still real for IC1-style centered starts** — keep the env var for centered-IC tuning runs and the analysis paper. The PRECISE+SOFT landings at xy=0.076 / vel=0.044 came at this setting.

## ✅ Virtual-compass + SO(3) yaw control made the DEFAULT 2026-05-20 (commit 7b52e49)

`controller.py` was split: now implements ONLY the manuscript-faithful inner loop (Section III-B1/B2, control_formulation.tex:219-251). Legacy Euler-PD path moved to `controller_v0.py` for reference. Active controller:
- ψ_d state, lazy-init at first `_attCtrl` call from current yaw (MATLAB line 127)
- ψ_d integrator in `_yawCtrl`: `psi_d ← wrap(psi_d + u_a · dt)` (Eq. `psi d integrator`)
- R_d construction in `_attCtrl` from `rd3=-I_a/||I_a||` + `a_h=[cos ψ_d, sin ψ_d, 0]` (Eq. `R_d construction`)
- SO(3) attitude error `e_R = 0.5·vee(R_d^T R - R^T R_d)` → body-rate setpoint `w_u = -K_R · e_R`
- `K_R = diag(5, 5, 5) · PLASMC_KR_SCALE` with **default scale = 0.4** since 2026-05-21 (effective `diag(2,2,2)`) — env `PLASMC_KR_SCALE` overrides uniform, `PLASMC_KR_{ROLL,PITCH,YAW}_SCALE` per-axis. Legacy `K_R=5.0` recoverable via `PLASMC_KR_SCALE=1.0`.
- No env gate; no LANDING_VIRTUAL_COMPASS anymore (always SO(3))

**IC1 performance (corrected with larger N):**
- ⚠️ **The 3-rep pre-split mean of 0.36/std=0.10 was a lucky sample.**
- 8-rep cumulative (3 pre-split + 5 post-split): **mean xy 0.65, std 0.34**
- v0 legacy reference (5 reps): mean 0.49, std 0.24
- So SO(3) is currently comparable-to-slightly-worse than legacy on IC1 with adequate sample size. The structural improvement is in alignment with the manuscript (compass-free virtual-compass + R_d tracking), not in measured precision.

**IC2-5 result (3-rep, 2 reps each):**
- IC2: mean 1.64 | IC3: mean 1.79 (~ noise) | IC4: 1.59 (regresses from 0.87 legacy) | IC5: 2.88

**Net assessment:** the SO(3) implementation is structurally correct (manuscript-faithful) but not empirically better than legacy. Needs gain tuning — that's the next step (95-run sensitivity sweep started 2026-05-20 evening). Data: `~/ws/Test_Data/VirtualCompass/`, `~/ws/Test_Based/ActiveIC1/`, `~/ws/Test_Data/ICValidation/20260520-2030*`.

## ⚠️ Rate-mode SO(3) port hit structural ceiling (2026-05-21)

After extensive tuning (sensitivity sweep, K_R sweep 0.3-1.0, τ_ia sweep, w_u clamp 1.0/1.5, multi-variable combos), **no configuration reaches consistent soft+precise on IC1**:
- Aggressive K_R (≥0.4): peak body rates 4+ rad/s → LK optical-flow tracking breaks (corner motion 17+ px/frame exceeds LK window) → TARGET_LOST
- Gentle K_R (≤0.3): stable, no target loss, but slow lateral convergence (mean xy=1.19m)
- Sweet spot K_R=0.4 single rep hit xy=0.10m (unreproducible, 2/3 next reps target-lost)
- w_u clamp PLASMC_W_U_MAX (commit 1d44cd5): eliminates target loss but caps precision

**Root cause: missing `-k_Omega·e_Omega` damping term.** Manuscript's full SO(3) torque law has gyro-feedback damping that operates without image-loop delay. Our rate-mode port pushes the damping into PX4's hidden rate controller (stock x500 gains, can't tune). The K_R proportional term alone either spikes (unstable) or under-damps (slow).

**TARGET_LOST mechanism (2026-05-21):**
1. Controller commands aggressive body rate (>1.7 rad/s on our 60Hz/f=270 setup)
2. LK loses corner positions between frames (motion exceeds 15-px search window)
3. "OPTIC FLOW UNAVAILABLE" → ArUco loses corner refinement → consecutive detection failures
4. After CHECK_NUM=80 failures, FEATURE_IS_VISIBLE flips → fallback engages → TARGET_LOST per strict criterion (commit 66df93d)

**Best IC1 results this session under strict criterion:**
- 1 PRECISE-only (P_Z×0.5, n=1 lucky shot in 95-run sweep, didn't reproduce in 5-rep validation)
- 0 SOFT+PRECISE
- Mean xy ~0.6-0.8 across all configurations tried
- TARGET_LOST rate 0-40% depending on aggressiveness

Data: `~/ws/Test_Data/{SensitivitySweep,KrTest,KrSweep,KrClamp,KrClamp2,KrCombo,SingletonValidate,VertCalm,TauIaTest,SlowDescent,ActiveIC1}/`.

## ✅ Best rate-mode operating point found (2026-05-21, IC1)

```
PLASMC_KR_SCALE=0.4                  # NOW DEFAULT (controller.py:216, 2026-05-21)
PLASMC_W_U_MAX=1.0                   # body-rate clamp, default since commit 1d44cd5
LANDING_REF_RAD_OPT_FLOW=-0.70       # env-only — regresses IC2-5, never default
IMG_FILTER_WIN=7                     # env-only — needs IC2-5 validation before default
```
**As of 2026-05-21, the K_R and W_U_MAX defaults give the baseline-best-stable config out of the box.** Only REF_RAD and IMG_FILTER_WIN remain as env overrides for an additional ~20-30% IC1 precision lift (but with IC2-5 regression risk for REF_RAD).

5-rep result: mean xy=**0.387 m**, max=0.58, std=0.13, **0/5 TARGET_LOST**, 1/5 SOFT.

**Compared to session-start baseline** (K_R=5, w_u no clamp, h_rd=-0.42, WIN=13):
- Mean xy: 0.69 → 0.387 m (−44%)
- Std xy: 0.26 → 0.13 (−50%)
- TARGET_LOST rate: 12-40% → 0%

**Why this works (mechanism):**
1. K_R + w_u clamp killed the LK-breaking body-rate spikes (visibility problem)
2. h_rd=-0.70 shortened flight by ~25% → less drift window
3. IMG_FILTER_WIN=7 cut centroid feedback delay from ~100ms to ~50ms → less position lag (biggest single effect)

**Plateau identified:** further gain perturbations either bring back TARGET_LOST (KP boost, KI boost, looser clamp, shorter τ_ia) or slow convergence (lower K_R, longer τ_ia, longer Savgol). Tested ~50 configurations, ~300+ SITL landings. No PRECISE (≤0.08m) singleton was reproducible; best repeatable mean ~0.39m.

**Defaults left unchanged** (per "validate IC2-5 before merging" lesson). All env vars optional; current best config is documented above and reproducible via env override.

## IC2-5 validation of best config (2026-05-21)

Tested `K_R×0.4 + W_U_MAX=1.0 + h_rd=-0.70 + IMG_FILTER_WIN=7` on the standard off-center IC set (2 reps each):

| IC | ENU pos | mean xy | max xy | notes |
|---|---|---|---|---|
| IC1 | (0,0,5) | 0.387 | 0.58 | tuned point |
| IC2 | (2,2,5) | 2.08 | 2.41 | 3× worse, 1 TL-pattern |
| IC3 | (-2,2,5) | 1.96 | 2.08 | 1 TL-pattern |
| IC4 | (2,2,7) | 1.59 | 1.95 | 1 TL-pattern |
| IC5 | (2,2,3) | 2.47 | 2.77 | largest regression |

3 of 8 off-center reps showed the long-flight (~18s) + low-vel pattern (= TARGET_LOST + open-loop fallback). The IC1 precision improvements (`h_rd=-0.70`, `IMG_FILTER_WIN=7`) do not transfer — they depend on starting near-centered.

**Bottom-line tuning state for the rate-mode port:**
- IC1 mean ≈ 0.39 m, std 0.13 m (best singleton 0.10 m, unreproducible)
- IC2-5 mean ≈ 1.6-2.5 m
- No soft+precise within strict criterion at any IC
- Defaults must not include the IC1-optimized envs (h_rd=-0.70, IMG_FILTER_WIN=7); they regress off-center starts
- Env-overridable knobs preserved for analysis/paper runs at IC1

## Sensor calibration refresh attempt (2026-05-21) — failed

Ran record_output_calibration.py fresh (5 valid runs out of 12 attempted). `aggregate_calibration.py` produced values 7-10× smaller than current `_sensor_cal_hw` and +5% on `_sensor_cal_s`. Methodology mismatch: aggregate uses `median(gt/raw)`, the current values came from a `std-ratio` path that produces factor-5-10 different scaling when raw-vs-gt correlation is low (0.2-0.5 per axis here). Tested the safer partial update (just `_sensor_cal_s` +5%): regressed mean xy 0.387 → 1.174 m. Reverted. Current cal stays at the 2026-05-13 9-run values.

Fresh calibration data preserved at `calibration_data/output_archive_*/` for future analysis.

## ❌ Still open

- **Combine cap=50 with prior per-axis best gains** (KP_X×1.5, KP_Y×0.75, N_Z=0.005) — none individually reached PRECISE; the cap was the bottleneck.
- **`alpha` (s[3]) auto-calibration not implemented** — needs target-yaw ground-truth derivation. Set to 1.0 in the applied matrix.
- **Optional: bypass savgol on centroid** — the slight centroid degradation could be avoided by reverting `getImgFeatureParam()` to its un-smoothed form. Worth A/B testing in SITL.

## How to apply

- Edit `img_data.py:50-51` with the proposed diag values.
- Run a fresh headless landing test (`HEADLESS=1 ./run_aruco_landing.sh`) to verify.
- If still unstable, re-run calibration via `./run_output_calibration.sh` and `python3 analyze_calibration.py`.

## Commit history (relevant)

- `b743f8e` — initial PX4_Gazebo/ creation + PLASMC port
- `8f2d0a1` — CLAUDE.md + SITL launcher + .gitignore
- (Pending) — image bumped to 640×480, sensor-cal analysis tools, pose validator
