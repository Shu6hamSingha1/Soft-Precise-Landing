# PLASMC Controller — Comprehensive Parameter Analysis & Failure Diagnosis

_Generated 2026-05-24 from 1205 SITL landing reps across 68 bundles + MATLAB Phase 1-5 results._

---

## Executive summary

Across **1205 landing attempts** in PX4/Gazebo SITL:

| Outcome | Count | Fraction |
|---|---|---|
| **SP (soft+precise)** | **1** | 0.08% |
| PRECISE-only (xy<0.08, vel>0.2) | 19 | 1.6% |
| SOFT-only (vel<0.2, xy>0.08) | 186 | 15.4% |
| near-SP (xy<0.15, vel<0.4) | 4 | 0.3% |
| MOD_PRECISE (xy<0.3) | 157 | 13.0% |
| POOR | 542 | 45.0% |
| CATASTROPHIC (xy≥1m) | 181 | 15.0% |
| TARGET_LOST | 115 | 9.5% |

**Causal chain (from analysis):**

1. **Architectural lag (MAVSDK 30 ms + PX4 8 ms ≈ 38 ms vs MATLAB ~13 ms)** is the dominant cause of mode-shifted convergence. Documented in `feedback_impulse_response.md`, `feedback_phase2_loop_latency.md`.

2. **Lag amplifies IC perturbations into σ-divergence in the lateral plane** (`feedback_phase5_sigma_divergence.md`).

3. **PRECISE-only reps lock to xy=0 but with high lateral velocity at touchdown**: the outer PID converges image error but never damps the lateral velocity that was used to chase the target. This is a **kinetic-energy issue not a position-tracking issue**.

4. **Marker-detection breakdown** at low altitude was the secondary stochastic-tail cause. Largely addressed by interventions 1+2+3 (`feedback_marker_detection_stale.md`).

5. **Sensor noise matches MATLAB** (Phase 4) — NOT a contributing factor.

---

## Parameter-by-parameter analysis

For each parameter:
- **Role**: where in the control law
- **Defaults**: current value
- **Knob**: env-override path
- **Empirical effect**: what we observed when varied
- **Suspected role in failures**: does varying this fix the dominant failure mode?

### 1. Outer-loop PID (image-error → desired-h)

The outer PID acts on normalized image error `s_e_n = (s - s_d)[:2] / p_10` and produces `ds_d`, a desired feature-derivative that becomes the optic-flow reference `h_d`.

#### `K_rp` (Proportional gain), default `diag(9, 9)`
**Role:** Drives image error to zero; the dominant "I see error → correct it" gain.
**Knob:** `PLASMC_KP_SCALE`, `PLASMC_KP_X/Y_SCALE`.
**Empirical:** Boosting (×1.5, ×2) made things worse — instability cascade. Reducing (×0.5) made convergence too slow. The 9.0 default is from MATLAB sweep.
**Role in failures:** Not a primary failure cause. Boost or cut both hurt.

#### `K_ri` (Integral gain), default `diag(1, 1)`
**Role:** Eliminates persistent offset (e.g., from sensor cal drift). 10× MATLAB's 0.1 because SITL has more sustained noise.
**Knob:** `PLASMC_KI_SCALE`, per-axis.
**Empirical:** Big sweep showed `KI_X × 1.5` gave one rep at xy=0.148 (near-SP). Boost on Y less effective. Reducing → drift accumulates.
**Role in failures:** Possible knob. KI_X boost showed promise in big sweep singletons.

#### `K_rd` (Derivative gain), default `diag(1.4375, 1.4375)`
**Role:** Damps the velocity component of image-error rate. Critical for SOFT — provides lateral-velocity braking.
**Knob:** `PLASMC_KD_SCALE`, per-axis.
**Empirical:** Sensitive to centroid noise; ↑ → chatter. Default 1.4375 is paired with K_rp=9 in MATLAB.
**Role in failures:** **Strong suspect** — if D-term effective braking is lag-degraded, lateral velocity won't damp. **NOT separately tested at large scale.**

### 2. Middle-loop SMC (PLASMC funnel + sliding)

The middle loop operates on optic-flow error `h - h_d` with a Performance-Constrained Funnel (Bechlioulis-Rovithakis style) + adaptive-gain SMC.

#### `gamma` (Ξ_2), default `diag(0.2, 0.2, 0.2)`
**Role:** Scales the sliding-variable computation: `ζ = gamma * (h - h_d) + integral term`. Higher → faster sliding-surface convergence.
**Knob:** `PLASMC_XI2_SCALE`, per-axis.
**Empirical:** Big sweep: `XI2_Y_1.5_rep1` gave a PRECISE-only (xy=0.058, but vel=1.22). Not consistently positive.
**Role in failures:** Secondary; tuning hasn't moved the needle.

#### `p_2_0` (initial funnel half-width), default `[25, 25, 4]`
**Role:** Initial bounds on `ζ`. Wide envelope → controller has authority; narrow → constrained.
**Knob:** `PLASMC_P20_SCALE`, per-axis.
**Empirical:** Big sweep: `P20_X_0.5_rep3` gave PRECISE-only at xy=0.077. P20_Z×0.5 had the best n=4 mean (0.233) but failed n=10 validation (`P20ZValidate`).
**Role in failures:** Z-axis singleton looks promising but doesn't reproduce — false positive at n=5.

#### `p_2_inf` (terminal funnel half-width), default `[2.5, 2.5, 1.5]`
**Role:** **LOAD-BEARING** per code comment. Final envelope at landing — limits how much optic-flow error is permitted at touchdown.
**Knob:** `PLASMC_P2INF_SCALE`, per-axis.
**Empirical:** Big sweep `P2INF_Y × 0.5` had mean xy=0.375 (top-10).
**Role in failures:** Possible knob for SP — tighter terminal envelope = forces lateral velocity to be smaller at touchdown. UNDERTESTED at terminal-only.

#### `Omega` (Ω), default `diag(0.05, 0.05, 0.025)`
**Role:** **LOAD-BEARING** — leakage rate on adaptive-gain κ. Controls κ-ODE convergence.
**Knob:** `PLASMC_OMEGA_SCALE`, per-axis.
**Empirical:** Most sensitive parameter per sensitivity sweep — small changes large effects.
**Role in failures:** Possibly. If κ doesn't reach the right magnitude in time, controller can't drive σ to zero.

#### `Gamma` (Γ), default `diag(0.4375, 0.5, 0.75)`
**Role:** Adaptation rate on κ-ODE. Higher → κ grows faster when σ is large.
**Knob:** `PLASMC_GAMMA_SCALE`, per-axis.
**Empirical:** Big sweep `GAMMA_X × 1.5` gave near-SP (xy=0.120, vel=0.383). Most promising of the SMC singletons.
**Role in failures:** **TOP SUSPECT for SOFT**. GAMMA_X×1.5 reduced vel from typical 1.5 to 0.38. UNDERTESTED in combos.

#### `E`, default `diag(1, 1, 1)`
**Role:** Boundary-layer thickness; sat() arg is divided by E. Larger → softer sat (less chatter).
**Knob:** `PLASMC_E_SCALE`, per-axis.
**Empirical:** Big sweep `E_X_1.5` PRECISE singleton (xy=0.038); E_Z_0.5 best softness (3/5 SOFT, but n=5).
**Role in failures:** Possible. E_X_1.5 + E_Z_0.5 combo gave one PRECISE singleton; YEEz combo at n=5 looked good but n=10 failed.

#### `N` (N̄), default `diag(0.02, 0.02, 0.02)` (z legacy 0.02 abs)
**Role:** e-modification factor in κ-ODE — drives κ → κ_0 slowly when σ is small.
**Knob:** `PLASMC_N_SCALE`, per-axis, plus `PLASMC_N_Z` legacy absolute.
**Empirical:** Big sweep `N_X × 0.5_rep4` PRECISE-only (xy=0.041 but vel=0.876).
**Role in failures:** Secondary.

#### `P`, default `diag(1.5, 1.5, 5.0)`
**Role:** Anti-windup-like gain inside κ-ODE.
**Knob:** `PLASMC_P_SCALE`, per-axis.
**Empirical:** `P_Z × 0.5` gave PRECISE singleton (xy=0.080, vel=0.596) — vertical envelope tightening. Best vel of all PRECISE-only.
**Role in failures:** Z-axis tightening helps vel modestly.

#### `kappa_0`, default `1.25 × [0.125, 0.125, 0.25]`
**Role:** Initial value for κ adaptive gain.
**Knob:** `PLASMC_KAPPA0_SCALE` (default 1.25 = best singleton from earlier sweep).
**Empirical:** Already at best-tuned value.
**Role in failures:** Tuned; doesn't move further.

### 3. Yaw SMC (κ_a)

Per supplement S3-A, the yaw channel is robust with low impact on landing outcomes.

| Param | Default | Knob | Empirical |
|---|---|---|---|
| `Omega_a` | 0.5 | `PLASMC_YAW_OMEGA_SCALE` | Big sweep YAW_OMEGA_1.5: mean 0.359 |
| `Gma_a` | 0.5 | `PLASMC_YAW_GAMMA_SCALE` | Big sweep neutral |
| `n_a` | 1.0 | `PLASMC_YAW_N_SCALE` | Big sweep YAW_N_0.5 gave xy_min=0.083 |
| `p_a` | 2.0 | `PLASMC_YAW_P_SCALE` | Neutral |
| `kappa_a_0` | 2.0 | `PLASMC_YAW_KAPPA0_SCALE` | Neutral |
| `E_a` | 3.0 | `PLASMC_YAW_E_SCALE` | Big sweep YAW_E_1.5 gave 1 PRECISE (xy=0.029, but vel=1.32) |

**Role in failures:** Low. Yaw mostly handles ψ_d tracking which isn't the bottleneck.

### 4. FoV cone-clamp (acceleration conditioning)

These prevent the SMC from demanding accelerations that would push the marker out of frame.

#### `rho_fov_0`, default `[290, 210]`
Initial pixel envelope for FoV cone. Big sweep neutral.

#### `rho_fov_inf`, default `[80, 80]`
**Role:** Terminal pixel envelope. Tightening forces lateral position to be closer at landing.
**Knob:** `PLASMC_RHOFOVINF_SCALE`.
**Empirical:** `RHOFOVINF × 0.5` gave the **CLOSEST near-SP rep ever observed**: xy=0.087, vel=0.220.

**Role in failures:** ⭐ **STRONGEST UNTESTED KNOB**. RHOFOVINF reduction may be the single most-promising lever — it directly constrains how close the drone must be at touchdown.

#### `l_fov`, default 0.1
Envelope decay rate. Untested at scale.

#### `theta_cap`, default 60°
**Role:** Soft cone ceiling — saturates desired tilt angle, preventing SMC from demanding accelerations that would tip the drone past 60°.
**Knob:** `PLASMC_THETACAP_SCALE`.
**Empirical:** Untested broadly.
**Role in failures:** If theta_cap is too tight, lateral correction is throttled → vh_end stays high. **WORTH TESTING** at 1.5× (90° effective).

### 5. Inner-loop (SO(3))

#### `K_R`, default `0.4 × diag(5, 5, 5)` (baked)
**Role:** Attitude-error proportional gain. Drives body rates to match desired attitude.
**Knob:** `PLASMC_KR_SCALE` (default 0.4), per-axis.
**Empirical:** `K_R × 0.4` is the stable sweet spot. Higher → LK breakage. Lower → slow convergence.
**Role in failures:** Tuned and load-bearing for stability.

#### `W_U_MAX` (body-rate clamp), default 1.0 rad/s
**Role:** Saturates body-rate command magnitude to prevent LK-tracking break.
**Knob:** `PLASMC_W_U_MAX`.
**Empirical:** 1.0 is the sweet spot. 1.5 reintroduces TL.
**Role in failures:** Tuned.

### 6. Anti-windup clamps (NOT env-overridable currently)

| Clamp | Default | Notes |
|---|---|---|
| `iV_s_e_n_clamp` (PID integral) | 5.0 | Per code comment — matches supplement S2-D |
| `izeta_clamp` (sliding integral) | 5.0 | Same |
| `ie_a_clamp` (yaw integral) | 2.0 | Yaw-specific |

**Role in failures:** Possibly limiting integral action. If the PID can't reach high enough integral to overcome steady drift, the system runs with persistent error.

**Status:** UNTESTED. Should probably be made env-overridable for sensitivity-sweep.

### 7. Misc control-loop

#### `tau_ia` (LPF on inertial accel command), default 0.08 s
**Role:** Low-pass filter on `I_a_cd` (commanded inertial accel) before R_d construction. Trades off lag vs noise rejection.
**Knob:** **NOT env-overridable**.
**Empirical:** Tested 0.04 and 0.20; both worse (per `feedback_precision_tuning_lessons.md`).
**Role in failures:** Tuned at the safe sweet spot.

### 8. Image pipeline (img_data.py)

| Param | Default | Knob | Empirical |
|---|---|---|---|
| `FILTER_WIN` (savgol) | 13 default, **user uses 7** | `IMG_FILTER_WIN` | 7 is best. 5 caused TL previously — interventions may now absorb it. |
| `FILTER_POLYORDER` | 1 | `IMG_FILTER_POLY` | Not swept |
| `STALE_THRESH` | 3 | `IMG_STALE_THRESH` | Intervention 2 default |
| `adaptiveThreshConstant` | 5.0 (intervention) | `ARUCO_ADAPT_THRESH_C` | Defaulted by intervention 1 |
| `errorCorrectionRate` | 0.8 | `ARUCO_ERR_CORRECT` | Same |
| `minMarkerPerimeterRate` | 0.02 | `ARUCO_MIN_PERIM_RATE` | Same |
| `minOtsuStdDev` | 3.0 | `ARUCO_MIN_OTSU_STD` | Same |

### 9. Landing-test (landing_test.py)

| Param | Default | Knob | Empirical |
|---|---|---|---|
| `REF_RAD_OPT_FLOW` (h_rd) | -0.42 (MATLAB), -0.70 (user) | `LANDING_REF_RAD_OPT_FLOW` | -0.70 best for IC1; -0.30 caused 40% TL; -0.85 untested |
| `MARKER_LOSS_GRACE` | 1.0 s | `LANDING_MARKER_LOSS_GRACE` | Tuned |
| `IC_VEL_TOL` | 0.5 m/s | `LANDING_IC_VEL_TOL` | Tighter helps σ0 but moves variance to vh0 |
| `IC_TILT_TOL_DEG` | 3.0° | `LANDING_IC_TILT_TOL_DEG` | Same |
| `IC_STABLE_HITS` | 20 | `LANDING_IC_STABLE_HITS` | More=longer hold |

---

## Cross-rep diagnostic findings

### What separates the 1 SP rep from the 19 PRECISE-only reps?

| Variable | SP rep (rep2) | PRECISE-only mean | Difference |
|---|---|---|---|
| **vh_end** (lateral velocity at touchdown) | **0.029 m/s** | **1.20 m/s** | ⭐ **40× lower** |
| vz_end (descent velocity at touchdown) | 0.128 m/s | 0.21 m/s | comparable |
| sigma_xy_tail | 0.30 | 0.42 | similar |
| h_z_tail (commanded descent rate) | -0.70 | -0.65 | similar |
| B_T (thrust) tail | 8.75 | 7.5 | similar |
| I_a_z (commanded vertical accel) | -5.67 | -6.0 | similar |

**Single distinguishing factor: lateral velocity at touchdown.** All PRECISE-only reps have the drone hitting the target while still moving sideways at 1+ m/s. The SP rep had the drone already-static when it touched down.

### Hypothesis on why SP rep worked

1. **Low IC vh0** (0.046 vs typical 0.14 m/s) → less lateral kinetic energy to dissipate
2. **Low sigma_xy0** (0.016) → controller starts close to sliding surface
3. **Marker detection robust** (intervention 1+2+3) → controller has fresh data throughout
4. **No need for active braking** — there was barely any lateral motion to brake

The current architecture is capable of HOLDING zero lateral velocity. It is NOT capable of actively BRAKING significant lateral velocity (>0.2 m/s) in the final phase.

### Why does the architecture fail to brake lateral velocity?

The PID's `K_rd` (derivative) provides braking. But:
1. With ~38 ms loop lag, the D-term acts on stale velocity estimates
2. The image-pixel error is what the PID sees — when image error is small, the D-term magnitude is small too
3. The SMC middle loop doesn't have explicit velocity-tracking — it tracks optic-flow rate `h`, which is proportional to (velocity / altitude). At low altitude, optic-flow magnitude is high for given velocity, so the SMC SHOULD see this — but the funnel envelope `p_2_inf` may not be tight enough to FORCE convergence.

**This is why RHOFOVINF reduction looked so promising in the big sweep** — it tightens the terminal envelope, forcing the controller to converge lateral position more aggressively. The `RHOFOVINF × 0.5` rep had **the lowest lateral velocity** of any PRECISE-or-near-SP rep.

---

## Recommended next experiments (ranked by expected SP-rate impact)

1. **⭐ `RHOFOVINF × 0.5` (terminal pixel envelope [80,80] → [40,40])** combined with all current interventions, n=10 at IC1. The big-sweep singleton (n=1) gave near-SP outcome. If this reproduces at n=10, we have a real lever.

2. **`p_2_inf × 0.5`** (terminal optic-flow envelope). Same logic as RHOFOVINF but at the SMC level instead of the cone clamp.

3. **`THETACAP × 1.5`** (relax tilt saturation from 60° → 90°). Lets the SMC demand more lateral acceleration in the final phase, providing more braking authority.

4. **`Gamma_X × 1.5`** singleton at n=10 with interventions. The big sweep showed it gave near-SP at n=1.

5. **`K_rd_X/Y × 1.25`** — boost lateral D-term gain to brake harder. Untested at fine granularity.

6. **Slower h_rd in terminal phase** — alternative implementation: piecewise h_rd, start at -0.70 then ramp to -0.30 below z=1m to give controller more time at low altitude.

---

## Data sources

- `~/Soft-Precise-Landing/PX4_Gazebo/test_data/` — 68 bundles, 1205 reps
- `/tmp/all_landings.json` — full classification table from `scan_all_landings.py`
- Memory: every `feedback_*.md` under `~/.claude/projects/-home-shubham-Soft-Precise-Landing/memory/`
- `controller.py`, `img_data.py`, `landing_test.py` — current source of truth
