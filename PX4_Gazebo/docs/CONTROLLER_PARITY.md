# PLASMC Controller: MATLAB ↔ Python Implementation Diff

Reference files:
- **MATLAB (canonical):** `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m` (+ `Common/kappa_Solver.m`, `kappa_a_Solver.m`, `Constants.m`, `Multi_init_cond/InitVar.m`)
- **Python (PX4 port):** `PX4_Gazebo/src/controller.py` (+ `apps/landing_test.py`, `src/img_data.py`)

Compiled 2026-06-02 at the start of the post-calibration gain-tuning campaign.

> 📌 **Addendum (current as of 2026-06-10):** the Python port has since **intentionally diverged** from
> the MATLAB canonical on several axes (all post-2026-06-04, deliberate SITL adaptations — not parity bugs):
> `CTRL_ZERO_WXY=1` (zeros the uncalibrated wx/wy flow feedforward), yaw **2π-wrap + conditional ie_a
> integration** (MATLAB has neither, and it spawns square), **K_rd=0** (MATLAB 1.4375), **gamma_s=1.0**
> outer funnel, **KAPPA0_Z=1.0 + KAPPA_MAX_Z=3.0 + κ-freeze on containment axes**, and **cbf2** visibility
> CBF (replaces the cone clamp). The §1 control-law math ports below remain valid; the *defaults* differ.
> See `test_data/Landing_Test/parameter_record.ods` (PX4_NewCal_Record) for the why behind each.

---

## 1. Control-law parity — what is mathematically IDENTICAL

Verified term-by-term; these blocks are exact ports:

| Block | MATLAB | Python | Status |
|---|---|---|---|
| Outer PID | `V_ds_d_xy = -rp*V_s_e_n - ri*iV_s_e_n - rd*dV_s_e_n` (l.366) | `_updateImgFeatureParam()` l.506-508 | ✅ identical form |
| Error normalization | `V_s_e ./ p_10`, p_10=[0.889,1.185] | `s_e[:2] / _p_10`, same values (240/270, 320/270) | ✅ identical |
| Desired flow | `V_h_d = V_ds_d + cross(V_w,V_s) + (h_rd − dot(cross(V_w,V_s),e3))·V_s` (l.369) | `_updateOptFlow()` l.537-541 | ✅ identical |
| Funnel envelope | `p_2 = expm(−Ξ·t)(p_20−p_2inf) + p_2inf` (l.377) | `_updatePerfFunc()` l.456-457 | ✅ identical |
| Log-barrier | `S=h_e/p` clamped ±(1−0.05), `ζ=log((1+S)/(1−S))`, `G=(e^ζ+1)²/(2e^ζ p)` (l.380-386) | l.547-555, same S_MARGIN=0.05 | ✅ identical |
| Sliding variable | `σ = ζ + Ω·∫ζ`, ∫ζ clamped ±5 per-component (l.393-397) | l.589, 601, same clamp | ✅ identical |
| c-term | `c = cross(V_dw,s) + cross(w,cross(w,s)) + 2cross(w,h) − dot(h+cross(w,s),e3)·h − V_dh_d` (l.407) | `PLASMC()` l.607-611 | ✅ identical |
| Θ & κ-ODE | `Θ=[−c+S·dp−G\(Ω ζ), I]`, `dκ/dt = ‖Θ‖·N·G·|σ| − N·P·κ`, RK5 | l.615-627, `_kappaSolver()` | ✅ identical |
| Control output | `u_sw+u_eq → V_a_cd = −G\(…)` | `a_v` / `a_u` l.635-641 (algebraically equal) | ✅ identical |
| FoV cone clamp | tilt + atan(d_min/f) capped at θ_cap, applied to I_a (l.443-464) | `_attCtrl()` l.728-775, incl. `I_a[2]≥0 → −3.0` rule | ✅ identical |
| I_a low-pass | `τ_ia = 0.08 s` first-order (l.131, 476) | l.254, 784-788 | ✅ identical |
| Yaw ASMC | `e_a` wrapped to ±π/2, `σ_a = e_a + Ω_a∫e_a`, κ_a-ODE, `u_a = Γ_a σ_a + sat(σ_a/E_a)κ_a + Ω_a e_a` (l.482-495) | `_yawCtrl()` l.658-686 | ✅ identical |
| Virtual compass | `ψ_d ← wrap(ψ_d + u_a·dt)`, init from body yaw (l.127, 502) | l.700-704, lazy init l.816 | ✅ identical |
| R_d construction | `rd3=−I_F/|I_F|`, `a_h=[cosψ_d,sinψ_d,0]`, Gram-Schmidt (l.508-523) | l.824-838 | ✅ identical |
| SO(3) error | `e_R = ½ vee(R_dᵀR − RᵀR_d)` (l.531-532) | l.842-843 | ✅ identical |

## 2. Architectural differences (forced by the platform)

These are not "errors" — they are what the PX4/Gazebo platform requires.

| # | Aspect | MATLAB | Python / PX4 | Reason |
|---|---|---|---|---|
| A1 | **Inner attitude loop** | Full SO(3) torque law: `τ = −k_R e_R − k_Ω e_Ω + ω×Jω`, thrust+torque → motor mixer → own dynamics | Body-**rate** setpoint `w_u = −K_R·e_R` shipped over MAVSDK; PX4's onboard rate controller supplies the damping (−k_Ω e_Ω), gyroscopic FF, and actuator allocation | We keep the rate-mode MAVSDK interface (thrust+torque refactor rejected — `feedback_thrust_torque`). K_R changes meaning: torque/rad → (rad/s)/rad |
| A2 | **Plant** | 13-state RK5 model + ground effect + 1-step actuator delay + wind/colored noise | Real Gazebo physics + PX4 EKF/mixer | Phase-2 of the project |
| A3 | **Perception** | Synthetic pinhole projection of 4 known corners + `pinv(L_s)` + depth-dependent pixel noise | ArUco board + LK optical flow + 8×6 LSTSQ + sensor-cal **M** + KF/savgol (`img_data.py`) | Real camera; needs detection, tracking and calibration |
| A4 | **Timing** | Fixed dt=10 ms, image ZOH every 3 steps (30 Hz cam / 100 Hz ctrl), zero latency | Async ~50 Hz control thread, ~60 Hz camera, variable dt, **~38 ms rate-loop lag + image pipeline lag** | SITL reality. The lag is the dominant MATLAB↔PX4 performance gap (see memory `feedback_impulse_response`, `phase5`) |
| A5 | **Thrust interface** | `T_cd = ‖m·I_a_filt‖` Newtons → mixer, saturated [0, 60] N | `B_T = m(I_a[2]+g)/cosφcosθ` (thrust *deficit* in N) → `thrust_norm = 0.738 − B_T/42.3` ∈ [0,1] | MAVSDK takes normalized throttle; slope 1/42.3 calibrated 2026-06-01 from input-cal |
| A6 | **Termination** | `alt_above ≤ zf=0.2 m` → stop sim | PX4 `LandedState` / impact-spike detection; then classification | **These are the SAME event**: zf = landing-gear height, so MATLAB's 0.2 m termination = gear contact = PX4's LandedState. The controller controls through touchdown on both platforms (clarified 2026-06-03) |
| A7 | **Run start** | Clean IC, controller active from t=0, V_h_d[0] already ≈ reference | Arm → takeoff → fly to IC → settle gate → 100 ms warmup → engage | SITL must reach the IC physically; gives rise to startup transients MATLAB never sees |

## 3. Deliberate parameter differences (and why)

| Parameter | MATLAB | Python default | Why different |
|---|---|---|---|
| `K_ri` | diag(0.1, 0.1) | diag(1.0, 1.0) — **10×** | SITL LK-centroid noise makes lateral PID conditionally unstable; integral nulls persistent drift |
| `K_rp` | diag(9, 9) fixed | **Gain-scheduled**: far=9 / close=4, tanh blend on \|s_e_n\| (thresh 0.1) | K_rp=4 gave softness at IC1 but regressed IC2-5 (insufficient authority); schedule keeps both |
| `N` | diag(0.02, 0.02, **0.05**) | diag(0.02, 0.02, **0.02**) | N_z slowed for SITL (κ_z adaptation too fast on noisy flow) |
| `P` | diag(1.5, 1.5, **5.0**) | diag(1.5, 1.5, **2.5**) | P_z×0.5 was the only reproducible above-baseline singleton in the Big Sensitivity sweep |
| `kappa_0` | [0.125, 0.125, 0.25] | **1.25×** that | Best singleton from IC1 sweep |
| `K_R` (inner) | diag(1.5, 1.5, 0.5) [N·m/rad] | 0.4 × diag(5,5,5) = diag(2,2,2) [(rad/s)/rad] | Different physical unit (A1). 0.4 scale found 2026-05-21: higher overdrives LK tracking |
| `k_Ω` | diag(0.3, 0.3, 0.1) | — (inside PX4) | A1: PX4 rate loop provides damping |
| `rho_fov_0` | [145, 105] px | [290, 210] px | Camera is 640×480 @ f=270 vs MATLAB 320×240 @ f=135 — same hfov, 2× pixels |
| `rho_fov_inf` | [40, 40] px | [80, 80] px | Same 2× pixel scaling |
| `h_rd` | −0.42 | −0.42 default (−0.70 used for IC1-only tuning) | −0.70 wins on IC1 but regresses IC2-5 |
| `g` | 9.81 | 9.80 | Gazebo world gravity is 9.8 |
| Savgol filter | window 11, order 2, on V_s/V_h/V_w/V_dw | `img_data.py` runtime savgol WIN=13 (or 7), order 1, sliding-window | Runtime must be causal; long windows lag the live signal out of phase |
| `tau_ia` | 0.08 s | 0.08 s | same |
| FILTER on ω | smooth4 on dω | smooth4 + ±5 rad/s clamp (`W_I_MAX`) | LK noise can spike \|w_i\|>10 rad/s — physically impossible |

## 4. Python-only protections (no MATLAB equivalent)

Added because SITL has noise/lag/startup transients MATLAB doesn't have:

| Protection | Where | Value | Purpose |
|---|---|---|---|
| `DH_D_MAX` clamp on dh_d | controller.py:570 | ±50 m/s³ (env `PLASMC_DH_D_MAX`) | Block startup dh_d spike (~160) from c-term blow-up. **2026-06-02 finding: 50 is too loose — this clamp value feeds the κ-runaway at touchdown (see §6)** |
| `W_I_MAX` clamp on w_i | controller.py:412 | ±5 rad/s | LK noise spikes |
| `W_U_MAX` clamp on body-rate cmd | controller.py:860 | ±1.0 rad/s (env) | Protect LK tracking window (~15 px/frame) |
| `DSD_CLAMP` on ds_d | controller.py:511 | off (env `PLASMC_DSD_CLAMP`) | Touchdown derivative-kick guard (added 2026-06-02) |
| `tau_ua` LPF on yaw rate | controller.py:691 | 0.1 s | α wraps at ±π/2; pixel noise near boundary flips e_a by π |
| `is_e_n` anti-windup | controller.py:481 | norm ≤ 5.0 | MATLAB has **no** clamp on iV_s_e_n (its noise never winds it up) |
| K_rp gain scheduling | controller.py:495-503 | tanh blend | IC1 vs IC2-5 trade-off |
| `CTRL_ZERO_WXY` | controller.py:422 | env, **=1 in current baseline** | Zeros w_x, w_y feeding cross(w,s): board-derived w_xy is height-corrupted + overdrives h_d |
| KLT marker fallback | img_data.py | 20 frames | Bridge ArUco detection outages |
| Stale-feature gate | img_data.py / landing_test.py | 3 frames | Don't act on extrapolated features |
| Marker-loss grace | landing_test.py:310 | 1.0 s | Brief dropouts shouldn't commit to open-loop descent |
| Removed: \|a_u\|>100 abort | (MATLAB l.470 breaks the run) | — | PX4 saturates internally; keeping controller alive lets SMC recover |

## 5. Known parity deviations (open items)

| # | Deviation | MATLAB | Python | Impact |
|---|---|---|---|---|
| D1 | **V→inertial accel transform** | `I_a_cd = I_R_V · V_a_cd − g` where `I_R_V = rotz(yaw)` (yaw-only) | `I_a_raw = R @ a_u − [0,0,g]` where R = **full body DCM** (controller.py:722) | a_u is a V-frame (level) vector — rotating it by the full DCM mis-rotates it by the current tilt. Zero when level; ~17% cross-axis error at 10° tilt. Compounds with lag during aggressive maneuvers. **Candidate fix: use rotz(yaw) only.** |
| D2 | **MATLAB filters V_s/V_h/V_w/V_dw all at the controller**; Python filters s and [h;w] inside img_data (KF or savgol) and derives dw via smooth4 in the controller | sgolayfilt(2,11) | savgol(13 or 7, 1) + KF | Different noise/lag distribution; covered by output-calibration |
| D3 | **is_e_n anti-windup** | none | norm ≤ 5.0 | Python-only; benign (protects against SITL drift) |
| D4 | dω derivation | `(V_w_i − V_w_i_prev)/dt/ZOH` at image rate, then smooth4 | `(w_i[k]−w_i[k−1])/dt` at control rate, then smooth4 | Python differentiates at ~50 Hz with variable dt → noisier dω; mitigated by W_I_MAX |
| D5 | Image-rate vs control-rate decoupling | ZOH explicitly holds image data 3 ticks | Python thread free-runs; same image sample may be re-used for several control ticks with new dt | Effective derivative noise differs |

## 6. The post-calibration gain mismatch (why this tuning campaign exists)

The 2026-06-02 multisine sensor cal (cal of record) changed the controller's *input scaling*:

| Input | May-12 cal (gains tuned against this) | Multisine M (now) | Ratio |
|---|---|---|---|
| `s` x/y (`_sensor_cal_s`) | 0.5814 / 0.5809 | 1.0986 / 1.0562 | **×1.89 / ×1.82** |
| `h` z (`_sensor_cal_hw` diag) | 0.0651 | 0.8583 | ×13 (pipeline also changed: single-marker → board) |

Consequences, confirmed by the explosion-chain analysis (n=6 defaults reps, 2026-06-02):

1. `s_e_n` and its derivative are ~1.85× larger than the values K_rp/K_ri/K_rd were tuned for → outer-PID output `ds_d` is 1.85× hotter.
2. Near touchdown, 1/Z geometry amplifies any residual lateral error → `ds_d` spikes (13–245 observed vs MATLAB ~2).
3. `dh_d` (smoothed d/dt of desired flow) pins at its ±50 clamp → feeds the SMC c-term directly **and** sets Θ_norm ≈ 50.
4. κ-ODE growth `dκ/dt = Θ·N·G·|σ|` with Θ≈50, G≈8 (barrier saturated), |σ|≈3.7 → κ runs to 10–100× κ_0 (decay time constant 1/(N·P) = 33 s — never recovers in-flight).
5. `a_u = c + Θ·sat(σ/E)·κ + …` → 400–7000 m/s² → hard impact / drift / TARGET_LOST.

The chain starts at the **outer PID** (cal-amplified) and is catastrophically amplified by the **κ-adaptation** (via the dh_d clamp). Gain relaxations under test (see `test_data/Landing_Test/parameter_record.ods → PX4_Gain_Tuning` sheet).

---

## 7. The definitive parameter taxonomy (2026-06-03 cleanup)

What is tunable, what is fixed, and why PX4 has anything beyond the manuscript set.

### 7.1 The real control parameters — 49 scalars, identical across manuscript / MATLAB / Python

| Group | Manuscript | MATLAB | Python (direct per-axis env knobs) | # |
|---|---|---|---|---|
| Image PID | K_rp, K_ri, K_rd | `K_ctrl.rp/ri/rd` | `PLASMC_KP/KI/KD_{X,Y}` | 6 |
| Descent ref | h_rd | `h_rd` | `LANDING_REF_RAD_OPT_FLOW` | 1 |
| Funnel | Ξ₂, p₂₀, p₂∞ | `gamma_2, p_20, p_2inf` | `PLASMC_XI2/P20/P2INF_{X,Y,Z}` | 9 |
| Optic-flow ASMC | 𝒳, Γ, 𝒫, 𝒩, κ(0), ℰ | `Omega, Gamma, P, N, kappa_0, E` | `PLASMC_OMEGA/GAMMA/P/N/KAPPA0/E_{X,Y,Z}` | 18 |
| Accel conditioning | p₁₀, p₁∞, ξ₁, θ_cap | `rho_fov_0/inf, l_fov, theta_cap` | `PLASMC_RHOFOV0/RHOFOVINF_{U,V}, LFOV, THETACAP_DEG` | 6 |
| Yaw ASMC | χ_α, γ_α, η_α, ρ_α, κ_α(0), ε_α | `Omega_a..E_a` | `PLASMC_YAW_OMEGA/GAMMA/N/P/KAPPA0/E` | 6 |
| SO(3) | k_R (k_Ω) | `kR (kOmega)` | `PLASMC_KR_{ROLL,PITCH,YAW}` (k_Ω → PX4 rate loop) | 3 |

Fixed design constants (all three sources, never tuned): ε_S=0.05 (Remark 5), ∫ζ clamp=5 (Suppl. S2-D),
acceleration floor 3.0 m/s² and LPF τ_a=0.08 s (Suppl. S1.5).

### 7.2 PX4-only parameters and their justification

**Category A — architecture translations** (the same MATLAB mechanism in rate-mode/async form; values set
by physics, never tuned):

| PX4 | MATLAB equivalent | Why the form differs |
|---|---|---|
| `PLASMC_W_U_MAX` = 1.0 rad/s | `tau_xy_max`/`tau_z_max` (torque saturation) | inner loop outputs body rate (A1), so the saturation is a rate; additionally protects the LK tracking window |
| `PLASMC_DH_D_MAX` = 50 | `if idx==1: raw_dh_d=0` | async engagement (A7) has no well-defined "first sample"; a high clamp kills the same transient |
| `iV_s_e_n`/`ie_a` anti-windups | `izeta_2_max` (S2-D) | the S2-D concept extended to the PID/yaw integrals, which can wind up in PX4's longer/drifting pre-engagement |

**Category B — real-perception protections** (MATLAB perception is synthetic and cannot fail):

| PX4 | Why MATLAB doesn't need it |
|---|---|
| `W_I_MAX` = 5 rad/s (w-measurement clamp) | synthetic V_w never spikes (MATLAB even defines the same bound, `w_max`, unused) |
| `PLASMC_TAU_UA` = 0.1 s (yaw cmd LPF) | synthetic α never wrap-flips; MATLAB's u_a runs unfiltered |

**Category C — the single control-law deviation:**

`CTRL_ZERO_WXY=1` zeros w_x, w_y inside cross(V_w, V_s). Valid for STATIC targets only (true V-frame
target-relative angular rate ≈ 0; the patch substitutes the known-true value for a noisy estimate with
R²≈0.49). MUST be removed for moving-target work — which requires fixing the w-estimation first.

### 7.3 Not control parameters at all

img_data.py (ArUco, KLT, savgol, stale, sensor cal, board) and landing_test.py (IC gates, grace,
stale-commitment, tolerances) parameterize perception and the experiment protocol. MATLAB needs neither
(perfect perception, ICs set by assignment). They are never part of controller tuning.
