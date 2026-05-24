---
name: tune-plasmc
description: Systematic parameter tuning + failure diagnosis for the PLASMC vision-based landing controller on PX4/Gazebo SITL. Use when the user asks to tune the controller, diagnose why a landing failed, sweep a gain, or understand parameter sensitivity. Encodes the complete parameter inventory, the n≥5 sweep methodology, known-bad and known-good configs, and the diagnostic chain that traces a failed rep back to root cause.
---

# Tune PLASMC

A systematic procedure for tuning the PLASMC controller in `PX4_Gazebo/` and for diagnosing why a given landing failed. Read this end-to-end before any tuning sweep. The memory under `~/.claude/projects/-home-shubham-Soft-Precise-Landing/memory/` has the historical findings — this skill is the **playbook** that ties them together.

## When to invoke

- "Tune the controller", "tune gain X", "sweep parameter Y"
- "Why did the landing fail / why is the controller diverging"
- "Should I change this parameter"
- "Run a sensitivity sweep" / "what parameters haven't we tried"
- Any time the user mentions a PLASMC gain by name (K_R, p_2_0, kappa, Omega, Gamma, E, N, etc.)

## Mental model — the seven failure modes

Every PLASMC landing in this SITL setup fails for exactly one of these reasons. Before tuning, identify which is happening.

| # | Failure mode | Diagnostic signal | Fix |
|---|---|---|---|
| **1** | **MAVSDK rate-loop lag** (architectural ceiling) | xy_mean stuck ~0.3-0.5 m across many gain configs; impulse-test t_d > 20 ms; PX4/MATLAB σ_xy ratio > 3× | Loop-lag reduction (see "Levers within MAVSDK" below). Not a tuning fix. |
| **2** | **Marker-detection breakdown** at z<0.3m | `Image Feature Pts` frozen for ≥100ms in final descent; sides stay <100 px through touchdown | Interventions 1+2+3 (committed 8baea2b). Already in default config. |
| **3** | **IC sensitivity amplified by lag** | vh0 > 0.2 m/s → σ_xy diverges; PX4 σ_trajectory deviates from MATLAB by 4× in σ_xy only | Tightening IC tol alone doesn't help (validated). Need lag reduction (mode 1). |
| **4** | **Kr too aggressive** | Peak body rates >1.7 rad/s → LK loses corners → TARGET_LOST | `PLASMC_KR_SCALE=0.4` (current default) + `PLASMC_W_U_MAX=1.0` |
| **5** | **PID-output saturation** | a_u peaks > 100 m/s² before LPF; tau_ia can't smooth | tau_ia ∈ [0.08, 0.20] is the safe range. Higher = lag, lower = noise. |
| **6** | **Funnel-envelope breach** | rho_fov_log near zero, theta_current near theta_cone | Widen `RHOFOV0` or relax `THETACAP`. Very rare in current config. |
| **7** | **Sensor cal drift** | Optic-flow lstsq returns clipped or non-finite | Don't refresh sensor cal — methodology mismatch in `aggregate_calibration.py` produces 7-10× wrong gains (see `feedback_calibration_lessons.md`). Sensor noise already matches MATLAB per Phase 4. |

If the failure is mode 1 → no controller-side tuning fixes it. Mode 2 → already fixed by defaults. Mode 4-7 → tuneable. Mode 3 → secondary to mode 1.

## Complete parameter inventory

### Outer loop (image-error → desired optic-flow rate)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `K_rp` (P gain) | `diag(9, 9)` | `PLASMC_KP_SCALE`, `PLASMC_KP_X/Y_SCALE` | Boost >1.5× → instability cascade |
| `K_ri` (I gain) | `diag(1, 1)` | `PLASMC_KI_SCALE`, `PLASMC_KI_X/Y_SCALE` | 10× MATLAB's 0.1; needed for SITL drift correction |
| `K_rd` (D gain) | `diag(1.4375, 1.4375)` | `PLASMC_KD_SCALE`, `PLASMC_KD_X/Y_SCALE` | Sensitive to centroid noise; ↑ → chatter |
| `PID_SCALE` uniform | 1.0 | `PLASMC_PID_SCALE` | Legacy uniform scaler |
| `DH_D_MAX` | 50.0 m/s³ | `PLASMC_DH_D_MAX` | Clamp on h_d derivative |

### Middle loop (PLASMC funnel + sliding)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `gamma` (Ξ_2) | `diag(0.2, 0.2, 0.2)` | `PLASMC_XI2_SCALE`, per-axis `_X/_Y/_Z` | Sliding-variable gain |
| `p_2_0` | `[25, 25, 4]` | `PLASMC_P20_SCALE`, per-axis | Initial half-width of funnel |
| `p_2_inf` | `[2.5, 2.5, 1.5]` | `PLASMC_P2INF_SCALE`, per-axis | Terminal half-width — LOAD-BEARING |
| `Omega` (Ω) | `diag(0.05, 0.05, 0.025)` | `PLASMC_OMEGA_SCALE`, per-axis | κ-leakage rate. LOAD-BEARING (controls SP rate) |
| `Gamma` (Γ) | `diag(0.4375, 0.5, 0.75)` | `PLASMC_GAMMA_SCALE`, per-axis | κ-adaptation rate |
| `E` | `diag(1, 1, 1)` | `PLASMC_E_SCALE`, per-axis | Boundary-layer thickness; sat() arg scaled by 1/E |
| `N` | `diag(0.02, 0.02, 0.02)` | `PLASMC_N_SCALE` + `PLASMC_N_Z` (legacy abs) | Drives e-modification term in κ-ODE |
| `P` | `diag(1.5, 1.5, 5.0)` | `PLASMC_P_SCALE`, per-axis | Anti-windup-like gain on κ-ODE |
| `kappa_0` (init κ) | `1.25 × [0.125, 0.125, 0.25]` | `PLASMC_KAPPA0_SCALE` (default 1.25) | Already at best singleton (big sweep) |

### Yaw SMC (low-impact per supplement S3-A)
| Param | Default | Env knob |
|---|---|---|
| `Omega_a` | 0.5 | `PLASMC_YAW_OMEGA_SCALE` |
| `Gma_a` | 0.5 | `PLASMC_YAW_GAMMA_SCALE` |
| `n_a` | 1.0 | `PLASMC_YAW_N_SCALE` |
| `p_a` | 2.0 | `PLASMC_YAW_P_SCALE` |
| `kappa_a_0` | 2.0 | `PLASMC_YAW_KAPPA0_SCALE` |
| `E_a` | 3.0 | `PLASMC_YAW_E_SCALE` |

### FoV-margin cone clamp
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `rho_fov_0` | `[290, 210]` | `PLASMC_RHOFOV0_SCALE` | Initial pixel envelope |
| `rho_fov_inf` | `[80, 80]` | `PLASMC_RHOFOVINF_SCALE` | Terminal pixel envelope |
| `l_fov` | 0.1 | `PLASMC_LFOV_SCALE` | Envelope decay rate (1/s) |
| `theta_cap` | 60° | `PLASMC_THETACAP_SCALE` | Soft cone ceiling — acceleration saturation |

### Inner loop (SO(3))
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `K_R` | `0.4 × diag(5, 5, 5) = diag(2, 2, 2)` | `PLASMC_KR_SCALE` (default 0.4); per-axis `PLASMC_KR_{ROLL,PITCH,YAW}_SCALE` | 0.4 is the stable sweet spot (committed 2026-05-21). Higher → LK breakage |
| `W_U_MAX` (body-rate clamp) | 1.0 rad/s | `PLASMC_W_U_MAX` | Caps body-rate command magnitude |

### Misc / control-loop
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `tau_ia` (LPF on I_a_cd) | 0.08 s | **not env-overridable** | Tested 0.04, 0.20 — both worse |
| `iV_s_e_n_clamp` | 5.0 | **not env-overridable** | Anti-windup on integral of normalized error |
| `izeta_clamp` | 5.0 | **not env-overridable** | Anti-windup on integrated sliding variable |
| `ie_a_clamp` | 2.0 | **not env-overridable** | Anti-windup on integrated yaw error |

### Image pipeline (img_data.py)
| Param | Default | Env knob |
|---|---|---|
| `FILTER_WIN` (savgol window) | 13 → user uses 7 | `IMG_FILTER_WIN` |
| `FILTER_POLYORDER` | 1 | `IMG_FILTER_POLY` |
| `STALE_THRESH` (intervention 2) | 3 frames | `IMG_STALE_THRESH` |
| `adaptiveThreshConstant` | 5.0 (intervention 1) | `ARUCO_ADAPT_THRESH_C` |
| `errorCorrectionRate` | 0.8 (intervention 1) | `ARUCO_ERR_CORRECT` |
| `minMarkerPerimeterRate` | 0.02 (intervention 1) | `ARUCO_MIN_PERIM_RATE` |
| `minOtsuStdDev` | 3.0 (intervention 1) | `ARUCO_MIN_OTSU_STD` |

### Landing-test (landing_test.py)
| Param | Default | Env knob |
|---|---|---|
| `REF_RAD_OPT_FLOW` (h_rd) | -0.42 (MATLAB), user uses -0.70 | `LANDING_REF_RAD_OPT_FLOW` |
| `MARKER_LOSS_GRACE` | 1.0 s | `LANDING_MARKER_LOSS_GRACE` |
| `IC_POS_TOL`, `IC_VEL_TOL`, `IC_TILT_TOL_DEG`, `IC_STABLE_HITS`, `IC_BUDGET_S` | 0.5 m / 0.5 m/s / 3° / 20 / 30s | `LANDING_IC_*` |
| `PX4_RATE_SCALE` | 1.0 (DEAD — see `feedback_mc_rate_p_dead.md`) | `PLASMC_PX4_RATE_SCALE` |

## Tuning methodology (RULES — violate at your peril)

1. **n ≥ 5 reps per cell.** Single runs are noise. The "winners" from n=1 sweeps reproducibly fail to replicate at n=10 (see `feedback_sensitivity_sweep_methodology.md`).
2. **IC2-5 validation is mandatory before defaulting.** IC1 improvements consistently regress off-center starts. `run_ic_validation.sh` exists.
3. **Validate combos at n ≥ 10.** Phase 3 found a "PRECISE singleton at n=5 (YEEz combo)" that gave 0/10 at n=10 — a fluke. Stacked-singleton optimism is the most common false positive.
4. **Trust direction-of-effect, not specific xy values.** Per-rep variance is ±0.2 m. A "winner at xy_mean=0.30 vs baseline 0.48" is only believable across n≥10.
5. **Stop sweeping after diminishing returns.** Memory says ~620 SITL runs were spent gain-sweeping before Phase 1 revealed the controller works in MATLAB. Lag is the architectural ceiling.

## Levers within MAVSDK architecture (sorted by remaining headroom)

The user has ruled out uXRCE-DDS migration unless it's the only option. So the levers within MAVSDK are:

| Lever | Expected lag reduction | Cost | Status |
|---|---|---|---|
| **A. MC_*RATE_P at post-takeoff** | ~10 ms (pitch) | 1-line move of the param-set call | Untried — original attempt failed at startup-time |
| **B. Savgol WIN=7 → WIN=5** | ~12 ms (image-side) | 1 env var | WIN=5 previously caused TARGET_LOST; **interventions might absorb now** |
| **C. Smith predictor in Python outer loop** | 20-30 ms (theoretical) | Few hundred LOC | Software-only model-based compensation |
| **E. Airframe-init MC_*RATE_P** | Same ~10 ms as A | Edit `~/PX4-Autopilot/...4014_*` | Outside-repo; bypasses MAVSDK timing |
| **F. PX4 MAV_*RATE params** | Possibly 5-15 ms | env knob via MAVSDK | Untried |

Lever D (image-based velocity feedforward) is **already implemented** via the optic-flow middle loop — the SMC middle loop uses optic-flow centroid Δ directly as feedforward.

## Diagnostic procedure — "why did this rep fail"

For a single failed rep:
1. Load with `analyze_timeseries.py <rep_dir>` (per-channel correlation with xy_end)
2. Look at:
   - `|σ|_tail` ρ vs xy_end — if high (>0.7) the controller didn't reach sliding surface
   - `|s_e_n|_tail` — if high, controller didn't drive image error to zero
   - `|w_u|_max` — if > 1.5 rad/s, LK tracking probably broke
   - Frozen `Image Feature Pts` in final 100ms → marker detection breakdown (mode 2)

For a comparison across reps:
- `diagnose_intervention_reps.py <bundle>` — produces per-rep summary table + correlations
- Look at `side_max_d`, `stale_pct_descent`, `switches`, `sigma_xy0` — the strongest correlates of SP outcomes

For lag characterization:
- `analyze_loop_latency.py` — cross-correlation; noisy
- `run_impulse_response.sh` + `analyze_impulse_response.py` — cleaner, gives per-axis τ + deadtime

For sensor-noise budget:
- `analyze_sensor_noise.py` — frame-to-frame Δ methodology; absolute std is motion-contaminated

For MATLAB-vs-PX4 comparison:
- `MATLAB/Multi_init_cond/phase1_baseline_sweep.m` (MATLAB controller alone)
- `MATLAB/Multi_init_cond/capture_sigma_trace.m` (matched h_rd σ trace)
- `PX4_Gazebo/analyze_sigma_compare.py` (compares MATLAB vs PX4 σ trajectories)

## Known dead-ends (do NOT retry without new evidence)

- `K_rp ≥ 13.5` or `K_ri ≥ 1.5` boost → instability cascade
- `K_R ≥ 0.5` without W_U_MAX clamp → LK breakage
- `tau_ia` not in [0.08, 0.20] → either lag or noise
- `LANDING_REF_RAD_OPT_FLOW < -0.7` (e.g., -0.30) → 40% TARGET_LOST
- Tight IC (vel_tol=0.05) → 33% TL rate, no xy benefit
- Stacking singletons from sensitivity sweeps → cancels benefits 90% of the time
- Sensor cal refresh via `aggregate_calibration.py` → 7-10× wrong values
- `MC_*RATE_P > 1.0` via MAVSDK runtime → SITL preflight failure (mode-1 lever still untried at post-takeoff)
- Enlarging small marker → violates the at-touchdown FoV match constraint

## Known winners (current defaults)

```
PLASMC_KR_SCALE=0.4         (baked into controller.py:221)
PLASMC_W_U_MAX=1.0          (default in controller.py:766)
LANDING_REF_RAD_OPT_FLOW=-0.70  (env-only — regresses IC2-5)
IMG_FILTER_WIN=7            (env-only — best of [5, 7, 13] for runtime delay)
ARUCO_*                     (interventions, env-overridable, sane defaults baked in)
IMG_STALE_THRESH=3          (intervention 2)
```

Best-known IC1 N=10 outcome:  PRECISE=1, SOFT=2, **SOFT+PRECISE=1**, TL=0, xy_mean=0.328 m, std=0.157 m.

## Procedure for new tuning

1. **Identify the failure mode** from the 7-mode table. If mode 1 (lag) → not tuneable; consider lever A/B/C/E/F.
2. **Pick ONE parameter** to vary (no combinations until singletons validated).
3. **Sweep at 3-5 values** spanning [0.5×, 2.0×] of default.
4. **Run n=5 per cell** at IC1.
5. **Save bundle to `~/ws/Test_Data/<sweep_name>/<timestamp>/`** for the analyzer.
6. **Reject any "winner" without n=10 follow-up** — first run only narrows the search.
7. **Run IC2-5 validation** before any default change (`run_ic_validation.sh`).
8. **Update memory** if a real winner appears; update this skill's "known winners" if it's baked into defaults.

## Anti-patterns observed in past sessions

- Running n=1 sweeps and treating them as data (Big Sensitivity Sweep, 84 cells × n=1 → mostly noise)
- Stacking "promising" singletons without n=10 (YEEz combo)
- Refreshing sensor cal (`aggregate_calibration.py` is broken)
- Tuning gains to fix a lag problem
- Tightening IC to fix a lag problem
- Suggesting the torque/thrust refactor (rejected per `feedback_thrust_torque.md`)

## Files

- `controller.py` — all PLASMC gains, env knobs
- `img_data.py` — image pipeline, ArUco params, savgol, stale-feature detection
- `landing_test.py` — IC convergence, marker-loss grace, control loop, autosave
- `flight_controller.py` — MAVSDK wrapper, telemetry rates, rate-gain hook (DEAD)
- `analyze_timeseries.py`, `analyze_loop_latency.py`, `analyze_sensor_noise.py`,
  `analyze_marker_switch.py`, `analyze_sigma_compare.py`,
  `diagnose_intervention_reps.py`, `analyze_impulse_response.py` — diagnostics
