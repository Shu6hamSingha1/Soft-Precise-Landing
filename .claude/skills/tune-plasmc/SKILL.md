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
| **2** | **Marker-detection breakdown** at z<0.3m | `Image Feature Pts` frozen for ≥100ms in final descent; sides stay <100 px through touchdown | Interventions 1+2+3 (committed 8baea2b). KLT fallback (`MARKER_KLT_MAX_STEPS=20`, default ON) further bridges short outages by LK-tracking last good corners — see [[klt-marker-fallback]]. |
| **3** | **IC sensitivity amplified by lag** | vh0 > 0.2 m/s → σ_xy diverges; PX4 σ_trajectory deviates from MATLAB by 4× in σ_xy only | Tightening IC tol alone doesn't help (validated). Need lag reduction (mode 1). |
| **4** | **Kr too aggressive** | Peak body rates >1.7 rad/s → LK loses corners → TARGET_LOST | `PLASMC_KR_SCALE=0.4` (current default) + `PLASMC_W_U_MAX=1.0` |
| **5** | **PID-output saturation** | a_u peaks > 100 m/s² before LPF; tau_ia can't smooth | tau_ia ∈ [0.08, 0.20] is the safe range. Higher = lag, lower = noise. |
| **6** | **Funnel-envelope breach** | rho_fov_log near zero, theta_current near theta_cone | Widen `RHOFOV0` or relax `THETACAP`. Very rare in current config. |
| **7** | **Sensor cal drift** | Optic-flow lstsq returns clipped or non-finite | Don't refresh sensor cal — methodology mismatch in `aggregate_calibration.py` produces 7-10× wrong gains (see `feedback_calibration_lessons.md`). Sensor noise already matches MATLAB per Phase 4. |
| **8** | **Compass yaw drift** under aggressive maneuvers | EKF yaw diverges 30-46° from GT in input-cal-style commands; body-frame analysis shows ~100m phantom position mismatch even though world position is correct within ~10m | Use `BODY_YAW_SOURCE=alpha` (control-side, **now the default**) — SO(3) loop uses the drift-free alpha feature instead of euler[2]. (NOT `V_YAW_SOURCE=alpha` — REMOVED 2026-06-04; it zeroed the yaw feature → open-loop yaw. See [[compass-free-yaw-sign]], [[v-yaw-source-alpha]].) Use `BODY_YAW_SOURCE=gt` in `plotter_input_calibration.ipynb` for analysis. |
| **8b** | **Compass drift at LANDING START → "yaw runaway"** (the real IC2-5 yaw failure) | At descent start GT yaw ≈ 77° while EKF yaw ≈ 0°; the IC rig holds/gates on the **EKF** yaw, so the drone BEGINS the descent physically yawed ~77° → `psi_d`→±180°, xy 2-16m. `alpha_start ≈ GT_yaw_start` every rep (**alpha is correct**). | **Don't touch alpha** — three alpha redesigns all hit 180° because the cause is the bad start, not the feature. Fix the **test rig** (`landing_test.py` IC convergence): servo NED yaw to null the **true (Gazebo)** yaw + gate on truth so the descent starts aligned. Controller untouched (yaw is image-alpha; compass only enters the rotation matrix, absorbed by the alpha outer loop). See [[yaw-compass-drift-ic-start]]. |

If the failure is mode 1 → no controller-side tuning fixes it. Mode 2 → already fixed by defaults. Mode 4-7 → tuneable. Mode 3 → secondary to mode 1. **Mode 8b → fix the IC rig, NOT the controller/alpha.**

## Complete parameter inventory

**2026-06-03 CLEANUP: all knobs are now DIRECT per-axis parameter values** (e.g. `PLASMC_KP_X=1.4`),
not scale factors. Obsolete mechanisms (K_rp scheduling, DSD clamps, THETA_FLOOR, VFRAME_ROT, PX4_RATE_SCALE)
were removed — see parameter_record.ods sheet `Removed_Parameters` for the full history.

**The validated IC1 config (28% SP @ 10cm, n=25 — parameter_record trial 46), in direct-value form:**
```
PLASMC_KP_X=1.4 PLASMC_KP_Y=1.4  PLASMC_KI_X=0.35 PLASMC_KI_Y=0.35  PLASMC_KD_X=0.5031 PLASMC_KD_Y=0.5031
PLASMC_YAW_OMEGA=0.2 PLASMC_YAW_GAMMA=0.2
PLASMC_RHOFOV0_V=315 PLASMC_RHOFOVINF_U=220 PLASMC_RHOFOVINF_V=300
PLASMC_GAMMA_Z=0.375
MARKER_KLT_MAX_STEPS=20 LANDING_STALE_COMMIT_EXTENT=100
CTRL_ZERO_WXY=1 BOARD_ALPHA0=1.23 BODY_YAW_SOURCE=alpha
```

### Outer loop (image-error → desired optic-flow rate)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `K_rp` (P gain) | `diag(9, 9)` | `PLASMC_KP_{X,Y}` (direct per-axis values; defaults 9.0) | Boost >1.5× → instability cascade |
| `K_ri` (I gain) | `diag(1, 1)` | `PLASMC_KI_{X,Y}` (direct; defaults 1.0) | 10× MATLAB's 0.1; needed for SITL drift correction |
| `K_rd` (D gain) | `diag(1.4375, 1.4375)` | `PLASMC_KD_{X,Y}` (direct; defaults 1.4375) | Sensitive to centroid noise; ↑ → chatter |
| `DH_D_MAX` | 50.0 m/s³ | `PLASMC_DH_D_MAX` | Clamp on h_d derivative. **LOAD-BEARING (2026-06-02): the clamp value feeds Θ_norm → κ-runaway; =5.0 eliminates IC1 hard impacts (n=5: rel_vel max 9.5→0.4 m/s, κ bounded, xy unchanged). IC2-5 gate PASSED 2026-06-03 (no regression; note both arms ~5-6m there — see memory multisine-cal-ic25-collapse).** |

### Middle loop (PLASMC funnel + sliding)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `gamma` (Ξ_2) | `diag(0.2, 0.2, 0.2)` | `PLASMC_XI2_{X,Y,Z}` (direct) | Sliding-variable gain |
| `p_2_0` | `[25, 25, 4]` | `PLASMC_P20_{X,Y,Z}` (direct) | Initial half-width of funnel |
| `p_2_inf` | `[2.5, 2.5, 1.5]` | `PLASMC_P2INF_{X,Y,Z}` (direct) | Terminal half-width — LOAD-BEARING |
| `Omega` (Ω) | `diag(0.05, 0.05, 0.025)` | `PLASMC_OMEGA_{X,Y,Z}` (direct) | κ-leakage rate. LOAD-BEARING (controls SP rate) |
| `Gamma` (Γ) | `diag(0.4375, 0.5, 0.75)` | `PLASMC_GAMMA_{X,Y,Z}` (direct) | κ-adaptation rate |
| `E` | `diag(1, 1, 1)` | `PLASMC_E_{X,Y,Z}` (direct) | Boundary-layer thickness; sat() arg scaled by 1/E |
| `N` | `diag(0.02, 0.02, 0.02)` | `PLASMC_N_{X,Y,Z}` (direct) | Drives e-modification term in κ-ODE |
| `P` | `diag(1.5, 1.5, 2.5)` | `PLASMC_P_{X,Y,Z}` (direct) | Anti-windup-like gain on κ-ODE (P_z=2.5 baked) |
| `kappa_0` (init κ) | `[0.15625, 0.15625, 0.3125]` (1.25× baked into base) | `PLASMC_KAPPA0_{X,Y,Z}` (direct) | Already at best singleton (big sweep) |

### Yaw SMC (low-impact per supplement S3-A)
| Param | Default | Env knob |
|---|---|---|
| `Omega_a` | 0.5 | `PLASMC_YAW_OMEGA` (direct) |
| `Gma_a` | 0.5 | `PLASMC_YAW_GAMMA` (direct) |
| `n_a` | 1.0 | `PLASMC_YAW_N` (direct) |
| `p_a` | 2.0 | `PLASMC_YAW_P` (direct) |
| `kappa_a_0` | 2.0 | `PLASMC_YAW_KAPPA0` (direct) |
| `E_a` | 3.0 | `PLASMC_YAW_E` (direct) |

### FoV-margin cone clamp
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `rho_fov_0` | `[290, 210]` | `PLASMC_RHOFOV0_{U,V}` (direct, px) | Initial pixel envelope (per image axis) |
| `rho_fov_inf` | `[80, 80]` | `PLASMC_RHOFOVINF_{U,V}` (direct, px) | Terminal pixel envelope (per image axis) |
| `l_fov` | 0.1 | `PLASMC_LFOV` (direct) | Envelope decay rate (1/s) |
| `theta_cap` | 60° | `PLASMC_THETACAP_DEG` (direct, deg) | Soft cone ceiling — acceleration saturation |
| `theta_floor` | 0° (legacy) | `PLASMC_THETA_FLOOR_DEG` | **Floor on θ_cone (2026-06-03). The d_min collapse was strangling terminal correction (94-100% of final-2s samples at IC1) — THETACAP is irrelevant when d_min=0. floor=60 → SP #6 (xy 0.060/vel 0.149, first mechanism-driven SP). See memory fov-cone-clamp-deadlock.** |

### Inner loop (SO(3))
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `K_R` | `diag(2, 2, 2)` (0.4×5 baked into base) | `PLASMC_KR_{ROLL,PITCH,YAW}` (direct; defaults 2.0) | 2.0 is the stable sweet spot. Higher → LK breakage |
| `W_U_MAX` (body-rate clamp) | 1.0 rad/s | `PLASMC_W_U_MAX` | Caps body-rate command magnitude |

### Misc / control-loop
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `tau_ia` (LPF on I_a_cd) | 0.08 s | **not env-overridable** | Tested 0.04, 0.20 — both worse |
| `iV_s_e_n_clamp` | 5.0 | **not env-overridable** | Anti-windup on integral of normalized error |
| `izeta_clamp` | 5.0 | **not env-overridable** | Anti-windup on integrated sliding variable |
| `ie_a_clamp` | 2.0 | **not env-overridable** | Anti-windup on integrated yaw error |

### Image pipeline (img_data.py)
| Param | Default | Env knob | Notes |
|---|---|---|---|
| `FILTER_WIN` (savgol window) | 13 → user uses 7 | `IMG_FILTER_WIN` | |
| `FILTER_POLYORDER` | 1 | `IMG_FILTER_POLY` | |
| `STALE_THRESH` (intervention 2) | 3 frames | `IMG_STALE_THRESH` | |
| `adaptiveThreshConstant` | 5.0 (intervention 1) | `ARUCO_ADAPT_THRESH_C` | |
| `errorCorrectionRate` | 0.8 (intervention 1) | `ARUCO_ERR_CORRECT` | |
| `minMarkerPerimeterRate` | 0.02 (intervention 1) | `ARUCO_MIN_PERIM_RATE` | |
| `minOtsuStdDev` | 3.0 (intervention 1) | `ARUCO_MIN_OTSU_STD` | |
| `IMG_EXTRA_PTS` (hybrid Shi-Tomasi corners) | 0 | `IMG_EXTRA_PTS` | Set ≥50 → over-determines 8x6 lstsq with extra corners; defaults to OFF |
| `BODY_YAW_SOURCE` (SO(3) yaw source) | `'alpha'` (default since 2026-06-04) | `BODY_YAW_SOURCE` | `'alpha'` = compass-free SO(3) (uses drift-free s[3], not euler[2]); `'compass'` = legacy. See [[compass-free-yaw-sign]]. (`V_YAW_SOURCE` was removed — marker-aligning V zeroed the yaw feature; see [[v-yaw-source-alpha]].) |
| `MARKER_KLT_MAX_STEPS` (KLT fallback cap) | `20` | `MARKER_KLT_MAX_STEPS` | KLT-tracks last good corners when ArUco fails; bridges short outages. Default ON. Set to `0` to disable. See [[klt-marker-fallback]]. |

### Landing-test (landing_test.py)
| Param | Default | Env knob |
|---|---|---|
| `REF_RAD_OPT_FLOW` (h_rd) | -0.42 (MATLAB), user uses -0.70 | `LANDING_REF_RAD_OPT_FLOW` |
| `MARKER_LOSS_GRACE` | 1.0 s | `LANDING_MARKER_LOSS_GRACE` |
| `IC_POS_TOL`, `IC_VEL_TOL`, `IC_TILT_TOL_DEG`, `IC_STABLE_HITS`, `IC_BUDGET_S` | 0.5 m / 0.5 m/s / 3° / 20 / 30s | `LANDING_IC_*` |
| `PX4_RATE_SCALE` | 1.0 (DEAD — see `feedback_mc_rate_p_dead.md`) | `PLASMC_PX4_RATE_SCALE` |

## Tuning methodology (RULES — violate at your peril)

0. **Per-axis knobs ONLY (user directive 2026-06-03).** All uniform scale multipliers
   were REMOVED from controller.py. Tune `_X/_Y/_Z` (or `_ROLL/_PITCH/_YAW`, `_U/_V`)
   variants. The two image axes run at different effective gains (x is 1.39× hotter —
   see memory per-axis-tuning); uniform scaling can never fix that.

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

**STANDARD PROCEDURE (2026-06-03, user methodology): failure root-causing + saturation audit.**

For every batch (not just failures):
1. `analyze_saturation_audit.py --glob '<bundle>/rep*'` — duty cycle of all 12 limits (6 code guards +
   6 manuscript limits), SP-vs-non-SP ranking. Active limits = silent performance loss even in good reps.
2. `analyze_saturation_audit.py --events <reps>` — every saturation event with when/why (auto-attributed
   reason: e.g. cone "large ask" vs "small allowance", σ/ℰ funnel-error ratio, accel-floor z-spike).
3. For each frequent event type: reason → the manuscript parameter that owns it → tune that parameter so
   the signal stays in its linear regime. NEVER respond by widening the limit itself (limits are a last
   resort, only when performance cannot improve any other way).

For a single failed rep:
1. `analyze_explosion_chain.py <rep_dir> [...]` — finds WHICH state departs its normal envelope FIRST and the resulting causal chain (ds_d → dh_d clamp → ζ saturation → κ runaway → a_u). Aggregates first-movers across reps. This is the per-term a_u attribution tool.
2. Load with `analyze_timeseries.py <rep_dir>` (per-channel correlation with xy_end)
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

## The precision-softness frontier (2026-05-24, mapped via 5 N=10 sweeps)

The architecture's lag forces a frontier — knobs move along it, not off it. Demonstrated empirically:

```
                          PREC  SOFT  SP    xy_min   vh_end_mean
Interventions only         1     2    1     0.039    1.2     ← still best
RHOFOVINF × 0.5            0     4    0     0.110    0.80
RHOFOVINF × 0.7            1     2    0     0.077    0.99
THETACAP × 1.5             0     1    0     0.099    0.94 (+ TL)
Stack ×0.5 + KP×1.25       1     2    0     0.031    0.88
─────────────────────────────────────────────────────────
Combined SP rate           1 / 50 = 2%
```

**The PRECISE-only failure mode** (across 19 PRECISE-only reps out of 1205): every rep has |vh_end| ≫ |vz_end|. The drone lands precisely on target but is still sliding laterally at 1+ m/s. The architecture can HOLD zero lateral velocity but cannot actively BRAKE significant lateral velocity in the final phase.

When the user proposes tuning the terminal-phase parameters (`RHOFOVINF`, `THETACAP`, `KP/KI/KD` boosts, or their combos), cite this finding. The frontier is mapped; further single-knob terminal-phase sweeps will land on a point already characterized.

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

**2026-06-03 (multisine-cal era): `PLASMC_THETA_FLOOR_DEG=60` + `PLASMC_DH_D_MAX=5` (default) → SP #6
(xy=0.060, vel=0.149) at IC1, 1/5 rate, 0 TL. The current best config. Neither knob is sufficient alone;
PID_SCALE=0.54 stacking adds nothing. Open: raise SP rate (terminal softness) + IC2-5 overshoot prevention.**

## Procedure for new tuning

1. **Identify the failure mode** from the 7-mode table. If mode 1 (lag) → not tuneable; consider lever A/B/C/E/F.
2. **Pick ONE parameter** to vary (no combinations until singletons validated).
3. **Sweep at 3-5 values** spanning [0.5×, 2.0×] of default.
4. **Run n=5 per cell** at IC1.
5. **Save bundle to `~/Soft-Precise-Landing/PX4_Gazebo/test_data/<sweep_name>/<timestamp>/`** for the analyzer.
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

- `src/controller.py` — all PLASMC gains, env knobs
- `src/img_data.py` — image pipeline, ArUco params, savgol, stale-feature detection
- `apps/landing_test.py` — IC convergence, marker-loss grace, control loop, autosave
- `src/flight_controller.py` — MAVSDK wrapper, telemetry rates, rate-gain hook (DEAD)
- `tools/analyze_saturation_audit.py` — duty cycle + per-event attribution of all 12 limits (2026-06-03; the standard batch diagnostic)
- `tools/analyze_explosion_chain.py` — first-exploding-state + causal-chain diagnosis (2026-06-02)
- `tools/analyze_timeseries.py`, `tools/analyze_loop_latency.py`, `tools/analyze_sensor_noise.py`,
  `tools/analyze_marker_switch.py`, `tools/analyze_sigma_compare.py`,
  `tools/diagnose_intervention_reps.py`, `tools/analyze_impulse_response.py` — diagnostics
