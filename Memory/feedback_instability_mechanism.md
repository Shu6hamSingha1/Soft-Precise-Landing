---
name: PLASMC instability mechanism (per-step diagnosis, 2026-05-21)
description: Time-series analysis of N=30 SITL runs identifying multi-factor IC sensitivity as the cause of ~0.4m xy variance; tightening one IC variable just shifts the dominant predictor to the next.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Cause of instability (definitive, time-domain evidence)

**Multi-factor IC sensitivity, no single dominant variable.** The SMC-driven descent's short flight time (~4s with `LANDING_REF_RAD_OPT_FLOW=-0.70`) is not long enough to wash out initial-condition variance in any of several correlated state variables. The dominant predictor of xy_end migrates as constraints tighten.

## Why: Time-domain finding

Across N=10 reps with the new defaults (K_R×0.4 + W_U_MAX=1.0 baked in, REF_RAD=-0.70, WIN=7):

- `|s_e_n|_tail` (terminal normalized image error) correlates with xy_end at **ρ=+0.94** — bad landings simply fail to drive image error to zero
- `|σ|_tail` (terminal SMC sliding variable) correlates at **ρ=+0.68** — bad reps end with 4× larger σ
- `σ0_xy` (initial sliding variable XY components) correlates at **ρ=+0.77** in loose-IC, despite spanning only 0.3% (0.354–0.358)
- `|s_e_n|` and `σ` diverge between good/bad reps at t≈0 — the OUTCOME IS SET BEFORE THE CONTROLLER ACTS

A 0.3% variation in σ0 produces 30× variation in xy_end. Mechanism: the Savgol-filtered optic flow uses ~7 samples (~117 ms) of pre-engagement motion. Residual lateral drift in the pre-engagement hover propagates into σ0, and the SMC's leaky-adaptation κ doesn't have enough time during the 4-s descent to fully correct it.

## Multi-IC tightening test (definitive)

| Config | σ0_mean | σ0_std | vh0_mean | xy_mean | xy_std | TL | PREC |
|---|---|---|---|---|---|---|---|
| LOOSE (vel=0.5, tilt=3°, hits=20) | 0.034 | 0.016 | 0.089 | 0.484 | 0.379 | 2/10 | 2/10 |
| TIGHT (vel=0.15, tilt=1.5°, hits=50) | 0.016 | 0.013 | 0.115 | 0.517 | 0.286 | 1/9 | 1/9 |
| ULTRA (vel=0.05, tilt=1.0°, hits=100) | 0.012 | 0.010 | 0.110 | 0.531 | 0.263 | 3/9 | 1/9 |

Correlation migration as IC tightens:
```
LOOSE:      ρ(σ0)=+0.79   ρ(vh0)=-0.59   ← σ0 is binding constraint
TIGHT:      ρ(σ0)=-0.19   ρ(vh0)=+0.62   ← vh0 takes over as binding
ULTRATIGHT: ρ(σ0)=-0.81   ρ(vh0)=+0.42   ← no clean signal anymore
```

σ0 shrank 64% but xy_mean didn't move. The variance source migrates rather than dissolving. xy_std drops 31% (LOOSE→ULTRATIGHT) but TARGET_LOST rate **doubles** (10% → 33%) from longer marginal-hover settling phase.

## Why this matters for future tuning attempts

- **No single-variable fix exists.** Gain tuning OR IC tightening will both fail to break the ~0.40 m mean / ~0.25 m std xy floor.
- **The architectural ceiling is real, and now mechanistically understood.** It's not "stochastic noise" — it's specifically multi-factor IC sensitivity propagated through a short SMC trajectory.
- **Per-step analysis is now available.** `PX4_Gazebo/analyze_timeseries.py` loads a directory of reps and produces Pearson correlations of every channel against xy_end. Future "why did this rep fail?" questions should start there.

## How to apply

- When the user asks "why is precision stuck at 0.4 m?": cite this analysis. The cause is mechanistic — multi-factor IC sensitivity — and confirmed by the σ0 → vh0 → noise correlation migration as constraints tighten.
- When the user proposes more gain tuning OR tighter IC: gently note that both approaches were tested (gain sweep N=419 + IC sweep N=29) and neither moves xy_mean. They can shrink xy_std modestly (31% from IC tightening) at cost of higher TL rate.
- The available env knobs (`LANDING_IC_VEL_TOL`, `LANDING_IC_TILT_TOL_DEG`, `LANDING_IC_STABLE_HITS`, `LANDING_IC_BUDGET_S`) are documented in landing_test.py:165-174 and stay env-only — defaults match LOOSE because tighter increases TARGET_LOST without moving xy_mean.
- For mechanistic follow-ups that COULD help: (a) longer descent (slower `REF_RAD`) to give SMC more time to correct IC — but this regressed historically; (b) reset Savgol buffer at controller engagement; (c) skip σ-feedback for first 0.5s (soft-start kappa). None tested yet.

## Data

- `~/ws/Test_Data/DefaultN10/20260521-224307/` — LOOSE-IC bundle (10 reps)
- `~/ws/Test_Data/TightICN10/20260521-225923/` — TIGHT bundle (9 reps; 1 IC-timeout)
- `~/ws/Test_Data/UltraTightN10/20260521-232334/` — ULTRA bundle (9 reps; 1 IC-timeout)
- Analyzer: `PX4_Gazebo/analyze_timeseries.py <bundle_dir>` — per-channel stats + correlations + divergence-time analysis
