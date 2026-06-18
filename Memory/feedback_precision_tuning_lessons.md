---
name: PLASMC precision-tuning lessons (rate-mode port)
description: Hard-won lessons from ~300 SITL landings in 2026-05-20/21. What works, what doesn't, what to NOT try again.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
## TL;DR

For the rate-mode SO(3) PX4 port (MAVSDK body-rate + thrust, not torque): **IC1 precision floor ≈ 0.4 m mean, ≈ 0.13 std; IC2-5 floor ≈ 1.6-2.5 m mean.** No soft+precise (xy ≤ 0.08 m AND vel ≤ 0.2 m/s) is reachable from any tested configuration under strict TARGET_LOST=failure criterion. Single-axis gain tuning has reached its empirical ceiling; further breakthroughs need a different mechanism (image-side, sensor cal, or architectural).

## What actually moves IC1 precision (in order of effect)

1. **`IMG_FILTER_WIN=7`** (from default 13): centroid feedback delay 100ms → 50ms. Cuts mean xy ~20 cm. **Biggest single lever found.** Risk: too short (e.g., 5) reintroduces TL via residual noise on the centroid signal. Env: `IMG_FILTER_WIN`.

2. **`LANDING_REF_RAD_OPT_FLOW=-0.70`** (from -0.42): faster descent → less drift accumulation window. Cuts mean xy ~10 cm. **Regresses IC2-5 badly** — keep env-only, never default.

3. **`PLASMC_KR_SCALE=0.4` + `PLASMC_W_U_MAX=1.0`**: stable inner loop. Does NOT improve precision directly (actually mildly worsens it 0.69→0.82 at constant other params); rather it ENABLES the above two changes to work without TARGET_LOST. The body-rate clamp commit is `1d44cd5`.

The best stable config: `PLASMC_KR_SCALE=0.4 PLASMC_W_U_MAX=1.0 LANDING_REF_RAD_OPT_FLOW=-0.70 IMG_FILTER_WIN=7` — mean xy 0.387 m, std 0.13 m, 0/5 TARGET_LOST on IC1.

## What does NOT help (negative findings to NOT retry)

- **K_rp / K_ri / K_rd boost** (×1.5 or ×2): unstable; outer loop overdrives clamped inner; xy mean explodes
- **K_R lower than 0.3**: too slow to converge (mean xy ≥ 1.2)
- **K_R higher than 0.5 without clamp**: peak body rates >2 rad/s → LK loses corners → TARGET_LOST
- **`τ_ia` longer** (× 2.5, × 5): partially reduces I_a oscillation but mean xy worsens
- **`τ_ia` shorter** (× 0.5): exposes controller noise, mean xy worsens
- **`κ_0_Z × 0.5 + N_Z × 0.5` combo**: over-detunes, worsens
- **REF_RAD slower** (-0.30): more time at low altitude → more TARGET_LOST events (40%!)
- **REF_RAD with KP boost**: instability cascade (one rep hit 24m crash)
- **Stacking "promising singletons" from sensitivity sweeps**: interactions cancel benefits, almost always worse than singletons alone
- **w_u clamp at 1.5** (vs 1.0): some target loss returns; 1.0 is the sweet spot
- **Sensor cal refresh via `aggregate_calibration.py`**: methodology mismatch (std-ratio vs median(gt/raw)) gives 7-10× different `_sensor_cal_hw`; even a +5% change to `_sensor_cal_s[0]` (0.5830→0.6104) destabilized the loop 3×

## Mechanism notes (for understanding future symptoms)

### TARGET_LOST root cause
Body-rate command spikes >1.7 rad/s cause corner motion >15 px/frame on our 60 Hz / f=270 setup. That exceeds LK's tracking window. Effect cascade:
1. LK loses corners → `OPTIC FLOW UNAVAILABLE` printed
2. ArUco refinement fails on stale corner positions
3. `CHECK_NUM=80` consecutive failures → `FEATURE_IS_VISIBLE` flips
4. landing_test.py grace expires (1s) → open-loop final descent
5. `landing_test.py:340` sets `target_lost = True` → classified as failure

Default `PLASMC_W_U_MAX=1.0` (commit `1d44cd5`) caps body rates well under the LK threshold.

### TARGET_LOST = failure (strict criterion)
Per user directive 2026-05-21 (commit `66df93d`): marker loss beyond the 1-s grace counts as TARGET_LOST failure, NOT a graceful soft-precise success. Many "SOFT" results from earlier in the session were actually marker-loss fallback landings that happened to be near target — those don't count.

### Precision plateau diagnosis
The residual ~0.4 m at IC1 decomposes roughly into: centroid measurement noise (50-100 mm), inner-loop attitude lag in PX4 rate controller (50-100 mm, unfixable from our side), outer-loop position lag from τ_ia LPF (~80 mm, can't shorten without exposing noise), per-rep stochastic initial state (50-150 mm variance), sensor-cal residual (~30-80 mm). No single intervention addresses more than one contributor.

## Big sensitivity sweep (2026-05-21) — comprehensive n=5 confirmation

84 cells × 5 reps + best-config baseline = 419 runs at IC1 on top of the best-stable
config (K_R×0.4 + W_U_MAX=1.0 + h_rd=-0.70 + WIN=7). Output `~/ws/Test_Data/BigSensitivity/20260521-073234/`.

**TOP n=5 PRECISION CELLS (stable, TL≤1):**
```
P20_Z × 0.5         xy_mean=0.233  xy_min=0.143
E_X   × 1.5         xy_mean=0.276  xy_min=0.038 ★ 1 PRECISE
P_X   × 0.5         xy_mean=0.317  xy_min=0.099
E_Z   × 0.5         xy_mean=0.343  xy_min=0.130   + 3/5 SOFT ★ best softness
YAW_OMEGA × 1.5     xy_mean=0.359  xy_min=0.108
P_Z   × 0.5         xy_mean=0.366  xy_min=0.080 ★ 1 PRECISE
GAMMA_Y × 1.5       xy_mean=0.373  + 2/5 SOFT
YAW_N × 0.5         xy_mean=0.373  xy_min=0.083 ★ near PRECISE
P2INF_Y × 0.5       xy_mean=0.375
YAW_E × 1.5         xy_mean=0.298  xy_min=0.029 ★ smallest single-rep xy (1 TL though)
```

**TOP n=5 SOFTNESS CELLS (stable, TL=0):**
```
E_Z × 0.5           3/5 SOFT, mean vel 0.65
P20_Y × 1.5         2/5 SOFT, mean vel 0.48
GAMMA_Y × 1.5       2/5 SOFT, mean vel 0.69
```

**Phase 3 combos (n=5 each):** All four orthogonal-pair candidates tested. YEEz combo (`YAW_E×1.5 + E_Z×0.5`) gave the best singleton ever: xy=0.0085m (8.5mm!). **But Phase 4 n=10 validation of YEEz showed it was a fluke** — mean xy=0.386, std=0.19, 0/10 PRECISE, 0/10 SOFT+PRECISE. Same as baseline.

**Definitive ceiling for the rate-mode SO(3) port (IC1):**
- Mean xy: ~0.39 m (baseline) to ~0.23 m (best singleton P20_Z×0.5)
- xy_min over ~600 reps: 0.0085 m (lucky single shot, doesn't repeat)
- Reproducible PRECISE rate: 0%
- Reproducible SOFT rate: up to 60% with E_Z×0.5
- **Reproducible SOFT+PRECISE: 0%**

**The precision variance is stochastic, not gain-tunable**: initial-state-at-engagement,
LK frame-by-frame noise, SMC κ-ODE trajectory, sub-pixel corner jitter at low altitude.
None of these are addressable from the gain set.

## How to apply

- When the user asks for a precision experiment: check whether it's been tried (DON'T list above). Quote the relevant past result and ask if they want to retest or do something new.
- When proposing a "stacked combo" of singletons: warn that this approach failed at n=10 even when n=5 looked promising (YEEz). Single-shot wins don't reproduce.
- When the user says soft+precise isn't yet hit: that's the architectural ceiling for the rate-mode port — ~0.4 m mean, 0% SP rate confirmed across ~600 runs.
- When defaulting any new behavior: validate on IC2-5 first. IC1-optimized configs reliably regress off-center starts.
- The strongest precision-improving singles are P20_Z×0.5, E_X×1.5, P_Z×0.5 (all reduce mean by 25-40%) but none cross to reproducible PRECISE.
- The strongest softness lever is E_Z×0.5 (3/5 SOFT, stable).
