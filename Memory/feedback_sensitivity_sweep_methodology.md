---
name: Sensitivity-sweep methodology — n=1 results are misleading
description: Lessons on interpreting per-gain sensitivity sweeps in noisy SITL — what to trust and what to retest
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
**Rule:** Per-gain singletons from sensitivity sweeps must be validated at n≥5 before drawing conclusions or stacking.

## Why

In the 2026-05-21 session, a 95-run sensitivity sweep (47 gains × 2 multipliers + 1 baseline, n=1 each) on IC1 was conducted. Two top singletons:

| Singleton (n=1) | xy at n=1 | xy at n=5 (validation) |
|---|---|---|
| `PLASMC_P_Z_SCALE=0.5` | **0.060** (PRECISE) | mean 0.49, min 0.24, max 0.71 |
| `PLASMC_E_X_SCALE=1.5` | xy=0.24 vel=0.037 (SOFT) | mean 0.63, min 0.27, max 1.03 |

The single-shot PRECISE landing was a **fortunate sample**, not a reproducible win. With 5-rep validation neither gain produced precision better than the baseline mean.

When stacked (P_Z×0.5 + E_X×1.5 combo): mean xy = 0.95, worse than either singleton alone. Combinations RARELY compose well in this SMC-cascaded controller.

## When sensitivity-sweep results CAN be trusted

- **Direction of effect**: gains that move xy or vel in one direction across multiple multipliers (×0.5, ×1.5) give credible "direction" info. E.g., we learned that increasing K_R increases body-rate spike magnitude reliably.
- **Cross-coupling map**: which gains spill into which control channels (B_T, w_x, w_y, w_z) is structural and trustworthy.
- **Hard failure indicators**: a singleton that target-loses or crashes tells us that gain is in a danger zone.

## When sensitivity-sweep results CANNOT be trusted

- **Specific xy_err values at n=1**: variance per-rep is ~0.3-0.5 m. A single 0.06 m result has wide uncertainty.
- **"Best singleton" claims**: pick the lucky tail, not the typical behavior.
- **Stack-of-bests recommendations**: the gains interact via cone-clamp + SMC κ-ODE coupling; combos almost always underperform.

## What to do

- After a sensitivity sweep finds a promising singleton: validate at n=5 BEFORE proposing combos.
- Use sensitivity-sweep singletons to PRUNE the space (eliminate clearly-bad directions), not to SELECT the answer.
- Compute std across singleton multipliers — large std (= mean) means the singleton effect is dwarfed by per-rep variance, treat as uncertain.

## Reference scripts

- `PX4_Gazebo/run_sensitivity_sweep.sh` — 47 perturbations × 2 mults, ~3 hr with SITL fixes
- `PX4_Gazebo/aggregate_sensitivity.py` — produces the channel-leverage matrix + precision-impact ranking
- IC1 baseline mean is ~0.5-0.7 m with std ~0.13-0.3 m — sensitivity-sweep "wins" below 0.4 m at n=1 are often fluky

## How to apply

- When the user wants to run a sensitivity sweep: that's fine, but caveat the single-rep variance.
- When a singleton looks "amazing" (e.g., xy=0.06): validate with n=5 before proposing it as a finding.
- When a "promising combo" is suggested: warn that combos in this SMC controller don't compose well unless mechanisms are clearly independent.
