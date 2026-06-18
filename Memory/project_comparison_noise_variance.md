---
name: Comparison harness has run-to-run variance
description: PLASMC in visualControl_comparison.m gives non-deterministic single-run outcomes; Monte Carlo or fixed seed needed for statistically meaningful comparison numbers.
type: project
---

**Open issue (2026-04-08):** The comparison harness `MATLAB/Comparison/visualControl_comparison.m` produces large run-to-run variance for PLASMC. Single-run results are not statistically meaningful.

**Empirical evidence:** Running PLASMC alone on Static x5 with `rng('shuffle')` gave 4 lands + 1 early break at 80 steps (t=0.8s). On Lissajous x5: 0/5 lands (all break between 200-800 steps). Yet a single seed=42 deterministic run on Lissajous landed cleanly at t=7.13s.

**Root causes (suspected, not fully verified):**
1. `rng('shuffle')` in `run_comparison.m:48` reseeds before each controller — every controller in every run gets a different noise realization.
2. `awgn(C_nP, SNR_IBVS, 'measured')` uses measured-signal-power scaling, so noise magnitude depends on the signal power at that simulation step. Across MATLAB function calls or sequential trajectory runs, accumulated state differs and produces different noise even with the same RNG seed.
3. PLASMC in the comparison config is at the edge of stability — small noise differences flip outcomes between land and cone-clamp break.

**Why:** Manuscript-quality comparison numbers require either Monte Carlo runs (N>=20 per controller per trajectory) with reported success rate + mean/stddev landing time, or a fixed seed reported alongside results. Single-run tables in the paper are misleading.

**How to apply:**
- Do NOT report single-run comparison numbers as definitive in the manuscript.
- For tuning the comparison-config PLASMC, use `rng(42)` (or any fixed seed) in `run_comparison.m:48` to get deterministic A/B testability, then revert to `'shuffle'` for the final dataset generation.
- For final paper numbers, run each (controller, trajectory) combination at least 20 times and report success rate + landing time mean ± std.
- The multi-init test (`Multi_init_cond/`) is the deterministic reference for PLASMC behavior — use it for tuning, not the comparison harness.

**Already addressed (commit 32566bc):** Cone clamp in comparison was 30° vs 35° in multi-init — fixed. Same-seed A/B confirmed cone=35 lands Lissajous PLASMC where cone=30 breaks. This was a real bug, not noise variance.

**Not addressed:** The deeper noise sensitivity. PLASMC has stability margin in the comparison config that gets eaten by single-realization noise spikes. Possible mitigations to investigate later:
- Lower SNR (more realistic noise floor) + retune
- Tighter `zeta_2` clamp / barrier saturation guard
- Add explicit pre-filter on `C_nP` instead of relying on `awgn 'measured'`
