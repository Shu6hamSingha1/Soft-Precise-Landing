---
name: run_simulation seed-passing convention
description: run_simulation.m clobbers caller-side rng() when no seed is passed; any deterministic sweep harness must pass seed via the 6th arg
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
`run_simulation(x0, trajType, K_override, speed_mult, cfg_override, seed)` — if both `K_override` and `seed` are empty/absent, the function internally calls `rng('shuffle')` which **silently clobbers any caller-side `rng(N)`**. A sweep loop that does `rng(1000+k); run_simulation(...)` with only 4 args will look deterministic but produce different draws on every run.

**How to apply:** For any deterministic sweep harness (speed sweep, deep sweep, multi-init, combo validation), pass the seed explicitly via the 6th argument:
```matlab
tmp = run_simulation(x0, trajType, K_override, speed_mult, [], 1000+k);
```
This worked around a real bug in `sweep_speed.m` discovered 2026-04-19 — the non-determinism had caused a spurious Circular trajectory failure at mult=1.0×.

**Why the API behaves this way:** The default shuffle branch exists so single-run harnesses (e.g., `visualControl_IBVS_adaptive.m` plots) get varied noise realizations. But it makes the 5-argument call-path a silent footgun for batch/reproducible experiments.
