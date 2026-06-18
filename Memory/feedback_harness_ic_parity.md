---
name: Keep PLASMC harness p0 matrices locked to canonical multi_Init_Var set
description: sweep_speed.m and multi_Init_Var.m (and any future PLASMC harness) must share the same canonical 5-IC p0; silently divergent ICs produce incomparable envelopes
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Any PLASMC validation harness that sweeps "the 5 canonical initial conditions" MUST use the multi_Init_Var.m p0 matrix verbatim:
```
p0 = [
    0,   0,  -5;
    2.0, 2.0, -5;
    2.0,-2.0, -5;
    2.0, 2.0, -7;
    2.0, 2.0, -3
];
```
Applies to: `MATLAB/Multi_init_cond/multi_Init_Var.m`, `MATLAB/Sweeps/sweep_speed.m`, `MATLAB/Multi_init_cond/multi_speed_cond.m`, any new sweep harness.

**Why:** Pre-2026-04-19 sweep_speed.m used a different IC set (`[0,0,-5; 0,0,-7; 0,0,-3; 2,2,-5; -2,-2,-5]`) than multi_Init_Var (`[0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3]`). Both sets contained a `(2,2,-5)`-family IC but at DIFFERENT row positions, so "IC5" meant `[2,2,-3]` in one harness and `[-2,-2,-5]` in the other. sweep_speed would FoV-fail at Linear mult=1.00 on its IC5 while multi_Init_Var passed Linear IC5 cleanly — because they were testing different ICs. Debugging wasted ~30 min before the silent divergence was caught. Envelope numbers citied in manuscript prior to the fix are not comparable across harnesses.

**How to apply:**
- When creating a new PLASMC sweep script, **copy** the p0 block from multi_Init_Var.m verbatim — do NOT retype. Short-IC-label references ("IC3", "IC4") only remain meaningful if ALL harnesses map the same row → same position.
- When reading results across harnesses, first verify p0 matches: `grep -A 6 "p0 = \[" <harness>`.
- If someone proposes using a *different* IC set in a harness (e.g., to stress a specific failure mode), they MUST rename the script and its output fields (`p0_stress`, not `p0`) to prevent cross-harness aliasing.
- The `fov_fail_ICs` / `fail_ICs` indices in result structs become ambiguous if p0 diverges — always co-load p0 with the results when tabulating.
