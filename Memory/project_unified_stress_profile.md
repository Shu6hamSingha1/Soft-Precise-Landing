---
name: Unified harder stress profile for 5x5 comparison
description: Current stress profile used for the 5-controller comparison rerun; Circular r=0.5 wz=0.3, matches Multi_init_cond traj_Gen
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
The 5-controller comparison now runs against a unified harder stress profile shared with the multi-init sweep. Circular trajectory parameters are `r=0.5 m, ω_z=0.3 rad/s` (sourced from `MATLAB/Multi_init_cond/Common/traj_Gen.m`, which the Comparison harness now `addpath`s into via `run_comparison.m`).

**Why:** Previously Comparison/ and Multi_init_cond/ used divergent traj_Gen copies. Collapsing to one shared generator means any tuning validated in the multi-init sweep transfers directly to the comparison study, and the comparison stress level matches what the manuscript claims.

**How to apply:**
- `run_comparison.m` must `addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Common'))` **inside** the function body (not before `function`, or MATLAB treats it as a script and errors with "Local function name must be different from the script name").
- Latest rerun log: `MATLAB/Comparison/rerun_all.log`. PLASMC lands on all 5 trajectories ~0.20 m xy in 9.6–12.4 s. Lin fails Linear (4.15 m), Zhang crashes <3 s on most, Chen hits 40 s budget on all, Cho hits budget on Linear.
- Landing criterion in force: altitude < 0.20 m AND horizontal error < 0.30 m (matches manuscript PDF, not the stale 0.10/0.05 in results.tex).
