---
name: traj_Gen.m canonical location
description: Single canonical traj_Gen lives at MATLAB/Common/traj_Gen.m; Multi_init_cond duplicate deleted 2026-04-14
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
There is now exactly one `traj_Gen.m` in the repo: `MATLAB/Common/traj_Gen.m`. The old duplicate at `MATLAB/Multi_init_cond/traj_Gen.m` has been deleted.

**Why:** Divergent copies caused Comparison/ and Multi_init_cond/ to run against subtly different trajectories, invalidating cross-study comparisons. The 2026-04-14 Circular `wz` sweep was the incident that forced consolidation.

**How to apply:**
- Edit `MATLAB/Common/traj_Gen.m` for any trajectory tuning — changes propagate to both Multi_init_cond and Comparison automatically.
- Both `multi_Init_Var.m` and `visualControl_IBVS_adaptive.m` already `addpath(fullfile(mfile_dir, '..', 'Common'))`, so no path fix needed.
- `run_comparison.m` does the same addpath **inside** the function body (not before `function`, or MATLAB treats the file as a script).
- Current locked values: Circular `r=0.5, wz=0.3`; Lissajous `A=0.5, B=0.8, w1=-0.8, w2=+0.4`; Sinusoidal `A=0.5, w0=0.8, v0=0.3`; Linear `v=[1;1;0]`.
