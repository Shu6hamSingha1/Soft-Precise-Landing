---
name: bestParam.mat — RETIRED (moved to Obsolete/ 2026-04-22)
description: bestParam.mat and regen_bestParam.m are retired; K.p_10 now sourced from Constants.m. Do NOT reintroduce.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Status: RETIRED 2026-04-22.** `bestParam.mat` and `regen_bestParam.m` moved from `MATLAB/Common/` to `Obsolete/Common/`. All four runtime consumers now source `K.p_10` from `Constants.m:32` (`K.p_10 = [res(2)/2/f; res(1)/2/f]`) — no binary file needed.

**Why:** Every tuned gain (`rp`, `ri`, `rd`, `gamma_2`, `Omega`, `kappa_0`, `E`, etc.) was already re-defined inline in each harness via `diag()` literals. The only downstream read from the stored `K` struct was `K.p_10`, and `Constants.m` already computed the same value from canonical `res`/`f`. The `.mat` existed only to carry one two-element vector, with a whole `regen_bestParam.m` script and cross-file coupling to keep it in sync with the gain literals.

**Retirement scope:**
- `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m` — the `if exist('K','var')/elseif isfile('bestParam.mat')` scaffold at the top replaced with plain `clear;`
- `MATLAB/Multi_init_cond/run_simulation.m` — `load("bestParam.mat");` at the top of the function removed
- `MATLAB/Comparison/visualControl_comparison.m` — the `bestParamFile` loader block (was L33–45) replaced with plain `clearvars -except MC_SEED CTRL_SEL TRAJ_TYPE c ctrl_list ctrl_names trajType all_results` (MC_SEED hoist semantics preserved)
- `MATLAB/Sweeps/sweep_deep.m` — `tmp_bp = load(...); K = tmp_bp.K;` removed; `Constants;` reordered BEFORE `InitGains_Comparison;` so `K.p_10` exists when the Comparison gain file reads it
- All four now source `K.p_10` uniformly from `Constants.m`

**How to apply:**
- Do NOT reintroduce `bestParam.mat` or any loader in any harness. If a new harness needs `K.p_10`, call `Constants;` first.
- `Adapt_Control_Params.m:4` still has a commented-out `% load("bestParam.mat");` line — harmless.
- If you need gain values inspectable from a standalone tool (e.g., reviewer-facing), read `InitGains_Comparison.m` directly — that file is the single source of truth for locked MDF-ASMC gains.
