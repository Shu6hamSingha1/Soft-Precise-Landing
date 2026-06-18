---
name: Per-trajectory baseline multi-speed comparison sweep
description: 80-run target-speed sweep (4 trajectories × 4 baselines × 5 λ multipliers) executed 2026-04-26. Data at MATLAB/Comparison/Datasets/<traj>_multi_speed_comparison.mat; PDFs at Soft_Precise_Landing/Figures/generated/comparison_multi_speed_<traj>.pdf. Confirmed: none of Lin/Zhang/Chen/Cho lands soft-precise at any λ on any moving trajectory.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The sweep (locked 2026-04-26):**

For Cases 2–5 (Linear, Sinusoidal, Lissajous, Circular), the four baseline controllers (`ctrls = [2 3 4 5]` = Lin 2022, Zhang 2026, Chen 2025, Cho 2022) were run at every speed multiplier `mults = [0.6, 0.8, 1.0, 1.2, 1.4]` from baseline IC = `[0,0,-5]` m, deterministic seed `MC_SEED = 1000`, full disturbance model (NOISE+GE+delay+wind+noise+ground effect).

**Data on disk:**
- `MATLAB/Comparison/Datasets/<traj>_multi_speed_comparison.mat` for traj ∈ {Linear, Sinusoidal, Lissajous, Circular}.
- 20 runs per file (4 baselines × 5 λ); fields `results`, `mults`, `ctrls`, `trajType`. Saved in `-v7` format (NOT `-v7.3`) so scipy.io.loadmat can read directly.

**Driver:** `MATLAB/Comparison/multi_speed_comparison.m`.
**Plotter:** `scripts/make_comparison_multi_speed_plots.py`.
**Output PDFs:** `Soft_Precise_Landing/Figures/generated/comparison_multi_speed_<traj>.pdf`.

**Key result:** zero soft-precise landings out of 80 baseline runs across the entire λ ∈ [0.6, 1.4] envelope. Compare with MDF-ASMC's 20/20 soft-precise touchdowns over the same λ range (`plasmc_multi_speed_landing.pdf`). Used in main-paper §IV-C item 3 to establish that the speed-robustness gap is structural, not a tuning artefact.

**Manuscript placement:**
- Main paper: Fig. 7 (`comparison_multi_speed_circular.pdf`) — Case 5 only.
- Supplement §S3-G: `comparison_multi_speed_{linear,sinusoidal,lissajous}.pdf`.

**Harness modifications required (2026-04-26):**
- `MATLAB/Comparison/visualControl_comparison.m`: added `SPEED_MULT` and `MS_STATE` to both `clearvars -except` lists (lines 34, 36); defaults `SPEED_MULT = 1.0` if unset; uses it in the `traj_Gen(...)` call.
- `MATLAB/Comparison/run_comparison.m`: added `SPEED_MULT` to inner-loop clearvars exceptions (lines 64, 66) so single-trajectory single-λ callers still work.

**Critical implementation note (clearvars wipe pattern):**
`visualControl_comparison.m` runs `clearvars -except <preservation list>` early, which wipes ANY workspace variable not in the list. Driver scripts that bundle state via local variables (e.g., `mu`, `cIdx`, `nC`, `nRuns`) will see those variables vanish after the call returns. Two consequences for any future driver:
1. **Bundle all driver state into a struct** (e.g., `MS_STATE`) and add THAT struct name to `visualControl_comparison`'s clearvars exception list.
2. **Outer-loop iteration variables get wiped mid-iteration** — MATLAB's for-loop only re-binds the iterator at iteration start, so accessing `cIdx` AFTER `visualControl_comparison` in the inner-loop body fails. Fix: assign `MS_STATE.cIdx = cIdx` ONCE per outer iteration (before the inner loop), not inside it.

This pattern is documented as a debugging trap because the failure mode is silent — the first run completes fine, then iteration 2 errors with "Unrecognized function or variable 'cIdx'".

**How to regenerate:**
```bash
# 1. Run the MATLAB driver (~80 runs, ~30 min wall-clock)
cd MATLAB/Comparison
matlab -batch "multi_speed_comparison"
# 2. Generate the 4 PDFs
cd ../..
python scripts/make_comparison_multi_speed_plots.py
```
