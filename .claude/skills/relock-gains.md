---
name: relock-gains
description: Orchestrate the full MDF-ASMC gain re-lock cascade after a sweep has identified winners. Walks the ordered dependency chain (source-of-truth gain file → bestParam.mat → multi-init rerun → speed sweep → Monte Carlo → figures → manuscript tex reconciliation) so nothing is missed. Use ONLY after a probe (validate_combo2 / validate_candidates) has confirmed the new gain set holds 20/20 × 4 trajs with strict improvement over baseline.
---

# /relock-gains

End-to-end re-lock orchestration. Assumes the user has already run a probe confirming the new gain set.

## Usage

```
/relock-gains                         -- interactive: walk every step, pause for confirmation
/relock-gains dry-run                 -- print the plan (edits + reruns) without applying
/relock-gains step <n>                -- execute a single step (1-7) — use to resume after a failure
```

## The ordered chain

Steps MUST be executed in this order. Skipping or reordering silently desyncs the project.

### Step 1 — Update `K_PLASMC` in `MATLAB/Comparison/InitGains_Comparison.m`

This is the **source of truth** for every harness (sweep_deep, validate_combo, multi_init, run_comparison). Edit only the specific fields that changed. Leave unchanged fields alone.

Record the exact old→new diff in a commit-message draft.

### Step 2 — Regenerate `MATLAB/Common/bestParam.mat`

Run `MATLAB/Common/regen_bestParam.m` so `K.p_10` etc. match `K_PLASMC`. All three harnesses read from this file — if step 1 is committed without step 2, the sweep will quietly use stale values because of the auto-sync block in `sweep_deep.m`.

**Pitfall**: `sweep_deep.m`'s auto-sync block pulls from `K_PLASMC` directly (not `bestParam.mat`), but `visualControl_IBVS_adaptive.m` reads from `bestParam.mat`. Both must match.

### Step 3 — Invoke `/sync-gains` audit

Confirms parity across:
- `run_simulation.m`
- `visualControl_IBVS_adaptive.m`
- `InitGains_Comparison.m`
- `bestParam.mat`

Plus non-gain constants (`izeta_2_max`, `att_cone`, precision criterion). Abort the cascade if `/sync-gains` reports any drift — fix at its source before proceeding.

### Step 4 — Rerun multi-init 50/50

Run `multi_Init_Var.m` (5 trajs × 5 idealistic ICs + 5 realistic ICs = 50 runs). Required before any manuscript table update — Table V is sourced from `Datasets/<traj>_multi_init.mat`. Abort cascade if land rate drops below the previous lock (50/50 → anything less than 50/50 on any realistic-subset traj).

### Step 5 — Rerun speed sweep

`MATLAB/Multi_init_cond/sweeps/sweep_speed.m`. The per-trajectory max speed multiplier is part of the robustness story (manuscript §IV). If any traj's ceiling drops vs the prior lock, decide with the user whether to revert or accept a regression.

### Step 6 — Rerun Monte Carlo

`MATLAB/Comparison/run_monte_carlo.m` — 5 trajs × 5 ctrls × 5 seeds = 125 runs (~25 min). Produces `Datasets/monte_carlo.mat` which drives the std-dev columns of the comparison table. All 5 controllers must be rerun, not just MDF-ASMC — the noise realizations for the baselines are seed-dependent too, and a selective rerun breaks the apples-to-apples comparison.

### Step 7 — Regenerate figures

Python figure scripts under `Soft_Precise_Landing/Figures/generated/`:
- `fig_multi_init.py`
- `fig_comparison.py`
- `fig_sweep_deep.py` (if it exists)
- any traj-specific scripts the latest draft pulls

Check the manuscript `\includegraphics` list to see which PDFs are currently referenced — only regenerate those, don't regenerate orphaned figures.

### Step 8 — Reconcile manuscript tex

Invoke `/refresh-tables` then manually reconcile:
- `results.tex:144` — pre-committed deep-sweep numbers (known stale, see `project_tex_reconciliation_pending.md`)
- Any Table V / Table VIII cells that moved
- §IV prose that quotes specific percentages
- Supplemental S4 (deep-sweep discussion)

Invoke `/verify-tex` for reference/citation/brace integrity before declaring done.

## Commit checkpoints

Commit after steps 3, 5, 7, and 8 — not per-step, but at logical milestones:

1. **After step 3**: "Relock MDF-ASMC gains: <summary>" (source files + bestParam.mat + audit clean)
2. **After step 5**: "Rerun multi-init + speed sweep on new lock"
3. **After step 7**: "Regenerate figures for new lock"
4. **After step 8**: "Reconcile manuscript tables/prose for new lock"

This keeps each commit atomic and revertible if a downstream step reveals the lock was wrong.

## Abort conditions

Stop the cascade immediately if:
- `/sync-gains` reports drift after step 2 (indicates a file was missed)
- Multi-init land rate drops below the prior lock on any traj (new gains are worse than probe suggested)
- Speed-sweep ceiling drops on >1 traj (regression in robustness envelope)
- Monte Carlo std-devs increase by >2× on any cell (noise sensitivity introduced)

On abort: revert the gain edit (step 1), regenerate bestParam.mat (step 2), rerun `/sync-gains`, and report which metric killed the lock.

## Why this skill exists

The 2026-04-15 lock-in session shipped with `izeta_2_max` drifted because a manual audit missed one call site. The 2026-04-16 session had `sweep_deep.m` operating on stale bases because its hardcoded `param_list` column 2 was never synced to `K_PLASMC`. Both failures were ordering/sync mistakes, not judgment mistakes. This skill encodes the ordering so future relocks don't repeat them.
