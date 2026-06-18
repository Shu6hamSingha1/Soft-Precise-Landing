---
name: comparison-reconciliation-history
description: "consolidated comparison-study reconciliation history (all RESOLVED 2026-04-16→22): Table IV swap, harness regression, fresh .mat, off-by-one"
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---

Consolidated history of the comparison-study (5 controllers × 5 trajectories) tex/harness reconciliation. All concerns RESOLVED across 2026-04-16 → 2026-04-22. Merges five point-in-time memories. Conventions below are verbatim and current.

**Canonical trajectory / Table order: `Linear, Sinusoidal, Lissajous, Circular`** (preceded by Static). Section IV-A defines Case 4=Lissajous, Case 5=Circular. Never insert Circular before Lissajous.

**Indexing / off-by-one convention (locked tex standard):** Tex Table IV uses the `idx-1` convention (the MATLAB "Landed at t=" printout); the Python aggregator reads `idx`. Off-by-one fix unified everything to **`t_f = (idx-1)*dt`, UAV state at `idx`, target state at `idx-1`**. Apply `idx-1` uniformly when refreshing tex from `.mat`. (See project_indexing_convention.md, project_termination_convention.md.)

## Harness consistency audit — all clear (2026-04-16)
Full harness consistency audit completed 2026-04-16; every analysis uses the same locked-in gains and updated ICs. Timestamps: bestParam.mat 04:31 (gains locked) → Multi-init 04:40 (25/25 landed) → Comparison 04:48 (5 controllers × 5 trajectories) → Deep sweep 13:24 (600 entries). Verified matches: Multi-init ICs = canonical `[[0,0,-5],[2,2,-5],[2,-2,-5],[2,2,-7],[2,2,-3]]`; Comparison IC = `[2,2,-5]` (IC2); deep-sweep ICs canonical; K_PLASMC in comparison = bestParam.mat (all 23 fields); comparison loads `../Common/bestParam.mat`; multi-init `cfg_override` only touches NOISE/GE/delay; shared `init_robustness.m` in Common/. (bestParam.mat later retired to Obsolete/Common/ — project_bestparam_location.md.)

## Tex reconciliation complete (2026-04-16)
All manuscript/supplemental tex reconciled with current `.mat`, locked gains, robustness model, ICs. Done: Table III gains (5 values), landing criterion 0.05→0.10 m, comparison table full rewrite, per-controller discussion (Lin/Zhang/Chen/Cho) rewritten with current numbers, deep-sweep paragraph generalized, supplement S3-D rewrite (42 Pareto/53 Fragile/15 Inert/10 Trade), speed envelope verified (8.38 cm Lissajous λ=1.0, 0.174 m/s Circular λ=1.4), all 29 figures regenerated, reference lines removed from comparison plot, IC label IC4→IC2. Verified current: abstract 25/25 / 7.3 cm / four baselines; 7.3 cm worst-case = actual 7.20 cm (Linear IC5); mean landing time ≤ 18.4 s = actual 18.20 s.

## Table IV Case 4/5 column-header swap — RESOLVED (2026-04-20)
**The Case 4/5 column-swap fix was applied across tex + MATLAB + Python.** An older `rerun_all_traj.m` (commit fb9349c, 2026-04-09) appended Circular at position 4 instead of 5 when extending 4→5 trajectories; that order propagated into Table IV headers, 10+ MATLAB drivers, and two Python plot scripts, while captions / multi-init Table V / Section IV-A stayed canonical — creating the contradiction. Fixed:
- `results.tex` — Table IV headers swapped AND data columns 4↔5 swapped across all five controller rows; prose updated ("worst-case … on Case 5" → "on Case 4", 0.021 m now under Case 4=Lissajous).
- MATLAB drivers: `Comparison/rerun_all_traj.m`, `Comparison/run_monte_carlo.m`, `Multi_init_cond/multi_speed_cond.m`, `Sweeps/sweep_speed.m`, `Sweeps/sweep_deep.m` (comment), `Sweeps/validate_{combo,combo2,gamma1_speed,kappa0_speed,candidates}.m` — all use `["Linear","Sinusoidal","Lissajous","Circular"]`.
- Python: `scripts/make_comparison_plots.py` L88, `scripts/make_multi_speed_plots.py` L47 (`make_multi_init_plots.py` already correct).

Data note: per-trajectory `.mat` files are keyed by NAME (`Lissajous_comparison.mat`, `Circular_comparison.mat`) so contents were never swapped — only the iteration order assigning each trajectory's numbers to a column/bar position. Canonical values: Case 4=Lissajous Proposed t_f≈23.48, xy≈0.023, z≈0.200; Case 5=Circular Proposed t_f≈23.12, xy≈0.018, z≈0.201 (`idx` convention); tex values `idx-1`: {23.47, 0.021, 0.200} / {23.11, 0.017, 0.199}.

## Comparison harness regression — FULLY RESOLVED (2026-04-20)
Fresh `*_comparison.mat` regenerated 2026-04-20 15:48; Proposed (MDF-ASMC) lands precise+soft on every trajectory (Static/Linear/Sin/Liss/Circ, t_f≈22.8–23.5 s, xy_e ≤ 2.3 cm). Validated against source: `Common/Constants.m` FILTER_WINDOW=11 in place since 2026-04-19 01:00 (all `.mat` post-date it); source diffs 2026-04-19 21:28–21:31 PURELY COSMETIC (`zp/zi/zd → rp/ri/rd` rename, same values, no re-run needed); all `.mat` reflect Approach 2 + Combo D. Harness stable — do not regenerate `.mat`s.

## results.tex staleness + hostile-reviewer MCs — RESOLVED (2026-04-22)
**All five Major Concerns (MC1–MC5) closed 2026-04-22; Table V + baseline prose reconciled with fresh `.mat`.**
- MC1 — deep-sweep classification rewritten to match harness output (33 axes / 132 combos / 3325 runs; classification table dropped; `results.tex:103`, `manuscript.tex:115`, supplement §S3-E); labels "Fragile/Inert" → "Load-bearing/Robust".
- MC2 — broken supplement cross-refs fixed.
- MC3 — pixel-box residual fixed in §III.
- MC4 — Table V data drift reconciled: comparison harness re-run 2026-04-22; Python aggregator off-by-one on `x_t[:,idx]`/`dx_t[:,idx]` → `[:,idx-1]` fixed; t_f unified to `(idx-1)*dt`. PLASMC shifts: Linear 0.016→0.005, Sinusoidal 0.021→0.016; baseline cells 3rd-decimal refresh; prose ranges updated (Lin t_f 3.53–16.43; Zhang Case 1 0.47 s; Chen xy_e upper 0.85; Cho t_f 0.35–1.67, xy_e 1.96–2.77); worst-case MDF-ASMC xy_e "on Cases 3–4" → "on Case 4".
- MC5 — PX4 deployability claim softened.

Infra cleanups same session: `bestParam.mat` + `regen_bestParam.m` retired to `Obsolete/Common/` (K.p_10 now sourced from `Constants.m` across all 4 consumers); 24+ stale `.mat`/`.m` files moved to `Obsolete/`; `Datasets_v1_preApproach2/` flattened.

**How to apply:** Tex/harness are current as of 2026-04-22. Before re-opening any staleness concern, compare current tex numbers against a fresh run of `scripts/_analyze_results_for_tex.py` — do not rely on specific numeric cells in memory, which decay on the next MATLAB re-run.
