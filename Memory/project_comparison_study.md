---
name: Comparison study status
description: 5-controller comparison — LOCKED as of 2026-04-16. All baselines best-effort retuned under shared robustness model.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
5-controller comparison (PLASMC, Lin 2022, Zhang 2026, Chen 2025, Cho 2022) is **locked** as of 2026-04-16 (commit 7fa0a02):

- Shared robustness model `MATLAB/Common/init_robustness.m` gates NOISE=1 in all 3 harnesses (run_simulation, visualControl_IBVS_adaptive, visualControl_comparison). Single source of truth — no drift between multi-init sweep, single-run PLASMC, and baseline comparison.
- Baselines retuned best-effort within each paper's published framework (paper values crash on IC=[2,2,-5] harness; see `project_baseline_paper_gains_failure.md`).
- Final per-trajectory numbers in `project_baseline_final_results.md`.

**Verdict:** PLASMC lands all 5 trajectories at 0.04–0.09m precision + soft v_z. Zhang 2026 is the strongest baseline (0.11–0.55m, moderate v_z). Lin/Chen/Cho fail on structural grounds, not tuning (each limitation documented).

**How to apply:** Do not retune baselines further — the structural ceiling is reached. Run `inspect_comparison` from `MATLAB/Comparison/` to regenerate telemetry if needed. For manuscript Tables V/VIII, pull numbers from `project_baseline_final_results.md`.
