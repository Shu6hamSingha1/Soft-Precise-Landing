---
name: Zhang2026 baseline failure mode — precise, not soft
description: Locked 2026-05-07. Zhang2026 (PBVS + adaptive disturbance observer) reaches the target surface on Cases 2-5 in the comparison study and on every (traj,λ) pair in the multi-speed sweep, but its binding failure is the precise threshold (terminal lateral error 0.17-0.24 m on Cases 2-5 of Table IV; 0.05-0.47 m across the 20-run multi-speed sweep). v_f is within 0.20 m/s on Cases 2-4 and on Linear/Sinusoidal/Lissajous λ runs; the soft bound only breaks on Circular (fastest target). Earlier §IV-D claims of "v_f = 0.56-0.66 m/s on four moving cases" or "0.67-0.85 m/s in speed sweep" are fabrications and have been removed.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Validated numbers (2026-05-07 audit)

### Comparison study (Table IV, IC$_2$, 5 cases)
| Case | $r_{xy,\text{f}}$ [m] | $v_\text{f}$ [m/s] | $r_{z,\text{f}}$ [m] | Notes |
|---|---|---|---|---|
| 1 Static | 2.696 | 0.206 | 5.05 | crashed at $t=0.47$s |
| 2 Linear | 0.189 | 0.088 | 0.20 | precise FAIL, soft OK |
| 3 Sinusoidal | 0.235 | 0.025 | 0.20 | precise FAIL, soft OK |
| 4 Lissajous | 0.184 | 0.016 | 0.20 | precise FAIL, soft OK |
| 5 Circular | 0.174 | 0.648 | 0.20 | precise FAIL, soft FAIL |

### Multi-speed sweep (4 traj × 5 λ, IC$_1$)
- $r_{xy,\text{f}}$ range: 0.05-0.47 m (precise OK on 1/20 only — Sin λ=1.2 with $r_{xy,\text{f}}=0.053$).
- $v_\text{f}$ range: 0.004-0.883 m/s. Soft OK on 14/20 (Linear/Sin/Liss all soft-OK; Circular all soft-FAIL).
- $r_{z,\text{f}}$ range: 0.20-0.27 m. Reaches surface on 20/20.
- Soft-precise: 0/20 strict ($r_{z,\text{f}} \le 0.20$); 1/20 with 5 mm tolerance.

## Mechanism

Zhang2026 uses a fixed-bandwidth PBVS controller with adaptive disturbance observer. Terminal time is ~8.25 s consistently across λ — the descent timescale is fixed and **does not depend on lateral progress**. Lateral chase converges at the fixed bandwidth, leaving steady-state offset proportional to target velocity. As λ grows, the offset grows.

On Case 5 (Circular, fastest target heading), the offset is also large enough in velocity space to exceed the 0.20 m/s soft bound.

## What MDF-ASMC has that Zhang2026 lacks

The optic-flow funnel is **depth-aware**: $\beta = 1/\,^\mathcal{V} z_\text{t}$ scales with depth, so the controller accelerates as $z\to h_\text{lg}$. Lateral chase converges within the precise bound at touchdown, regardless of target velocity (within the [0.6, 1.4] envelope). This is the structural advantage cited in §IV-C and §IV-D.

## How to apply

1. **Never claim Zhang2026's failure mode is "soft-touchdown criterion violation"** outside Case 5 / Circular runs. Its real failure mode on Cases 2-4 (and Linear/Sin/Liss multi-speed) is **precise**.
2. **Never use the v_f range "0.56-0.66 m/s" or "0.67-0.85 m/s"** — both are fabrications from earlier draft passes.
3. The **canonical Zhang2026 numbers** for §IV-D are in the tables above. If retuning the harness changes them, re-validate.

## Related conventions

- `project_baseline_final_results.md` — overall baseline retune record.
- `project_baseline_paper_gains_failure.md` — why baselines need best-effort gains.
- `project_chen2025_limitation.md` — analogous structural-failure record for Chen2025.
- `feedback_validate_critiques_against_cited_works.md` — sweeping claims about cited works must be checked against data.
