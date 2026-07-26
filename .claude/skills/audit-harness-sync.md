---
name: audit-harness-sync
description: Audit sync between the probe/sweep harnesses (sweep_deep.m, validate_combo2.m, any ad-hoc probe) and the ground-truth multi-init harness (multi_Init_Var.m). Catches IC-list drift, trajList drift, cfg_override drift, and seed-injection drift — the exact failure modes that caused the 2026-04-16 COMBO_C regression to pass the probe but fail multi-init on the [2,-2,-5] off-axis IC under realistic disturbances.
---

# /audit-harness-sync

Compare every sweep/probe harness against `multi_Init_Var.m` (the ground truth) and report drift.

## Usage

```
/audit-harness-sync                      -- audit sweep_deep.m + any validate_*.m in sweeps/
/audit-harness-sync <file>               -- audit a single harness file
```

## What to check

`multi_Init_Var.m` is the authoritative gate. Every lighter harness that feeds a relock decision must be a strict subset of (or identical to) it on these five dimensions:

| Dim | Ground truth | How to check |
|---|---|---|
| **IC list** | `[0,0,-5; 2,2,-5; 2,-2,-5; 2,2,-7; 2,2,-3]` | Grep the harness for its `p0 = [...]` block and diff. The off-axis asymmetric `[2,-2,-5]` is the one that's historically dropped — always verify it's present. |
| **trajList** | `["Static","Linear","Sinusoidal","Lissajous","Circular"]` | Static is the one historically dropped from sweeps. Its absence hides a whole failure class. |
| **cfg_override** | Both noiseless `(NOISE=0,GE=0,delay=0)` and realistic `(NOISE=1,GE=1,delay=1)` | Any harness passing `[]` / nothing inherits `InitVar.m` defaults — auditable only by reading that file. Require explicit cfg_override. |
| **seed injection** | `run_simulation(..., 1000+k)` (6th arg) | Older harnesses use internal `rng(1000+k)` before the call. Both yield deterministic results but on different code paths; prefer the 6th-arg form. |
| **success criterion** | `tmp.success` (precise ≤ 0.08 m AND soft ≤ 0.20 m/s) | Confirm no harness redefines success locally — they must all consume `run_simulation`'s flag. |

## Output format

```
Auditing: sweep_deep.m vs multi_Init_Var.m
  IC list           OK  (5/5 ICs match)
  trajList          DRIFT  missing: ["Static"]
  cfg_override      DRIFT  uses InitVar defaults (implicit realistic only)
  seed injection    DRIFT  uses rng() inside run_5ic, not 6th arg
  success           OK
  → 3 drifts; fix before trusting this harness for relock decisions
```

## Why this skill exists

The 2026-04-16 COMBO_C relock passed `validate_combo2.m` (20/20 × 4 trajs) but failed `multi_Init_Var.m` (23/25 realistic) because:

- `validate_combo2.m` inherited its IC set from `sweep_deep.m`, which was missing `[2,-2,-5]` — the exact IC where Static/Lissajous realistic failed under the new gains.
- Neither sweep nor probe tested Static at all.
- Neither tested noiseless, so noiseless regressions would also be invisible.

Post-fix, `sweep_deep.m` was synced to multi-init's 5 ICs + Static + explicit cfg_override. This skill encodes the audit so future probes can't drift back into the same failure mode.

## Paired skill

After running this audit and fixing drift, re-run the probe, then run `/relock-gains` — which itself calls the multi-init harness as step 4 as the real gate. The two skills together form: *audit → probe → relock cascade → manuscript*.
