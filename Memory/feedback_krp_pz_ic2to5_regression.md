---
name: K_rp=4 + P_z=2.5 default regression on IC2-5 (2026-05-25)
description: Defaults committed as K_rp 9→4 + P_z 5→2.5 produced 2nd ever SP at IC1 but caused 31-67% regression on IC2-5. Mechanism diagnosed: K_rp=4 has insufficient gain authority to drive 2m initial lateral offset to zero in 3s descent window. Project-rule revert pending user direction.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

> ⛔ **STAMP 2026-07-03: historical gain-era record** (K_rp/P_z parametrization predates the combined-surface defaults). Enduring lesson = the mandatory IC2-5 gate (core rule in ../MEMORY.md).
## What happened

Per user direction 2026-05-25, defaults were changed:
- `K_rp` (outer PID P-gain): 9.0 → 4.0 (commit `3503a84`)
- `P_z` (κ-ODE anti-windup Z-component): 5.0 → 2.5 (commit `c934dfa`)

Validation at IC1 (n=10) showed a 2nd-ever SP landing (rep 5, xy=0.026, vel=0.148).  But the mandatory IC2-5 validation (n=2 per IC) showed catastrophic regression:

| IC | OLD default | NEW default | Δ |
|---|---|---|---|
| IC2 (2,2,-5) | 1.64 m | 2.16 m | **+31%** |
| IC3 (-2,2,-5) | 1.79 m | 1.91 m | +7% |
| IC4 (2,2,-7) | 1.59 m | 2.65 m | **+67%** |
| IC5 (2,2,-3) | 2.88 m | 1.72 m | -40% |

All 8 IC2-5 reps landed 1.7-2.7 m off-target, vs the 0.03 m SP at IC1.

## Mechanism (diagnosed via per-rep trajectory analysis)

**Root cause: K_rp=4 has insufficient gain authority for large initial offsets.**

At IC1 the drone starts centered, so initial |s_e_n| ≈ 0.025.  At IC2 (2m offset), initial |s_e_n| ≈ 0.315 — 12× larger.  IC5 has |s_e_n| ≈ 0.6 — 24×.

The maximum commanded lateral acceleration `I_a_xy` is BOUNDED by the funnel envelope.  At K_rp=4:
- IC1: max I_a_xy = 3.95 m/s²  (sufficient — initial error is small)
- IC2-4: max I_a_xy = 2.0-2.4 m/s²  (insufficient — needed ~1.8 m/s² with no margin)

The arithmetic: to cover 2m laterally in 3s descent (accel-then-decel), peak required acceleration ≈ 1.8 m/s².  K_rp=4 delivers ~2.0 m/s² — no margin.  K_rp=9 would deliver ~4.5 m/s² — 2.5× margin.

The smoking gun: PID integral `|is_e_n_end|` at touchdown:
- IC1 SP: 0.021 (PID drove error to ~0, integral didn't accumulate)
- IC2-4: 1.0 to 1.8 (integrator fully wound — PID could not drive error to zero before landing)

**The outer PID literally runs out of time at K_rp=4 when there's a real offset to close.**

P_z reduction is secondary — vz_end is small across all ICs.  K_rp is the binding constraint.

## Why this validates the project rule

`feedback_ic_validation.md` explicitly warned: "IC1 improvements consistently regress off-center starts; `run_ic_validation.sh` is mandatory pre-merge."

K_rp=4 was tuned on IC1 (centered) where the gain reduction's only effect was slowing the already-near-zero pursuit, giving softer touchdown.  At IC2-5 the same reduction starved the loop of authority.  This is exactly the failure mode the rule was designed to catch.

## Status

Defaults remain at K_rp=4 + P_z=2.5 pending user direction.  Three remediation options on the table:

1. **Revert** to MATLAB defaults (K_rp=9, P_z=5).  Project-rule-compliant; loses IC1 SP gain.
2. **Gain scheduling**: K_rp=4 when |s_e_n|<0.1 (close-in), K_rp=9 when |s_e_n|>0.1.  Few-line change in controller.py.
3. **Keep current + accept IC2-5 regression**: only defensible if user prioritizes IC1 operationally and ignores IC2-5.

## How to apply

- Future tuning sessions: when reducing a gain that affects loop authority, ALWAYS validate at IC2-5 BEFORE merging.  This finding is the explicit illustration.
- Any future default change for K_rp or P or middle-loop gains needs the same gate.
- The IC1 SP rate of ~10% is consistent across configs (lucky-IC distribution); not worth IC2-5 trades.

## Data

- `~/ws/Test_Data/NewDefaults/20260525-015341/` — IC1 n=10 (SP at rep5)
- `~/ws/Test_Data/ICValidation/20260525-021929/` — IC2-5 n=2 each, all bad
- Diagnostic script ad-hoc, not committed
- Commits: `3503a84` (K_rp=4), `c934dfa` (P_z=2.5), `8f956b2` (N=10 validation log)
