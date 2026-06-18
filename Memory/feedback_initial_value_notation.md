---
name: Initial-value notation — parentheses for time-varying signals, subscript for design constants
description: Locked 2026-05-09. Time-varying signals evaluated at $t=0$ use parentheses ($\boldsymbol{\kappa}(0)$, $\boldsymbol{h}_\text{e}(0)$, $\kappa_\alpha(0)$); design constants that happen to be the "initial" funnel widths use subscripts ($\boldsymbol{p}_{1_0}$, $\boldsymbol{p}_{2_0}$). Don't write $\boldsymbol{\kappa}_0$ — the adaptive gain is time-varying.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

| Symbol | Form | Reason |
|---|---|---|
| Adaptive gain (translational) | $\boldsymbol{\kappa}(0)$ | Time-varying signal: $\boldsymbol{\kappa}(t)$ adapts via leakage law |
| Adaptive gain (yaw) | $\kappa_\alpha(0)$ | Time-varying signal: $\kappa_\alpha(t)$ adapts |
| Optic-flow error | $\boldsymbol{h}_\text{e}(0)$ | Time-varying signal evaluated at $t=0$ |
| Sliding surface | $\boldsymbol{\sigma}(0)$ | Time-varying signal |
| Funnel initial half-width | $\boldsymbol{p}_{2_0}$, $\boldsymbol{p}_{1_0}$ | Design constant (locked at design time) |
| Funnel terminal half-width | $\boldsymbol{p}_{2_\infty}$, $\boldsymbol{p}_{1_\infty}$ | Design constant |

## Why

The adaptive gain $\boldsymbol{\kappa}(t)$ is a time-varying state of the leakage-type adaptive law (main paper eq `adaptive law`). Its value at $t=0$ is one point on its trajectory — parenthesized-time form is correct.

By contrast, $\boldsymbol{p}_{2_0}$ is a fixed design parameter of the funnel envelope $\boldsymbol{p}_2(t)=(\boldsymbol{p}_{2_0}-\boldsymbol{p}_{2_\infty})e^{-\Xi_2 t}+\boldsymbol{p}_{2_\infty}$. The "0" subscript is part of the parameter name, not a function evaluation.

The supplement initially had mixed notation: Table~S1 used $\boldsymbol{\kappa}_0$ (subscript) while the yaw row used $\kappa_\alpha(0)$ (parens). Audit on 2026-05-09 normalized to $\boldsymbol{\kappa}(0)$ across Table~S1 (L309), §S3-A.2 prose (L332), and §S3-B prose (L379) to match main paper eq `adaptive law`.

## How to apply

1. **Drafting / editing**: when writing the initial value of an adaptive (or otherwise time-varying) gain, use $\boldsymbol{\kappa}(0)$ form, not $\boldsymbol{\kappa}_0$.
2. **Funnel envelope widths**: keep subscript form ($\boldsymbol{p}_{2_0}$, $\boldsymbol{p}_{2_\infty}$). Do NOT "normalize" these to $\boldsymbol{p}_2(0)$ — they are constants, not function evaluations.
3. **Reviewing**: if you spot $\boldsymbol{\kappa}_0$ in the manuscript, supplement, or figures, flag it.
4. **Drafts/* backups**: leave `Drafts/supplemental_v2.tex` and `_v3.tex` alone — they are version snapshots from earlier epochs.

## Related conventions

- `feedback_funnel_symbol_convention.md` — funnel symbols ($\boldsymbol{p}_1$, $\boldsymbol{p}_2$, $\xi_1$, $\Xi_2$).
- `feedback_notation_subscript_convention.md` — `_u` for plant inputs.
