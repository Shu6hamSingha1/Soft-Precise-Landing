---
name: $\delta/\delta t$ denotes discrete-time forward-difference
description: Locked 2026-05-09. In main paper §III-A1's PID law $K_{rd}\frac{\delta\bar{\boldsymbol{r}}_\text{e}}{\delta t}$, the $\delta/\delta t$ notation is INTENTIONAL — it denotes a discrete-time forward-difference (since the implementation runs at $\Delta t=0.01$~s). Do not "fix" this to $d/dt$. The mixed notation in the equation (continuous-time $\dot{}$, $\int$, plus discrete-time $\delta/\delta t$) reflects the implementation reality where the PID derivative term is computed by finite differences while the integral and the LHS retain continuous-time analytical form.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Rule

**Don't replace $\delta/\delta t$ with $d/dt$** in the Virtual Image Point PID law (main paper §III-A1, eq `s_e_dot_d`).

## Why

The user clarified 2026-05-09: *"$\delta/\delta t$ is meant to denote a discrete-time forward-difference"*. The implementation runs at $\Delta t=0.01$~s, so the PID derivative term is computed via forward-difference, not analytical differentiation.

## How to apply

1. **Don't flag this as an inconsistency**: the mixed notation ($\dot{}$ on LHS, $\int$ for integral, $\delta/\delta t$ for derivative) is intentional — it captures the analytical-form sliding-mode setup where the derivative is approximated discretely while the rest is treated continuously.

2. **If reviewing**: skip this item. Don't propose changing $\delta/\delta t$ to $d/dt$.

3. **Other locations**: the same convention may apply elsewhere if discrete-time forward-difference is the intended semantics. Check before suggesting changes.

## Related conventions

- `feedback_short_sentences_no_colons_no_emdash.md` — paper-wide style.
