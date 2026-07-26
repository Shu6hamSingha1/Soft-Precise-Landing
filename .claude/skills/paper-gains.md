---
name: paper-gains
description: Extract published gains from a reference paper and diff them against the current values in InitGains_Comparison.m. Surfaces drift between what we're running and what the paper actually used.
---

# /paper-gains — baseline gain fidelity audit

Usage:
- `/paper-gains` — audit all 4 baselines (Lin 2022, Zhang 2026, Lin 2023, Cho 2022)
- `/paper-gains Lin` — audit a single controller by author surname
- `/paper-gains fix` — after audit, propose per-row Edits to `InitGains_Comparison.m` to restore paper values; confirm with user before applying

## What to extract from each paper

For each baseline, grep the PDF (via `pymupdf`/`fitz`, not pdftotext — drops math diacritics) for:
- **Controller gains** — named gain matrices/scalars (`k1`, `k2`, `Kc1..5`, `kr`, `lambda_IBVS`, `rho_inf`, `l_p/l_v`, `Kv`, etc.)
- **Simulation section tables** — usually "Table II/III" or "simulation parameters"
- **Initial conditions** (depth estimate `ẑ*(0)`, reference height `zstar0`, desired features)
- **Saturation limits** if published
- Note which **simulation case** / **experiment** the gains come from; papers often publish multiple tunes

Ignore experiment-only values unless simulation values are absent — our harness is a simulation, so the sim tune is the fair comparison.

## Diff rules

Side-by-side per gain:
- **Match**: our value equals paper within 1e-3 — print `OK`
- **Drift**: differs — print `DRIFT: paper=X, ours=Y, ratio=Y/X`
- **Translated**: we use a sign-flipped or rescaled value for NED/downward-camera convention — recognize and mark `OK (sign-flipped NED)` rather than flagging drift
- **Not wired**: paper gain exists but our controller file doesn't use it (e.g. Zhang `Kc4/Kc5` — shared SO(3) inner loop replaces paper's attitude law). Mark `NOT WIRED (shared SO3)`; do not flag.
- **Justified deviation**: if `InitGains_Comparison.m` has a code comment explaining why we deviate (e.g. Chen `kr=1.0: paper 8 was too aggressive under our noise`), surface the comment in the diff as context.

## Output format

```
==================== Lin 2022 ====================
k1             paper=4.5   ours=4.5   OK
k2             paper=2.0   ours=2.0   OK
rho_inf_p      paper=0.05  ours=0.05  OK
l_p            paper=0.5   ours=0.5   OK
...
==================== Zhang 2026 ====================
Kc1            paper=diag(0.2,0.2,0.6)   ours=diag(0.2,0.2,0.6)   OK
Kc4            NOT WIRED (shared SO3)
omega_AFm      paper=1.0   ours=1.0   OK
...
```

End with a one-line verdict per controller: `FAITHFUL` (no drifts) / `PARTIAL (N drifts, M justified)` / `DIVERGED (N unjustified drifts)`.

## Fix mode

When invoked with `fix`, do not edit blindly. For each unjustified drift:
1. Propose a single `Edit` operation that restores the paper value
2. Show old→new
3. Wait for user confirmation before applying

Never fix a drift that has a justification comment in the code — those are deliberate. Surface them and stop.

## Key references

- `L:/Claude/Soft Landing/References/Lin 2022.pdf` — p7, after "parameters of the controllers were set"
- `L:/Claude/Soft Landing/References/Zhang 2026.pdf` — Table III (p8), Table II for conditions
- `L:/Claude/Soft Landing/References/Lin 2023.pdf` — robust circle-feature IBVS landing on a moving vehicle (\S~IV sim/experiments)
- `L:/Claude/Soft Landing/References/Cho 2022.pdf` — Table 2 (sim, p13), Table 3 (exp, p14)

## When not to use

- Do not run this for MDF-ASMC — there is no "paper value" to compare to, the manuscript is the source
- Do not run during an active retuning loop where drift is intentional — wait for the tune to settle first
