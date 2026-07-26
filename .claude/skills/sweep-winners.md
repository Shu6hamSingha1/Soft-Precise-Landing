---
name: sweep-winners
description: Parse a deep/speed sweep log (sweep_deep_rerun.log or similar) and extract strict dual-Pareto winners, single-metric champions, and re-lock candidates. Replaces the manual log scroll → winner table that every tuning round currently redoes by hand.
---

# /sweep-winners

Parse a sweep log and produce a decision-ready winners table.

## Usage

```
/sweep-winners                               -- default: sweep_deep_rerun.log
/sweep-winners <log-path>                    -- custom log (e.g. sweep_speed.log)
/sweep-winners <log-path> strict             -- strict dual-Pareto only (both t and xy better)
/sweep-winners <log-path> top 5              -- top 5 by each axis (t, xy, combined score)
```

All paths are relative to `MATLAB/Sweeps/`.

## What it extracts

From the `CROSS-TRAJ SUMMARY` section:
1. **Baseline row** — `aggT`, `maxXY`, `minL`
2. **Candidate rows** — `(param, grp, idx, mult, value, minL, aggT, maxXY)` for every `*` row
3. Derives `%ΔT`, `%ΔXY` vs baseline

## Output format

```
Baseline: 20/20 land, aggT=18.052 s, maxXY=0.0820 m

STRICT DUAL-PARETO (5/5 × 4 trajs, both t and xy strictly better)
  param          idx  mult  value      aggT      ΔT%    maxXY    ΔXY%
  ----------------------------------------------------------------------
  zp             0    1.50  9.0        17.74   -1.7%   0.0555  -32.3%
  Gamma          3    1.50  1.125      12.94  -28.3%   0.0782   -4.6%
  ...

TOP BY TIME            TOP BY PRECISION        TOP BY COMBINED SCORE
  1. p_2inf(z) ×0.5     1. zp ×1.50             1. Gamma(z) ×1.50
  2. Gamma(z)  ×1.5     2. Gamma(x) ×1.50       2. zp ×1.50
  ...
```

Combined score = `0.5 × (%ΔT + %ΔXY)` — a simple balanced metric for ranking.

## Decision rules for the caller

After extracting:
- **≥3 strict dual-Pareto cells on orthogonal parameters** → recommend a combo probe (`validate_combo2.m` pattern); don't re-lock yet.
- **Exactly 1 strict dual-Pareto cell** → safe to re-lock that single knob — UNLESS it is `zp` or `zd`.
- **0 strict dual-Pareto** but single-axis champions present → flag trade-off; defer to user for priority (time vs precision).
- **0 candidates at all** → baseline is locally Pareto-optimal; reconcile tex and move on.

## Paired-gain constraints (do not tune alone)

- **`zp` and `zd` are an outer-PID P+D pair.** Any sweep cell that lights up `zp` alone must be paired with a matched `zd` bump before re-locking — bumping zp solo sharpens position but re-breaks the soft margin (confirmed 2026-04-19 session: Combo D = zp×1.5 + zd×1.25 is the locked pattern). Same rule in reverse: don't bump `zd` without `zp`.
- **Cross-cluster `Γ(2)` conflicts with paired zp/zd bumps.** If a combo already includes zp×1.5 + zd×1.25, adding `Gamma(2)×1.5` re-breaks soft margin (Combo E failed for this reason). Treat Γ(2) as orthogonal-only, not additive.

## Pitfalls

- The `*` marker in the log uses the loose OR criterion (`t better OR xy better`). Strict dual-Pareto requires re-checking both deltas — do not trust the asterisks blindly.
- Rows with `minL < 5/5` must be excluded even if their numbers look good. A 4/5 row with a great `aggT` is noise from dropping the hardest IC.
- `value` column in the log is the scaled entry for 1-based `idx`, or the scalar for `idx=0`. When reporting, say `param(idx)` not just `param`.
- Multiple `*` rows per param (e.g. both `×1.25` and `×1.50`) → pick the stronger one for the table, list the weaker as "also admissible" in a footnote.

## Why this skill exists

Every tuning round, we run `sweep_deep.m` → 30 params × 4 mults → 120-row log. Manually scrolling it for Pareto winners takes ~10 min and is error-prone (asterisks use the loose criterion; I misread them in the 2026-04-14 session). This skill is a 30-second deterministic extraction.
