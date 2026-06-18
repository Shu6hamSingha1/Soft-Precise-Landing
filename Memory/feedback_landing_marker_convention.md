---
name: 3D-plot landing marker convention — 5-category outcome scheme
description: All 3D figures that show UAV touchdowns use a 5-category marker scheme distinguishing soft-precise (▲ filled triangle), precise-only (◇ open diamond), soft-only (○ open circle), touched-down-but-neither (▽ open down-triangle), and failed-to-reach-surface (× cross). Classification reads `run.success`, `run.precise`, `run.soft` from the MATLAB `result` struct (NOT reconstructed from `data` time series). Locked 2026-05-07, replacing the earlier 2026-04-30 2-marker scheme.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## The rule (locked 2026-05-07)

5-category marker scheme based on touchdown outcome:

| Outcome | Condition | matplotlib marker | Fill | Caption symbol |
|---|---|---|---|---|
| **soft-precise** | `success && precise && soft` | `'^'` | filled (`color=color`) | `$\blacktriangle$` |
| **precise only** | `success && precise && !soft` | `'D'` | unfilled (`facecolors='none', edgecolors=color`) | `$\Diamond$` |
| **soft only** | `success && !precise && soft` | `'o'` | unfilled | `$\bigcirc$` |
| **touched down, neither** | `success && !precise && !soft` | `'v'` | unfilled | `$\bigtriangledown$` |
| **failed to reach surface** | `!success` (FoV break or time-out) | `'x'` | n/a | `$\times$` |

The start-of-trajectory marker remains a circle (`marker='o'`, filled) — unchanged.

## Source of truth — MATLAB result struct flags

Verified 2026-05-07. The `.mat` files saved by `run_simulation.m` (multi-init, speed-envelope) and `visualControl_comparison.m` (comparison) carry the following **at the run/result struct level** (NOT inside `data`):

```matlab
% From run_simulation.m L638-647:
result.success     = landed && ~fov_fail;
result.final_xy    = norm(I_p_c(1:2) - x_t(1:2,idx));
result.final_alt   = abs(I_p_c(3) - x_t(3,idx));
result.final_rel_vel = norm(I_v_c - dx_t(1:3,idx));   % 3-D speed
result.precise     = result.success && (result.final_xy <= 0.08);
result.soft        = result.success && (result.final_rel_vel <= 0.2);
result.fov_fail    = fov_fail;
result.fov_fail_t  = fov_fail_t;
result.data        = load(scratch);   % time series goes inside .data
```

Python access pattern in plot scripts:

```python
m = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
results = m["results"]
for run in results:
    success = bool(getattr(run, "success", False))
    precise = bool(getattr(run, "precise", False))
    soft    = bool(getattr(run, "soft",    False))
    d       = run.data        # time-series for trajectory plotting
```

**Important:** `precise` and `soft` are **already AND-ed with `success`** in MATLAB. So `run.precise` is `True` ONLY if the run touched down AND xy_err ≤ 0.08. A failed-to-reach-surface run has `success=False, precise=False, soft=False` — drop into the `'x'` bucket.

## Classification helper

```python
def _classify_outcome(run):
    """Return (marker, filled) tuple for the 5-category scheme."""
    if not bool(getattr(run, "success", False)):
        return ('x', True)            # × failed to reach target surface
    p = bool(getattr(run, "precise", False))
    s = bool(getattr(run, "soft",    False))
    if   p and s:   return ('^', True)    # ▲ soft-precise
    elif p:         return ('D', False)   # ◇ precise only
    elif s:         return ('o', False)   # ○ soft only
    else:           return ('v', False)   # ▽ neither, touched down
```

Pass the **`run` object** (not `run.data`) to this helper, since the flags live at the `run` level.

## Marker rendering

```python
marker, filled = _classify_outcome(run)
if filled:
    ax.scatter(X[0,-1], X[1,-1], -X[2,-1],
               color=color, marker=marker, s=30)
else:
    ax.scatter(X[0,-1], X[1,-1], -X[2,-1],
               facecolors='none', edgecolors=color, marker=marker,
               s=30, linewidths=1.0)
```

For `'x'`, `filled=True` but `'x'` has no fill concept in matplotlib — the `color=color` form just sets the line/edge colour.

## Caption convention (locked 2026-05-07)

Use the compact form:
> "Touchdown markers: $\blacktriangle$ soft-precise; $\Diamond$ precise only; $\bigcirc$ soft only; $\bigtriangledown$ touched down but missed both; $\times$ failed to reach target surface."

This replaces the old 2-marker line: *"Soft-precise touchdowns are triangles ($\blacktriangle$), crosses ($\times$) otherwise."*

Do NOT repeat the numerical thresholds (0.08 m, 0.20 m/s) inside captions — the threshold definitions live in §IV.A and stay there.

## Affected scripts (current)

- `scripts/make_multi_init_plots.py` — helper currently `_is_soft_precise`; rename to `_classify_outcome` per the helper above. Multiple call sites (`plot_3d`, `plot_image_plane`, possibly `plot_combined`).
- `scripts/make_multi_speed_plots.py` — single call site in `plot_multi_speed`.
- `scripts/make_comparison_plots.py` — inline check (no helper) at two sites: comparison loop in main figure + per-baseline subplot loop. Add the helper or refactor.
- `scripts/make_comparison_multi_speed_plots.py` — helper `is_soft_precise`; rename + extend.

The constants `PRECISE_XY_M = 0.08`, `SOFT_V_REL_MPS = 0.20`, `Z_F_M = 0.20` at the top of each script remain — but the new helper does not need them, since classification reads MATLAB-saved flags directly.

## Why source-of-truth flags beat reconstruction

Earlier (2026-04-30) the helper recomputed `xy_err`, `v_rel` from `d.X_DS[:, -1]` and compared against thresholds. That worked but had two failure modes:
1. **Off-by-one indexing** — `idx` vs `idx-1` ambiguity (per `project_indexing_convention.md`); reconstruction sometimes used the wrong sample.
2. **No FoV-fail signal** — couldn't distinguish "touched down but missed both criteria" (zhang2026 case) from "failed to reach target surface" (lin/chen/cho FoV break) without re-checking the z-gap. The 5-category scheme makes this distinction load-bearing.

The MATLAB `result.success`, `result.precise`, `result.soft` flags are computed at the authoritative termination sample inside MATLAB and survive the .mat round-trip. Reading them directly is correct by construction.

## Affected captions (paper-wide)

**`results.tex`:**
- Fig. 3 (multi-init Case 5 combined) L95
- Fig. 4 (speed envelope) L110
- Fig. 5 (comparison combined) L181
- Fig. 6 (comparison multi-speed Case 5) L188

**`supplemental.tex`:**
- §S3-A multi-init Cases 1–4 (4 captions, L271, L278, L285, L292)
- §S3-G comparison multi-speed Cases 2–4 (3 captions — verify on regen)

## Outcome distribution by sweep (expected)

- **Multi-init MDF-ASMC (5×5=25 runs):** 25 ▲, 0 others (all soft-precise).
- **Speed-envelope MDF-ASMC (4×5=20 runs):** 20 ▲, 0 others.
- **5-controller comparison (5×5=25 runs):**
  - MDF-ASMC: 5 ▲
  - lin2022: 5 × (FoV break)
  - zhang2026: 1 × (Case 1 fails) + 4 ▽ (Cases 2–5: touched down, neither precise nor soft per Table IV xy_err > 0.08 AND v_rel > 0.20)
  - chen2025: 5 × (stalls above target surface, †)
  - cho2022: 5 × (immediate FoV break)
- **Comparison multi-speed (4 baselines × 4 traj × 5 λ = 80 runs):** 0 ▲, 0 ◇, 0 ○ per the §IV-D narration claim "0/80 soft-precise". Mostly × (lin/chen/cho FoV break) and ▽ (zhang2026 touched down but neither).

## Related conventions

- `feedback_landing_corridor_convention.md` — shaded box around target trajectory; new markers sit *on* the corridor for ▲, *near* the top face for ◇/○/▽, *above* for × (which fails to reach the corridor).
- `feedback_figure_label_sync.md` — when scripts/PDFs are regenerated, captions must stay in sync.
- `project_indexing_convention.md` — confirms the `idx`/`idx-1` ambiguity that motivated reading flags directly.

## Regen procedure (after script edits)

```bash
cd "L:/Claude/Soft Landing"
PYTHONIOENCODING=utf-8 python scripts/make_multi_init_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_multi_speed_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_comparison_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_comparison_multi_speed_plots.py
```

Each script writes PDFs under `Soft_Precise_Landing/Figures/generated/`.
