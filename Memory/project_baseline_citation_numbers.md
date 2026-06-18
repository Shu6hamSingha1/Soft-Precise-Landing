---
name: Hard-coded baseline citation numbers in comparison figures
description: The 5-controller comparison figures embed IEEE citation numbers ("Baseline A [1]", "Baseline B [2]", etc.) directly in the figure legends/titles. These numbers are hard-coded in two Python plot scripts and MUST be updated whenever the bibliography order shifts (e.g., new cites added). Locked 2026-05-07.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Hard-coded numbers (as of 2026-05-10)

| Cite key | Main paper | Supplement |
|---|---|---|
| `lin2022`   | [1]  | [9]  |
| `zhang2026` | [2]  | [8]  |
| `chen2025`  | [10] | [10] |
| `cho2022`   | [9]  | [11] |

Main paper and supplement have separate IEEEtaes bibliographies, so the numbers differ.

## Where the numbers live

### `scripts/make_comparison_plots.py`
- `CTRL_DISPLAY_MAIN` dict — used by Plot H (`comparison_combined_circular.pdf`, main paper Fig. 5).
- `CTRL_DISPLAY_SUPP` dict — used by Plot F+ (`comparison_traj3d_combined.pdf`, supplement).

### `scripts/make_comparison_multi_speed_plots.py`
- `CTRL_TITLE_MAIN` dict — used for `comparison_multi_speed_circular.pdf` (main paper Fig. 6).
- `CTRL_TITLE_SUPP` dict — used for `comparison_multi_speed_{linear,sinusoidal,lissajous}.pdf` (supplement).
- Routing controlled by `TRAJ_IS_MAIN = {"Circular": True, ...}` flag dict.

## When to update — TRIGGER

If any of the following happens, the numbers above may shift:
1. **A new citation is added before any of `lin2022`/`zhang2026`/`chen2025`/`cho2022`** in the main paper or supplement.
2. **An existing citation is removed** in the main paper or supplement.
3. **A `\cite{}` is moved earlier in the document** (IEEE ordering is by first occurrence, not alphabetical).
4. **The bibliography style changes** (e.g., switch from IEEEtaes.bst to another).

## How to update

1. **Compile** `manuscript.tex` (main paper) and `supplemental.tex` (supplement).
2. **Read the rendered citation numbers** for `lin2022`, `zhang2026`, `chen2025`, `cho2022` from each compiled PDF.
3. **Update the four entries** in:
   - `CTRL_DISPLAY_MAIN` and `CTRL_DISPLAY_SUPP` in `make_comparison_plots.py`.
   - `CTRL_TITLE_MAIN` and `CTRL_TITLE_SUPP` in `make_comparison_multi_speed_plots.py`.
4. **Regenerate** the affected PDFs:
   ```bash
   cd "L:/Claude/Soft Landing"
   PYTHONIOENCODING=utf-8 python scripts/make_comparison_plots.py
   PYTHONIOENCODING=utf-8 python scripts/make_comparison_multi_speed_plots.py
   ```
5. **Update this memory** with the new numbers.

## How to detect drift

Search the rendered PDFs for "Baseline A [N]" (etc.) in the figure legends and verify that the bracketed N matches what \cite{lin2022} renders to in that PDF's reference list. If they disagree, the numbers have drifted and the dicts need updating.

## Why hard-coded (and not via `\cite{}` in matplotlib)

Matplotlib's mathtext does NOT support `\cite{}` (LaTeX citation commands). Real LaTeX rendering (`text.usetex=True`) would, but LaTeX is not on the user's PATH (verified 2026-05-07), so we fall back to mathtext + hard-coded numbers.

## Related conventions

- `feedback_landing_marker_convention.md` — 5-category outcome scheme for the same comparison figures.
- `feedback_figure_label_sync.md` — figure scripts must stay in sync with paper notation.
- `feedback_figure_title_case.md` — Title Case for figure titles (does not apply to bracketed citation numbers).
