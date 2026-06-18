---
name: matplotlib 3D-axes layout gotchas (z-labels, titles, tight_layout)
description: Three reproducible matplotlib quirks discovered while polishing the 3D plot scripts in 2026-04-26 — bbox_inches="tight" drops 3D z-axis labels; positive set_title pad triggers horizontal subplot shifts under tight_layout; subplots_adjust after tight_layout only modifies named parameters.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**Three reproducible matplotlib 3D-axes pitfalls (locked 2026-04-26):**

1. **`bbox_inches="tight"` drops 3D z-axis labels.** matplotlib's tight-bbox detection works on 2D Text-artist bboxes, but 3D z-axis labels (`set_zlabel`) are rendered as Text positioned in the 3D coordinate system; the bbox computation misses them, so the saved PDF crops the labels off — typically only on the *right* column of a 2×2 grid because the z-axis sits between the two columns and gets clipped by the gutter detection. **Fix**: drop `bbox_inches="tight"` from `fig.savefig` for 3D plots; use `pad_inches=0.05` or similar. Single-axes 3D plots are fine with `bbox_inches="tight"` (no inter-axes clipping); the bug bites only when multiple 3D subplots share a figure.

2. **Positive `pad` on 3D `set_title` shifts the entire subplot horizontally** under `fig.tight_layout`. matplotlib 3D titles report a non-trivial bbox extent that `tight_layout` interprets as *both* vertical and horizontal padding requirement, so increasing `pad` slides the subplot right (or left). **Fix**: use `y=...` to position the title (axes-fraction coords, ignored by tight_layout's bbox math), not `pad=...`. `y=1.0` puts title at the top edge; `y=0.95` pulls it 5% inside the axes area, eliminating the gap between title and 3D content.

3. **`subplots_adjust` after `tight_layout` only modifies named parameters** — the rest stay at whatever tight_layout computed. Useful for piecemeal overrides (e.g., set only `top=0.95` to reduce suptitle gap) but a trap if you forget to also set `wspace`/`hspace` and the inter-subplot gutter ends up wrong for 3D z-labels (see point 1).

**How to apply:**
- For any new 3D-plot script: `fig.savefig(path, pad_inches=0.05)` (no `bbox_inches`).
- For 3D subplot titles: `ax.set_title(..., fontsize=8, y=0.95)` — never `pad=` on a `tight_layout` figure.
- When tight_layout misbehaves, prefer explicit `fig.subplots_adjust(left=, right=, bottom=, top=, wspace=, hspace=)` with all 6 params named.

**Related pattern (locked):**
- 3D figure label/legend conventions live in `reference_3d_plot_format.md` (labelpad=2, nbins=4, tick_params(pad=1, labelsize=7), title fontsize=8, view_init(22,-58)).
- Soft-precise corridor visualization in `feedback_landing_corridor_convention.md`.

**Affected scripts (2026-04-26 polishing pass):**
- `scripts/make_multi_speed_plots.py` (legend at lower center, ncol=6; bbox_inches dropped)
- `scripts/make_comparison_multi_speed_plots.py` (suptitle layout, wspace adjustment)
- `scripts/make_multi_init_plots.py` (legend at right or bottom right; title `y=0.95`; image-plane inset)
- `scripts/make_comparison_plots.py` (legend at right, fontsize=6; same conventions)
