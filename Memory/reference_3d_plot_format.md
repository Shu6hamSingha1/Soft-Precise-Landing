---
name: Uniform 3D-plot format across all plot scripts
description: All matplotlib 3D plot scripts (make_multi_init_plots.py, make_multi_speed_plots.py, make_comparison_plots.py) share a uniform tick / label / title format. Locked 2026-04-26.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The format (locked 2026-04-26):**

Every 3D `Axes3D` instance produced by the plot scripts uses these settings:

```python
ax.set_xlabel(r"$\,^\mathcal{I}x$ [m]", labelpad=2)
ax.set_ylabel(r"$\,^\mathcal{I}y$ [m]", labelpad=2)
ax.set_zlabel("altitude [m]",            labelpad=2)
ax.locator_params(axis="x", nbins=4)
ax.locator_params(axis="y", nbins=4)
ax.locator_params(axis="z", nbins=4)
ax.tick_params(pad=1, labelsize=7)
ax.set_title(<title-string>, fontsize=8)
ax.view_init(elev=22, azim=-58)
```

**Why each value:**
- `labelpad=2`: positive padding so axis names don't overlap tick numbers (negative `-2` from earlier caused overlap on small subplots like Sinusoidal x ∈ [-0.5, 0.5]).
- `nbins=4`: caps tick density; small ranges with 6+ default ticks crowded each other in 4×4-inch subplots.
- `tick_params(pad=1)`: positive (was -2) — pulls tick numbers off the axis line.
- `tick_params(labelsize=7)`: 7-pt tick labels — matches the smaller subplot scale.
- `title fontsize=8`: ~20% smaller than 10-pt default; required for the longest velocity/trajectory equations (Case 5 Circular) to fit a 4.25-inch subplot. Multi-speed titles use 2-line layout `"<Case N: TrajName Target Trajectory>\n<velocity equation>"`.
- `view_init(elev=22, azim=-58)`: standard 3D viewing angle used across all scripts.

**How to apply:**
- When adding a new 3D plot script, copy this block verbatim. Don't re-tune individually.
- If a future plot has different axis labels (e.g., not in the inertial frame), still use `labelpad=2` and the locator/tick settings unchanged.
- For an obvious tradeoff: bump `nbins` to 5 only if a specific axis range genuinely needs 5 ticks AND the labels still fit at 7 pt.
- The 3D landing corridor (`draw_landing_corridor` helper) is a separate convention — see `feedback_landing_corridor_convention.md`.

**Source of changes:** Iterative tightening 2026-04-26 to fix x-axis tick label overlap in the Sinusoidal subplot of `plasmc_multi_speed_landing.pdf`. User then asked for the format to be uniform across all plots.

**Scripts using this format (as of 2026-04-26):**
- `scripts/make_multi_init_plots.py`
- `scripts/make_multi_speed_plots.py`
- `scripts/make_comparison_plots.py`
- `scripts/make_comparison_multi_speed_plots.py`

**Manuscript figure-width convention (added 2026-04-26):**
- Multi-init 3D plots (e.g., `Circular_3D.pdf`) use `width=0.8\columnwidth` in main paper Fig. 3 and supplement §S3-A; image-plane companions stay at `0.6\columnwidth`.
- Multi-speed plots (e.g., `plasmc_multi_speed_landing.pdf`, `comparison_multi_speed_*.pdf`) use `width=0.8\textwidth` in `figure*` blocks.
- `comparison_traj3d_*.pdf` plots use `width=\columnwidth`.
- Apply the same width on every supplement instance of a given plot type so the visual sequence is uniform across the document.

**Matplotlib 3D-axes pitfalls (added 2026-04-26):**
- Do NOT use `bbox_inches="tight"` for 3D plots in a multi-subplot figure — drops z-axis labels (matplotlib bbox detection is 2D-only). Use `pad_inches=0.05` instead.
- Do NOT use positive `pad` on `set_title` for 3D axes under `tight_layout` — shifts the subplot horizontally. Use `y=0.95` (axes-fraction) instead.
- Detail in `feedback_3d_matplotlib_gotchas.md`.
