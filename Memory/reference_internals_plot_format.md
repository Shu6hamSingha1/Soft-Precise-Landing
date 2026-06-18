---
name: MDF-ASMC internals plot format (sup-sliding / sup-kappa / sup-thrust)
description: Locked 2026-05-10. Format pattern for column-width supplement time-series plots (sliding surface, adaptive gains, thrust+accel) — figsize 10.5×4.0/4.3, fonts 14-24pt to compensate for ~33% column-width scaling, fig.suptitle for figure-level title, subplots_adjust(wspace=0.3) for horizontal panel gap.
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
## Format spec

**Figure dims**:
- Single panel (sup-sliding): `figsize=(10.5, 4.0)`
- 1×2 panels (sup-kappa, sup-thrust): `figsize=(10.5, 4.3)`
- LaTeX placement: `\begin{figure}[!t]` + `\includegraphics[width=\columnwidth]` (column-width, ~33% effective scale)

**Font sizes** (chosen to render 4.6–7.9pt at column-width):
- Axis labels: `fontsize=20, labelpad=4`
- Tick labels: `tick_params(pad=1, labelsize=16)`
- Per-axes title: `fontsize=20`
- Legend: `fontsize=14`
- Figure suptitle: `fig.suptitle("...", fontsize=24, y=0.98 or 0.99)`

**Layout cleanup**:
- `locator_params(axis="x", nbins=4)` — sparse x-ticks
- `fig.tight_layout(pad=0.3)` — minimal subplot padding
- `fig.subplots_adjust(wspace=0.3)` — horizontal gap between panels (1×2 only)
- `fig.savefig(..., bbox_inches="tight", pad_inches=0.02)` — tight outer margins

**Legend placement** (when curves dominate the plot area):
- Inside legend with `framealpha=0.85` to visualize against curves
- `loc="center right"` with `bbox_to_anchor=(1.0, 0.425)` for sup-kappa subplot 1 (between center and lower)
- Drop the legend entirely if a subplot has only a single curve (sup-kappa subplot 2, Yaw ASMC Gain)

## Suptitles (figure-level, 2026-05-10)

| Figure | Suptitle |
|---|---|
| sup-sliding (Fig. S2) | "Sliding-Surface Evolution" |
| sup-kappa (Fig. S3) | "Adaptive Switching Gains" |
| sup-thrust (Fig. S4) | "Thrust and Lateral Acceleration" |

## Subplot titles (per-axes)

| Figure | Subplot 1 | Subplot 2 |
|---|---|---|
| sup-kappa | "Optic Flow ASMC Gains" | "Yaw ASMC Gain" |
| sup-thrust | "Thrust Command" | "Cone-Constrained Lateral Acceleration" |

## Why this pattern

Column-width supplement figures scale at ~33% (3.5"/10.5"). Default rcParams fonts (8-9pt) render at 2.6-3pt — too small to read. The 14-24pt source fonts compensate so rendered fonts sit at 4.6-7.9pt, matching the surrounding sup-funnel-combined and multi-init-combined figures.

## Related

- `reference_3d_plot_format.md` — multi-init 3D plot format (smaller fonts, columnwidth)
- `feedback_multi_init_combined_layout.md` — multi-init combined plot layout
- `feedback_figure_label_sync.md` — figure scripts must match paper notation
- `feedback_3d_matplotlib_gotchas.md` — bbox_inches="tight" + subplots_adjust gotchas
