---
name: Multi-init combined plot layout (3-D + image-plane side-by-side)
description: Re-locked 2026-04-30. Multi-init manuscript figures use one PDF per trajectory with the 3-D trajectory on the left and the image-plane evolution on the right, sharing one figure-level IC legend (2 rows × 3 cols) and one in-axes style legend. Only `_combined.pdf` is produced by `make_multi_init_plots.py`. Layout values were re-tuned 2026-04-30.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The convention (re-locked 2026-04-30 after iterative resizing):**

`scripts/make_multi_init_plots.py` produces one PDF per trajectory:
`Soft_Precise_Landing/Figures/generated/multi_init/<Traj>_combined.pdf`

Locked layout values:
- `figsize = (10.5, 6.0)` inches.
- `gs = fig.add_gridspec(1, 2, width_ratios=[1.4, 1.0], left=0.05, right=0.98, wspace=0.10)`. The 3-D bbox needs to be **wider** than the image-plane bbox because matplotlib leaves significant internal horizontal padding around the rendered cube — with these ratios the visible 3-D cube and the aspect-locked image-plane content render at comparable sizes.
- 3-D axes additionally shifted left after `subplots_adjust` via `ax3.set_position([pos.x0 - 0.10, pos.y0, pos.width, pos.height])`.
- `subplots_adjust(bottom=0.17, top=0.93)` — small fractional margins so the suptitle (~0.5 in) and the 2-row IC legend (~0.7 in) get only what they need and the subplot fills the rest. (At an earlier `(11.0, 6.5)` configuration the margins were `bottom=0.230, top=0.865`; both yielded ~5.1 in of subplot height.)
- **Suptitle** `fig.suptitle(f"Landing for Multiple Initial Conditions for {TRAJ_CASE[traj]}", fontsize=24, y=0.99)`.
- **Subplot titles** `fontsize=20`: 3-D at `y=0.97`, image-plane at `y=1.03`. (3-D's title sits *inside* the bbox at the top; image-plane's sits *just above* its bbox top — different conventions because 3-D titles compete with the suptitle.)
- **Axis labels** `fontsize=20`; 3-D x/y use `labelpad=12`, z uses `labelpad=2`; image-plane y uses `labelpad=-6`.
- **Tick labels** `labelsize=18` (main axes), `labelsize=10` (image-plane inset).
- **Style legend** (`desired` / `start` / `end`) inside `axI` at `loc="lower right"`, `fontsize=14`, `ncol=1`.
- **IC legend** at `fig.legend(handles=ic_handles, loc="lower center", bbox_to_anchor=(0.5, 0.0), ncol=3, fontsize=14, handlelength=1.6, columnspacing=1.0, handletextpad=0.6)` — 2 rows × 3 cols with the last cell empty.
- **IC labels use LaTeX subscripts**: `rf"IC$_{k+1}$: ..."` (was bare `f"IC{k+1}: ..."` until 2026-04-30 — see `feedback_ic_notation.md`).

**Manuscript / supplement usage:**
- Main paper Fig. 3 (Circular) uses `\includegraphics[width=\columnwidth]{Figures/generated/multi_init/Circular_combined.pdf}` inside `\begin{figure}[!t]`.
- Supplement §S3-A Cases 1–4 use the same pattern with `Static_combined.pdf`, `Linear_combined.pdf`, `Sinusoidal_combined.pdf`, `Lissajous_combined.pdf` — each inside `\begin{figure}` (single-column).
- Captions read "*Left*: 3-D trajectories ... *Right*: image-plane evolution ..."

**Why two legends instead of fully merged:**
- IC entries have varying widths (each shows IC coords + `Δ_max`). Merging them with the static-style entries made the bottom-center legend either too wide or too tall.
- Splitting keeps the static-style entries close to the image-plane content and the IC entries at the figure foot where they apply to both panels.

**Image-plane axes are aspect-locked**:
`axI.set_aspect("equal", adjustable="box")` with `xlim=(-160, 160)` and `ylim=(-120, 120)`. So the image-plane bbox renders at fixed 4:3 regardless of bbox shape; the 3-D bbox has no aspect lock. Implication: changing `width_ratios` redistributes width but the image-plane content only grows/shrinks to the bbox-limited extent.

**Affected scripts (current):**
- `scripts/make_multi_init_plots.py` — `plot_combined()` is the only function called by `main()`; legacy `plot_3d()` and `plot_image_plane()` retained but inactive.

**Files moved to Obsolete (2026-04-29):**
- All `<Traj>_3D.pdf` and `<Traj>_image_plane.pdf` for `Static`, `Linear`, `Sinusoidal`, `Lissajous`, `Circular` (10 files) → `Obsolete/Figures/generated/multi_init/`.

**Related conventions:**
- Marker triangle/cross: `feedback_landing_marker_convention.md`.
- Corridor visualisation: `feedback_landing_corridor_convention.md`.
- 3-D format defaults (still apply to non-combined plots): `reference_3d_plot_format.md`.
- IC notation in legends: `feedback_ic_notation.md`.
- 3-D axes never tolerate `bbox_inches="tight"` (drops z-label): `feedback_3d_matplotlib_gotchas.md`.
- Backup convention before structural script edits: `Obsolete/scripts/make_multi_init_plots_v3.py` is the pre-2026-04-29 snapshot.
