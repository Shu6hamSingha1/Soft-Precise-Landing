# /multi-init-plots

Regenerate the multi-initial-condition figures used in the manuscript from the latest `.mat` datasets produced by `MATLAB/Multi_init_cond/multi_Init_Var.m`.

## Usage

```
/multi-init-plots
```

No arguments. Processes every trajectory under `MATLAB/Multi_init_cond/Datasets/` (single regime — the noiseless/idealised variant was removed 2026-04-26 along with the idealised case in the paper).

## Instructions

Run the generator script from the repo root:

```bash
cd "L:/Claude/Soft Landing" && python scripts/make_multi_init_plots.py
```

## What it does

`scripts/make_multi_init_plots.py` loops over `{Static, Linear, Sinusoidal, Lissajous, Circular}` and writes, for each:

- `Soft_Precise_Landing/Figures/generated/multi_init/<Traj>_combined.pdf`

Each combined PDF places the 3-D trajectory on the left and the image-plane evolution on the right, with a single shared figure-level legend (5 IC entries + Δ_max in pixels, in 2 rows × 3 cols at the bottom-center) and a per-axes style legend (`desired`/`start`/`end`) at the lower-right of the image-plane axes; common suptitle uses `<Case>: <TrajName>` form.

Inputs (must exist beforehand):
- `MATLAB/Multi_init_cond/Datasets/<Traj>_multi_init.mat`

Missing `.mat` files are skipped and reported; re-run `MATLAB/Multi_init_cond/multi_Init_Var.m` first if anything is missing.

## Expected output

```
Wrote 5 figures to L:\Claude\Soft Landing/Soft_Precise_Landing/Figures/generated/multi_init
```

5 = 5 trajectories × 1 combined figure each. The legacy `plot_3d()` and `plot_image_plane()` helpers are retained in the script for future use but not invoked from `main()`. Old separate `_3D.pdf` and `_image_plane.pdf` files were moved to `Obsolete/Figures/generated/multi_init/` on 2026-04-29.

## Notes

- Script uses matplotlib Agg backend; no display needed.
- NED convention: altitude plotted as `-z`.
- Image-plane plots use `P_DS` columns 9..12 (raw pixel corners).
- Target path is trimmed to the latest actual landing index so periodic trajectories don't wrap back to the origin.
- 3D plots draw a "soft-precise allowable landing region" corridor around the target trajectory (xy ±0.08 m, vertical 0–0.20 m above true target, follows heave) instead of a single dashed line. See `feedback_landing_corridor_convention.md` in memory.
- Touchdown markers: triangle (`^`) for soft-precise (xy ≤ 0.08 m AND |v_z_rel| ≤ 0.20 m/s), cross (`x`) otherwise. See `feedback_landing_marker_convention.md`.
- Uniform 3D format (labelpad=2, nbins=4, tick_params(pad=1, labelsize=7), title fontsize=8, view_init(22,-58)) is shared with `make_multi_speed_plots.py` and `make_comparison_plots.py`. See `reference_3d_plot_format.md`.
- Do not commit figures unless the user asks — they regenerate from data.
