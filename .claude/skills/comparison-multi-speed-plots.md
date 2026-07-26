# /comparison-multi-speed-plots

Regenerate the per-trajectory baseline-controller multi-speed comparison figures used in the manuscript supplement (Cases 2–4) and main paper (Case 5).

## Usage

```
/comparison-multi-speed-plots
```

No arguments. Reads `MATLAB/Comparison/Datasets/<traj>_multi_speed_comparison.mat` for traj ∈ {Linear, Sinusoidal, Lissajous, Circular}.

## Instructions

```bash
cd "L:/Claude/Soft Landing" && python scripts/make_comparison_multi_speed_plots.py
```

If any `.mat` is missing, the script reports it and exits gracefully. To regenerate the data, run the MATLAB driver first:

```bash
cd "L:/Claude/Soft Landing/MATLAB/Comparison"
matlab -batch "multi_speed_comparison"
```

The driver runs 80 simulations (4 trajectories × 4 baselines × 5 λ multipliers); ~30 min wall-clock.

## What it does

`scripts/make_comparison_multi_speed_plots.py` produces 4 PDFs:

- `Soft_Precise_Landing/Figures/generated/comparison_multi_speed_linear.pdf`
- `Soft_Precise_Landing/Figures/generated/comparison_multi_speed_sinusoidal.pdf`
- `Soft_Precise_Landing/Figures/generated/comparison_multi_speed_lissajous.pdf`
- `Soft_Precise_Landing/Figures/generated/comparison_multi_speed_circular.pdf`

Each PDF: 2×2 grid (Lin 2022 / Zhang 2026 / Lin 2023 / Cho 2022). Each subplot shows the baseline's UAV descent at λ ∈ {0.6, 0.8, 1.0, 1.2, 1.4} (viridis-colored, dark→bright), per-λ shaded soft-precise corridor, and triangle/cross touchdown markers.

## Manuscript placement

- Main paper Fig. 7: `comparison_multi_speed_circular.pdf` (Case 5).
- Supplement §S3-G: the other three PDFs (Cases 2–4).

## Notes

- MATLAB driver state lives in `MS_STATE` struct, preserved across `visualControl_comparison.m`'s `clearvars -except`. Do NOT rely on plain locals (mu, cIdx, etc.) surviving the inner script's workspace reset. See memory `project_comparison_multi_speed.md`.
- Save format is `-v7` (not `-v7.3`); scipy.io.loadmat reads it directly. If you change the driver to `-v7.3`, the Python script will need `mat73` or h5py.
- The plot script shares the uniform 3D format with `make_multi_init_plots.py`, `make_multi_speed_plots.py`, `make_comparison_plots.py` — labelpad=2, nbins=4, tick_params(pad=1, labelsize=7), title fontsize=8, view_init(22, -58). See memory `reference_3d_plot_format.md`.
- Do not commit figures unless the user asks — they regenerate from data.
