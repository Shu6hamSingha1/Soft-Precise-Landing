---
name: Multi-init image-plane plot design
description: scripts/make_multi_init_plots.py — single-axis overlaid image-plane design with all ICs, start/end quads, corner trajectories
type: reference
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
`scripts/make_multi_init_plots.py` produces the multi-init manuscript figures.

Image-plane plot (per trajectory × condition):
- Single axis (replaced an earlier 2×3 subplot grid)
- 4 corners × 5 ICs overlaid + desired quad (black, solid, lw=2)
- Start quad: solid, faded (alpha=0.4), IC color
- End quad: long-dash `(0,(5,2))`, IC color, labeled with IC coordinates
- Corner trajectories: IC color (C0–C4), lw=0.7, alpha=0.5, linestyle varies by corner
- Legend: lower-right, ncol=2, fontsize=7

3D plot function is the original layout (5 ICs, identical styling across trajs).

Outputs: `Soft_Precise_Landing/Figures/generated/multi_init/<traj>_3D[_noiseless].pdf` and `<traj>_image_plane[_noiseless].pdf`.

**History (2026-04-19):** earlier `make_multi_init_plots_v2.py` was promoted to the canonical name; the original (v1) is in `Obsolete/scripts/make_multi_init_plots_v1.py`.
