---
name: Figure scripts must match paper notation; regenerate PDFs after label edits
description: Tex caption changes alone don't fix figures — the generating Python/MATLAB script must also be updated and the PDF re-rendered. Audit `scripts/make_*.py` (and `MATLAB/.../plotter_*.m` if applicable) whenever a notation or numeric convention changes in the manuscript.
type: feedback
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
**The rule (locked 2026-04-24):**

When the manuscript's notation or numeric convention changes (e.g., $u/v \to \,^\mathcal{C}\hat{x}/\hat{y}$, $w_e \to h_e$, cone-cap angle 35° → 60°), three things must be in sync:
1. The tex caption / prose
2. The figure-generating script's axis labels, title, and any reference-line annotations
3. The rendered PDF in `Soft_Precise_Landing/Figures/generated/` (or its subdirectory)

If only (1) is updated, the compiled PDF still shows the OLD axis label or value, and a reader/reviewer immediately spots the mismatch with the caption.

**Catalog of generating scripts (figures used in the manuscript/supplement):**
- `scripts/make_plasmc_plots.py` → `plasmc_outer_funnel.pdf`, `plasmc_inner_funnel.pdf`, `plasmc_sliding.pdf`, `plasmc_adaptive_gain.pdf`, `plasmc_thrust_accel.pdf`. Source data: `MATLAB/Multi_init_cond/Datasets/Sinusoidal_multi_init.mat` (representative IC2 run).
- `scripts/make_multi_init_plots.py` → 20 PDFs in `Figures/generated/multi_init/` (per trajectory × {realistic, idealised} × {3D, image-plane}). Source data: `MATLAB/Multi_init_cond/Datasets/{Static,Linear,Sinusoidal,Lissajous,Circular}_multi_init[_noiseless].mat`.
- `scripts/make_multi_speed_plots.py` → speed-envelope figures.
- `scripts/make_comparison_plots.py` → comparison summary + 3D trajectory plots.

**Notation alignment check before regeneration:**
- Image-plane axis labels: $\,^\mathcal{C}\hat{x}, \,^\mathcal{C}\hat{y}$ (NOT $u, v$ — see `feedback_image_plane_axis_vs_component.md`).
- Optic-flow error: $h_e$ (NOT $w_e$ — legacy from older naming epoch).
- Cone-cap: $\theta_\text{cap} = 60°$, lateral acceleration cap $g\tan\theta_\text{cap} \approx 16.99~\text{m/s}^2$ (NOT $\phi_\text{max} = 35°$).
- Target image funnel: $\boldsymbol{p}_1(t)$ on the camera-frame physical-pixel coordinates (NOT virtual-frame, see `project_visibility_constraint_location.md`).

**Regeneration command:**
```bash
cd "L:/Claude/Soft Landing"
PYTHONIOENCODING=utf-8 python scripts/make_plasmc_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_multi_init_plots.py
# ... etc per script
```
Each script overwrites its PDFs in-place. Report file timestamps to user as confirmation.

**Why:** User flagged 2026-04-24 that the supplement caption I had fixed (`E_2(t) → \hat{r}_e(t)`) didn't fix the figure itself, because the underlying script still plotted `(u, v)` labels. A reader would see the mismatch immediately. This rule prevents that class of bug.

**How to apply:**
- Whenever you change a figure's caption, also grep the matching script for the symbols/values in that caption.
- After a notation-convention update across the manuscript, regrep ALL figure scripts for the old symbols and update them in lockstep.
- Always regenerate the affected PDFs as the final step.
- If a script depends on a `.mat` file, the data is the source of truth — verify against the script's loaded variables (e.g., the worst-case xy-error claim "5.7 cm" was validated against `Linear_multi_init.mat` IC5 = 5.7126 cm).
