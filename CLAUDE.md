# CLAUDE.md

## Repository

- **GitHub:** https://github.com/Shu6hamSingha1/Soft-Precise-Landing
- **Remote:** `git@github.com:Shu6hamSingha1/Soft-Precise-Landing.git`
- **Default branch:** `main`

## Environment

- Platform: Windows 11, shell: bash (Git Bash / MSYS2)
- GitHub CLI path: `export PATH=$PATH:"/c/Program Files/GitHub CLI"`
- Git user: `Shu6hamSingha1 <shubhamsing2@iisc.ac.in>`

## Git Workflow

Commit at **logical checkpoints** (feature complete, tuning round done), not after every single edit. Batch related changes into one commit. Always push after committing:

```bash
export PATH=$PATH:"/c/Program Files/GitHub CLI"
git add <files>
git commit -m "descriptive message"
git push origin main
```

## Project Structure

```
IBVS_Manuscript.pdf          — PLASMC paper draft (IEEE TAES)
References/                  — Reference papers (PDF)
MATLAB/
  visualControl_IBVS_adaptive.m       — Main PLASMC simulation (single run)
  visualControl_IBVS_adaptive_loop.m  — Monte Carlo / loop version
  InitVar.m                  — Simulation parameters (shared)
  Constants.m                — Physical constants (m, J, g, camera params)
  UAVDyn.m                   — Quadrotor dynamics (13-state, RK5)
  RK5.m                      — Runge-Kutta 5th order integrator
  traj_Gen.m                 — Target trajectory generator
  image_feature.m            — Pixel → image features (s, h, w, dw)
  bestParam.mat              — Tuned PLASMC gains
  Comparison/
    visualControl_comparison.m   — 5-controller comparison sim (~600 lines)
    run_comparison.m             — Entry point: run_comparison(ctrl_id)
    InitGains_Comparison.m       — Gains for all 5 controllers
    InitVar.m                    — Comparison-specific params (overrides parent)
    ctrl_Lin2022.m               — Controller 2: Lin et al. (PBVS+PPC)
    ctrl_Zhang2026.m             — Controller 3: Zhang & Wu (PBVS+AEDO)
    ctrl_Chen2025.m              — Controller 4: Chen et al. (IBVS observer)
    ctrl_Cho2022.m               — Controller 5: Cho et al. (FF-IBVS)
    plotter_comparison.m         — Plot comparison results
    result_ctrl_{1-5}.mat        — Saved simulation results per controller
    comp_result.mat              — Combined comparison results
```

## Key Conventions

- **Coordinate frame:** NED (x=North, y=East, z=Down). Altitude is negative z.
- **Controllers 2-5** use geometric SO(3) inner loop (bypass shared attitude PID).
- **IBVS** controllers (1,4,5) get pixel noise via `awgn(C_nP, 50, 'measured')`.
- **PBVS** controllers (2,3) get position/velocity noise (`sigma_pos=0.01m`, `sigma_vel=0.02m/s`).
- **Ground effect + delay** applied to controllers 2-5 matching PLASMC case 1.
- User runs MATLAB simulations themselves — don't run `matlab -batch`. Check `.mat` result files instead.
- Read PDFs with `pymupdf` (Python `fitz`), not `pdftotext` — pdftotext drops math diacritics.

## Token Optimization

- Use `offset`/`limit` when reading large files (especially `visualControl_comparison.m`).
- Don't re-explore the codebase — use the structure above.
- Batch git commits — don't commit after every single line edit.
