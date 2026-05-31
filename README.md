# Soft Precise Landing

MATLAB simulation framework for quadrotor UAV autonomous soft-precise landing on a moving target from monocular vision. Implements **VDF-ASMC** (Visual Dual-Funnel Adaptive Sliding-Mode Control), an optic-flow-based landing controller with closed-loop guarantees on soft touchdown, precise touchdown, and target visibility. Includes a comparative benchmark against four state-of-the-art controllers.

Developed at the **Indian Institute of Science (IISc), Bengaluru**, for an IEEE Transactions on Aerospace and Electronic Systems (TAES) submission. Manuscript source in `Soft_Precise_Landing/`.

## Overview

The core challenge is autonomously landing a quadrotor onto a moving platform using only a downward-facing monocular camera, with **no metric depth** and **no external heading reference**. Existing visual-servoing laws built for tracking do not guarantee a soft-precise touchdown.

### VDF-ASMC architecture

A two-funnel image-parameter-driven cascade:

1. **Outer loop** — *Dual funnel*
   - The **optic-flow funnel** $\boldsymbol{p}_2(t)$ constrains the optic-flow error and is enforced by a *leakage-type adaptive sliding-mode law* that produces the inertial commanded acceleration $\,^\mathcal{I}\boldsymbol{a}_\text{d}$. The leakage adaptation identifies the unknown disturbance bound online (no *a priori* bound required).
   - The **target image funnel** $\boldsymbol{p}_1(t)$ sizes a state-dependent attitude cone — the *funnel-margin cone clamp* — that kinematically clips $\,^\mathcal{I}\boldsymbol{a}_\text{d}$ so the four image feature points stay inside the camera FoV.

2. **Inner loop** — *Virtual-compass yaw ASMC + Geometric SO(3) tracker*
   - A leakage-type adaptive sliding-mode law on the image-orientation error $\alpha_\text{e}$ produces the desired yaw rate; an angle-wrapping integrator (the *virtual compass*) yields the desired heading $\psi_\text{d}$ — magnetometer-free.
   - A geometric SO(3) attitude tracker (Lee 2010) produces the body torque.

### Closed-loop guarantees

A layered Lyapunov certificate delivers:

- **Theorem 1** — Adaptive Optic-Flow Funnel Invariance: $\boldsymbol{h}_\text{e}(t) \in (-\boldsymbol{p}_2(t),\,\boldsymbol{p}_2(t))$ for all $t \ge 0$ under bounded disturbances.
- **Corollary 1** — Closed-loop target visibility: every image feature point stays inside the FoV envelope kinematically.
- **Theorem 2** — Adaptive Yaw Ultimate Boundedness: image-orientation error is UUB with magnetometer-free heading alignment.

## Comparative benchmark

Five controllers exercised under a shared disturbance model:

| # | Controller | Type | Reference |
|---|-----------|------|-----------|
| 1 | **VDF-ASMC** (Proposed) | IBVS + Optic Flow + Adaptive SMC | Singhal et al. (this work) |
| 2 | **Lin 2022** | PBVS + Performance-Constrained Control | Lin et al., IEEE TII 2022 |
| 3 | **Zhang 2026** | PBVS + Adaptive Extended Disturbance Observer | Zhang & Wu, IEEE TIE 2026 |
| 4 | **Lin 2023** | Robust IBVS + Performance Funnel | Lin et al., IEEE T-ASE 2023 |
| 5 | **Cho 2022** | Feed-Forward IBVS | Cho et al., Aerosp. Sci. Technol. 2022 |

Baselines 2–5 share a common geometric SO(3) inner loop.

### Test setup

- **Platform**: Holybro X500 quadrotor; mass $m=2.1$ kg, inertia $J=\text{diag}(0.026, 0.026, 0.044)$ kg-m².
- **Camera**: 320×240 px, focal length $f=135$ px, downward-facing.
- **Initial conditions (5 ICs)**: IC$_1$ nominal drop $[0,0,-5]$ m; IC$_2$–IC$_3$ ±diagonal lateral offsets; IC$_4$–IC$_5$ high/low altitude with lateral offset.
- **Trajectories (5 cases)**: Static, Linear (ship-deck), Sinusoidal, Lissajous, Circular (ship-deck). Cases 2 and 5 include $0.2\sin(0.5t)$ m heave + $(\pm 15°, \pm 8°)$ roll/pitch deck oscillations.
- **Disturbances**: depth-dependent pixel noise $\sigma_\text{px}(z)=0.3+0.5/(z+0.5)$ px, 0.5% Bernoulli 5-px outliers, $\pm 0.2$ m/s mean wind + Ornstein–Uhlenbeck turbulence, $\pm 5\%$ mass/inertia uncertainty, $\pm 5$ mm CoG offset, aerodynamic drag $C_d=0.25$, rotor ground effect, one-step computational delay, $60°$ attitude-cone projection.
- **Sample rate**: 100 Hz dynamics ($\Delta t=0.01$ s), 30 Hz camera (ZOH$=3$).
- **Coordinate frame**: NED (x=North, y=East, z=Down).
- **Termination**: when UAV altitude above target falls below landing-gear height $h_\text{lg}=0.20$ m, or 40 s timeout.
- **Soft-precise thresholds**: terminal horizontal error $r_{xy,\text{f}} \le 0.08$ m; terminal relative speed $v_\text{f} \le 0.20$ m/s.

### Headline results

- **Multi-initial-condition robustness** (5 trajs × 5 ICs = 25 deterministic runs): VDF-ASMC achieves $25/25$ soft-precise touchdowns with worst-case horizontal error 5.7 cm and mean landing time $\le 18.2$ s.
- **Target-speed envelope sweep** ($\lambda\in\{0.6,\ldots,1.4\}$): $20/20$ soft-precise touchdowns; worst-case 5.03 cm / 0.154 m/s.
- **Comparative benchmark** (single-speed, IC$_2$): VDF-ASMC is the only controller to land soft-precise on every trajectory ($\le 2.1$ cm); baselines fail either by feature-visibility break above the surface or by exceeding the precise/soft bounds.
- **Cross-trajectory deep sweep**: 3325-run perturbation study validates the locked gain set as single-axis Pareto-optimal up to numerical precision.

## Repository structure

```
IBVS_Manuscript.pdf                      -- Main paper (compiled, IEEE TAES)
IBVS_Supplemental.pdf                    -- Supplementary material (compiled)
References/                              -- Cited reference papers (PDF)
Soft_Precise_Landing/                    -- LaTeX source for paper + supplement
  manuscript.tex                         -- Main paper (intro, conclusion, bibliography)
  control_formulation.tex                -- §II–III (preliminaries + VDF-ASMC design + stability)
  results.tex                            -- §IV (numerical results, multi-IC + speed sweep + comparison)
  supplemental.tex                       -- Full supplement (§S1–S3)
  bibliography.bib                       -- Shared bib file
  Figures/                               -- Generated and hand-drawn figures
MATLAB/
  Common/                                -- Shared modules across all sweeps
    Constants.m, InitVar pieces          -- Physical constants + locked gains
    UAVDyn.m, RK5.m                      -- 13-state quadrotor dynamics + integrator
    traj_Gen.m                           -- Target trajectory generator (canonical location)
    image_feature.m                      -- Pixel → image features (s, h, w, dw)
    kappa_Solver.m, kappa_a_Solver.m     -- Adaptive switching-gain ODEs (transl. + yaw)
    centered_moment.m, moment.m          -- Image-moment helpers
    skew.m, sat.m, smooth4.m             -- Numerical utilities
  Comparison/                            -- 5-controller comparison harness
    run_comparison.m                     -- Entry point: run_comparison(ctrl_id, [traj])
    visualControl_comparison.m           -- Per-controller closed-loop simulation
    InitGains_Comparison.m               -- Locked gains for all 5 controllers
    ctrl_Lin2022.m, ctrl_Zhang2026.m     -- Baseline implementations
    ctrl_Lin2023.m, ctrl_Cho2022.m
    multi_speed_comparison.m             -- Per-traj per-λ speed-sweep driver
    plotter_comparison.m                 -- MATLAB-side plotting
    Datasets/                            -- Saved .mat results (per-traj, per-ctrl)
  Multi_init_cond/                       -- Multi-IC sweep harness (VDF-ASMC alone)
    multi_Init_Var.m, run_multi_init.m   -- 5-IC sweep entry point
    multi_speed.m                        -- 5-λ speed-envelope sweep
    Datasets/                            -- Saved .mat results (per-traj)
  Sweeps/                                -- 33-axis cross-trajectory deep sweep
    sweep_deep.m                         -- 3325-run perturbation study driver
    Datasets/sweep_deep.mat              -- Aggregated sweep results
scripts/                                 -- Python analysis + plot generators
  make_plasmc_plots.py                   -- VDF-ASMC internals: funnel, sliding, kappa, thrust
  make_comparison_plots.py               -- Comparison combined plot (main paper) + 2x2 supp
  make_comparison_multi_speed_plots.py   -- Per-traj multi-speed baseline plots
  make_multi_init_plots.py               -- Per-traj multi-IC plots (3D + image plane)
  make_multi_speed_plots.py              -- VDF-ASMC speed-envelope landing plot
  analyze_results.py, analyze_multi_init.py  -- Diagnostic analysis utilities
  summarize_sweep.py                     -- Deep-sweep summary tables
```

## Usage

### Requirements

- MATLAB R2022b or later (Signal Processing Toolbox for `awgn`)
- Python 3.8+ with `scipy`, `numpy`, `matplotlib` (for analysis + plot generation)
- `pymupdf` (Python `fitz`) for PDF reading utilities

### Running the comparison benchmark

```matlab
% Single controller on a single trajectory
run_comparison(1, "Circular")   % VDF-ASMC on Case 5

% All five controllers on all five cases
for cid = 1:5
    for tr = ["Static" "Linear" "Sinusoidal" "Lissajous" "Circular"]
        run_comparison(cid, tr);
    end
end
```

Results are saved to `MATLAB/Comparison/Datasets/<traj>_comparison.mat`.

### Running the multi-IC robustness sweep

```matlab
cd MATLAB/Multi_init_cond
run_multi_init("Sinusoidal")   % 5 ICs on Case 3
```

Results saved to `Datasets/<traj>_multi_init.mat`.

### Regenerating manuscript figures

```bash
cd "L:/Claude/Soft Landing"
PYTHONIOENCODING=utf-8 python scripts/make_plasmc_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_comparison_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_comparison_multi_speed_plots.py
PYTHONIOENCODING=utf-8 python scripts/make_multi_init_plots.py
```

All PDFs are written to `Soft_Precise_Landing/Figures/generated/`.

### Analysing comparison results

```bash
cd scripts
python analyze_results.py            # Summary across all 5 controllers
python analyze_results.py 4 --plot   # Detailed Lin2023 diagnostics with plots
```

Reports: landing status, final position/velocity, attitude excursions, phase-by-phase breakdown, optic-flow noise diagnostics, funnel-margin proximity, and per-baseline crash signatures.

## Quadrotor dynamics

13-state rigid-body model in NED with FRD body frame:

- **State**: $\boldsymbol{x} = [\boldsymbol{p}; \boldsymbol{q}; \boldsymbol{v}; \boldsymbol{\omega}]$ — position, quaternion, velocity, body angular velocity
- **Input**: $\boldsymbol{u} = [\,^\mathcal{B}T_u; \,^\mathcal{B}\boldsymbol{\tau}_u]$ — total thrust along $-\hat{Z}_b$ and body torque
- **Integration**: 5th-order Runge–Kutta (RK5) at 100 Hz

Actuator limits: total thrust $0$–$60$ N, attitude-cone ceiling $\theta_\text{cap}=60°$, body-rate limits $\pm 4$ rad/s (roll/pitch), $\pm 2$ rad/s (yaw).

## Acknowledgements

This work is supported by:
- Science and Engineering Research Board (SERB), Core Research Grant **CRG/2021/006872**
- Space Technology Cell, IISc, Grant **STC/P-486**

## Authors

- **Shubham Singhal** — Robert Bosch Centre for Cyber-Physical Systems, IISc Bengaluru
- **SriKrishna TKVSS** — Department of Mechanical Engineering, IISc Bengaluru
- **Suresh Sundaram** — Robert Bosch Centre for Cyber-Physical Systems + Department of Aerospace Engineering, IISc Bengaluru
- **Jishnu Keshavan** — Robert Bosch Centre for Cyber-Physical Systems + Department of Mechanical Engineering, IISc Bengaluru

## License

This repository contains research code associated with a manuscript under preparation. Please contact the authors for licensing information.
