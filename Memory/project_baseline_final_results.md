---
name: Final baseline retune results (5-controller comparison)
description: Lock-in results for Lin/Zhang/Chen/Cho under shared robustness model, best-effort retuned gains. Supersedes per-trajectory memories from 2026-04-07.
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Final 5-trajectory comparison (2026-04-16, commit 7fa0a02) after propagating the shared `init_robustness.m` (depth-dep pixel noise, outliers, wind + OU turbulence, parametric uncertainty) into `visualControl_comparison.m`. Gains in `InitGains_Comparison.m` are best-effort retunes *within each paper's published framework* — paper-authoritative values all crash on our IC=[2,2,-5] harness (see `project_baseline_paper_gains_failure.md`).

**Results (p_xy in meters; PLASMC landing criterion p_xy<0.10m AND v_rel soft):**

| Ctrl       | Static | Linear     | Sinusoidal | Lissajous | Circular    |
|------------|--------|------------|------------|-----------|-------------|
| PLASMC     | 0.04   | 0.04       | 0.04       | 0.09      | 0.08        |
| Lin 2022   | 1.38   | 2.71 (T=49N)| 1.52      | 1.48      | 2.51        |
| Zhang 2026 | 0.24   | 0.11       | 0.55       | 0.22      | 0.18        |
| Chen 2025  | 1.71†  | 2.68†      | 0.54       | 0.65      | 11m (div)†  |
| Cho 2022   | 0.15   | 0.38       | 0.43       | 0.30      | 0.29        |

† = doesn't land (stalls or diverges). Chen shows seed-variant catastrophic failure on rotating targets (finite-diff `de` noise → unbounded thrust).

**Structural limits reached per baseline:**
- **Lin 2022** — PPC barrier. Widened `rho_inf_v(3)=0.15` covers Lin heave (A_z=0.2, w_z=0.5) so xi_v(3) no longer saturates on z-oscillation. But on some rng seeds the xy barrier still hits 49N because PPC cannot simultaneously absorb 0.2 m/s wind disturbance AND track moving targets.
- **Zhang 2026** — strongest baseline, lands all 5 with 0.11–0.55m precision. z slowed (Kc1(3)=0.03, Kc2(3)=0.15, Kc3(3)=0.5, omega_n_z~0.4 rad/s) gives moderate v_z 0.3–1.1 m/s — softer than prior tune but still not "soft" by PLASMC's threshold.
- **Chen 2025** — structural. Finite-diff `de` noise through (r-s+eps)*kr produces unbounded thrust on ~half of rng seeds. Paper assumes analytical `dq/dt`; no filter prescribed. Framework cannot be fixed via gains alone. Already documented in `project_chen2025_limitation.md`.
- **Cho 2022** — tuning sensitivity discovered: `k_sigmoid=0.1` (fast `ad_z` saturation) broke moving targets because feature centroid jitters → `ad_z` jitters → z-command jitters. Must stay at `k_sigmoid=0.02`. Accepted terminal stall ~0.3m on moving trajectories.

**Final locked gains in `InitGains_Comparison.m`:**
- Lin: k1=0.6, k2=4.0, rho_inf_{p,v}=[0.30;0.30;0.15], l_{p,v}=[0.03;0.03;0.10], margin=1.5
- Zhang: Kc1=diag(0.25,0.25,0.03), Kc2=diag(2.0,2.0,0.15), Kc3=diag(2.5,2.5,0.5), omega_AFm=1.0
- Chen: kr=1.0, k2_obs=5, k3_obs=0.5, zstar0=1.5, q_d=[0;0;0.0667]
- Cho: lambda=[-1.2;-1.2;-2.0;0;0;0], Kv=diag(3.5,3.5,2.0), k_sigmoid=0.02, v_sat=[1;1;0.7;0.2]

**How to apply:** These are the manuscript Table V/VIII values. Do NOT retune further without evidence a specific baseline was wrong — we have hit the structural ceiling for all four. Any seed-variance noise in subsequent runs is not a regression. Lock before generating final plots.
