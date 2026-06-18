---
name: Controller analysis script
description: Python script to analyze comparison sim results (result_ctrl_{1-5}.mat) — landing/crash detection, barrier proximity, optical flow noise, crash diagnostics
type: reference
---

`MATLAB/Comparison/analyze_results.py` — run from the Comparison directory.

Usage:
- `python analyze_results.py` — summary of all 5 controllers
- `python analyze_results.py 4` — detailed analysis of controller 4
- `python analyze_results.py 4 --plot` — detailed + matplotlib diagnostic plots

Features:
- **Termination detection**: distinguishes LANDED / CRASHED / TIMEOUT (not just idx < N_MAX)
- Summary: final errors, attitude envelope, thrust stats, control energy, oscillations
- Detailed: phase analysis (2s windows), stability diagnostics, altitude convergence
- Optical flow noise (h_i std + max step jumps from V_X_DS rows 3:6)
- PLASMC barrier proximity (|h_e / p_2| ratio per axis, 1.0 = crash)
- Crash diagnostics: last 5 steps with |a_cd|, E_crd, |dE_cd|, identifies likely break cause

State vector indexing: `X_DS` is `[pos(3); quat(4); vel(3); omega(3)]` (13 states, NED frame). Target `x_t` is uint8 — cast to float before use.
