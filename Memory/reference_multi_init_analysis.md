---
name: Multi-init analysis script
description: Python script to analyze PLASMC multi-initial-condition results from Multi_init_cond/Datasets/<trajType>_multi_init.mat
type: reference
---

`MATLAB/Multi_init_cond/analyze_multi_init.py` analyzes results from `multi_Init_Var.m`.

**Run from `MATLAB/Multi_init_cond/` directory.**

## Usage
```bash
python analyze_multi_init.py                    # default: Static
python analyze_multi_init.py Linear             # any trajectory name
python analyze_multi_init.py Static --plot      # add matplotlib plots
python analyze_multi_init.py Static 3           # detailed view of run 3
python analyze_multi_init.py Static 3 --plot    # detailed + plots
```

## Input
Reads `Datasets/<trajType>_multi_init.mat` (saved by `multi_Init_Var.m`):
- `results` — struct array, fields: `success`, `final_error`, `data` (full sim workspace)
- `p0` — Nx3 initial-position matrix
- `numRuns` — number of runs

`results(k).data` is the full workspace from `run_simulation.m`, containing:
`X_DS` (13×N+1), `U_DS` (4×N), `x_t` (7×N), `idx`, `dt`, `tRange`, `K_ctrl`, etc.

## Landing thresholds (must mirror run_simulation.m)
`LAND_ALT = 0.20 m`, `LAND_XY = 0.30 m` — both must hold simultaneously.

## Per-run metrics extracted
- `t_end` = `idx * dt` — time at termination (lower than tend means landed or crashed)
- `final_alt`, `final_xy`, `final_3d` — distance to target at termination
- `final_vel` (3D speed), `final_vz` — landing velocity
- `max_climb` — peak altitude above start (initial-overshoot diagnostic)
- `yaw_drift` (deg) — start-to-end yaw change
- `landed` — true iff `final_alt ≤ 0.20 && final_xy ≤ 0.30`

## Failure pattern fingerprints
| Symptom | Likely cause |
|---|---|
| `t_end ≈ 0.03 s`, `idx=3` | NaN/inf early in controller (kappa, I_a_cd, dE_cd safety break at iter 3) |
| `t_end < 1 s`, large `final_3d` | Controller diverges immediately |
| `final_alt ≤ 0.20` but `final_xy > 0.30` | UAV reached ground but missed target laterally |
| `final_xy ≤ 0.30` but `final_alt > 0.20` | UAV stalls/hovers above target |
| `max_climb > 1 m` | Initial vertical overshoot — vertical channel too aggressive at startup |
| `final_vel > 1 m/s` | Hard landing — track noise amplification at low altitude |

## Counterpart for comparison study
`Comparison/analyze_results.py` analyzes per-controller results (`result_ctrl_{1-5}.mat`).
The two scripts share the same X_DS / x_t / idx conventions but iterate over different
axes (controllers vs. initial conditions).
