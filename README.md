# Soft Precise Landing

MATLAB simulation framework for quadrotor UAV autonomous soft landing using visual servoing. Implements **PLASMC** (Performance-constrained Leakage-type Adaptive Sliding Mode Control), a novel optic flow-based landing controller, along with a comparative study against four state-of-the-art controllers from the literature.

Developed at the **Indian Institute of Science (IISc), Bengaluru**, for an IEEE Transactions on Aerospace and Electronic Systems (TAES) submission.

## Overview

The core challenge is autonomously landing a quadrotor onto a moving platform using only a downward-facing camera. Unlike position-based methods that require metric depth estimation, PLASMC uses **optic flow** (image feature velocities) as scale-independent feedback, making it robust to unknown altitudes.

### PLASMC Control Architecture

A cascaded three-loop structure:

1. **Outer loop** -- PID on normalized position error with prescribed performance constraints (barrier functions) that guarantee the landing trajectory stays within a decaying funnel
2. **Middle loop** -- Adaptive sliding mode control (ASMC) on optical flow error with leakage-type adaptive gains that prevent wind-up while maintaining robustness to unknown disturbances
3. **Inner loop** -- PID on roll/pitch attitude + adaptive SMC on yaw channel for heading alignment

The controller provides **formal guarantees**: Lyapunov-based proof of exponential convergence to a uniformly ultimately bounded (UUB) region.

## Comparative Study

Five controllers are benchmarked under identical conditions (sensor noise, ground effect, computational delay, 30 Hz camera rate):

| # | Controller | Type | Reference |
|---|-----------|------|-----------|
| 1 | **PLASMC** (Proposed) | IBVS + Optic Flow + ASMC | Singhal et al. |
| 2 | **Lin 2022** | PBVS + Prescribed Performance Control | Lin et al., IEEE TII 2022 |
| 3 | **Zhang 2026** | PBVS + Adaptive Extended Disturbance Observer | Zhang & Wu, IEEE TIE 2026 |
| 4 | **Chen 2025** | IBVS + Robust Observer | Chen et al., IEEE TCST 2025 |
| 5 | **Cho 2022** | Feed-Forward IBVS | Cho et al., Aerosp. Sci. Technol. 2022 |

Controllers 2-5 use a shared geometric SO(3) inner loop for attitude tracking, while PLASMC uses its own cascaded PID + ASMC inner loop.

### Simulation Conditions

- **Platform**: Holybro X500 quadrotor (m = 2.114 kg, J = diag(0.0256, 0.0256, 0.0440) kg-m^2)
- **Camera**: 320x240 resolution, f = 135 px focal length, downward-facing
- **Initial conditions**: 5 m altitude, 2.83 m lateral offset from target
- **Target trajectory**: Circular motion on the ground plane
- **Sensor noise**: IBVS controllers get pixel noise via `awgn(SNR=50)`; PBVS controllers get position/velocity Gaussian noise (sigma_pos = 0.01 m, sigma_vel = 0.02 m/s)
- **Ground effect**: Thrust amplification model `T_eff = T / (1 - (r/4z)^2)` active below ~1 m altitude
- **Computational delay**: One-step actuator delay
- **Sample rate**: 100 Hz dynamics, 30 Hz camera (ZOH = 3)
- **Coordinate frame**: NED (x = North, y = East, z = Down); altitude is negative z

## Repository Structure

```
IBVS_Manuscript.pdf                  -- Paper draft (IEEE TAES)
References/                          -- Reference papers (PDF)
MATLAB/
  visualControl_IBVS_adaptive.m      -- Main PLASMC simulation (single run)
  visualControl_IBVS_adaptive_loop.m -- Monte Carlo / parameter sweep version
  run_simulation.m                   -- Entry point for standalone PLASMC
  InitVar.m                          -- Simulation parameters & initial conditions
  Constants.m                        -- Physical constants (mass, inertia, camera, limits)
  UAVDyn.m                           -- Quadrotor dynamics (13-state ODE)
  RK5.m                              -- Runge-Kutta 5th order integrator
  traj_Gen.m                         -- Target trajectory generator (7 trajectory types)
  image_feature.m                    -- Pixel coordinates to image features (s, h, w, dw)
  kappa_Solver.m                     -- ASMC adaptive gain ODE (translational)
  kappa_a_Solver.m                   -- ASMC adaptive gain ODE (yaw)
  plotter_adaptive.m                 -- Plotting utilities for single-run results
  bestParam.mat                      -- Tuned PLASMC gains
  Comparison/
    run_comparison.m                 -- Entry point: run_comparison(ctrl_id)
    visualControl_comparison.m       -- 5-controller comparison simulation
    InitGains_Comparison.m           -- Gains for all 5 controllers
    InitVar.m                        -- Comparison-specific parameters
    ctrl_Lin2022.m                   -- Controller 2: Lin et al. (PBVS + PPC)
    ctrl_Zhang2026.m                 -- Controller 3: Zhang & Wu (PBVS + AEDO)
    ctrl_Chen2025.m                  -- Controller 4: Chen et al. (IBVS observer)
    ctrl_Cho2022.m                   -- Controller 5: Cho et al. (FF-IBVS)
    plotter_comparison.m             -- Plot comparison results
    analyze_results.py               -- Python analysis script (detailed diagnostics)
    result_ctrl_{1-5}.mat            -- Saved simulation results per controller
    comp_result.mat                  -- Combined comparison results
```

## Usage

### Requirements

- MATLAB R2022b or later (Signal Processing Toolbox for `awgn`)
- Python 3.8+ with `scipy` and `numpy` (for analysis script only)

### Running the PLASMC Controller (Standalone)

```matlab
run_simulation()
```

This runs `visualControl_IBVS_adaptive.m` with gains from `bestParam.mat`, then plots the results.

### Running the Comparison Study

```matlab
% Run a single controller (1-5)
run_comparison(1)     % PLASMC
run_comparison(4)     % Chen2025

% Run all controllers
run_comparison([1 2 3 4 5])
```

Results are saved to `result_ctrl_{N}.mat` and combined into `comp_result.mat`.

### Analyzing Results

```bash
cd MATLAB/Comparison

# Summary of all controllers
python analyze_results.py

# Detailed analysis of a specific controller
python analyze_results.py 4

# Detailed analysis with diagnostic plots
python analyze_results.py 4 --plot
```

The analysis script reports:
- Landing status (LANDED / CRASHED / TIMEOUT)
- Final position error, velocity, attitude envelope
- Phase-by-phase breakdown (2-second windows)
- Altitude convergence timeline
- Optical flow noise diagnostics
- PLASMC barrier function proximity (distance to constraint violation)
- Crash diagnostics (last 5 steps before failure)

## Quadrotor Dynamics

The UAV is modeled as a 13-state rigid body in the NED frame:

- **State**: `x = [p(3); q(4); v(3); omega(3)]` -- position, quaternion, velocity, angular velocity
- **Input**: `u = [tau_x; tau_y; tau_z; T]` -- body torques and total thrust
- **Integration**: 5th-order Runge-Kutta (RK5) at 100 Hz

Actuator limits: thrust 0-60 N, torque +/-1.85 Nm (roll/pitch), +/-1.0 Nm (yaw), angular rate +/-4 rad/s (roll/pitch), +/-2 rad/s (yaw).

## Acknowledgements

This work is supported by:
- Science and Engineering Research Board (SERB), Grant CRG/2021/006872
- Space Technology Cell, IISc, Grant STC/P-486

## Authors

- **Shubham Singhal** -- IISc Bengaluru
- **Suresh Sundaram** -- IISc Bengaluru
- **Jishnu Keshavan** -- IISc Bengaluru

## License

This repository contains research code associated with a manuscript under preparation. Please contact the authors for licensing information.
