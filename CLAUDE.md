# CLAUDE.md

## Repository

- **GitHub:** https://github.com/Shu6hamSingha1/Soft-Precise-Landing
- **Remote:** `git@github.com:Shu6hamSingha1/Soft-Precise-Landing.git`
- **Default branch:** `main`

## Environment

User works on this project from **both Ubuntu and Windows** — check the live session before assuming paths.

- **Ubuntu (current PX4/Gazebo phase):**
  - Project root: `/home/shubham/Soft-Precise-Landing/`
  - PX4 install: `~/PX4-Autopilot/`
  - Working baseline Python pipeline (do not edit in place): `~/ws/scripts/soft_precise_landing/`
  - Python venv: `~/ws/scripts/env2025/`
- **Windows (Git Bash / MSYS2, mostly for paper / MATLAB work):**
  - GitHub CLI: `export PATH=$PATH:"/c/Program Files/GitHub CLI"`
- **Git user:** `Shu6hamSingha1 <shubhamsing2@iisc.ac.in>`

## Git Workflow

Commit at **logical checkpoints** (feature complete, tuning round done), not after every single edit. Batch related changes. Always push after committing:

```bash
git add <files>
git commit -m "descriptive message"
git push origin main
```

On Windows, prepend `export PATH=$PATH:"/c/Program Files/GitHub CLI"` if `gh` isn't found.

## Project Structure

```
IBVS_Manuscript.pdf            — PLASMC paper draft (IEEE TAES)
IBVS_Supplemental.pdf
README.md                      — overview of MATLAB phase
References/                    — reference papers (PDF)
Soft_Precise_Landing/          — LaTeX manuscript source (manuscript.tex, supplemental.tex, etc.)
scripts/                       — Python analysis scripts (NOT the SITL pipeline)
                                 analyze_results, plotter scripts, sweep summarizers

MATLAB/                        — Phase 1: numerical simulation (done)
  Common/
    Constants.m                — physical constants (m, J, g, camera params)
    InitVar.m                  — sim parameters & ICs (top-level shared)
    UAVDyn.m, UAVDyn_robust.m  — 13-state quadrotor dynamics
    RK5.m                      — 5th-order Runge-Kutta integrator
    traj_Gen.m                 — target trajectory generator (7 types)
    image_feature.m            — pixel → image features (s, h, w, dw)
    kappa_Solver.m             — ASMC adaptive-gain ODE (translational)
    kappa_a_Solver.m           — ASMC adaptive-gain ODE (yaw)
    smooth4.m, sat.m, moment.m, centered_moment.m, skew.m, init_robustness.m
  Multi_init_cond/
    visualControl_IBVS_adaptive.m       — main PLASMC single-run (CANONICAL)
    visualControl_IBVS_adaptive_loop.m  — Monte-Carlo / parameter sweep
    InitVar.m, InitVar_loop.m           — local overrides
    multi_Init_Var.m, multi_speed_cond.m, Adapt_Control_Params.m, probe_lateCrash.m
    plotter_adaptive.m
    plotters/                  — figure generators
  Comparison/
    visualControl_comparison.m — 5-controller comparison sim
    run_comparison.m           — entry: run_comparison(ctrl_id)
    InitGains_Comparison.m, InitVar.m
    ctrl_Lin2022.m             — Lin et al. (PBVS+PPC)
    ctrl_Zhang2026.m           — Zhang & Wu (PBVS+AEDO)
    ctrl_Chen2025.m            — Chen et al. (IBVS observer)
    ctrl_Cho2022.m             — Cho et al. (FF-IBVS)
    plotter_comparison.m, plotter_adaptive.m
    run_comparison_all.m, run_monte_carlo.m, multi_speed_comparison.m
    rerun_circular.m, rerun_other4.m, rerun_lissajous_plasmc.m, rerun_all_traj.m
    inspect_comparison.m, dump_comparison.m
    Datasets/                  — saved .mat results
  Sweeps/
    (parameter sweep scripts + Datasets/)

PX4_Gazebo/                    — Phase 2: PX4 SITL + Gazebo Harmonic (active)
  landing_test.py              — main entry: arms, takes off, runs landing loop
  controller.py                — PLASMC port aligned to MATLAB single-run
  flight_controller.py         — MAVSDK wrapper (uXRCE-DDS over udp:14540)
  gz_subscriber.py             — ROS 2 subs for /pose, /clock, /image
  img_data.py                  — ArUco detection + Lucas-Kanade optical flow
  img_data_LK.py               — alt LK tune (not in active pipeline)
  numerical_methods.py         — RK5, smooth4, extrapolate
  posctl.py                    — bare position-control demo
  target_tracking_mtr.py       — pose-only target centering demo
  calibration.py, input_calibration.py, output_calibration.py
  Images/                      — ArUco marker images (DICT_4X4_50)
  tips.txt                     — manual launch sequence (ArUco + rover scenarios)
  run_aruco_landing.sh         — one-command launcher (MicroXRCEAgent + PX4 SITL
                                 + Gazebo + 3 bridges + QGC + landing_test.py)
```

## PX4/Gazebo Phase — Quick Reference

**Stack:** PX4 SITL + Gazebo Harmonic + ROS 2 (`ros_gz_bridge`) + uXRCE-DDS (`MicroXRCEAgent`)
**Drone:** `x500_mono_cam_down` (airframe 4014); downward camera 1280×960 @ f≈540 px, hfov=1.74 rad
**Scenarios:** `aruco` world (stationary target), `rover` world (moving rover with marker, airframe 4022)
**Interface:** MAVSDK over UDP (`udp://:14540`); body-rate + thrust setpoints

**Launch (stationary ArUco):**
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo/
./run_aruco_landing.sh        # spawns all background components + runs landing_test.py
```
See `tips.txt` for the manual 7-pane launch (and the rover/moving-target variant).

**Control architecture (post-port):**
- **Outer loop (Python):** PID on raw normalized pixel error → desired feature-time-derivative `V_ds_d`
- **Middle loop (Python):** adaptive SMC on optical-flow error with log-barrier envelope; κ-ODE leakage form via RK5
- **Yaw (Python):** κ_a adaptive SMC → body yaw-rate setpoint
- **Inner attitude/rate loop:** **PX4 handles it** (we ship body rates + thrust via MAVSDK; PX4's geometric controller does actuator allocation)

## Key Conventions

- **Coordinate frame:** NED everywhere (x=North, y=East, z=Down). Altitude is negative z.
- **MATLAB phase only:**
  - Controllers 2–5 use a shared geometric SO(3) inner loop (bypass PLASMC's cascaded PID inner loop).
  - IBVS controllers (1, 4, 5) get pixel noise via `awgn(C_nP, 50, 'measured')`.
  - PBVS controllers (2, 3) get position/velocity Gaussian noise (`sigma_pos=0.01 m`, `sigma_vel=0.02 m/s`).
  - Ground effect + 1-step actuator delay applied to controllers 2–5 to match PLASMC case 1.
- **PX4/Gazebo phase:**
  - The `~/ws/scripts/soft_precise_landing/` files are the **working baseline**; never edit in place.
  - `PX4_Gazebo/` in this repo holds the MATLAB-aligned, git-tracked Python.
  - `img_data.py` keeps `_sensor_cal_s = diag(1/6, 1/6, 1, 1)` and `_sensor_cal_hw = diag(1, 1, 1, 1/3, 1/3, 1)` — Gazebo-tuned, do not remove.
  - Camera intrinsics live in `~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` (hfov=1.74, 1280×960 → fx=fy≈540).
- **Don't run things for the user:**
  - MATLAB sims: user runs them; check `.mat` result files instead. Don't invoke `matlab -batch`.
  - Gazebo SITL: GUI-bound; user runs `./run_aruco_landing.sh` themselves.
- **PDF reading:** prefer `pymupdf` (`fitz`); pdftotext drops math diacritics.

## Token Optimization

- Use `offset`/`limit` when reading large MATLAB files (`visualControl_IBVS_adaptive.m` ≈ 600 lines, `visualControl_comparison.m` ≈ 600 lines).
- Use `PX4_Gazebo/controller.py` (~600 lines) and `img_data.py` (~430 lines) judiciously — read by section when iterating.
- Don't re-explore the codebase — the structure above is the map.
- Batch git commits; don't commit after every line edit.
