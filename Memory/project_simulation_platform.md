---
name: Simulation platform: MATLAB done, PX4/Gazebo port underway
description: MATLAB PLASMC phase is complete and submitted; project's active phase is the PX4 SITL + Gazebo Harmonic port at PX4_Gazebo/
type: project
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---
**Phases**

1. **MATLAB (done):** PLASMC controller + 5-controller comparison + Monte-Carlo sweeps. Manuscript drafted at `Soft_Precise_Landing/manuscript.tex`. Code under `MATLAB/{Common, Multi_init_cond, Comparison, Sweeps}/`.
2. **PX4/Gazebo (active):** Python port of PLASMC running on PX4 SITL + Gazebo Harmonic + ROS 2 + uXRCE-DDS. Lives at `PX4_Gazebo/` in this repo (added commit `b743f8e`, 2026-05-12).

**Why:** Natural progression from idealized MATLAB simulation toward higher-fidelity SITL validation, then HIL, then real flight.

**How to apply:**
- When the user says "the simulation," disambiguate: legacy MATLAB run (`MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`) vs the PX4/Gazebo run (`PX4_Gazebo/landing_test.py`).
- The Python port keeps PX4's internal rate controller — only the **outer position loop + middle adaptive-SMC loop + yaw κ_a SMC** are ported. PX4 owns the inner rate/attitude loop. Don't try to port the geometric SO(3) inner loop.
- Original Python pipeline at `~/ws/scripts/soft_precise_landing/` is the **working baseline**; the `PX4_Gazebo/` copy in this repo is the **edited, MATLAB-aligned version**. A/B comparison expected during tuning.
- For new controller iterations: edit `PX4_Gazebo/controller.py`, not `~/ws/...`.
- `MATLAB/Comparison/` contains four reference controllers (Lin 2022, Zhang 2026, Chen 2025, Cho 2022) — not yet ported to PX4/Gazebo. Likely next paper / experiment.
