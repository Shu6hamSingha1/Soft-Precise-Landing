---
name: Phase 1 MATLAB baseline — controller works, PX4 plant is the bottleneck
description: Same controller + gains + IC + noise model achieves 100% soft+precise in MATLAB but 0% in PX4 SITL. The 16× xy_err ratio is SITL-specific, not a controller-design ceiling.
type: feedback
originSessionId: 3ffdaf6f-0da4-4c9f-b4df-fb9a91a7eb04
---

## Top finding (2026-05-22)

The PX4 SITL precision plateau is NOT a controller-design ceiling. The same PLASMC controller, same gains, same noise model, same IC achieves **10/10 soft+precise** in MATLAB but **0/30** in PX4. The 16× gap is SITL-specific (sensor pipeline, actuator response, rate-loop lag).

| Condition | n | xy_mean | xy_min | vel_mean | SP rate |
|---|---|---|---|---|---|
| MATLAB @ IC(0,0,-5) + 50dB noise | 10 | 0.030 m | 0.008 m | 0.120 m/s | 10/10 (100%) |
| MATLAB @ IC(0,0,-5) noiseless | 1 | 0.0019 m | — | 0.115 m/s | 1/1 |
| MATLAB @ IC(2,2,-5) + 50dB noise (canonical) | 5 | 0.032 m | 0.021 m | 0.094 m/s | 5/5 |
| PX4 SITL @ IC(0,0,5) loose | 10 | 0.484 m | 0.033 m | 0.890 m/s | 0/10 |
| PX4 SITL @ IC(0,0,5) tight | 9 | 0.517 m | 0.047 m | 0.840 m/s | 0/9 |
| PX4 SITL @ IC(0,0,5) ultra-tight | 9 | 0.531 m | 0.019 m | 0.640 m/s | 0/9 |

## What this overrides

`feedback_instability_mechanism.md` correctly identified **multi-factor IC sensitivity** as the proximate mechanism on the PX4 side. But its statement that the ~0.4 m floor is "a structural property of the SMC-driven 4-second descent" needs nuance:

- The structural sensitivity to ICs is **real on PX4** because PX4 ICs have ~16× more variance than MATLAB ICs (sensor noise on initial state estimation, EKF jitter, image-pipeline lag).
- Eliminate the noise → MATLAB shows the same controller can handle ICs with sub-cm precision.
- Therefore: gain tuning + IC tightening on PX4 were diagnosing a symptom of plant noise, not a controller-design issue.

## What changes about future work

- **Stop chasing PX4 gain tunings.** N=620 SITL runs concluded "stochastic-variance-limited." Now confirmed as a property of the PX4 plant, not the controller.
- **Phase 2 (loop-latency budget)** and **Phase 4 (sensor-noise floor)** become the actionable next steps. Quantify which SITL-pipeline noise source dominates the 16× gap — image SNR, MAVSDK→PX4 rate-loop lag, PX4 actuator response, EKF drift.
- **If image SNR turns out to dominate**: improve sensor calibration / use larger marker / increase camera resolution.
- **If MAVSDK rate-loop lag dominates**: this is the longstanding "do we need torque commands instead of rate commands" question (currently forbidden per `feedback_thrust_torque.md`).

## MATLAB toolchain bootstrap (one-time, completed 2026-05-22)

The MATLAB pipeline as committed required several Robotics System Toolbox functions that aren't in the standard education bundle. Replaced with local shims in `MATLAB/Common/`:
- `quat2rotm.m` — quaternion → 3×3 rotation matrix
- `quat2eul.m` — supports both 'ZYX' (default) and 'XYZ' sequences used in the codebase
- `rotx.m` / `roty.m` / `rotz.m` — degree-input axis rotations

`InitVar.m` previously referenced undefined `K.kappa_0` / `K.kappa_a_0` (the canonical script overwrites these later from `K_ctrl.kappa_0`, but the unconditional reads on a fresh workspace throw). Now uses `isfield` guards with sensible defaults.

The `phase1_baseline_sweep.m` wrapper uses a `global WS` struct to survive the canonical script's `clear`.

## How to apply

- When the user asks why PX4 can't hit soft+precise: answer is **the SITL plant adds 16× more variance than MATLAB's reference**. Not the controller. Cite this finding.
- Before suggesting any controller-side change for precision: check whether the suggestion would help the PX4-specific noise/lag sources, not just nominal performance.
- Reuse the MATLAB toolchain for any future MATLAB-vs-SITL comparison (Phase 3 trajectory replay, etc.). The shims + IC override are now permanent.

## Data

- `MATLAB/Datasets/Phase1/phase1_summary.mat` — 16-run aggregate
- `MATLAB/Datasets/Phase1/rep_*.mat` — per-rep tiny metadata files
- Analyzer: `PX4_Gazebo/analyze_matlab_phase1.py`
- MATLAB wrapper: `MATLAB/Multi_init_cond/phase1_baseline_sweep.m`
