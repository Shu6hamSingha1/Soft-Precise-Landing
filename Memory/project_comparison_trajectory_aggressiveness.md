---
name: Comparison-paper test trajectories vs. our multi-init harness
description: Per-paper target motion for Lin 2022, Zhang 2026, Chen 2025, Cho 2022 and why our multi-init trajectories are strictly more aggressive (lateral course-reversal rate, deck attitude, IC dispersion)
type: project
originSessionId: 3e15f5af-ebb0-48e4-ae6b-fd086be8fe9b
---
Source review of the 4 comparison-controller papers to quantify how aggressive their test trajectories are relative to the multi-init harness (`MATLAB/Common/traj_Gen.m`). Source files: `References/Lin 2022.pdf`, `Zhang 2026.pdf`, `Chen 2025.pdf`, `Cho 2022.pdf`.

**What each paper actually tests:**

- **Lin 2022** (IEEE TII) — four scenarios, all slow-lateral:
  - Ex 1: straight line 5 m/s + 0.2 rad/s yaw (no lateral tracking)
  - Ex 2: straight line 0.5 m/s + 0.2 rad/s yaw
  - 3-D Case: 2 m/s + `zr = 0.2 sin(0.5 t)` heave (this is where we inherited our heave term verbatim)
  - Robustness "8-shape": `xr = 5 sin(0.1 t), yr = 5 sin(0.2 t)` → peak v = 0.5/1.0 m/s, peak a = 0.05/0.2 m/s², course-reversal ω = **0.1/0.2 rad/s** (period 31–63 s). This is the hardest lateral maneuver in any of the 4 papers.

- **Zhang 2026** (IEEE TAES) — experiment-only, straight-line fast ground platform at 8.3 m/s surge, quadrotor co-flies at 5.94 m/s with 0.4 m/s descent. No lateral tracking, no attitude oscillation, no heave. Aerodynamic/wind disturbance is the stress axis, not trajectory tracking.

- **Chen 2025** (IEEE TCST) — **STATIC target**. Simulation: MAV from `(−0.4,−0.1,7)` to `(0,0,4)` hovering over stationary feature plane. Experiment: same, stationary target on ground. Zero trajectory tracking demand — it is pure position regulation with a depth observer. Comparing a moving-target result vs. Chen is giving Chen the easiest possible scenario.

- **Cho 2022** (Aero Sci Tech) — Gazebo/PX4 sim + real ship experiments. Ship moves forward at 10 knots (~5.14 m/s) with 6-DOF Sea-State-4 wave disturbance (roll, pitch, yaw, surge, sway, heave). Stress is wave-rejection on top of a straight feed-forward line, not a curved reference the controller must actively track.

**Our multi-init trajectories:**

- Linear: 1 m/s diagonal drift + Lin heave `0.2 sin(0.5 t)` + deck roll `±15°@0.9 rad/s` + pitch `±8°@0.6 rad/s`
- Circular: r=0.5, wz=0.25 rad/s + same heave + same deck osc + tangent yaw
- Sinusoidal: A=0.5, ω=**0.8 rad/s**, drift 0.3 m/s
- Lissajous: A=0.4, B=0.8, w1=**−0.8 rad/s**, w2=**0.4 rad/s**

**Key observations:**

1. **Our lateral course-reversal rate is ~4× Lin's hardest case.** Lin's 8-shape peaks at ω = 0.2 rad/s (31 s period). Sinusoidal and Lissajous use 0.8 rad/s (period ~8 s). This is the regime where the cone-clamp IBVS limit cycle emerges — none of the 4 papers touch this regime.

2. **Peak lateral acceleration is 1.3–6× higher.** Lissajous 0.26 m/s² and Sinusoidal 0.32 m/s² vs. Lin's 0.2 m/s² max. Zhang/Chen have zero lateral accel demand.

3. **Only Cho has comparable perturbation bandwidth**, but as 6-DOF wave *disturbance* — not as a reference the controller has to track. Our deck ±15°/±8° at 0.9/0.6 rad/s is roughly Sea-State-4-class, layered *on top of* the aggressive lateral tracking.

4. **Chen 2025 doesn't test a moving target at all** — static regulation in both sim and experiment. This is the one to call out if a reviewer questions why Chen's numbers look good.

5. **Zhang's 8.3 m/s is impressive on paper but pure surge** — a constant feed-forward solves it. Our harness never runs that scenario because it's trivial under any lateral controller.

6. **Lin's heave `0.2 sin(0.5 t)` is our heave source** — retained verbatim in Linear/Circular, so Cases 2 and 3 in our harness match Lin's heave bandwidth exactly. Everything else (lateral ω, deck roll/pitch, IC dispersion ±2 m, wind, 25/25 Monte Carlo sweep) is strictly harder.

**How to apply — manuscript / reviewer-defense use:**

- When comparison tables show PLASMC outperforming the 4 baselines on Lissajous/Sinusoidal: the honest framing is that those trajectories exceed the tracking bandwidth the baselines were designed for. The baselines were each tuned for one narrow axis (Lin = slow 8-shape, Zhang = straight-line surge, Chen = static hover, Cho = wave-rejection). PLASMC's 50/50 lock-in is the superset.
- If a reviewer asks "why does Chen 2025 fail so badly on moving targets", the answer is that Chen's paper never tested a moving target — the algorithm has no target-velocity compensation path. Cite Chen Sec. V: "Simulation 1: Feature point coordinates ... set under Fw initially as (0.25, 0.2, 0 m) ..." (stationary).
- If a reviewer questions our frequency choices (0.8 rad/s) as unrealistic: Lin 2022's 8-shape at 0.2 rad/s is the state-of-the-art in the cited literature, and 0.8 rad/s is what a real gimbal-stabilized shipboard target or agile ground vehicle actually produces. Sea-State-4 wave spectra peak around 0.7–1.0 rad/s (reference Cho's Perez & Blanke citation).
- **Peak values to quote in defense:**
  - Lin 8-shape peak v = 1.0 m/s, peak a = 0.2 m/s², ω = 0.2 rad/s
  - Ours Lissajous peak v = 0.32 m/s, peak a = 0.26 m/s², **ω = 0.8 rad/s**
  - Frequency, not peak velocity, is the stress axis (because (f/z) image-plane amplification at low altitude multiplies it).
