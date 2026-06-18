---
name: scale-free-depth-free
description: "HARD CONSTRAINT: the PLASMC/VDF-ASMC formulation is scale-free and depth-free — no depth (Z/altitude) or metric-scale data may enter the control law, its gains, or its mode switching. This is the core novelty of the approach."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

**User directive (2026-06-03): "Remember that it is a scale-free and depth-free approach. So we can't use such data in our control formulation."**

The controller's core contribution is landing WITHOUT depth estimation, range sensing, or metric scale. Everything it uses is image-based: normalized centroids, image moments (s), optic flow (h, w — ratios with units 1/s), and pixel-space envelopes.

**What this forbids (in the control path):**
- Altitude/Z-triggered mode switches (e.g., the reverted "IBVS handoff at 0.25 m" — it read Gazebo altitude)
- Altitude-scheduled gains, Z-dependent clamps, depth-based saturation guards
- Any use of Gazebo truth pose, PX4 EKF altitude, or marker-size-derived depth inside controller.py / the command path of landing_test.py

**What remains admissible:**
- All manuscript control parameters (they act on image-space / flow-space quantities): K_rp/K_ri/K_rd, Ξ₂, p₂, 𝒳, Γ, 𝒩, 𝒫, κ(0), ℰ, p₁ (pixel-space), θ_cap, yaw set, k_R, h_rd (desired flow, 1/s)
- Gain scheduling on image quantities (e.g., |s_e_n|) — scale-free
- Test-harness uses of truth data for IC setup and for EVALUATION/classification (not control)

**Boundary clarification:** physical touchdown detection (PX4 LandedState = gear contact) is a physical event, not a depth measurement — terminating the run on it is fine. zf=0.2 m in MATLAB is the gear height (the same physical event), not a depth threshold the controller knows about.

**Why this matters for tuning:** the terminal-phase difficulties (flow and noise growing as the target gets close) must be solved by the controller's own image-based mechanisms (per-axis gains, funnel, envelope) — that IS the research problem. Any fix that sneaks in depth defeats the contribution.
