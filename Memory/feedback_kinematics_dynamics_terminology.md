---
name: feedback_kinematics_dynamics_terminology
description: "Kinematics-vs-dynamics terminology — use 'image position/orientation kinematics' everywhere (never '…-error dynamics'); litmus: forces/actuators in the statement → dynamics, geometry-of-motion only → kinematics (coupling=kinematic, disturbance-sensitivity=dynamic)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: cee6e71e-7634-484c-b772-85dc1fc380cb
---

Consolidated kinematics-vs-dynamics terminology. Merges two feedback memories: the fixed "image position/orientation kinematics" phrasing and the force-litmus for the word choice paper-wide.

## Phrasing — "image position kinematics" / "image orientation kinematics" (uniformity)

The equations $\dot{\alpha}_e = -\dot{\psi}_d + d_\alpha$ (orientation) and $\dot{\boldsymbol{s}}_e = \boldsymbol{h} - \omega\times\boldsymbol{s} - \ldots$ (position) are introduced in `control_formulation.tex` under the exact labels:
- "image position kinematics" (line 130)
- "image orientation kinematics" (line 149)

**Why:** User flagged 2026-04-24 when I wrote "image-orientation-error dynamics" in Theorem 2's preamble. The paper already uses "image orientation kinematics" at the definition site, so any downstream reference must reuse the same phrase for uniformity.

**How to apply:**
- When referring back to these equations in Theorem statements, proofs, or prose, use the same noun phrases — "image orientation kinematics" / "image position kinematics" — NOT improvised variants like "orientation-error dynamics", "α_e dynamics", "image-orientation kinematics" (hyphenated), etc.
- Applies across both the main paper and supplement.

## Litmus — kinematics vs dynamics word choice (locked 2026-06-10)

**Litmus test (user-endorsed 2026-06-10):** in any sentence, ask whether forces, masses, or actuator effects participate in the statement.

- **Kinematics** — relations among motion quantities (position, velocity, acceleration) imposed by projection geometry; no forces. The s–h coupling (cross products of w, ẇ, s, h), the 1/z scaling growth, the interaction-matrix relations, §II's "image parameter kinematics". Even ḣ written in terms of acceleration is kinematic — acceleration is a motion quantity.
- **Dynamics** — the force-driven system: anything involving thrust, wind gusts, ground effect, actuator nonlinearities, stability under disturbance, or what a controller stabilizes. "Image-based landing dynamics" = rigid-body dynamics cascaded with image kinematics.

**Why:** quadrotors are force-controlled and underactuated; classical velocity-input IBVS is kinematic control, thrust/torque-input UAV visual servoing is "dynamic visual servoing". Only optic flow ever acquires dynamics (ḣ contains acceleration → force); position-type image parameters have only kinematics. Coupling = kinematic; disturbance sensitivity / stabilization = dynamic.

**How to apply (corrected-sentence examples):**
- "Coupled through nonlinear landing kinematics" (abstract sentence 2) — correct, structural claim.
- "Renders the image-based landing dynamics sensitive to ground effects/gusts/actuator nonlinearities" (§I para 6) — corrected from "image kinematics" 2026-06-10; sensitivity to forces is a dynamics statement.
- "Controllers address these coupled nonlinear landing dynamics" (§I para 7) — corrected likewise; controllers stabilize dynamics, not geometry.
- Contributions 1–2 "image-based landing dynamics" — correct (stabilizing the uncertain forced system).
- Mixed-layer sentences should split: coupling = kinematics; force sensitivity / stabilization = dynamics.

Related: [[feedback_target_image_parameters]] (only optic flow couples to body force).
