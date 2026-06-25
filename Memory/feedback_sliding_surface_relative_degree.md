---
name: feedback_sliding_surface_relative_degree
description: "Can't stack ζ_s and ζ_h in one sliding surface — position is relative-degree-2, flow is relative-degree-1; the cascade is the correct backstepping"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ce5926b8-f1ea-4718-84c8-9a29e27ac4c6
---

> ⛔ SUPERSEDED/CORRECTED 2026-06-26: The literal σ=ζ_h+λζ_s rejection stands, but the blanket 'measurability ≠ actuated, cannot combine' is overturned — the baked σ=ζ_h+χ_r·ζ_r is well-posed because the MEASURED ζ̇_r enters u_eq as feedforward. The PX4 lateral "wall" was a gain-parity bug + the velocity-damping lever (tighten the lateral flow funnel XI2_xy), NOT a perception/architecture/inner-loop-velocity limit; the combined sliding surface σ=ζ_h+χ_r·ζ_r is baked default-on and gives 10/10 bounded landings. The residual is a terminal SOFT velocity kick (≈38ms lag), not a precision wall. See [[feedback_flow_funnel_zetah_works]]. Content below kept as history.

**REJECTED 2026-06-15 (user: "your control formulation is wrong, both"):** redefining the
middle-loop sliding surface as `σ = ζ_h + Λζ_s` (stacking the image-position barrier `ζ_s` onto
the optic-flow barrier `ζ_h=ζ_2`). WRONG on two counts, both confirmed:

1. **Relative-degree mismatch.** From the thrust-accel `a_d`: `a_d → ḣ` (flow `h` is
   RELATIVE-DEGREE-1), and `a_d → ḣ → ṡ` (image position `s` is RELATIVE-DEGREE-2). So `ζ_h` is
   rel-deg-1 but `ζ_s` is rel-deg-2: `ζ̇_s` contains NO `a_d`. You cannot place a rel-deg-2 barrier
   in a single FIRST-ORDER sliding surface — a first-order SMC on `σ` can neither enforce the
   manifold through `ζ_s` nor stabilize the position dynamics on it. (Claiming "`λζ̇_s` is a known
   measured feedforward" does not fix this — measurability ≠ actuated.)
2. **`ζ_h ≠ ζ̇_s`.** `ζ_h` is the barrier of the flow ERROR `h_e=h−h_d`, not the position-error
   rate. The "PD surface = velocity + λ·position" reading needs `h_e ≈ ṡ_e`, which drops the
   loom/descent coupling `∝ s_xy` — unfounded.

**Why:** the plant is a cascade `a_d → flow h (rel-deg-1) → position s (rel-deg-2)`. The EXISTING
ARCHITECTURE is the correct backstepping/cascade for a rel-deg-2 output: the image-feature
(position) funnel sets a DESIRED FLOW `h_d` (a desired velocity), and the rel-deg-1 optic-flow
ASMC drives `h→h_d`. That two-loop structure is not an accident to be collapsed.

**How to apply:** do NOT propose combining position and flow barriers in one first-order surface.
A legitimate single combined surface for the rel-deg-2 position would be built from `ζ_s` AND its
MEASURED RATE `ζ̇_s` (e.g. `σ = ζ̇_s + λζ_s`), where `ζ̇_s` comes from the actual optic flow via the
s-dot kinematics — NOT from the flow-error barrier `ζ_h`. Also note the honest-centroid motivation
([[feedback_sen_authority_analysis]], flow under-reports velocity) does NOT require a combined
surface — the cascade already uses the honest position to set `h_d`. Relates to
[[feedback_plasmc_two_task_framework]] (the two funnels act on physically distinct quantities).
