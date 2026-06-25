---
name: feedback_terminal_smc_actuator_wall
description: "⭐⭐ The terminal fly-away is an SMC CONVERGENCE failure: sigma breaches E (boundary-layer saturation -> bang-bang) because the DELIVERABLE authority is actuator-bounded (g*tan(theta_cap)=17 m/s^2) while the 1/Z disturbance is UNBOUNDED. NOT tunable on the SMC side (more kappa/Gamma/theta_cap fail or backfire). The lateral theta explosion (flow h_xy=v_lat/Z, 1/Z) -> tilt -> steals vertical thrust -> loom h_e_z breach. Fix = keep the disturbance SMALL (arrest v_lat EARLY), not the reaching law. Loom cycle killed by E_z=3 (E_z=0.5 baked too tight)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**THE terminal fly-away = an SMC convergence failure at an actuator-bounded wall (2026-06-25, user-led GT-FB).**

**Why sigma->0 is lost at the deck.** The reaching law has UNLIMITED commanded authority (a_u_xy explodes to
703 m/s^2) but the actuator delivers at most **g*tan(theta_cap=60deg) = 17 m/s^2** (2% of commanded; the tilt
clamps). Meanwhile the disturbance — the flow-rate dynamics ~ v*vz/Z^2 — grows UNBOUNDED as Z->0. **Bounded
deliverable authority vs unbounded 1/Z disturbance => sigma*sigma_dot<0 fails => sigma breaches E** (sig_xy/E
0.3->1.9, sig_z/E ->1.26). And it self-worsens: the reaching law commands MORE a_u_xy -> tilt pins 60deg ->
steals the vertical thrust (cos60=0.5) -> sig_z breaches right after. **You cannot hold a sliding mode with a
bounded actuator against an unbounded disturbance** — NOT fixable by more kappa/Gamma/theta_cap (they fail or
backfire). The ONLY fix: keep the disturbance bounded = **arrest v_lat EARLY (while Z is large)** so the 1/Z
hasn't blown it past the 17 m/s^2 ceiling. VERIFIED premise: clean reps arrest v_lat by mid-descent (<=0.5 m/s
@1m) -> breach LATE (touchdown, benign) -> sub-meter; the fly (IC5_rep2, v_lat 2.5 @1m) breaches HIGH (0.93m)
-> fly. Breach altitude TRACKS v_lat.

**The lateral theta explosion = the trigger (corrects my earlier "1/Z^2" claim — it's 1/Z).** The scalar
theta=||Theta||_F goes 4->859. ROOT: the LATERAL FLOW h_xy=v_lat/Z explodes 1/Z (->122). The c-term loom*flow
`-(h.e3)h = -h_z*h_xy` SCALES it by the BOUNDED loom h_z (~0.84-1.85, clamped) — it is NOT a 1/Z^2 squaring
(the loom is bounded). theta=sqrt(vector^2+1) -> a_u_xy 432 -> tilt 46deg -> vertical thrust collapses (0.40*
cos46=0.28, below hover) -> drone FALLS (vz->-3.5) -> h_z=vz/Z runs away -> h_e_z BREACH. So **s_e_n (Task-1)
and h_e_z (Task-2) are NOT independent at the terminal — the lateral drives the loom through TILT**
(actuator-level coupling; per-axis theta decoupled the COMMAND but not the actuator). Lateral theta explosion
PRECEDES the loom breach in every rep where both occur. h_d_z is CONSTANT (h_rd=-0.42, dh_d=0) — the breach is
h_z running away, not the desired moving.

**The loom limit cycle = boundary-layer saturation (sat(sigma_z/E_z)>=1 -> bang-bang).** E_z=0.5 (baked
2026-06-21) was MIS-SIZED TOO TIGHT -> sat saturates (5-8) -> the bang-bang relaxation oscillation (descent
balloon, period ~10s = loom-law recovery, NOT the 38ms lag). **E_z=3 KILLS it** (uncapped base: sat 5-8->0.6-
1.4, vz-flips=0, 7/8 sub vs 5/9; Ez_combo IC2 catastrophe did NOT recur b/c E_xy stayed 1.0). RESIDUAL: sigma_z
still peaks **3.66 = log((1+0.95)/(1-0.95))** — the barrier CEILING at the S_MARGIN=0.05 ratio clamp (h_e_z
pins the funnel r->0.95). E_z=4 (or P2INF_Z 1.5->2.0 to un-pin the funnel) closes it. NOT a band-aid: E_z is the
boundary-layer thickness, a core SMC param. kappa-uncap = SAFE now (no runaway — c-term/per-axis-theta fix) but
NOT the cycle fix (sat-saturation is upstream of kappa); loom CLAMP rejected (band-aid).

**How to apply.** Terminal is unwinnable on the SMC reaching side — win it BEFORE the terminal by keeping the
disturbance small: engage velocity damping (tighten flow funnel -> [[feedback_kappa_4axis_hexy_param_map]]),
keep h_rd CONSTANT. Continues [[feedback_sp_task2_terminal_limit_cycle]], [[feedback_descent_softness]],
[[feedback_terminal_launch_flow_loop]]. DEAD on the SMC side: kappa caps, Gamma, theta_cap, clamps.
