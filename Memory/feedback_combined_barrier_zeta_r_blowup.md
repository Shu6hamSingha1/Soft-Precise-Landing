---
name: feedback_combined_barrier_zeta_r_blowup
description: "⭐ WHY the baked combined-barrier FLIES OFF (2026-06-20): the position log-barrier zeta_r=log((1+r)/(1-r)) sits DIRECTLY in sigma, so when the terminal 1/Z amplification pushes s_e_n toward the funnel edge p_r, zeta_r->infinity -> sigma explodes (0.13->3.3->7.0) -> a_u SPIKES to ~130 m/s2 -> max-tilt command -> marker whipped out of FoV -> TARGET_LOST. Smoking gun: at the 130 spike h_e is only 0.07 (NOT the flow error -- it's ALL chi_r*zeta_r). OPPOSITE failure from back-mapped (G_s^-1->0 = UNDER-react); combined = OVER-react (unbounded log-barrier). Fix = command-bounding cap (bound the command so zeta_r blow-up cant whip out): combined+cap."
metadata:
  node_type: feedback
  type: feedback
---

**The combined-barrier bake (`PLASMC_COMBINED_BARRIER`+`VDS_KF` default-on, a82328b) REGRESSES: flies
off at IC2-5** (IC2 4/5 fly, IC5 5/5 fly 0/5 land vs back-mapped's 3/5 fly 2/5 land — a NET regression).
Loom+KF gate (FLOW_LOOM_DECOUPLE+DHD_KF+DW_KF on top) = MIXED, no clean pass.

**MECHANISM (traced on a LoomKF_IC2_base fly-away rep, s_e_n converges 0.04@1.7m then flies to 6.5m):**
the combined surface `sigma_xy = zeta_h + chi_r*zeta_r` puts the POSITION log-barrier `zeta_r` directly
in sigma. As the terminal 1/Z amplification drifts s_e_n toward the funnel edge `p_r`, `zeta_r → ∞`:
| t | alt | s_e_n | sigma_xy | a_u_xy | h_e_xy |
| 2.7| 1.70| 0.04 | 0.13 | 9.8 | 0.25 |
| 3.1| 1.67| 1.29 | 3.32 | **129.5** | **0.07** |  ← a_u SPIKE, h_e tiny
| 3.8| 1.30| 1.19 | 7.03 | 26.6 | 3.77 |
The 130 m/s2 a_u spike → max-tilt (theta_cap) command → marker whipped out of FoV → fly-away. SMOKING
GUN: at the spike `h_e=0.07` (flow error tiny) → the blow-up is ALL the position barrier `chi_r*zeta_r`
(end: sigma_xy=[-0.74,0.51], zeta_h=[2.37,-2.61] → chi_r*zeta_r=[-3.1,+3.1] dominates). Feedback loop:
violent tilt → w_i spikes → cross(w_i,s) → h_d spikes to 7-9 → h_e spikes → more a_u.

**THE TWO FORMULATIONS FAIL IN OPPOSITE WAYS at the terminal breach (answers the Q2 authority question):**
- BACK-MAPPED: `G_s^-1=(1-r^2)p_s/2 → 0` as r→1 → demand COLLAPSES → UNDER-react (not enough authority).
- COMBINED: `zeta_r=log(...) → ∞` as r→1 → sigma explodes → OVER-react (130 m/s2 whip-out).
So the combined surface FIXED the authority deficit — too well; its position authority is an UNBOUNDED
log-barrier. **FIX = COMMAND-BOUNDING CAP** (`PLASMC_COMMIT_DSD_MAX`): bound the lateral command so the
zeta_r blow-up can't produce the catastrophic spike → combined+cap = position authority delivered WITHOUT
the violent terminal command. NEXT = gate combined+cap IC2-5 vs back-mapped. Relates to
[[feedback_lateral_wall_anti_restoring_au]] (command-bounding closed IC1) + [[feedback_combined_surface_divergence]].

**⛔ combined+cap GATE FAILED — the V_ds_d cap is INERT in combined mode (2026-06-20).** IC2-5 (IC2,IC5,
IC3 done): combined+cap WORSE than back-mapped (IC2 STD 7.3->24.9 + 63m blowup; IC5 3/5fly 2land ->
5/5fly 0land). ROOT: PLASMC_COMMIT_DSD_MAX caps V_ds_d (controller.py:806) but the combined-barrier path
RETURNS at line 724 (before the cap) and uses ds_d=0 — its lateral demand is zeta_r->sigma->a_u, NOT
V_ds_d. So the cap NEVER fires in combined mode -> combined+cap == combined == fly-away. To bound the
combined-barrier's a_u=130 spike you must cap **a_u_xy DIRECTLY** (in PLASMC after a_u computed, when
committed), NOT V_ds_d. (The sigma is actually bounded ~7 by the S_r/S clamps; the a_u 130 is G^-1
amplification of a_v when the interaction matrix G goes ill-conditioned near terminal -> cap the OUTPUT
a_u.) NEXT = either implement an a_u_xy commit-cap for combined mode + re-gate, OR REVERT the combined
bake (a82328b) since it regresses and the existing cap can't fix it.
