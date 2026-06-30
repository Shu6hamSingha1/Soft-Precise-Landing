---
name: project_why_sp_achieved
description: "Causal synthesis (2026-06-30/07-01): why the GT-FB campaign flipped from chronic 0 SP to 19/25. The terminal 'wall' was substantially a TEST-HARNESS ARTIFACT (fake unbounded 1/Z below the gear floor, Z_REG=0.01); fixing it + cleaning the residual terminal limit cycle = SP. Not new gains."
metadata:
  node_type: memory
  type: project
---

**WHY we now achieve SP (GT-FB IC1-5), reconstructed from the session's full test history.**

## Evidence: SP onset is a sharp terminal-velocity collapse at the 06-29 -> 06-30 boundary
Same config (XIR=0.10 / P2INF_xy=1.0, "Arm B" family), `rel_med` (terminal relative
velocity = the softness tell) across the whole campaign:
- 06-25 -> 06-29 (all day): **0 SP**, rel_med **1.3-2.8** (fly-away / terminal blow-up)
- 06-30 00:12 (...012527): 3/8, rel_med **0.385**  <- collapse here
- 06-30 15:09 (...150907): 7/9, rel_med 0.026
- 06-30 22:03 (...220308, Arm B): **19/25**, rel_med 0.023

Terminal velocity drops ~50x (2.0 -> 0.02) overnight. SP did NOT arrive by gradual gain-tuning.
The 06-29 runs already had the full stability stack ([[project_bake_and_sp_walls]] 787cf2d:
funnel-ref + TERMINAL_COMMIT + TOUCHDOWN_LOOM + SAVGOL_PREDICT, banner-confirmed) and were
STILL 0 SP -> the stability stack was necessary scaffold but NOT sufficient.

## Ranked causal factors
1. **Z_REG 0.01 -> 0.2 (gear-floor fix) = THE enabler** ([[feedback_zreg_gear_floor_artifact]]).
   Lands exactly at the 06-29->06-30 collapse (before the 06:06 W_U_MAX commit). GT-FB fed
   `1/(z+Z_REG)`; at 0.01 the COMPUTED z fell below the physical ~0.2 m gear height -> fake
   unbounded 1/z->100 -> kappa_eq 107 -> sigma 3.66 ejected from the boundary layer -> blow-up
   -> fly-away. Z_REG=0.2 caps 1/z<=5 -> restores Lyapunov Assumption 1 (beta=1/Z bounded) ->
   leakage-ASMC back in its design envelope -> kappa_eq 5, sigma 1.35 -> converges -> SP.
   **The prior "0-SP terminal wall" was substantially a TEST-HARNESS ARTIFACT, not a controller limit.**
2. **W_U_MAX 1.0 -> 2.0** (b5c992d) — the 1.0 body-rate clamp's DISCONTINUITY seeded the residual
   ~1 Hz terminal limit cycle; 2.0 keeps cmd under the cap -> not seeded -> first IC5 SP. [[project_residual_cycle_wumax_bake]]
3. **VDS_KF_Q 10 -> 1** (b5c992d) — smooths `s_dot_meas` -> drift-osc amp 4.7->0.9 (the chi_r*zeta_r_dot/G cycle driver).
4. **Breach-leak fix** (uncommitted ~midday 06-30) — corrected the kappa-leak on funnel breach
   (un-froze protective kappa); took the climbing config to 19/25 (the 15->20 jump on the
   P2INF=1.5 pair 103813->133048 is this fix, NOT noise/config — user-confirmed).
5. **GAMMA_xy 0.4375/0.5 -> 0.25 + h_rd -0.42 -> -0.30** — lower reaching gain shrinks the
   terminal-cycle forcing; slow descent gives short-runway IC5 the runway to arrest v_lat.

## CAVEAT (load-bearing)
This is **GT-FB SP**, and the #1 enabler (Z_REG) is a HARNESS fix that does NOT exist under
perception-ON (there Z comes from vision, not `1/(z+Z_REG)`). What's proven: with physically-honest
depth feedback the control law converges to SP across IC1-5 -> validates the controller, exonerates
the "terminal wall = fundamental" framing. The W_U_MAX / VDS / breach-leak / GAMMA fixes DO transfer
(real controller changes); Z_REG transfers as UNDERSTANDING (bound 1/Z near the deck), not as the literal value.
The perception-ON terminal (decode/loom collapse at the deck) is a SEPARATE unclosed problem.

**Why:** the whole "0 SP / terminal wall / not gain-tunable" history was confounded by a fake
1/Z singularity the GT-FB harness manufactured below the gear floor; once removed, the controller
was already capable.
**How to apply:** when a GT-FB terminal blow-up looks "fundamental," FIRST check the harness feeds a
physically-bounded 1/Z (computed z >= gear height); judge SP onset by the `rel_med` collapse, not gains.
Don't chase new gains for a terminal wall before ruling out a harness/observability artifact.

## Side-finding this session (the XIR bake regression)
The 486f713 bake flipped XIR 0.10 -> 0.15 (validated at P2INF=1.5), but at the baked P2INF=1.0 a
controlled same-binary A/B (212524 vs 220308, soft-breach off both, only XIR differs) shows
XIR=0.15 REGRESSES: 12/25 vs 19/25 (gain concentrated in off-center IC2 1->5 / IC3 2->5, the XIR
funnel-contraction mechanism; IC5 4->2 = the predicted short-runway tradeoff). Revert XIR -> 0.10
recommended. Soft-breach (Idea 1) re-confirmed NET REGRESSION (155825/175837 baseline 5,7/10 ->
160942/181147 soft-breach 4,4/10). Relates [[feedback_validate_on_establishment_base]].
