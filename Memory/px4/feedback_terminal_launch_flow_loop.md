---
name: feedback_terminal_launch_flow_loop
description: "The terminal lateral fly-away is a POSITIVE-FEEDBACK LOOP THROUGH THE FLOW (motion→big flow h→c-term feedforward explodes→a_u_xy→max tilt→thrust saturates→launch→more motion). Single c-term-sub-term attacks are whack-a-mole (CTERM_DWS_MAX dead-end). Break it at the OUTPUT (COMMIT_AU_MAX) or SOURCE (tight s_e_n convergence), not the middle."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**TERMINAL LATERAL FLY-AWAY = a positive-feedback LOOP THROUGH THE FLOW (2026-06-25, GT-FB, traced
end-to-end on the IC1 32m launch + the CTERM_DWS_MAX cap test).** The loop:
```
terminal lateral motion -> big optic flow h (h_lat 11->133) -> c-term feedforward EXPLODES
 -> a_u_xy explodes (100s-1000s) -> demands max tilt (42-60deg) -> thrust law T=m*a_d_z/cos(tilt)
 SATURATES (full throttle) -> vertical force > weight -> LAUNCH up (3-8m) -> even more motion -> ...
```
The z command stays BOUNDED (per-axis theta, |a_u_z|~4) — the launch is purely the LATERAL->TILT->
THRUST-SATURATION coupling (command-level decoupling does NOT break the actuator-level coupling: a
lateral command can only be delivered by tilting, and the thrust law inflates thrust 1/cos(tilt) to
hold the vertical demand -> saturates -> climbs). [[feedback_theta_per_axis_decoupling]]

**The c-term is the AMPLIFIER and it is UNGATED.** With the switching term tamed (kappa-cap 0.03 +
per-axis theta -> switching bounded to ~+-4), the c-term `c = w_dot x s + w x(w x s) + 2w x h
- (h.e3)h - dh_d` (default form, CH_CLEAN off) became the DOMINANT terminal a_u_xy driver. At the IC1
32m launch: a_u_x = -146 = reach -1.7 + switch -4.2 + CTERM -137.7 + posbar -2.2 (cterm = 94%).
Within the c-term the `w_dot x s` (angular-accel FF) dominated (-121; |w_dot x s| is ~0 mid-descent
but EXPLODES to 40-266 in the last 15cm even on CLEAN reps that land — universal terminal spike).

**SINGLE-SUB-TERM ATTACK = WHACK-A-MOLE (the CTERM_DWS_MAX dead-end).** Added PLASMC_CTERM_DWS_MAX
(env-gated default-off) capping |w_dot x s|. Cap=10: a_u_xy STILL hit 449 — the explosion just shifted
to `-(h.e3)h` (loom x flow = 234), because ALL c-term sub-terms are fed by the SAME flow h (which blew
to 133). Clean retry (n=2, IC1-5): 8/10 sub BUT 3/10 still LAUNCHED to 5-7m (recovered stochastically),
2 fly. The cap does NOT break the loop — it just gambles on recovery. CTERM_DWS_MAX is a DEAD-END knob
(env-gated default-off, leave inert or strip). Lesson: you cannot fix this in the MIDDLE of the loop
(any c-term sub-term) — the flow feeds them all; clip one, the next takes over.

**Why / How to apply.** Break the loop at the OUTPUT or the SOURCE, h_rd CONSTANT:
- OUTPUT (robust loop-breaker): `COMMIT_AU_MAX` — cap a_u_xy at the thrust-saturation threshold so the
  drone physically can't over-tilt -> no thrust saturation -> no launch -> motion bounded -> flow
  bounded -> c-term bounded, regardless of WHICH term spikes. Already coded (default-off). A band-aid,
  but a load-bearing one for the actuator-coupling failure.
- SOURCE (principled): PREVENTION — keep s_e_n tightly converged at ALTITUDE so there's no terminal
  lateral motion to start the loop (clean reps: small terminal flow -> small c-term -> land). chi_r up
  under per-axis theta, BUT watch the terminal: any terminal lateral over-command launches (IC1 canary).
DEAD-ENDS: breach-recovery (SEN_RECOVERY_K combined-mode — fired at the universal terminal s_e_n spike
-> launched centered IC1 to 32m, [[feedback_terminal_root_lateral_zeta_r]]); c-term sub-term caps
(whack-a-mole, this file). Continues [[project_gt_feedback_control_tuning]].
