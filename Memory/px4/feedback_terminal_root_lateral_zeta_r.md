---
name: feedback_terminal_root_lateral_zeta_r
description: "RE-FRAMING (2026-06-25, GT-FB axis-by-axis dig): the terminal fly-away ROOT is the LATERAL position barrier zeta_r driving the shared scalar theta, NOT altitude/loom. Prior 'loom collapse->kz over-brake->balloon->lateral runaway' framing had causality BACKWARDS — z over-brake is collateral from shared theta. Yaw exonerated, altitude is a universal symptom."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**Axis-by-axis Control_Data dig (GT-FB, user order yaw->alt->lateral), 2026-06-25.** Contrasted
CLEAN reps (bundle 20260624-234905 IC4_rep2 0.119, IC1_rep1 0.174) vs FLY-AWAYS (002430 IC3_rep1
4.4m / IC1_rep2 2.4m; 012051 IC2_rep2 29m). Binned every loop by altitude + correlated theta.

**YAW = EXONERATED.** e_a within +-8deg the WHOLE descent on every rep (clean AND fly), |u_a|<=0.1
rad/s, kappa_a small, no growing cycle, no clean-vs-fly difference. Omega_a=0.1 holds. Not in the
causal chain.

**ALTITUDE = UNIVERSAL SYMPTOM, NOT CAUSE.** The terminal loom spike (h_z -0.42 -> -1.8 at alt<0.12m)
is INHERENT + UNIVERSAL — present even in the CLEAN 0.119 rep — structural to h_rd*Z (as Z->0 the
drone can't shed downward speed fast enough so h_z=vz/Z grows). Mid-descent loom tracking is clean in
ALL reps (h_e_z~0). At the theta peak the LOOM barrier zeta_z is SMALL (0.7-3.3). The z over-brake
(a_u_z -86 fly vs -6.5 clean) is COLLATERAL from the shared theta, not a driver.

**LATERAL = THE ORIGINATOR.** The shared scalar theta=||Theta||_F is DRIVEN by the lateral POSITION
barrier zeta_r (the combined-surface barrier on s_e_n). At EVERY theta peak, zeta_r is the LARGEST
barrier of all axes — decisively on the worst flys: FLY2.4 zeta_r=5.2 vs zz=0.8; FLY29 zeta_r=5.2 vs
zz=0.7/zxy=0.7. Mechanism: s_e_n=lat_offset/Z is 1/Z-AMPLIFIED + carries a DESCENT-DRIVEN POSITIVE
FEEDBACK (s_e_n_dot ⊃ +|h_rd|*s_e_n — descending GROWS the bearing error). Near the deck the lateral
loop can't null s_e_n fast enough -> it runs to the position-funnel floor p_r -> zeta_r->5+ -> shared
theta DETONATES -> switching term explodes on ALL axes at once (a_u_xy->1110 AND a_u_z->-86) -> fly.

**ENTRY DIFFERENCE (why clean survives):** CLEAN rep stays laterally converged (s_e_n 0.02-0.05) until
the last 12cm -> touches down BEFORE zeta_r runs away. FLY reps ENTER the terminal with un-nulled
lateral offset — IC1_rep2 drifted from the START (s_e_n 1.5 at alt 2m, on the CENTERED IC1!), IC3_rep1
took a mid-descent KICK (s_e_n 0.05->1.52 at alt 0.8m) — so zeta_r breaches at ALTITUDE with room to
detonate. OPEN: what kicks IC3 lateral at 0.8m / drifts IC1 early (next dig).

**Why:** CORRECTS the load-bearing prior framing in [[project_descent_terminal_failure]] ("terminal
descent-control failure: loom collapse -> kz over-brake -> balloon -> 1/Z lateral runaway"). Causality
is BACKWARDS there: the z over-brake is COLLATERAL; the LATERAL zeta_r drives the shared theta which
detonates everything incl z. The kappa caps (z AND xy) are all downstream band-aids on theta's output
([[feedback_kappa_clamp_bandaid]]); kappa_z uncapping catastrophe + the kappa_z cap value tradeoff are
SYMPTOMS of this lateral-driven theta explosion, not an altitude problem.

**How to apply.** Target the LATERAL position barrier, keeping h_rd CONSTANT (user: h_rd const = the
soft-landing mechanism, [[project_descent_terminal_failure]]). Two threads: (1) DECOUPLE theta
per-axis so the lateral barrier can't collateral-detonate z (fixes z over-brake; not the lateral
runaway). (2) The lateral runaway ITSELF is binding: s_e_n must be nulled (metric) BEFORE Z small —
the 1/Z amplification + descent positive-feedback make late lateral offset unrecoverable. Clean rep
proves it works when entered centered, so the lever = keep lateral offset small INTO the terminal
(lateral convergence speed / chi_r PD balance [[feedback-gtfb-lateral-orbit-divergence]] / p_r funnel
shaping), NOT anything on the altitude loop. Stop diagnosing the terminal as a descent/loom problem.
Continues [[project_gt_feedback_control_tuning]].
