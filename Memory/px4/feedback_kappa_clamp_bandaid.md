---
name: feedback_kappa_clamp_bandaid
description: "User: the KAPPA_MAX clamp that fixes the terminal fly-aways is a BAND-AID — later tune control params so the clamp rarely triggers. Gate-wide result: KAPPA_MAX=0.03 (GT-FB) clears the IC1/IC4 fly-aways but the clamp is the binding element, not an inert backstop."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**User guidance (2026-06-25): the κ-clamp is a band-aid; the real work is to tune the
control law so the clamp is triggered RARELY** (it should be an inert backstop, not the
binding element). Same principle as [[feedback_clamps_during_tuning]] ("clamps are
band-aids") and [[feedback_fix_causes_not_limits]], reaffirmed for the GT-FB
saturation context.

**Why.** Capping κ (KAPPA_MAX_X/Y/Z) bounds the OUTPUT of the switching term
`θ·sat(σ/E)·κ` so PX4 doesn't saturate, but it does not stop the term from WANTING to
over-command. The terminal fly-away is a PX4-saturation-induced divergence
([[feedback_gtfb_lateral_orbit_divergence]], [[feedback_gtfb_kappa_z_bounce]]): near the
deck (1) σ leaves the boundary layer (sat → ±1), (2) κ adapts upward during the limit
cycle, (3) the shared scalar θ inflates from the terminal barrier / c-term blow-up. The
clamp masks all three; it doesn't remove them. On CLEAN mid-descent reps κ already sits
at its natural ~0.03 and the cap is inert — the goal is to make the WHOLE descent (incl.
terminal) look like that.

**Empirical anchor (GT-FB IC1-5 gate, this session, bundle 20260624-234905).**
KAPPA_MAX=(0.03,0.03,0.03) + XI2=(0.2,0.2,1.0) + VDF gains + K_R rp=2.5 + Ω_a=0.1 +
h_rd=-0.42: **first gate-wide confirmation the κ-cap clears the terminal fly-aways** —
max xy over 10 reps = 1.04m (vs baseline IC1 7.8m / IC4 11.7m blow-ups); 8/10 sub-meter,
4 sub-0.25 (IC1 0.174, IC4 0.119/0.138, IC3 0.082). BUT the cap is the binding element,
not a backstop, and it introduced a NEW failure: **no-descent hover-stall** (IC1_rep2 sat
at ~6m for 46s, never descended) + slow-descent struggles (IC3_rep1 61s, IC4_rep1 38s),
because KAPPA_MAX_Z=0.03 forces κ_z down from its bootstrap κ0_z=0.25 at step 1 →
starves DESCENT-INITIATION authority at altitude (loom≈0 above 4m). The Z-cap has
competing jobs (high enough at altitude to descend; ≤0.034 at the deck to avoid the
G_z≈99 bounce) → one constant can't serve both → that tension IS the symptom of treating
output instead of cause. Soft touchdown still unmet (0/10, vel 0.5-3.4, lag-limited).
Follow-up in flight: KAPPA_MAX_Z 0.03→0.08 (X/Y kept 0.03) to restore descent authority
(bundle 20260625-002430, log ic_gate_kmaxz008.log).

**How to apply.** Use KAPPA_MAX caps as a DIAGNOSTIC/scaffold to prove the
saturation-divergence mechanism and unblock the gate — NOT as the final fix. The real
levers (future work) are the ones that keep κ small and σ inside E NATURALLY through the
terminal: boundary layer E, funnel/σ shaping (XI2/P2INF), κ-ODE adapt/leakage (N/P), the
shared-θ / c-term blow-up, and the terminal loom-collapse that drives σ_z (reference
governor — [[project_descent_terminal_failure]]). Success metric for that work = the κ
clamp is rarely/never the active limiter (κ stays ≈natural; sat() stays <1). Continues
[[project_gt_feedback_control_tuning]].
