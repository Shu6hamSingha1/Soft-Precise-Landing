---
name: feedback_vds_kf_q_severity_bandaid
description: "V_ds KF q re-baked 1.0→10.0 (2026-07-04). The 06-30 \"terminal-osc\" rationale is stale (PR0=10 handles the 1/Z); low q only damped drift-term-noise, masking the real deck-overflow root. q is a fly-away SEVERITY knob, not the fix."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

`PLASMC_VDS_KF_Q` re-baked **1.0 → 10.0** (2026-07-04, controller.py ~411; uncommitted pending
the a_u decomposition + terminal-commit work). Supersedes the 06-30 lowering to 1.0.

**The 06-30 rationale is stale.** q=1.0 was set to damp the "terminal 1/Z² s_dot oscillation"
that feeds the drift term `chi_r·ζ̇_r/G`. But the **PR0=10 funnel-shape fix (06-29)** already
absorbs the terminal 1/Z (A/B: terminal lateral-osc std stays 0.009-0.012 even at q=10). So the
terminal-osc justification is REDUNDANT (a second mitigation of what PR0=10 already holds).

**What the low q actually did:** damp DRIFT-TERM NOISE. A/B (IC2 perception ×5 each): q=10 gives
**7.6m and 31m deck fly-aways** the q=1 baseline never produced (max ~3.3m); q=1 gives milder ~1m
TLs. So q modulates the SEVERITY of the terminal over-reaction — it is a **band-aid, not the
root**. The root is the deck marker-overflow → the drone reaches the deck clean, marker overflows,
loom+lateral go blind/stale, drone bounces + reacts blind → launch
([[feedback_terminal_overflow_deck_flyaway]]). At q=10 the responsive V_ds jumps on the terminal
perc spike → aggressive a_u → big launch; at q=1 it's damped → small drift.

**Why re-baked to 10 anyway:** it restores low-lag off-center velocity (the same attenuation cost
seen on the observer KF, [[feedback_centroid_rate_observer_fixes]]), and the fly-aways it "prevents"
are a terminal-commit problem to fix at the source, not by lagging the whole-descent velocity.
**⚠ WITHOUT a terminal-commit fix, q=10 shows the 7-31m deck fly-aways** — env-revert to
`PLASMC_VDS_KF_Q=1.0` if landing on the current code before that fix lands.

**How to apply:** the V_ds KF feeds (1) the a_u drift term `chi_r·ζ̇_r/G` (dominant a_u driver)
and (2) the commit gate `ds_e_n = s_dot_meas/p_10`. If mid-velocity V_ds attenuation ever binds,
the right lever is an altitude-scheduled q (high off-center, low terminal), NOT a flat value.
Do NOT re-lower q to "fix fly-aways" — that's treating the symptom.
