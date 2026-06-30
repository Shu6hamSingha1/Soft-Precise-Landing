---
name: project_soft_breach_idea1
description: "IDEA 1 (soft funnel-breach handling) IMPLEMENTED 2026-06-30 (PLASMC_SOFT_BREACH, default-off): on a terminal breach, pull h_e (flow) AND s_e_n/S_r (position) to FRAC*last-in-funnel (off the singular edge) instead of hard-hold/clip-at-edge; NO kappa freeze. Coexists with terminal commit (commit=clean settle; soft-breach=the breach the commit aborts on). ⚠ TODO before perception-ON: gate soft_breach on feature_fresh + freeze lateral on loss-during-breach."
metadata:
  node_type: memory
  type: project
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

> **⛔ VERDICT 2026-07-01 — SOFT-BREACH IS A DEAD-END AT EVERY FRAC (frac sweep done; keep default-off / delete path).**
> Frac sweep on the XIR=0.10 base, GT-FB, IC4/IC5 n=5 each, vs a fresh same-job soft-breach-OFF baseline (8/10):
> `frac=0.05 -> 4/9` (kappa_xy 1.07), `0.15 -> 7/10` (0.92), `0.30 -> 7/10` (0.70), `0.50 -> 6/10` (0.84).
> **NO frac reaches OFF (8/10); kappa_xy_max RISES as frac SHRINKS** (OFF 0.62 -> 0.05 1.07) — the naive
> "smaller frac -> smaller zeta_h -> bounded kappa_eq" hypothesis is REFUTED. Pooled config-matched historical
> A/B agrees: OFF (155825+175837) 12/20=60% vs SB frac0.30 (160942+181147) 8/20=40%.
> **WHY (root cause, decomposed from the sigma-breach):** the terminal sigma-breach is the FLOW barrier zeta_h
> pinning at 3.66 (NOT position — ζ_r≈0, S_r≈0, position CONVERGED) because under HD_FUNNEL_REF h_d collapses
> to ~0 once s_e_n converges, so `h_e ≈ h = v_lat/Z` = the raw 1/Z-amplified RESIDUAL LATERAL VELOCITY past p_2.
> This is a GENUINE, SUSTAINED physical error (re-breaches every frame), not a glitch. Soft-breach's two arms
> BOTH fail: (a) the POSITION arm never fires (wide PR0=10 funnel, S_r<0.95); (b) the FLOW arm is the SAME
> `if/else` branch as the kappa-freeze (the `contained[idx]=True` gate, controller.py ~1451 -> kappa-ODE ~1648)
> so enabling soft-breach INHERENTLY removes the freeze, and with the breach physically persistent kappa then
> ratchets unfrozen -> runaway (worse the smaller the frac). The kappa-freeze (OFF) is LOAD-BEARING.
> **No breach-handling (symptom-side) fix works for an un-arrested-velocity (cause-side) problem.** The only live
> lever for the lateral zeta_h breach = arrest v_lat earlier (P2INF_xy/chi_r velocity damping) or cut the 38ms lag.
> Also closed same session: W_U_MAX=2.0 INERT (0% duty), P2INF_z reduction DEAD (1.5=19/25 > 0.5=17 > 1.0=12;
> vz is lag-set not funnel-set). See [[project_why_sp_achieved]], [[feedback_terminal_smc_actuator_wall]].

**IDEA 1 — soft funnel-breach handling (user's 2026-06-29 idea, implemented 2026-06-30).**
Recalled from session 13b15a0c. Idea 1 = "instead of zeroing zeta_r, continue using zeta_r; instead of
clipping s_e_n at S_margin use ~30% of previous; instead of holding previous h_e at breach use ~30% of
previous h_e." (Idea 2 = forward prediction / lag compensation — separate, hit the scale wall in the last cm.)

**WHY (this-session diagnosis it fixes).** At a breach the hard clamp pins the feature at the funnel edge
(S_margin) where G/g_r are LARGEST -> sigma blows up AND control is starved (a_u=-G^-1*a_v->0); the Singhal
containment additionally HOLDS h_e (= freezes zeta_h, the velocity feedback) AND freezes kappa on the impact
axis (verified: kappa_x locks 0.8055, h_e_x locks, at the same step). Soft-breach pulls the breaching feature
to FRAC*(last in-funnel value) -> OFF the singular edge -> G/g_r bounded -> zeta_h/zeta_r stay LIVE (not
saturated/frozen) AND kappa_eq bounded (so NO kappa freeze AND no runaway — resolves the containment-off
dilemma where un-freezing kappa alone un-caps it toward the un-deliverable kappa_eq~124).

**IMPLEMENTATION (controller.py, env-gated default-off).**
- `PLASMC_SOFT_BREACH=1` (default 0), `PLASMC_SOFT_BREACH_FRAC=0.30` (init ~line 512).
- FLOW barrier (containment block ~1426): on `|h_e/p|>=1`, branch — soft: `ratio = FRAC*S[-1]`, reconstruct
  h_e/h, and do NOT set `contained[idx]=True` (kappa keeps adapting). The hard-hold path (original Singhal
  + contain_hold_full) is the else-branch (unchanged when soft off). The containment block is NOT skipped
  (user: "don't skip the containment block, improve the breach handling").
- POSITION barrier (~1184): on `|s_e_n/p_r|>=1-S_margin`, `S_r = FRAC*tanh(zeta_r[-1]/2)` (= FRAC*last S_r)
  instead of clip at +/-(1-S_margin). On a sustained breach both DECAY (0.30->0.09->...) -> feature relaxes
  toward center instead of fighting at the edge ("continue using zeta_r", just softly).
- Replaced an earlier `PLASMC_CONTAIN_OFF` (skip-the-block) approach — user rejected skipping.

**COEXISTS WITH TERMINAL COMMIT (kept ON).** Commit handles the CLEAN path (centered+settled -> case(b)
zeros zeta_r); it ABORTS on a breach (case(a), off-center/diverging). Soft-breach sits IN the barrier
computation so it acts on every breach step INDEPENDENT of commit -> it catches exactly the IC4r2-class case
the commit walks away from. They don't conflict (clean reps: feature centered -> soft-breach never triggers).
Self-gates to the terminal (|feature/p|>=1 only happens near the deck; funnel wide above) -> stays depth-free.

**⚠ TODO BEFORE PERCEPTION-ON VALIDATION (user-confirmed 2026-06-30).** GT-FB hides this (GT features are
CONTINUOUS to touchdown -> soft-breach always acts on valid data; the running A/B is valid for the CONTROL
question). In PERCEPTION mode, after the marker is NOT visible the features go STALE (last-decode / KLT-extrap
/ nan) -> soft-breach would (a) trigger on garbage (stale/nan h_e reads |h_e/p|>=1) and (b) decay a stale
feature (tells the controller it's centering while blind -> MASKS a real divergence). FIX: **`soft_breach &=
feature_fresh`** — AND it with the marker-visibility/decode-fresh signal the controller already uses
(KLT-exhausted / nan-quat sentinel). On loss-during-breach: **freeze lateral, keep vertical(loom)+yaw via
rings** = the commit-style handoff (the design already intends the commit as the marker-loss handler on the
CLEAN path; the gap is the BREACH path where the commit aborts -> currently no marker-loss handler). NOT yet
wired — add before perception-ON.

**FIRST TEST (running 2026-06-30).** GT-FB A/B at baked {h_rd=-0.30, XIR=0.15, P2INF=1.0}, IC4/IC5 n=5:
baseline (commit only) vs `PLASMC_SOFT_BREACH=1`. Reads: fly count, rel_vel mean+VARIANCE (does the soft
handling kill the breach kick / shrink the terminal-velocity tail?), and mechanism confirm — on breach reps
kappa_x should ADAPT (not lock 0.8055) + he_held->0; on clean reps soft-breach stays dormant. Relates
[[feedback_terminal_kick_commit_vs_live]], [[project_terminal_kick_commit_design]],
[[feedback_hd_funnel_ref_zeta_h_degenerate]] (the kappa/zeta_h freeze diagnosis), [[feedback_terminal_smc_actuator_wall]].
