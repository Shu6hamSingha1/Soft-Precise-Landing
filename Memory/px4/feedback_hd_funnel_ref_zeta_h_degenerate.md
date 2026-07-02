---
name: feedback_hd_funnel_ref_zeta_h_degenerate
description: "VALIDATED: HD_FUNNEL_REF is BAKED ON (controller.py:888, default '1', 2026-06-29) — h_d's lateral rate is the POSITION-based funnel-ref (S_r*dp_r - hd_kr*zeta_r/g_r), NOT measured s_dot. BUT empirically zeta_h is STILL degenerate (h_e exactly constant) — so HD_FUNNEL_REF is NOT un-degenerating zeta_h as its comment claims. Mechanism OPEN."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**2026-06-30 (user-led validation). Corrects several in-session mis-reads about h_d / zeta_h.**

**VALIDATED facts (code + data):**
- **HD_FUNNEL_REF is BAKED ON** — `controller.py:888` defaults `"1"`, baked 2026-06-29. The line-887
  "default-off / restores s_dot_meas" comment and the memory's "moving-target candidate, default-off"
  framing are **STALE** (predate the bake). It is NOT off / NOT at 0.
- **With HD_FUNNEL_REF on, `h_d`'s lateral rate is the POSITION-based funnel reference**
  `_hd_rate = p_10*(S_r*dp_r - hd_kr*zeta_r/g_r)` (controller.py:1358-1366), computed from `s_e_n`.
  It is **NOT** `s_dot_meas` (that's the *else* branch, line 1367, inactive). So **"h_d uses the
  MEASURED flow rate" is FALSE** for the baked config (I asserted it — wrong).

**The unresolved discrepancy:**
- Empirically (GT-FB IC4r2, the D-gate fly) **`h_e = h - h_d = -0.4519 EXACTLY constant`** for 12+ steps
  → `zeta_h = -0.62` constant → **the lateral flow barrier is DEGENERATE (carries no dynamics)**.
- So **HD_FUNNEL_REF=1 is ON but zeta_h is STILL degenerate** — the funnel-ref is NOT achieving the
  "un-degenerate zeta_h" its comment claims. The combined surface `sigma = zeta_h + chi_r*zeta_r` has
  NO live velocity-damping term (zeta_h is a constant bias); all dynamics are in `zeta_r` (position) →
  the terminal cycle grows undamped in zeta_r.
- **WHY h_e is exactly constant despite the position-based funnel-ref is OPEN.** Reconstruction showed
  the net rate term tracking `h` at corr 0.99, but the transport/descent reconstruction was imperfect
  (constant h_rd, ignored dgate + the h-vs-s_rate blend at controller.py:1304). NOT validated.
- **RESOLVED (2026-06-30, logging added — controller.py logs hd_rate/h_d_noS/s_dot_meas):** the funnel-ref
  rate term **`hd_rate` is ~0 (|hd_rate| ≈ 0.000-0.002)** in the operating regime → **HD_FUNNEL_REF is
  EFFECTIVELY INERT** (contributes ~nothing to h_d; h_d ≈ h_d_noS = transport+descent). ⚠ audit 2026-07-02:
  the "inert" scope is TERMINAL-window only — mid-flight |hd_rate| p50 0.02-0.15 (max 0.18-0.44 on Jun-30/Jul-1
  IC4 reps), ~64% of |h_d_xy| — a real h_d contributor away from the deck; "contributes ~nothing" over-generalizes. Cause:
  `_hd_rate = p_10*(S_r*dp_r - hd_kr*zeta_r/g_r)`, and S_r is tiny (wide PR0=10 funnel) + dp_r tiny (slow
  XIR / funnel at floor) → `S_r*dp_r ≈ 0`; centered ζ_r small → the hd_kr term ~0 too. So the funnel-ref
  it's supposed to inject is ~0 → it can't un-degenerate ζ_h (no rate to inject). DESIGN CONFLICT: making
  hd_rate non-negligible needs a TIGHTER/faster funnel (larger S_r*dp_r) = exactly the EDGE-FORCING that
  flew IC4r2 at XIR=0.15. So HD_FUNNEL_REF's velocity reference can't be engaged without the edge-forcing
  it's meant to avoid. ⚠ ALSO: the "ζ_h degenerate -0.62" was BREACH-TERMINAL-specific (h_e grew there);
  in normal flight h_e std ~0.01-0.04, ζ_h ~0. And the "no velocity damping" framing was OVERSTATED — the
  DRIFT `chi_r*ζ̇_r` (via s_dot_meas, in u_eq) carries velocity feedback regardless of ζ_h. The narrow
  VALIDATED fact: hd_rate≈0 → HD_FUNNEL_REF inert, gated by wide-funnel-vs-edge-forcing.

**RESOLVED 2026-06-30 — the kappa-freeze AND the "h_e_x constant / zeta_h degenerate" are the SAME
thing: the Singhal OUTLIER CONTAINMENT firing on x.** Computed kappa_eq=theta*G*|sigma|/P with the REAL
P_xy=1.5 (not 15): it ramps 16->124 through the IC4r2 breach, so the ODE DEMANDS dkappa/dt~2-18/s (rise
~0.4-3.0), yet kappa_x sits EXACTLY 0.8055 -> the ODE output is being DISCARDED = frozen, not slow.
Traced the lock: kappa_x AND h_e_x lock to constants at the SAME step (~1028). That's the containment
(controller.py:1408-1412 reconstructs/HOLDS h_e to ratio*p, line 1610 freezes kappa on the contained
axis). The |h_e_x/p|=0.30 I kept citing is the POST-reconstruction HELD value, NOT the trigger (the
containment fires on the GENUINE pre-reconstruction |h_e/p|>=1, then clamps h_e down). So: (1) the "0.30<1
so not contained" reasoning was wrong; (2) the "h_e_x exactly -0.4519 constant / zeta_h degenerate" was
NEVER degeneracy — it's the containment HOLDING h_e_x. MECHANISM: at the terminal the violent lateral
motion spikes the x-flow error past the funnel -> containment reads it as a perception GLITCH -> freezes
kappa_x + holds the flow feedback ON THE IMPACT AXIS, exactly when the adaptive law is needed. In GT-FB
there are NO glitches -> containment is purely harmful here (already flagged in the tuning skill).
**"Make kappa fast" does NOT fix it: (a) N is irrelevant while the containment overwrites the ODE output;
(b) even unfrozen, kappa_eq=124 is UN-DELIVERABLE (actuator g*tan(60)~17) -> fast kappa chases it ->
the N=1.0 runaway/launch (kappa 0.42->13.5, a_u 64x).** The adaptive law works at ALTITUDE (deliverable,
slow disturbance) and fails ONLY in the terminal sliver, for TWO reasons that are NOT speed: containment
disables it + the disturbance exceeds the actuator envelope. LEVERS: disable/gate the containment in GT-FB
(immediate freeze fix, necessary-not-sufficient) + keep the disturbance deliverable (arrest v_lat early /
cut the 38ms lag so kappa_eq never spikes to 124). NEXT A/B candidate: containment-off in GT-FB on the
IC4r2-class case. Relates [[project_residual_cycle_wumax_bake]], [[feedback_terminal_smc_actuator_wall]],
[[feedback_validate_on_establishment_base]].
