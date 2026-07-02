---
name: feedback_zreg_gear_floor_artifact
description: "The GT-FB terminal limit cycle / unbounded-1/Z / kappa_eq explosion was an ARTIFACT of Z_REG=0.01 (computed depth below the physical gear floor); Z_REG=0.2 (gear height) is physical -> bounded 1/Z -> SP. Resolves the whole terminal-cycle campaign."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**2026-06-30 (user insight). The terminal limit cycle was a GT-FB FEATURE-SYNTHESIS ARTIFACT, not a real control failure.**

gt_feedback.py computes every 1/z-dependent feature as `1/(z+Z_REG)`. Z_REG was 0.01 → let the COMPUTED
relative depth z fall to 0.01 m, FAR below the physical ~0.2 m LANDING-GEAR floor → synthesized a 1/z
spiking to ~100 that NEVER PHYSICALLY OCCURS (the gear stops descent at z>=z_gear≈0.2; image perception
also saturates there — marker fills FoV, decode fails). That fake unbounded 1/z drove the whole campaign's
terminal pathology: kappa_eq=θG|σ|/P → 107, σ → barrier ceiling 3.66 → bang-bang, the "leakage-ASMC can't
reject unbounded 1/z" failure, the "two SP walls", the governor conclusion.

**FIX: Z_REG = the PHYSICAL depth floor (BAKED gt_feedback.py default 0.01→0.1).** z>=z_floor → true 1/z
bounded → Lyapunov Assumption 1 (β=1/z bounded) HOLDS → leakage-ASMC in its DESIGN ENVELOPE. Z_REG sweep
{0.01,0.05,0.1,0.2} IC2 n=1: κ_eq 107→15→**5**, σ_max 3.66→1.87→**1.35** (audit 2026-07-02: the three cited points are exact but the unlisted 0.05 run has κ_eq 168 — strictly NON-monotone over all 4, n=1 noise) (back in boundary layer,
no bang-bang); Z_REG=0.2 → SP (rel 0.058, xy 0.064). ⚠ EMPIRICAL FLOOR CHECK (user-requested, 27 reps):
min relative z=|UAV_z-Target_z| MEDIAN **0.096 m** (TOUCHDOWN_LOOM fires z~0.11-0.14), abs-min 0.001 (1
post-disarm glitch), 96% of reps reach <0.2m. So the "z never <0.2m / gear=0.2m" claim is REFUTED — the
real floor is **~0.1 m**, NOT 0.2. Z_REG=0.2 over-clamped (capped 1/z at 5 vs the real ~10 + ~17% altitude
under-read); **BAKED Z_REG=0.1** (caps 1/z at 10 = the measured floor, less over-clamp). Additive 1/(z+0.1)
under-reads ~9% at z=1m; 1/max(z,0.1) is exact if altitude bias bites. Confirmation IC2/4/5 n=3 running.

**IMPLICATIONS (supersede much of the 2026-06-2x terminal-cycle framing):**
- The leakage-ASMC was NEVER broken. It provably can't reject an UNBOUNDED 1/z — but the real 1/z is
  BOUNDED by the gear. The "fundamental terminal wall" was a synthesis artifact.
- Γ/χ_r/E_xy/N/P/K_R all "failed to tame the cycle" because you can't out-tune an unbounded disturbance —
  but the disturbance was never really unbounded. (Those characterizations are still individually correct;
  the cycle they were fighting was the artifact.) Γ=0.25/PR0=10 bakes STAND (still the optimum); E_xy=4 is
  no longer needed (no bang-bang at Z_REG=0.2).
- Only the LOOM-commit (open-loop settle, LANDING_COMMIT_EXTENT, already default-off) was REJECTED by the
  user. TERMINAL_COMMIT (controller-side s_e_n→0 ramp / zero-ζ_r commit) is KEPT (baked ON) — user
  explicitly wants it (I wrongly disabled it then reverted). TOUCHDOWN_LOOM is a DETECTOR (not a commit),
  kept. No descent governor needed — THE GEAR IS THE PHYSICAL GOVERNOR (it bounds z).
- ⚠ GT-FB-specific: Z_REG only exists in gt_feedback.py (GT mode). In PRODUCTION (perception) the image
  features are physical and saturate naturally at the gear/FoV — so perception never sees the fake 1/z
  either. Z_REG=0.2 just makes GT-FB FAITHFUL to perception.
- NUANCE: the additive form 1/(z+0.2) under-reports ~17% at altitude (z=1: 0.83 vs 1.0); 1/max(z,0.2) is
  more physically exact. Additive kept (simpler, user-specified); refine to clamp if altitude bias matters.
- Confirmation run (clean baked config, IC2/IC4/IC5 n=3) launched 2026-06-30. Baked state: Z_REG=0.2,
  TERMINAL_COMMIT off, W_U_MAX=1.0 re-engaged (inert), Γ=0.25, PR0=10, E_xy=1.0.
Refines/supersedes [[feedback_prinf_standing_condition]] terminal-cycle conclusions, [[project_bake_and_sp_walls]] (the 2 SP walls were artifact-driven), the loom-commit memories.
