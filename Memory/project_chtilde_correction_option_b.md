---
name: project_chtilde_correction_option_b
description: "Manuscript c̃_h kinematics correction — TASK 1 option (b): corrected controller + regenerated results"
metadata: 
  node_type: memory
  type: project
  originSessionId: 5ef5a2d7-329a-4539-b101-9f7e6204c84a
---

Manuscript session (Windows, 2026-06-15) resuming after the Ubuntu MATLAB robustness work.
Handoffs: `WINDOWS_HANDOFF.md` (root), `Soft_Precise_Landing/MANUSCRIPT_DATA_HANDOFF.md`.

**TASK 1 — c̃_h / d_h kinematics correction.** manuscript.tex ALREADY corrected (ṡ L196, ḣ L208,
d_h L212 — no Coriolis). supplemental.tex S1-B still OLD (c̃_h L96-97, d_h L99 Coriolis, text L101);
control_formulation.tex is LEGACY (standalone, superseded by single-file manuscript.tex) → SKIP.

**USER CHOSE OPTION (b)** (over my recommended (c)): present corrected model AND regenerate results
with a corrected controller that doesn't regress. The old full-w c̃_h is analytically falsified
(transport theorem) + numerically (2.5-4x residual vs .mat GT). C_SIMPLE (global, default-off) =
the simple manuscript c̃_h = -psi_dot_b*(e3 x h) - (e3.h)*h using clean IMU yaw rate.

**FEASIBILITY DIAGNOSIS (decisive):** C_SIMPLE regresses IC5 noisy SP 10/12→6/12, BUT the regression
is purely TERMINAL — trajectories near-identical through t<4.6s (alt 0.35m); C_SIMPLE often SOFTER on
easy seeds. 3/4 regressions (seeds 10,11,12) = marginal LATERAL-velocity overshoot at touchdown
(|v_xy|=0.27 vs |v_z|=0.15; vx ramps -0.10→-0.27 in final 0.12s chasing moving Circular target,
xy stays precise). Seed 8 = terminal/CBF-observability WALL (theta_cone→55° saturates, marker at FoV
edge, correction stripped at high 1/Z) — SAME documented IC5 wall as seeds 4,6; NOT a c-term bug; the
leaner-correct feedforward just has less margin. kappa_z clamp didn't fix seed 8 (spike drives u_eq
ahead of kappa).

**FIX: K_rd 1.4375→2.5** (SEN-funnel lateral damping, via SEN_RD_OVERRIDE). Sweep on seeds {1,5,10,11,12}:
K_rd=2.5 → 5/5 SP (marginals 0.28-0.34→0.13-0.15, easy stay SP, land earlier=cleaner). Narrow sweet
spot (4.0 over-damps xy, 6.0 diverges). Added global-gated KAPPA_MAX/KAPPA_MAX_Z clamp (default-off,
anti-runaway backstop; didn't fix seed 8 so not adopted).

**RUNNING:** gate_csimple.m — IC1-5 no-regression gate, old-c base (K_rd=1.4375) vs C_SIMPLE+K_rd=2.5
(noiseless n=1 + noisy n=5). If no-regression on IC1-4 (esp IC1 centered) holds → option (b) viable →
bake C_SIMPLE+K_rd=2.5, regenerate results, then supplement.tex edits (corrected c̃_h/d_h).
**GATE PASSED + FINALIZED (2026-06-16):** gate_csimple.m IC1-5: C_SIMPLE+K_rd2.5 = SP 30/30 vs old-c
base 29/30 — NO regression on any IC (IC1 centered perfect), IC5 noisy 4/5->5/5. Full 12-seed IC5:
C_SIMPLE+K_rd2.5 = 9/12 vs old-c 10/12, BUT eliminates catastrophic fly-aways (old-c seed4 v1.43->0.094;
seed8 v2.07->0.26), softer mean (0.32 vs 0.42). K_rd refine {2.5,2.8,3.0} ALL = 9/12 — marginal misses
just SHUFFLE (seed6 = genuine CBF-strip wall, ~2 of 11 landable always tip just over 0.2 cutoff). NOT a
tunable. **DECISION: K_rd=2.5** (gate-validated, no need to re-run gate at 2.8). Corrected controller =
equivalent-to-SAFER. Option (b) VIABLE: model+code agree, results don't regress.

**MULTI-TRAJECTORY TEST (2026-06-16, the decisive one):** tested corrected controller across the FULL
suite (5 traj x 5 IC, noisy n=1, TRAJ_OVERRIDE hook added). C_SIMPLE+K_rd2.5 = 23/25 vs old-c 24/25.
EQUIVALENT on Linear/Sinusoidal/Circular/Lissajous; regresses 1 cell at **Static IC5 [2,2,-3]**. Drill-
down (Static IC5, 5 seeds): old-c 3/5, C_SIMPLE+K_rd2.5 2/5, C_SIMPLE+K_rd1.44 1/5 — Static IC5 is a hard
wall for BOTH (old-c also fov-fails seeds 3,4); C_SIMPLE is ~1 seed worse. **K_rd is NOT the cause**
(C_SIMPLE fails Static IC5 at both 1.44 AND 2.5) — it's the CORRECTED FEEDFORWARD being marginally less
robust on IC5, exactly as the handoff warned ("full-w terms do real feedforward work"). CONCLUSION:
option (b)'s premise (no regression) DOESN'T fully hold — corrected controller is ~1-seed-worse on IC5
across trajectories, equivalent elsewhere. This STRENGTHENS option (c): corrected MODEL is right, no
robustness payoff to regenerating with the (marginally worse) corrected controller; keep old-code results,
frame the extra feedforward as bounded residual ->0 as s_xy->0 absorbed into d_h. **DECISION PENDING:
(b) accept modest IC5 regression for full consistency, vs (c) revisit (now better-supported).**

**IC5 RECOVERY ATTEMPTS FAILED -> option (b) BLOCKED (2026-06-16).** Tried to recover the corrected
controller's IC5 deficit without the old (incorrect) feedforward: (a) reaching gain Gamma_xy {0.5,1.5,3.0}
made it WORSE on BOTH Static+Circular IC5 (over-aggressive destabilizes); (b) K_rd doesn't fix it (fails
both values); (c) demand V_h_d is shared with old-c so funnel can't differentiate. No clean principled
lever. The old c-term's benefit is a PRODUCTIVE SUSTAINED FLOW ERROR (old-c |Vhe|~1.0-1.7 sustained drives
closure; C_SIMPLE |Vhe|~0.5-0.9 tracks cleanly but UNDER-closes) -- not replicable by gain.

**s_e_n BOUNDEDNESS ANALYSIS (the decisive diagnostic, answers "enough authority?"):** captured Static
IC5 seed1. OLD-c: s_e_n_y bounded (max residency S_s=0.88 <1), CONVERGES to ~0, 0% funnel breach -> funnel
guarantee HOLDS. C_SIMPLE: s_e_n_y PINS at the S_s=-0.95 clamp from t~2.6s then BREACHES to -2.55 (~6x the
funnel p_s=0.43), 46% of steps in breach, NO convergence -> prescribed-performance |s_e_n|<p_s VIOLATED.
ANSWER: NO, the corrected formulation does NOT have enough control authority for s_e_n convergence/
boundedness. Mechanism: at the boundary the back-mapped demand saturates (G_s^-1 ∝ p_s -> 0, §9 demand-
starvation) AND the leaner-correct c-term under-delivers closing motion; old-c's richer feedforward
supplies the authority the funnel needs. The funnel guarantee REQUIRES authority the correct form lacks.

**CONSTRAINT (user 2026-06-16): p_20 (optic-flow/velocity-funnel INITIAL width, K_ctrl.p_20=[25;25;4])
is LOCKED — do NOT retune it.** It's load-bearing (tied to the prescribed-performance bounds in the
Theorem-1 proof); changing it moves the guarantee, not just a knob. Retune other levers (SEN p_s_0/K_rp/
K_rd, middle-loop Gamma/E/kappa_0) but never p_20.

**SEN HARD-CONTAINMENT RECOVERY TESTED (2026-06-16) — FAILS, inner-bound confirmed.** SEN_RECOVER_ST
{0.3,0.5,0.7}: ZERO effect on Static IC5 (still 2/5 fov3) — triggers on actual breach (raw_ratio>=1) but
the killer is the PRE-breach stall at the S_s=0.95 clamp (raw_ratio<1) where demand already starves; fires
too late. Wider p_s_0 {2.0,3.0}: WORSE on both Static (2->1/5) AND Circular (5->3->2/5) — looser funnel =
weaker barrier = less position authority. Outer SEN levers EXHAUSTED. Deficit is genuinely INNER-BOUND
(middle loop can't deliver closing motion), as the controller caveat (L199-200) warned. K_rp sweep running.

**UNTESTED RECOVERY CANDIDATE (next):** §9 HARD outlier-containment (SEN_CONTAIN_MODE / SEN_RECOVER_ST,
already-implemented toggles, controller ~L546-557: on breach EXPAND p_s_eff=|s_e_n|/(1-margin) or pin S_s
at small ST -> restores ~full -rp restoring gain past breach). Directly addresses the s_e_n breach. May
recover IC5 boundedness under the corrected formulation. NOT YET TESTED (session wrapped). Note: SEN funnel
demand-starvation is the SAME §9 issue being worked on Ubuntu [[feedback_shared_issue_fix_in_ubuntu]].

**RECOMMENDATION (well-supported): option (c)** -- present corrected MODEL in all 3 .tex, keep old-code
results (marginally better on IC5), frame extra feedforward as bounded residual ->0 as s_xy->0 absorbed
into d_h. The whole investigation EMPIRICALLY justifies (c)'s claim. PENDING: user's final b/c call; IF the
SEN hard-containment recovers IC5 -> reconsider (b). .tex edits identified (supplement S1-B c̃_h L96-97 /
d_h L99 / text L101; control_formulation.tex ṡ L47 + h_d L102 -- it's MAINTAINED not legacy, has CBF/Fix-B
updates). TASK 2 (CoG-FF contribution) still pending. Test affordances added to single-run: TRAJ_OVERRIDE,
KAPPA_MAX/KAPPA_MAX_Z (gated). Harnesses: gate_csimple/test_new_formulation/csimple_*/recover_ic5.m.
See [[project_kinematics_correction_2026_06_11]].
