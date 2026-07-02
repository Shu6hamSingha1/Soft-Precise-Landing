---
name: project_handoff_terminal_oscillation
description: "HANDOFF (2026-06-30): containment/soft-breach is a confirmed NET REGRESSION (revert or leave default-off). The terminal softness wall = a 1/Z-gain x 38ms-lag LIMIT CYCLE in the lateral velocity loop; lag is the lever. Baked {h_rd=-0.30, XIR=0.15, P2INF=1.0} — RESOLVED: committed 486f713 (06-30 19:59); XIR later REVERTED 0.15→0.10 f068774 (07-01, high-n confirmed). Next: lag fix / c-term cap / XIR-PRINF-P2INF_xy sweep."
metadata:
  node_type: memory
  type: project
  originSessionId: a378d3e9-67aa-42fc-ae09-63da27f370a9
---

**HANDOFF 2026-06-30 (long session, user-led).** Continue from here in a fresh chat.

## CURRENT BAKED CONFIG (⚠ resolved 2026-07-02 audit: COMMITTED same day in 486f713; XIR then REVERTED 0.15→0.10 in f068774 on 07-01 — pooled 56/98=57% vs 33/75=44%, see [[project_why_sp_achieved]])
`{h_rd=-0.30, XIR=0.15→0.10, P2INF_xy=1.0}`.
- **h_rd=-0.30** (landing_test.py:29): slow descent, IC5 runway; D-gate 20/25 SP vs 15/25 at -0.42.
- **XIR=0.15** (controller.py:264): off-center precision (IC2/IC3 5/5); ⚠ edge-forces high-alt IC4.
- **P2INF_xy=1.0** (controller.py:273): A/B WINNER (7/9 SP, rel med 0.026) vs 1.5 (5/9) vs 2.0 (5/9).
  Smaller p_2 = steeper zeta_h = MORE lateral velocity damping. The old "1.5 un-saturates zeta_h" claim
  REVERSED at this config (see [[project_residual_cycle_wumax_bake]] banner). Needs a full IC1-5 n=5 gate.
- DECIDE: commit this config (logical checkpoint) or keep iterating.

## ⛔ CONTAINMENT / SOFT-BREACH (Idea 1) = CONFIRMED NET REGRESSION — do NOT re-enable
`PLASMC_SOFT_BREACH` (default-off, env-gated) + the source-fake of `s_e_n`/`s_dot` + the `h_e` containment
soft-clamp. **Two A/Bs: 7/10 -> 4/10, a fly each.** Why it fails (all verified):
1. **Position-containment is INERT** — the wide funnel (PR0=10) means `|s_e_n/p_r|` peaks ~0.92, NEVER hits
   the 0.95 trigger (0/10 reps breach). So the whole p_r source-fake never fires; can't even be exercised.
2. **The flow branch un-freezes kappa** (drops `contained[idx]`) -> kappa-RUNAWAY (kappa_x -> 5.3) -> the fly.
   The **kappa-freeze is PROTECTIVE**; un-freezing it is the regression.
3. **Faking a genuine error is wrong** — the terminal 1/Z spike is a GENUINE large error (drone IS off /
   moving), not a glitch; faking it blinds the controller. Idea 1 solves the wrong problem (funnel breach),
   which isn't the actual failure (marginal precision/softness).
RECOMMEND: revert the soft-breach code (dead/regressing) OR leave default-off. The diagnostic logging added
(`hd_rate`/`h_d_noS`/`s_dot_meas` in the output dict) is harmless + useful — keep.

## ⭐ THE TERMINAL SOFTNESS WALL = a 1/Z-gain x 38ms-lag LIMIT CYCLE (the key finding)
IC4r1 (the n=5 soft-fail, rel 0.266) is the canonical case: terminal `v_lat` does NOT fail to slow — it
**rings** (9 zero-crossings, v_lat swings 0.08<->0.61), and touchdown caught a PEAK. CAUSE (identified):
- **Loop gain ∝ 1/Z** (scale-free feature `h=v_lat/Z`, `s_e_n=lat/Z` divide by depth) -> gain rises without
  bound as Z->0. Forced by depth-free (can't down-weight near deck without Z).
- **Fixed 38ms inner-loop lag** (PX4 rate loop + MAVSDK).
- Near the deck the **velocity feedback (flow / drift chi_r*zeta_r_dot via s_dot) flips from damping to
  PUMPING** (phase margin -> 0 at high gain) -> limit cycle, INTENSIFYING as Z->0 (v_lat 0.61 at touchdown).
- Evidence: oscillation amplitude GROWS toward the deck (rising-gain signature). NOT the kappa-runaway
  (ARM A has kappa frozen -> bounded, no fly; the freeze caps amplitude but can't damp the ring).
- **Gain knobs (kappa/Gamma/funnel) can't fix it** — they change loop gain/amplitude, NOT phase margin.
  The ring is in EVERY rep; only the deck-PHASE varies -> SP is phase-gated ("luck" is the touchdown phase).
- **LEVER = the 38ms lag** (the one term in the phase margin you can move): cut it (uXRCE-DDS rate path /
  faster inner loop) or add lag-compensation / phase-lead on the velocity feedback. Pushes the ring below
  the deck -> v_lat settles -> soft every rep. (Lag is both the cause and the lever.)

## ⭐ THE c-TERM `-h_z*h_xy` = a 1/Z^2 PHANTOM (why kappa can't handle the terminal)
`c = h_z*h_xy = (v_z/Z)(v_lat/Z) = v_z*v_lat/Z^2`. VERIFIED: at a blow-up, v_lat=0.74, v_z=0.05 (TINY, soft
landing!) but Z=0.10 -> `v_z*v_lat/Z^2 ≈ 3.9 ≈ measured c=4.18` (×99 the median). So c is huge PURELY from
1/Z^2 geometry on small real velocities — a feature-space PHANTOM, not a physical disturbance. Why kappa
can't handle it: (1) c is in the EQUIVALENT-CONTROL/feedforward branch `G*(-c)`, NOT kappa's SWITCHING
`theta*kappa*sat` -> kappa has no lever on it; (2) it INFLATES `theta=||Theta||` (Theta includes -c) ->
poisons the switching `theta*kappa` even with kappa bounded -> un-freezing kappa = chase the inflated
kappa_eq=124 -> runaway; (3) kappa_eq=124 is UN-DELIVERABLE (actuator g*tan60≈17). FIX = bound the c-term
at SOURCE (`cterm_loom_scale` / cap `|h_z*h_xy|`, depth-free) OR arrest v_lat so h=v/Z never spikes. NOT kappa.

## THE TWO WALLS (per-IC, both marginal at the baked config; ARM A baseline 7/10)
- **IC4 (7m, long descent) -> SOFTNESS** (rel just >0.2): position centers fine (xy<0.08), but the terminal
  v_lat RING catches a peak. The lag/phase-margin wall above.
- **IC5 (3m, short runway) -> PRECISION** (xy 0.13-0.26, soft): short runway -> can't fully center position
  before deck (convergence-rate-vs-runway). Velocity IS arrested.

## NEXT STEPS (open)
1. **Commit the baked config** (or finalize P2INF=1.0 with an IC1-5 n=5 gate first).
2. **Revert the soft-breach code** (net regression) — or leave default-off.
3. **The lag lever** (softness): uXRCE-DDS rate path (built+impulse-validated, blocked on rclpy ctx —
   [[feedback_dds_lag_fix_blocker]]) OR lag-compensation/phase-lead on the velocity feedback. THE softness fix.
4. **The c-term cap** (the kappa-runaway / fly root): `cterm_loom_scale` or a terminal cap on `|h_z*h_xy|`,
   depth-free. Bounds the feedforward AND theta (un-poisons kappa).
5. **Proposed (un-run) terminal-aggression sweep** for softness: XIR=0.10 (gentler, costs precision),
   PRINF=1.0 (RAISE per Standing-Cond-1, not reduce), P2INF_xy=0.7 (more lateral damping; NOT P2INF_z=vertical).
Relates [[project_residual_cycle_wumax_bake]], [[feedback_terminal_smc_actuator_wall]],
[[project_soft_breach_idea1]], [[feedback_hd_funnel_ref_zeta_h_degenerate]], [[feedback_impulse_response]].
