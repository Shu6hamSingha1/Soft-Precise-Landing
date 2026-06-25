---
name: feedback_flow_funnel_zetah_works
description: "⭐⭐ THE VELOCITY-DAMPING LEVER WORKS (current frontier). Tightening the lateral flow funnel (XI2_xy 0.2->0.5) engages zeta_h (share of sigma_xy 7%->18%, p2_xy 6-9->2-4) -> arrests v_lat during the descent -> drops the SAFE FLOOR (altitude where the terminal 1/Z loop triggers) ~1m->~0.15m -> GT-FB tally 175559 4/2/3 -> 6 sub/2 marg/2 fly (bundle 20260625-202813). No limit cycle (SUPPRESSED the loom cycle at E_z=0.5); kappa healthy all 4 axes. h_xy NOT eliminated (irreducible 1/Z at Z->0) but pushed to touchdown. NEXT: XI2_xy 0.5->0.7."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 31fd53ca-48b0-48f4-81a8-2e081955028f
---

**THE VELOCITY-DAMPING LEVER IS CONFIRMED — the fix the whole terminal thread pointed to (2026-06-25, user-led GT-FB).**

**The lever.** sigma_xy = zeta_h + chi_r*zeta_r. zeta_h (the flow/velocity barrier) was DORMANT (~7% of sigma_xy) because the lateral flow funnel was too loose (p2_xy stayed 6-9 during the descent; XI2_xy=0.2 contracts too slowly). **Tighten it: XI2_xy 0.2->0.5** (env `PLASMC_XI2_X=0.5 PLASMC_XI2_Y=0.5 PLASMC_XI2_Z=0.2`; keep Z=0.2 = VDF, else the single-axis env BYPASSES the auto-align and reverts XI2_z hot). Result: p2_xy contracts to ~2-4, **zeta_h share 7%->18%** -> the SMC reaching term now drives v_lat->0 *during* the descent (while Z is large and the disturbance is under the 17 m/s^2 actuator ceiling).

**Result: GT-FB tally 6 sub / 2 marg / 2 fly** (bundle 20260625-202813) vs the same-config baseline **175559 (XI2_xy=0.2): 4/2/3**. A clean win at the SAME N_xy=0.1, chi_r=0.5, P2INF=[0.5,0.5,1.5].

**Mechanism = the SAFE FLOOR drops.** The terminal fly is the 1/Z feedback loop: h_xy=v_lat/Z -> control over-reacts -> tilt -> lateral accel -> v_lat grows -> bigger h_xy. The loop GAIN is ~1/Z, so it always wins eventually as Z->0. zeta_h damping doesn't remove the loop — it **lowers the altitude where the loop overpowers the damping** from ~1m (175559: v_lat survived to 1m -> breach@1m -> fly) to **~0.15m** (most reps now touch down BEFORE the loop triggers -> sub). vlat@1m: subs 0.02-0.73 (the SP condition <0.5), the fly IC4_rep1 = 1.0 (not arrested).

**h_xy is NOT eliminated and CAN'T be (irreducible geometric 1/Z).** h_xy=v_lat/Z spikes as Z->0 for ANY non-zero v_lat. On SUBS the "spike" (hxy_deck up to 50) is the TOUCHDOWN sample (Z<0.1, drone landed, GT bounces) — h_xy is <=2.5 through the real descent = benign. On FLYS v_lat RE-GROWS in the last ~15cm (0.34->2.29 as Z 0.15->0.10) -> real spike (174). **The right metric is the SAFE-FLOOR ALTITUDE, NOT the hxy_deck magnitude.** (On flys h_xy=174 >> v_lat/Z=24 because the violent terminal tilt adds ROTATIONAL flow on top of v_lat/Z.)

**Side benefits (clean lever).** (1) **NO limit cycle introduced** — all 4 axes sat(sigma/E)<=1.48, velocity sign-flips 1-3 (not a sustained oscillation). (2) **The LOOM cycle is SUPPRESSED** even at the baked E_z=0.5 (the setting that earlier gave sat_z 5-8 bang-bang balloon): here sat_z=1.48, no balloon — arresting v_lat cuts the lateral->tilt->loom coupling that fed the cycle. (3) **kappa healthy in all 4 axes** on the subs: x/y ADAPT (std 0.23, the N_xy=0.1 wake working) with benign terminal growth (3.8, NOT the fly-only 13-46 explosion); z low (0.095, term 0.55); yaw decays (0.11->0.10). The kappa explosion is FLY-ONLY (needs the v_lat re-growth loop to trigger).

**NEXT STEPS (NOT baked — iterate then validate).**
1. **Tighten more: XI2_xy 0.5->0.7** (and/or lower p_2_0_xy from 25) -> push zeta_h share past 30% and the safe-floor below ~0.05m -> should catch IC4_rep1 (the v_lat-not-arrested fly).
2. **Trace IC2_rep1** (flew 1.87 with v_lat ARRESTED to Z=0.11 then a SUDDEN late spike v_lat 0.04->2.92) — NOT a v_lat-arrest failure; smells like a woken-kappa (N=0.1) terminal transient. Different mechanism.
3. After a clean XI2_xy gate: validate n>=5, then perception-ON (not GT-FB), THEN consider baking XI2_xy.
4. h_rd CONSTANT throughout (the vz=h_rd*Z taper IS the soft-landing mechanism — never reference-govern it).

Continues [[feedback_terminal_smc_actuator_wall]] (the "arrest v_lat early" prescription — now VERIFIED), [[feedback_kappa_4axis_hexy_param_map]] (the velocity-damping lever was the predicted fix), [[feedback_matlab_gains_not_portable]] (the lag is why we can't just copy MATLAB's chi_r). Reproduce: `HEADLESS=1 PLASMC_GT_FEEDBACK=1 PLASMC_XI2_X=0.5 PLASMC_XI2_Y=0.5 PLASMC_XI2_Z=0.2 N_REPS=2 bash scripts/run_ic_validation.sh`.
