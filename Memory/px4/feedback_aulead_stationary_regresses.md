---
name: feedback_aulead_stationary_regresses
description: "B4 gate (2026-09-09): PLASMC_AU_LEAD=1+RATIO=0.5 REGRESSES stationary cross-marker HARD under PERCEPTION — A/B IC1-5 n=5 = 2/25 precise vs 20/25, xy 2-6x worse. But GT-FEEDBACK A/B (same config) shows NO regression on IC1-4 (xy 0.013→0.015) and RESCUES the fragile IC5 (0/5→5/5). ⇒ cause is PERCEPTION-NOISE AMPLIFICATION (the ×3.9 HF lead multiplies terminal extent-saturation centroid/flow corruption), NOT a control-side instability. AU_LEAD stays default-OFF; the deployability gate is 'feature measurement is clean', not 'no 1/Z'."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-09T17:36:43.165Z
---

**B4 (turning-target lateral limit cycle) — the mandatory stationary IC gate for
`PLASMC_AU_LEAD` was run 2026-09-09. AU_LEAD FAILS it under PERCEPTION but the failure is
a perception-noise-amplification artifact, not a control defect (GT-FB shows no regression
— see Mechanism below).**

Concurrent A/B, `WORLD=cross_marker MARKER_TYPE=cross`, IC1-5 stationary, n=5, HEADLESS
(PERCEPTION path):
- **Arm A (baseline, AU_LEAD off):** 20/25 precise; per-IC mean xy 0.024-0.090 m.
- **Arm B (`PLASMC_AU_LEAD=1 PLASMC_AU_LEAD_RATIO=0.5`, ω_z/ω_p defaults 0.9/3.5):**
  **2/25 precise**; per-IC mean xy 0.109-0.209 m = **2.2x (IC4) to 5.7x (IC2) worse**.
- All 50 reps landed; no TL, no crash, no fly-away. The failure is pure terminal
  sloppiness, not instability.
- Bundles: `test_data/ICValidation/20260909-191246` (A) / `20260909-193638` (B).
- ⚠ Both arms ran on `CBF_DRIFT_TAU=0.15` (reverted to 0 same day, commit `b731ed22`, after
  its own gate showed ~3× hard touchdowns on stationary). A/B is matched so the verdict
  holds; but arm-A ABSOLUTE numbers here are on the degraded tau=0.15 base — re-take the
  baseline on tau=0 if AU_LEAD is ever revisited.

## Mechanism — CONFIRMED perception-noise amplification (GT-FB discriminating test, 2026-09-09)

**First hypothesis (from the perception log dive) was a control-side "terminal κ-ratcheted
HF command pump": terminal `I_a_raw` peak 4.7→14.6, κ_xy peak 0.21→0.41, κ growth
+0.08→+0.37, `corr(lead-delta, κ_xy)` −0.01→+0.42, CBF `theta` peak 0.09→0.18, lead delta
pinned at exactly 0.50·|I_a_raw| (RATIO clamp saturated). All REAL in the perception logs
— but they are DOWNSTREAM SYMPTOMS, not the cause.**

**GT-FEEDBACK A/B (`PLASMC_GT_FEEDBACK=1`, GT channels: all; same AU_LEAD config; IC1-5
n=5; bundles `ICValidation/20260909-221339` A / `20260909-224018` B) REFUTES the
control-side story:**
- **IC1-4: NO regression.** xy_err 0.013→0.015 m, rel_vel 0.050→0.064, 19/19 vs 20/20
  precise+soft, min_alt identical.
- **IC5 (GT-FB baseline is a known terminal-divergence fly-away regime): AU_LEAD RESCUES
  it** — baseline 0/5 precise / mean xy 4.14 m (two 7-8 m fly-aways + 1 TL) → AU_LEAD
  **5/5 precise, mean xy 0.027 m**. The lead damped a genuine divergence = its design job.
- Mechanism signals under GT-FB (IC1-4 pooled, B−A): `I_a_raw` peak +0.01 (was +9.9),
  κ_xy peak +0.003 (was +0.19), κ growth −0.051→−0.052 = **κ DECAYS in both arms** (was
  +0.37), CBF `theta`/`vis_active` ~0 both. Only surviving fingerprint: `I_a` spectral
  centroid +0.5 Hz — the ×3.9 HF gain is there but its ABSOLUTE amplitude is ~0.008 m/s²
  because clean-feature `|I_a_raw|` is only ~0.02, so it does no damage and the RATIO
  clamp is irrelevant.

**Correct mechanism:** AU_LEAD's HF gain **×3.9** (ω_p/ω_z = 3.5/0.9) is a **terminal
perception-noise amplifier**. Under real perception the frame-saturating extent (318 px,
BOTH arms) corrupts centroid `s` and LK flow `h_xy` (the documented terminal-overfill
coherent corruption, same family as the #1 `h_y`/`w_z` blocker). The lead multiplies that
MEASUREMENT NOISE into the lateral command; the `I_a_raw`-peak inflation / κ-ratchet /
CBF-thrash then follow as the loop reacting to amplified sensor noise — a driven
response, not a self-excited instability. Remove the sensor noise (GT-FB) and there is
nothing to amplify. NOTE this means the earlier "extent saturates in both arms so it's
NOT feature corruption" reasoning was wrong: extent saturates in both, but only the lead
turns that corruption into a large command.

**Why the RATIO=0.5 clamp doesn't help under perception:** `|I_a_raw|` there is inflated
BY the amplified noise feeding back, so a scale-free (multiplicative) clamp rides the
noise-driven excursion. Under clean features `|I_a_raw|`≈0.02 so the clamp never matters.

**How to apply:**
- `PLASMC_AU_LEAD` stays **default-OFF in `controller.py`**; do NOT bake it on globally —
  under the live perception path it amplifies terminal extent-saturation noise (2/25
  precise).
- Enable it ONLY per-scenario via launcher env (the [[project_rover_turning_open]] "BEST
  CURVED CONFIG" pattern).
- The deployability gate is **"the feature measurement is clean"**, NOT "no 1/Z". Under
  clean features (GT-FB, and by extension a well-conditioned perception window) the lead
  is benign-to-helpful — it even rescued the fragile GT-FB IC5. So a viable path for
  turning-target use = gate the lead on a **perception-quality signal** (extent below
  frame-saturation / corner count healthy / flow residual low) so it disengages exactly
  when terminal overfill corrupts `s`/`h_xy`. A curvature/`w_z` gate alone is NOT enough;
  a perception-quality gate is the actual lever. `rel_resid`/`cond(A)` from
  `_solve_jacobian` is a candidate quality signal.
- The perception 2/25-vs-20/25 result is real for the LIVE path and reproducible — don't
  re-run expecting it to pass as-is. The open question is whether a perception-gated lead
  clears the stationary gate AND helps the turning target; that needs the gate built +
  tested, not another ungated A/B.

`run_ic_validation.sh` does NOT set `WORLD`/`MARKER_TYPE` — it defaults to the ArUco world
(stale 640x480 cal, comparison-only). Always `export WORLD=cross_marker MARKER_TYPE=cross`
before calling it for a real cross-marker gate. [[feedback_reject_on_single_failure]]
