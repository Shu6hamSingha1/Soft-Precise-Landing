---
name: feedback_aulead_stationary_regresses
description: "B4 gate (2026-09-09/12): PLASMC_AU_LEAD regresses stationary cross-marker under PERCEPTION (2/25 vs 20/25) via HF-gain noise amplification (GT-FB clean, confirmed not a control instability). FIX IMPLEMENTED+VALIDATED 2026-09-12: PLASMC_AU_LEAD_QGATE attenuates the lead by MARKER_EXTENT_PX fill-fraction (scale-free). 3-arm A/B: IC1-4 FULLY RECOVERED (13/20 precise, >= baseline's 12/20, vs 3/20 ungated). IC5 (steep low-alt start) still fails (0/5) — open, needs tighter/different threshold for that regime. Default ON whenever AU_LEAD=1."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-11T22:23:22.474Z
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

## Perception-quality gate on the lead — IMPLEMENTED + VALIDATED (2026-09-12)

`PLASMC_AU_LEAD_QGATE` (default ON whenever `PLASMC_AU_LEAD=1`): attenuates the lead
delta by `fill = MARKER_EXTENT_PX / frame_min` — full lead below `QGATE_LO=0.55`, zero
by `QGATE_HI=0.85`, linear ramp between (scale-free, no depth/altitude). Logged
`au_lead_qgate(t)`. Directly targets the confirmed cause (the ×3.9 HF gain amplifying
terminal extent-saturation noise) rather than the adaptive-law symptom the
[[feedback_adaptive_law_noise_behavior]] dead-zone tried and failed to fix.

**3-arm A/B, IC1-5 n=5, perception, `WORLD=cross_marker`** (bundles
`ICValidation/{20260912-023429 A-base, -030211 B-nogate, -032559 C-qgate}`):

| arm | precise | mean xy | IC1-4 precise | IC5 precise |
|---|---|---|---|---|
| A base (no AU_LEAD) | 17/25 | 0.073 | 12/20 | 5/5 |
| B AU_LEAD, no gate | 3/25 | 0.116 | 3/20 | 0/5 |
| **C AU_LEAD + QGATE** | **13/25** | **0.085** | **13/20** (≥ baseline) | **0/5** |

**IC1-4 fully recovered — 13/20 precise vs baseline's 12/20, vs 3/20 without the gate.**
Gate confirmed engaging correctly in logs (`au_lead_qgate` → 0 in the terminal 3 s on
every checked rep, extent saturating 300-319 px in both IC2 and IC5). **IC5 (3 m start,
shortest/steepest descent, ~9 s flight) does NOT recover** (0/5 in both B and C, xy
0.10-0.28 m) despite the gate engaging identically (frac_zero terminal =1.0, frac_full
mid-flight ~0.33). Diagnosis: IC5's full-lead (`fill<0.55`) window carries ~2.3× the
command magnitude of IC2's (`I_a_xy` mean 1.13 vs 0.49, max 3.09 vs 1.84) — the
same relative gate leaves proportionally more room for HF-amplified noise to matter on
IC5's inherently hotter, tighter-margin low-altitude approach. **Open**: QGATE_LO/HI are
fixed fractions of frame_min (scale-free, altitude-independent by design) but IC5's
failure suggests the RIGHT threshold may need to be tighter for a steep/low-altitude
start — not yet retuned. Do not claim IC5 is fixed.

**Verdict: the perception-quality-gated lead is the correct fix for 4/5 ICs and is the
viable path to a rover-scenario bake, PENDING an IC5 fix** (tighter QGATE_LO, or a
different quality/altitude-adjacent scale-free proxy). Still needs: the curved-target
benefit re-validated WITH the gate on (does gating the lead during any terminal
overfill on a moving/curving target still damp the [[project_rover_turning_open]]
cycle?), and a real-rover pass.

## Prior "how to apply" (superseded in part by the gate above; kept for the AU_LEAD
## mechanics/history it still documents)
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
