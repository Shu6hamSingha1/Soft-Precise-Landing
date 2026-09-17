---
name: feedback_session_20260909_12_audit
description: "Statistical audit (2026-09-16) of the 2026-09-09→12 AU_LEAD/B4 session's 11 SITL arms. KEY CALIBRATION: the perception IC1-5 n=5 baseline is only reproducible to 17-20/25 (pooled 56/75=0.747), so ANY single-run difference under ~3/25 is inside noise. Two recorded claims were WRONG and are corrected here: (1) 'AU_LEAD rescues GT-FB IC5' is CONFOUNDED (the GT-FB A/B ran unmatched CBF_DRIFT_TAU, A=0.0 vs B=0.15, because a peer re-baked the default 13 min before arm B started); (2) 'ext+mag gate 18/25 beats baseline 17/25' is p=1.000, indistinguishable. Also: IC5 is the EASIEST perception IC (15/15 baseline), not the hardest; IC4 is the hardest (8/15)."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 878fdadb-dd99-4085-bcf2-e19879f48082
  modified: 2026-09-16T13:44:21.425Z
---

**Audit of the 2026-09-09→12 B4/AU_LEAD session (11 SITL arms, 274 reps).** Re-derived
every number from the raw `summary.tsv` + `Control_Data.npy` rather than the session's own
running commentary. Two load-bearing claims did not survive.

## ⭐ THE CALIBRATION EVERY FUTURE GATE NEEDS: baseline reproducibility

Three *independent* runs of the **identical** perception baseline config (no AU_LEAD,
`WORLD=cross_marker`, IC1-5 n=5, all on `CBF_DRIFT_TAU=0.15`, verified from
`Control_Params.resolved`):

| run | bundle | precise |
|---|---|---|
| P1-A | `20260909-191246` | 20/25 |
| K-A  | `20260911-003120` | 19/25 |
| Q-A  | `20260912-023429` | 17/25 |
| **pooled** | | **56/75 = 0.747**, Wilson95% [0.638, 0.831] |

**Run-to-run spread is 3/25 (12 pct pts) with NOTHING changed.** ⇒ **any single-run
25-rep difference smaller than ~3-4/25 is inside baseline noise and must not be reported
as an effect.** At n=25 the Fisher-exact resolution is roughly: a drop to ≤8/25 is
unambiguous; 13/25 is marginal (p≈0.05); ≥16/25 is indistinguishable from baseline.

## ⛔ CORRECTION 1: "AU_LEAD RESCUES the fragile GT-FB IC5" — CONFOUNDED, RETRACTED

The GT-FB A/B (`20260909-221339` A / `20260909-224018` B) was **not config-matched**:
`Control_Params.resolved` shows **A ran `CBF_DRIFT_TAU=0.0`, B ran `0.15`**. Cause — a
peer session was live-editing `controller.py`'s default in the same window:
`6701d143` (21:40, revert →0) … **my arm A started 22:13** … `827933b5` (22:27, re-bake
→0.15) … **my arm B started 22:40**. My harness never pinned the value (and I told the
peer it did — it didn't), so the two arms silently straddled the flip.

- **The IC5 result (A 0/5 with two 7-8 m fly-aways, B 5/5) CANNOT be attributed to
  AU_LEAD.** `CBF_DRIFT_TAU` is an equally-plausible cause. Retract the "rescues IC5"
  claim wherever it appears.
- **The load-bearing GT-FB conclusion SURVIVES**, because it rests on IC1-4, where BOTH
  arms sat at ceiling and at zero noise: IC1-4 precise 20/20 (A) vs 19/19 (B); END-window
  `sigN_std` 0.0015 (A) vs 0.0022 (B); `k_grow` −0.006 both. With σ-noise ≈0 in both arms
  the tau difference has nothing to act on. So "under clean features the lead is harmless
  ⇒ the perception regression is noise amplification" still holds.
- Lesson: **pin every non-default env var explicitly in the harness**, don't inherit
  code defaults, when a peer session may be editing the same file. Inherited defaults are
  not a controlled variable on a shared worktree.

## ⛔ CORRECTION 2: "ext+mag gate 18/25 BEATS the 17/25 baseline" — NOT SUPPORTED

Fisher exact, 18/25 vs 17/25: **p = 1.000**. Against the pooled 56/75 baseline: p = 0.797.
The honest claim is **"restored to statistically indistinguishable from baseline"** —
which is the correct success criterion anyway (a gate should make the lead *harmless*,
not beat the no-lead case). Never again quote a 1/25 delta as an improvement.

## Corrected result table (all vs pooled baseline 56/75, Fisher exact 2-sided)

| arm | config | precise | p vs pooled base | verdict |
|---|---|---|---|---|
| P1-B | AU_LEAD r0.5, ungated | 2/25 | 3.1e-09 | **massively degraded** |
| Q-B | AU_LEAD r0.5, ungated | 3/25 | 3.4e-08 | **massively degraded** |
| Q-C | + extent gate | 13/25 | **0.046** | **still degraded** (marginal) |
| Q-D | + extent × magnitude gate | 18/25 | 0.797 | **indistinguishable = FIXED** |
| K-B | `KAPPA_DZ=0.06` alone | 16/25 | 0.315 | indistinguishable |
| K-D | AU_LEAD + `KAPPA_DZ` | 4/25 | 5.0e-07 | **massively degraded** |

Head-to-head: extent-gate vs ungated p=0.0054 (real partial fix); ext+mag vs ungated
p=3.4e-05 (real full fix); **ext+mag vs extent-only p=0.244 (NOT significant on the
pooled metric** — the magnitude term's value shows up only in the IC5 slice, below).

## ⛔ CORRECTION 3: IC difficulty ranking was backwards

Pooled perception baseline, per IC (3 runs, x/15): **IC1 10/15, IC2 12/15, IC3 11/15,
IC4 8/15 (0.53 — the HARDEST), IC5 15/15 (1.00 — PERFECT, the EASIEST).** The session
repeatedly described IC5 as the "steepest/hardest" IC. It is not: at baseline IC5 never
fails. IC5 is *only* broken by AU_LEAD (0/5, 1/5, 0/5 across the ungated/extent-gate
arms) and restored by the magnitude gate (5/5) — i.e. it is the IC most *sensitive to the
lead*, which is the opposite of intrinsically hard. Conversely the final arm's **IC4 1/5
is unremarkable against IC4's own 53% base rate** (P(≤1/5 | p=0.53) ≈ 15 %) — the
"unrelated flake" call was right, but for the wrong reason; the right reason is IC4's
base rate, not that rep's signature.

## What DID hold up (and why)

The **mechanism** measurements are continuous, sampled ~125 Hz over hundreds of frames
per rep, so they carry far more power than the binary precise-rate and they replicate
cleanly across two independent run-series:

| END-window (IC1-4 pooled) | base | ungated lead | gated |
|---|---|---|---|
| κ growth, P1-series | −0.005 | **+0.198** | — |
| κ growth, Q-series | +0.022 | **+0.150** | +0.046 (ext) / +0.051 (ext+mag) |
| `I_a` xy, Q-series | 0.779 | 1.015 | 0.957 / **0.773** |
| GT-FB both arms | −0.006 | −0.006 | (σ-noise ≈ 0.002, nothing to amplify) |

⚠ One sub-claim to soften: **"AU_LEAD inflates σ-noise ~1.6×"** replicates in the
P1-series (0.0568→0.0923, +62 %) but only weakly in the Q-series (0.0656→0.0714, +9 %),
and the two baselines themselves differ 0.0568 vs 0.0656 (~15 %). The **κ-growth**
signature is the robust discriminator, not the σ-noise ratio.

**IC5 mechanism (the magnitude-gate target) is unambiguous** — END window:
base `k_grow +0.009 / k_peak 0.115 / Iaraw_pk 3.02`; ungated `+0.227 / 0.344 / 15.55`;
extent-only `+0.090 / 0.286 / 8.33`; **ext+mag `+0.003 / 0.104 / 3.97` = back to
baseline.** IC5-specific Fisher: ext+mag vs extent-only 5/5 vs 0/5, **p = 0.0079,
significant.** So the magnitude term's benefit is real but localised to IC5.

## ⚠⚠ FORWARD RISK, now quantified — the magnitude gate probably kills the curve case

The magnitude gate fixes IC5 **by turning the lead off**: measured duty on IC5 is
**mean qg = 0.185, full-strength only 5.8 % of the flight** (vs extent-only's 0.398 /
32.5 %). Transmission curve (`MAG_LO=0.5`, `MAG_HI=1.2`):

| \|I_a_raw\| | 0.5 | 0.7 | 0.9 | 1.0 | 1.2+ |
|---|---|---|---|---|---|
| lead transmitted | 100 % | 71 % | 43 % | **29 %** | **0 %** |

[[project_rover_turning_open]] records the curved target as a **SUSTAINED**
`|I_a_raw|` ≈ 1.0-1.5 m/s² (the standing centripetal demand). **⇒ on the curve this gate
transmits 0-29 % of the lead — i.e. it very likely neuters the exact use case AU_LEAD was
built for.** This is no longer a hypothetical caveat; it is arithmetic from the shipped
defaults. Before any rover bake, either (a) re-validate the curve WITH both gate terms
and accept the answer, or (b) replace instantaneous `|I_a_raw|` with a
persistence/duration discriminator (rolling mean, or a "still-transient" latch) that can
separate IC5's decaying startup spike from the curve's sustained demand.

**How to apply:** run ≥2 baseline repeats before believing any new gate result; pin all
env vars in the harness; report 25-rep deltas with a Fisher p, not as bare counts.
[[feedback_aulead_stationary_regresses]] [[feedback_adaptive_law_noise_behavior]]
[[feedback_sensitivity_sweep_methodology]]
