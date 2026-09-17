---
name: project_20260917_visibility_predictor_residual
description: "Measured the visibility CBF's own one-step predictor against realized centre motion on the IC1-5 gate (25 reps, ~24k frame pairs, attitude time-aligned with a proven self-check). Three results: (1) the predictor's meaningful horizon is the ~125-144 ms ATTITUDE-REALIZATION time, not one control step -- at 1 step it is WORSE than assuming no motion (p95 0.0183 vs null 0.0152) because L_e amplifies attitude noise when dy~0; (2) buffer b=0.15 covers the bulk but not the tail -- residual p95 0.084 vs per-axis buffer [0.133,0.178], but p99 0.194 and p99.9 0.307 both EXCEED it, over-buffer on 1.2% of frames (consistent with the independently measured 0.27% buffered-set exits / 0% sensor exits); (3) the tau*d drift term is a MODEST MEDIAN correction only -- reduces residual on 54.1% of frames, mean 9.9%, p50 0.0194->0.0169, but does NOT improve the safety-relevant tail (over-buffer 1.235%->1.319%). So tau's SCALE is a plant property (attitude-realization horizon), but tau*d is not a fix for the frames that matter."
metadata:
  type: project
---

**Stating positively what the 2026-09-17 audit block left implied** (peer
`soft-precise-landing-42` asked for this, correctly: if τ equals the attitude-realization
horizon then it is a **plant property, not a scenario property**, so framing
`CBF_DRIFT_TAU=0.15` as "for moving targets" is mis-framed — the stationary case has
self-motion flow too). Continues [[project_20260909_visibility_projection_wire_in]].

## Method (the alignment is proven, not assumed)

`Img_Data` runs longer than `Control_Data` (image node has its own cadence: 1306 vs 1168
samples on IC1_rep1) — truncating to `min` silently misaligns, the
[[feedback_recurring_analysis_mistakes]] §1 trap. Instead: interpolate `Img_Data.Quat`
onto the control clock `Control_Data["t"]` via `Img_Data["Time"]`, renormalise, and
**self-check** by reconstructing `arccos(R33)` and comparing to the independently-logged
`theta_current(t)` → p50 err **0.013°**, p99 **0.33°**. Frames with a >40 ms nearest-sample
gap (1.9%) are dropped. Any future offline use of `Quat` against control-rate logs should
carry this same self-check.

## 1. The predictor's horizon is the realization time, not one control step

Residual of `ĉ = c + L_e·Δy_now` vs the realized centre, by horizon (IC1-5, ~24k pairs):

| horizon | rot-only p95 | null (no-motion) p95 |
|---|---|---|
| 1 step (~10 ms) | 0.0183 | **0.0152** |
| ~125 ms | 0.0776 | 0.0830 |
| ~144 ms | 0.0841 | 0.0915 |

Over one control step the attitude barely moves, so `Δy ≈ 0` and `L_e` mostly amplifies
attitude noise — **the model is worse than doing nothing there.** It only earns its keep at
≳125 ms. Measured attitude-realization horizon (high-passed lean-command → realized-tilt
cross-correlation) is ~144 ms, IQR 112-288.

**This is NOT a safety hole for the rotational term** (an earlier framing of mine, corrected):
the QP bounds the centre at the *fully-realized* lean, and the realized path travels the
segment `c → c_next`, so by convexity of the box every intermediate state is inside too. What
is *not* on that segment is translation during the realization window — which is where the
residual actually lives.

## 2. `b = 0.15` covers the bulk, not the tail

At the realization horizon, per-axis buffer `b·R/(2f) = [0.133, 0.178]`:

| predictor | p50 | p95 | p99 | p99.9 | % frames over buffer |
|---|---|---|---|---|---|
| null | 0.0219 | 0.0915 | 0.2133 | 0.3525 | 1.653% |
| rot-only | 0.0194 | 0.0841 | 0.1941 | 0.3074 | **1.235%** |
| rot+drift | 0.0169 | 0.0807 | 0.1968 | 0.2974 | 1.319% |

p95 sits comfortably inside the buffer; **p99 and p99.9 both exceed it.** Consistent with the
independently measured 0.27% buffered-set exits and 0% sensor exits — the buffer absorbs the
bulk, the box is breached occasionally, the sensor never is. A residual-aware (per-axis,
possibly altitude-dependent) buffer now has a number behind it rather than a guess.

## 3. `τ·d` is a modest MEDIAN correction — not a tail fix ⚠

Reduces residual on **54.1%** of frames, **mean 9.9%**, p50 0.0194 → 0.0169 — but the
over-buffer rate goes **1.235% → 1.319%**, i.e. **no improvement where it matters.**

⚠ **Scope correction to how this got recorded elsewhere.** `5a5fc2a6` (peer) records the
`gT²/(6Z)` framing as "a better justification" for τ=0.15. Correct about τ's *scale*;
overstated about its *benefit*, on two counts:
- The `gT²/(6Z)` shortfall is an exact geometric quantity (0.6% of predicted displacement at
  Z=5 m, 6% at Z=0.5 m) but it is **small compared to the measured total residual** — so the
  translational term is **not the dominant residual source**. Measurement noise and unmodelled
  dynamics dominate. That is exactly why correcting it buys only ~10%.
- Benefit is median-only and does not touch the tail.

Honest scope of the check: used raw `h[:2]` and actual elapsed `dt`, **not** the
`condition_drift`-conditioned `d` at fixed τ — a proxy for the implemented term, not the term
itself.

**Net:** keep `CBF_DRIFT_TAU=0.15`; justify it as *the plant's attitude-realization horizon*
(applies to stationary and moving alike, no scenario framing); claim a ~10% median
prediction improvement and **no** tail/safety improvement. Do not cite it as closing a
moving-target gap ([[feedback_dont_judge_cbf_by_sp]]).
