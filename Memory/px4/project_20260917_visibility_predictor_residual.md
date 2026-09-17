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

---

## CORRECTION 2026-09-17 (same day): `phi` is TRANSPOSED against `c` in the live code

**Found while trying to implement a per-axis buffer.** `marker_tangent()` applies
`_SWAP = [[0,1],[-1,0]]`; `fov_limit()` does **not** apply it to the intrinsics. So:

    c[0] = +(y_px - cy)/f  -> spans +-cy/f = +-1.185   (the 320-tall axis)
    c[1] = -(x_px - cx)/f  -> spans +-cx/f = +-0.889   (the 240-wide axis)
    phi  = CENTER/focal*(1-b) = [0.889, 1.185]*(1-b)   <-- NOT reversed

The physical half-extent in `c`'s own axis order is `CENTER` **reversed**, `[1.185, 0.889]`.
Consequences, both live:
- **axis 1 barrier is INERT**: `phi_1 = 1.007` at `b=0.15` is OUTSIDE the physical edge
  `0.889` -- on that image axis the constraint cannot bind before the marker has left.
- **axis 0 over-tight by 36%**: `phi_0 = 0.756` against a true edge of `1.185`.

Airtight from `center = _resolution/2 = (120,160)` on the 240-wide x 320-tall rotated frame.
Data agree asymmetrically: `|c[0]|` reaches 1.504 and exceeds 0.889 on 0.92% of frames;
`|c[1]|` exceeds it on 0.19% and never passes 1.124.

**Both my tools inherited it** (they mirrored the code). Fixed in `tools/scan_vis_safeset.py`
and `tools/measure_vis_predictor_residual.py`; v1 copies in `Obsolete/tools/*_v1_transposed_phi.py`.

### What changed in the recorded numbers

| quantity | as first recorded | corrected |
|---|---|---|
| IC1-5 sensor exits | 0.00% | **0.00%** (unchanged -- headline survives) |
| IC1-5 buffered-box exits | 0.27% | **1.11%** |
| rover raw sweep, off / lead | 2.24% / 2.10% | **0.45% / 0.28%** (lead better) |
| rover conditioned, off / lead | 0.47% / 1.36% | **0.29% / 0.63%** (lead worse) |
| residual %>buffer, rot / drift | 1.235% / 1.319% | **1.711% / 1.619%** |
| per-axis b to cover p99 | [0.218, 0.164] | **[0.164, 0.218]** (I had it backwards) |

Residual *quantiles* are unaffected (norms of residual vectors, independent of `phi`):
p50 0.0194 / p95 0.0841 / p99 0.1941 / p99.9 0.3074 all stand, as does the 1-step-vs-null
result and the ~144 ms horizon.

**Two conclusions flip:**
1. `tau*d` now **slightly improves** the over-buffer tail (1.619% vs 1.711%), where the
   transposed numbers made it look slightly worse. §3 above overstated the case against it --
   still only ~10% median, but it is no longer "no tail improvement".
2. The rover sweeps now **disagree in direction** (raw: lead better; conditioned: lead worse).
   That is what n=2/cell noise looks like, and it **reinforces** rather than weakens
   [[project_20260909_visibility_projection_wire_in]]'s one-armed-statistic finding: the
   same-metric comparison does not support "tau*d closes the moving-target gap" in either
   direction. The 278-frame figure being a tau=0-arm-only count is unaffected -- that was
   about which arms were compared, not the extents.

**Fix priority: this outranks both the `y_max=0` degenerate-ball defect and any buffer
re-sizing** -- a per-axis `b` is meaningless until the box is on the right axes, and one axis
of the guarantee is currently not running. Preferred fix is to reverse the intrinsics in
`fov_limit()` (keeps `c` in the frame the `h_xy` identity-map was validated against) rather
than touching `marker_tangent()`. It is a genuine behaviour change -- it activates a
previously-inert constraint -- so it needs the IC2-5 gate, with `vis_slack` watched.

### FIXED 2026-09-17 — `96271ba6`, three sites, and why "15/15" never caught it

1. `src/visibility_projection.py` `fov_limit()` — reverse the `center/focal` quotient.
2. `tools/validate_visibility_projection.py` — **the reason it survived validation.** The
   validator passed `CENTER=[160,120]` (reversed vs what `controller.py` passes) and set
   `SENSOR = CENTER/FOCAL`, transposed for its own geometry too. **Module and oracle carried
   the SAME error**, so they agreed with each other while both disagreed with the real
   camera. Fixed module + fixed oracle = 15/15 on seeds 0-4; fixed module + old oracle =
   10/15 (checks 1 and 12 fail). → **An independent validator that shares the code's
   convention is not independent.** When an oracle hard-codes intrinsics, check them against
   what the live caller actually passes.
3. `src/controller.py` `_p_10_tan` — was deliberately un-reversed with the comment
   "marker_tangent applies its own [y,-x] swap". Backwards: `c` being swapped is *why* the
   half-extent must be swapped to match. The drift-off pull-back therefore fired early on one
   image axis and could never fire on the other.

**`y_max=0` "degenerate ball" fix RETRACTED** (was listed as the top defect before this).
Implementing it failed validator checks 10/11 and the checks are right: at `a_z >= a_cap`,
`||a*|| = a_z*sqrt(1+||y||^2)` exceeds `a_cap` already at `y=0`, so the feasible set is
genuinely EMPTY and `y_max=0` is the correct answer. Widening the ball manufactures a lean
the vehicle cannot deliver and breaks deliverability-by-construction. The infeasibility is in
the CALLER's `a_z` — a caller-side clamp before the solve is the principled fix, but it
modifies the vertical channel and breaks the two-tier separation, so it needs its own
decision. Reasoning left in-code so it is not retried.

**Still open:** IC2-5 gate (the fix activates a previously-inert constraint — expect
`vis_active`/`vis_slack` to rise; slack going routinely non-zero is the tripwire). Buffer
re-sizing deliberately deferred until AFTER the gate: per-axis effective margins change with
this fix, so `b` must be re-derived on top of it, not alongside.

---

## IC2-5 gate run 2026-09-17 — 0/25 precise: NOT the axis fix, a perception-layer environment collapse

Ran the mandated IC2-5 gate for `96271ba6` (N_REPS=5, HEADLESS=1, cross-marker):
`test_data/ICValidation/20260917-224720`. Result: **0/25 precise, 0/25 soft**, IC2-5 mean xy
1.3-1.8 m (vs the Sep-12 bundle used for the residual analysis, `20260912-040029`: 16/25
precise, xy mostly 0.01-0.15 m). Looked catastrophic at first read.

**Before attributing this to the fix, validated the bisect endpoint** — this project's own
hard-learned rule ([[feedback_recurring_analysis_mistakes]] §10-15,
[[project_20260916_curve_qgate_revalidation]]/`e7882829`: "a worktree rebuilds CODE not the
EXPERIMENT... out-of-repo camera SDF"). Same-day, same-environment, interleaved A/B on IC2
(5 reps/arm, `git worktree` at `4ba07bb8` = `96271ba6^` for OLD, `LANDING_OUT_BASE` set per
arm per the `run_visproj_gate.sh` autosave-collision lesson):

| arm | xy_err (5 reps) | precise |
|---|---|---|
| OLD (pre-fix code, today) | 2.36, 0.99, 0.87, 1.09, 3.67 | 0/5 |
| NEW (fixed code, today) | 2.69, 2.71, 2.39, 2.61, 2.31 | 0/5 |

**OLD code fails exactly as badly as NEW, today.** The `96271ba6` fix is NOT implicated.

Root cause traced one level further — **marker-alive rate** (`N Flow Corners > 0` fraction)
per rep:

| | Sep 12 (good) | Sep 17 OLD | Sep 17 NEW |
|---|---|---|---|
| marker-alive % | **100.0%** every rep | 25.9 / 78.6 / 91.9 / 40.8 / 25.5 | 23.6 / 23.9 / 56.1 / 28.6 / 23.5 |

**This is a perception-layer collapse present identically in both code versions.** Something
in the environment (Gazebo world state, marker rendering, camera plugin, lighting — not yet
isolated) degraded between 2026-09-12 and 2026-09-17, independent of any `src/` change.
Camera SDF checked and unchanged (320x240, hfov 1.74) at the time of this test; no other
SITL/PX4/bridge process was running before either test. Not yet root-caused further — this
smells related to the still-unexplained out-of-repo state that caused the curve-cycle
mystery ([[project_20260916_curve_qgate_revalidation]]), possibly the same drift, but that
is a hypothesis, not established.

**Consequence for the axis fix (`96271ba6`):** cannot be validated as beneficial OR harmful
under the current environment — no landing-quality signal is trustworthy right now for
ANY change. The fix's correctness stands on its own terms (mathematical derivation from the
SDF + validator's independent oracle, 15/15 across 5 seeds) and is NOT reverted. Do not
re-attempt an IC2-5 landing-quality gate until the marker-alive collapse is diagnosed —
otherwise every gate from here forward returns the same false "everything is broken" signal
regardless of what changed.

**Action item, higher priority than any further CBF tuning:** diagnose the marker-alive
collapse. Suspect areas to check first: Gazebo world/marker model state (a stale spawn,
lighting, or renderer setting), ros_gz_bridge health, whether any residual state was left by
recent SDF experimentation (peer 29's temporary 640x480 restore — SDF file itself reads
correct 320x240, but check for cached/stale Gazebo model resources), PX4 firmware/parameter
drift. This blocks all landing-outcome gating, not just this thread.
