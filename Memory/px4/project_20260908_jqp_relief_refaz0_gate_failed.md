---
name: project_20260908_jqp_relief_refaz0_gate_failed
description: "CBF_JQP_RELIEF_REF_AZ0 (joint-QP descent-rate-relief self-inflation fix, pdf open item #2) FAILED its IC2-5 cross-marker SITL A/B gate — regressed mean xy_err at all 4 ICs and nearly doubled target-loss. Left default OFF as an A/B hatch; do NOT re-try as an untested win. Separately, the per-outer-iterate joint-QP convergence residual instrumentation (commit 7b81ac1f) is committed and stands."
metadata:
  node_type: memory
  type: project
---

Follow-up to the 2026-09-08 CBF work (residual logging + the relief fix). Continues
[[project_joint_qp_nonconvergence_kappa_ratchet]] and the `CBF_visibility.pdf` S4
open items.

## What was tried

`CBF_visibility.pdf` S4 flags, unresolved: *"the descent-rate relief of S3.1 shrinks
|az|, which inflates Ni(az) and so tightens the box ... relief and box can fight
across outer iterates."* Concretely, in `cbf_visibility.py`'s joint-QP the relief
measured the box-suppressed lean as `||th_desired - P@(Ia_lat/_az_now)||` with the
RUNNING `_az_now` — which the relief itself drives more negative each outer iterate.
Larger `_az_now` shrinks `P@(Ia_lat/_az_now)` geometrically -> the suppression term
LOOKS bigger -> more relief -> `Ia_z` ratchets toward `-g` whenever the FoV box binds
at all. That is the terminal descent-stall chain in
[[project_20260901_rover_cross_perception_diagnosis]] ("folds ~5 m/s^2 UP into I_a[2]
-> B_T->0 -> stall").

**Fix (`CBF_JQP_RELIEF_REF_AZ0=1`, `cbf_visibility.py`):** evaluate the Δθ term at the
FIXED unconstrained `a_z` (`_az0` == `th_desired`'s own denominator), so it reflects
only the box's lateral suppression, not the relief's own geometric angle shrink.
Then `th_desired - P@(Ia_lat/_az0) == P@((I_a[:2]-Ia_lat)/_az0)` — exactly "lateral
accel the box removed, over the original a_z" (pdf eq 7).

**Offline looked good:** `validate_cbf.py` new `test_joint_qp()` 17/17 (this path had
had ZERO automated coverage — every other test leaves `A_CAP=None` -> theta path
only). In 926/926 box-binding below-hover synthetic cases the fix retained strictly
more descent authority than the buggy path (mean 0.30, up to 0.82 m/s^2) and was a
fixed point.

## The gate — REJECT

`test_data/JQP_ReliefRefAz0_AB/20260908-160523/`, `scripts/run_jqp_reliefref_gate.sh`
-> `run_cbf_ab.sh` per IC. Cross-marker world (`WORLD=cross_marker MARKER_TYPE=cross`
via `EXTRA_ENV`), headless, arms interleaved per rep, **n=5/arm/IC**, IC2-5. No
concurrent SITL (checked before launch). `relief_off` = `CBF_JQP_RELIEF_REF_AZ0=0`
(buggy path, current default), `relief_on` = `=1` (the fix).

| IC | arm | mean xy | median xy | max xy | TL |
|---|---|---|---|---|---|
| IC2 | off | 0.58 | 0.39 | 1.23 | 0 |
| IC2 | **on** | **0.97** | 0.43 | **3.16** | 0 |
| IC3 | off | 1.17 | 0.98 | 2.83 | 0 |
| IC3 | **on** | **1.93** | **1.61** | **4.39** | **1** |
| IC4 | off | 1.88 | 0.64 | 6.80 | 1 |
| IC4 | **on** | **2.34** | 0.32 | **8.50** | **2** |
| IC5 | off | 0.71 | 0.66 | 1.42 | 3 |
| IC5 | **on** | **1.49** | **0.79** | **3.31** | **4** |

Totals (n=20/arm): mean xy 1.08 -> **1.68**; TL 4 -> **7**; reps with xy>2 m: 2 -> **7**.

- `relief_on` regressed **mean xy_err at all four ICs** and equalled-or-worsened
  median at all four.
- **Target-loss (the CBF's actual job) nearly doubled** — worse or equal at every IC
  (IC3 0->1, IC4 1->2, IC5 3->4).
- The only thing that improved: IC4 median 0.64->0.32 and IC4 mean rel_vel 2.12->0.36
  (softer) — but IC4 also went 1->2 TL and produced the single worst miss (8.50 m).
  Net negative.

**Mechanism:** the "principled" fix UNDER-relieves relative to what the off-center
terminal geometry actually needs. The self-inflating relief, ugly as it is, is
empirically buying the descent-slowing that keeps the marker in frame while the
lateral loop converges. Same class as [[feedback_backstep_tried_clamps_are_lever]]
(the barrier-inversion "correct" h_d over/under-demands vs the tuned band-aid) and
the reverted `Rz_p90b` / the reversed-direction `CBF_MARGIN_RESERVE`
([[project_20260825_cbf_margin_reserve_fix]]).

## Status

- `CBF_JQP_RELIEF_REF_AZ0` stays **default OFF** (it was never baked; it is inert
  unless the env is set). Kept in-tree as an A/B hatch with the gate result stamped
  in `cbf_visibility.py`'s comment. **Do NOT re-try / bake as an untested win.**
- Per-outer-iterate joint-QP convergence residual instrumentation (commit
  `7b81ac1f`: `state["joint_qp_resid*"]` -> `Control_Data.npy`
  `jqp_resid_final/max/rising/converged(t)`, plus
  `tools/analyze_joint_qp_convergence.py`) is committed and STANDS — pure
  observability, unaffected by this reject.
- `test_joint_qp()` in `validate_cbf.py` STANDS (17/17) — real new coverage of the
  `CBF_JOINT_QP` path regardless of this env's fate.
- `CBF_visibility.pdf` S4 open items unchanged: #1 (corrected joint feasibility
  condition with a thrust conjunct) untouched; the fixed 6x5 iterate budget still
  has no convergence test / early-exit (only the residual is now logged); the
  relief<->box coupling of #2 is real but its obvious fix regresses — leave it.
- **Methodology reminder:** `validate_cbf.py` passing (17/17 here, 13/13 for
  `Rz_p90b`, 12/12 for `CBF_MARGIN_RESERVE`) is NOT evidence a CBF change helps. Only
  an IC2-5 cross-marker SITL A/B is. Three synthetic-clean CBF "fixes" have now
  regressed in SITL.

## MECHANISM — box<->relief closed-loop DEADLOCK (traced 2026-09-08 from the bundle)

Terminal-window (last 25-40%) trace of the regressed `relief_on` reps (IC2 rep2/rep3,
IC3 rep2, IC5 rep2) vs their matched `relief_off` reps, from `Control_Data.npy`
(`I_a`/`I_a_raw`/`az_joint_delta` -> relief proxy, `B_T`, `s_e_n`, `a_u`, `kappa`,
`theta_cone`, `MARKER_EXTENT_PX`) + GT altitude:

| | regressed `relief_on` | matched `relief_off` |
|---|---|---|
| terminal mean relief (m/s^2) | **0.73** (0.24 / 0.56 / 1.34 / 0.79) | 0.09 (0.01 / 0.08 / 0.10 / 0.18) |
| frac of terminal frames `I_a[2]` pinned at exactly `-g` | up to **0.71** | ~0.00-0.19 |
| frac of terminal frames `B_T < 0.1` (thrust collapsed) | **0.48-0.82** | 0.26-0.35 |
| descent stall (consec. frames \|vz\|<0.1 at alt 0.3-1.6 m) | **4-8 s** | 0-0.2 s (except IC3) |
| terminal `s_e_n` (lateral error; FoV edge = 1.0) | 0.47-0.96, **not shrinking** | similar-or-higher but flight ends |
| terminal `a_u` max | 5 -> **115 -> 1666** | 30-85 |

The corrected relief is **proportional and persistent**: while the FoV box binds it
relieves a steady ~0.7-1.3 m/s^2 EVERY frame. Near a tight-margin off-center
touchdown the box binds every frame (marker fills the frame + off-centre -- the exact
condition IC2-5 are built to stress), so the relief fires every frame, and with the
`min(Ia_z, ...)` it **pins `I_a[2]` at `-g` -> `B_T` -> 0 -> the descent freezes**
for seconds at 0.4-1.5 m. The freeze does NOT fix the off-centre condition (the box
is suppressing the very lateral authority that would centre it), so the box keeps
binding, so the relief keeps firing: **bind -> relieve -> freeze -> still bind.** A
closed-loop deadlock. `s_e_n` sits pinned near the FoV edge the whole stall, often
drifting out to TARGET_LOST, then a terminal `a_u` blow-up (kappa integrating against
a frozen error) ends it.

**Why the BUGGY self-inflation avoids the deadlock:** it is unstable, so it SPIKES
(0.3 -> 0.9 -> 1.5 in a couple of frames) and in doing so perturbs the state enough
that the box momentarily stops binding -> relief collapses to ~0 -> the descent
resumes. It is an accidental **dither / limit-cycle** that keeps the vehicle creeping
down instead of freezing. Ugly, over-reactive, theoretically wrong -- but it never
deadlocks. The "fix" removed the accidental escape and turned a bursty limiter into a
sustained descent lock.

**When the fix is benign:** reps where the box does NOT bind hard terminally (IC2
rep1/4/5, IC5 rep1/3/4/5) show `relief_on` terminal relief 0.02-0.14, ~= `relief_off`,
and land fine. The fix only bites when the box binds every frame -- which is exactly
the stress case.

**IC4 rep3 (`relief_on`, xy 8.50) is NOT this mechanism** -- `MARKER_EXTENT_PX` stuck
~40 px the whole flight (vs 280-318 normal) and the z-SMC itself (`I_a_raw[2]` -10.0
-> -10.2, relief=0) drove a climb 7 m -> 12.5 m: a perception non-acquisition flake,
counts as SITL variance, not a relief effect.

**Takeaway for `CBF_visibility.pdf` S4 #2:** the within-solver "relief and box fight
across iterates" is real, but the deployed relief's *closed-loop* danger is a
box<->relief DEADLOCK, and the current (buggy) code is inadvertently immune to it
because it is unstable. A correct fix has to break the deadlock loop itself -- e.g.
cap the relief's cumulative/duration (not just per-cycle), or require evidence the
lateral error is actually shrinking before continuing to relieve, or make the box
back off (not just the descent) when both have been binding for N frames. Just making
the per-cycle relief "correct" makes it worse.
