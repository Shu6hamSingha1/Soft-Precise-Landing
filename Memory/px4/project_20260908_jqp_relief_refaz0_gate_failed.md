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
