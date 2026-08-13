---
name: diagnose-flight-data
description: How to correctly diagnose a recorded PX4/Gazebo landing flight (Control_Data.npy / Ground_Truth.npy / Img_Data.npy) — always compare a perception or control signal against PROPERLY-COMPUTED ground truth, never against the controller's own internal desired/reference value alone, and always verify timestamp sync directly rather than assuming it. Use whenever diagnosing a landing failure, a hard touchdown, an unexpected controller signal, or any post-flight "why did X happen" investigation.
---

# Diagnosing a recorded flight

This skill exists because of a real mistake (2026-08-11): a hard-touchdown
diagnosis initially concluded "the perceived signal is wrong" by comparing
it only against the controller's own internal desired/reference value
(`h_d(t)`) — not against ground truth. That's not a diagnosis of a
perception error, it's just a report that the controller wasn't tracking
its own setpoint, which could mean either the perception OR the reference
was the outlier. Redone properly against real GT, the finding held up but
was also revealed to be a bigger/different problem than first thought (see
`PX4_Gazebo/docs/HANDOVER_cross_marker_headless_flight_testing_20260811.md`).
Don't repeat this mistake.

## The hard rule

**A signal logged in `Control_Data.npy` (or any per-controller-step log) is
what the CONTROLLER believed/commanded — it is never ground truth, even
when it's named for a physical quantity (`h(t)`, `s(t)`, `w_i(t)`, etc.).**
To diagnose whether a perception or control signal was *correct*, always
compute an independent ground-truth value from `Ground_Truth.npy`'s raw
`UAV Pose`/`Target Pose` and compare against THAT — never conclude a
perception error from a mismatch against `h_d(t)` or any other internal
reference/desired-value log alone. A mismatch against an internal reference
only tells you tracking wasn't perfect; it says nothing about which side
(measurement or reference) was actually wrong.

## Which GT tool to use — this determines whether the answer is even valid

Two GT-flow computations exist in this codebase with **different depth
conventions**, and using the wrong one for the altitude range you're
diagnosing silently produces garbage or `NaN`:

| Tool | Depth convention | Valid altitude range | Use for |
|---|---|---|---|
| `tools/aggregate_calibration_phased.py::compute_gt_signals` | raw `v/Vz`, no regularization | **gates to `NaN` below `Vz<=1.0m`** | calibration-derivation, phased-excitation z-phase checks, anything comfortably above 1m |
| `tools/gt_optical_flow.py` / `tools/validate_cross_marker_flow.py::_compute_gt_flow_zreg` | `v/(z+Z_REG)`, `Z_REG` matches `gt_feedback.py`'s convention (default 0.2, `PLASMC_GT_Z_REG` env) | valid all the way to touchdown, `1/(z+Z_REG)` stays bounded | **anything touchdown-adjacent, near-ground, or below ~1.5m** — landing diagnosis, terminal-descent analysis, hard-landing root-causing |

**If you're diagnosing anything within ~1.5m of the ground (a landing's
final approach, a touchdown, a terminal-kick/commit event), you MUST use
the `Z_REG`-regularized tool.** `compute_gt_signals` will either silently
return `NaN` (if you're lucky enough to notice) or, if used to define your
altitude bins from a DIFFERENT quantity, quietly exclude the exact window
you care about without any error. `gt_optical_flow.py` is the canonical GT
flow/loom/s/alpha reference for this project generally — prefer it over
ad-hoc finite-difference gradients on raw pose arrays too (those produce
`0`/spike artifacts from duplicate-timestamp/quantized logging — see the
sync section below).

## Always verify timestamp sync directly — don't assume it

Every recording has (at least) two clocks that must be reconciled before
any per-sample comparison is meaningful:
- `Ground_Truth.npy['Time']` is RELATIVE to `Ground_Truth.npy['Start Time']`
  — the absolute clock is `t_abs = gt['Time'] + gt['Start Time']`.
- `Control_Data.npy['t']` (or `Img_Data.npy['Time']`) is typically already
  on the SAME absolute clock (both ultimately derive from Gazebo's
  `/clock` topic via `Clock_Node`/`msg.header.stamp`) — but verify this,
  don't assume it. Check `Ground_Truth.npy['Start Time']` against the
  first `Control_Data.npy['t']` value; they should be close.

**Before trusting any cross-array comparison, compute the actual gap:**
```python
idx = np.clip(np.searchsorted(t_gt_abs, t_other), 0, len(t_gt_abs)-1)
gap = t_other - t_gt_abs[idx]
print(np.median(gap), np.percentile(np.abs(gap), 95), np.max(np.abs(gap)))
```
A healthy sync shows a median gap of 0 and a 95th-percentile gap under one
sample period (~10-20ms range in this project's typical logging rates). If
it's not that tight, the comparison isn't trustworthy yet — find out why
before drawing conclusions from it.

**Watch for quantization/staircase artifacts when computing a rate via
`np.gradient`/finite-difference on a raw pose array.** If the underlying
position log updates at a coarser rate than the polling loop, several
consecutive samples read identical and the interval AFTER them shows an
implausible spike (seen firsthand: a computed "-20 m/s" instantaneous
velocity from a position signal that in reality moved smoothly). Sanity-
check any such spike against the surrounding window's AVERAGE rate before
treating it as real.

## Quick reference: what each file actually contains

- `Ground_Truth.npy` — the ONLY ground-truth source. `UAV Pose`/`Target Pose`
  (raw Gazebo poses), `Time`/`Start Time` (absolute clock), often phase
  labels (`Phase`) and diagnostic logs (`Flow Diag Log`, `Radial Diag Log`,
  `Point Diag Log` if present).
- `Control_Data.npy` — the CONTROLLER's own per-step internal state:
  perceived signals it computed (`h(t)`, `s(t)`), its own references/desired
  values (`h_d(t)`, `ds_d(t)`), and its outputs (`w_u(t)`, `B_T(t)`,
  `kappa(t)`, `a_u(t)`, etc.). None of this is ground truth by construction.
- `Img_Data.npy` — the perception layer's own raw (pre/post-cal) logs,
  typically at the TRUE per-detect-rate (not the outer polling rate) —
  see `feedback_dt_staleness_after_detection_dropout` /
  the oversampling-artifact history in `tools/validate_cross_marker_flow.py`'s
  `prep()` docstring for why this file's own `Time` field, not GT's outer
  polling rate, is the right per-frame clock for perception-side analysis.

## Procedure

1. Identify the altitude range in question. If it touches within ~1.5m of
   the ground, use the `Z_REG`-regularized GT tool — not `compute_gt_signals`.
2. Compute the GT quantity independently from `Ground_Truth.npy`'s raw poses.
3. Verify timestamp sync between the GT array and whatever you're comparing
   it against (print the gap stats above — don't skip this).
4. Compare the CONTROLLER's logged signal against this independently-computed
   GT, not against its own internal reference/desired value.
5. Only after that comparison holds up, additionally check the internal
   reference if useful for understanding WHY the controller reacted the way
   it did (a reference/tracking-error view is a valid SECOND step, just not
   a substitute for the GT comparison).
