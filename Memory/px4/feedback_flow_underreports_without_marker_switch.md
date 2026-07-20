---
name: feedback_flow_underreports_without_marker_switch
description: "GT-validated (tools/gt_optical_flow.py, Z_REG 0.01->0.2 FIXED 2026-07-11) on IC1_rep3 (2026-07-11 batch): measured h_z shows a real but MODEST sustained under-report vs GT loom (diff grows ~0.1->0.6 over ~1s, not the originally-reported 1.6 -- that larger number was substantially the tool's OWN broken 1/(z+0.01) regularization, not the real signal) over ~1s of CONTINUOUS single-marker tracking (extent smoothly 66->164px, NO marker-ID switch). A genuine, smaller under-conditioning effect, distinct from the marker-switch misattribution mechanism in project_ic1_terminal_kick_root_cause_chain."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a8922284-2fe3-4a78-9355-9949c3be5a10
---

## CORRECTION (2026-07-11): the originally-reported magnitude was a tool artifact

The first pass of this finding used `tools/gt_optical_flow.py` with its ORIGINAL
`1/(z+0.01)` depth regularization, which — like the `gt_feedback.py` `Z_REG` bug documented in
`feedback_zreg_gear_floor_artifact` — lets computed depth fall below the physical ~0.2m gear
floor near touchdown, producing a fake, unbounded loom blowup (GT reached −1.7 to −6+, then NaN).
**Fixed: `gt_optical_flow.py`'s regularization changed `0.01`→`0.2`** (both the bearing `V_s_g`
and flow `B_h_g`/`V_h_g` terms), matching the `Z_REG=0.2` gear-height precedent.

**Re-run with the fix: GT loom is now properly bounded** (max ≈ −0.64 in the previously-analyzed
window, not −1.7+) and the measured-vs-GT diff is **real but much more modest**:

```
tg=8.53  GT_loom=-0.25  meas=-0.15  diff=0.10
tg=8.91  GT_loom=-0.42  meas=-0.21  diff=0.21
tg=9.35  GT_loom=-0.54  meas=-0.32  diff=0.22
tg=9.73  GT_loom=-0.64  meas=-0.00  diff=0.63
```

(vs the original, artifact-inflated report of diff growing to 1.59 — retract that specific
number; the mechanism/direction below still holds, just smaller.)

## Finding (GT-validated via tools/gt_optical_flow.py, IC1_rep3, bundle 20260711-051856)

With the SAME marker continuously, successfully tracked throughout (`N Flow Corners` stayed
130-170, no miss; `MARKER_EXTENT_PX` grew smoothly 66→164px, no discontinuity, no marker-ID
switch), the measured `h_z` **still shows a genuine, one-directional under-report** relative to
GT, growing from ~0.10 to ~0.6 over roughly 1 second approaching touchdown — smaller than first
reported, but consistent in DIRECTION and not attributable to noise (every sample in the window
shows measured-below-GT, never the reverse).

The genuine KLT-fallback miss window in this rep (tg≈9.93-9.95s) remains too close to touchdown
to validate directly — GT still goes NaN just before it, now from the separate `abs(zB)>=0.1`
too-close-to-compute guard (a legitimate exclusion, not the regularization artifact).

## Relationship to other findings this session

- **Distinct from [[project_ic1_terminal_kick_root_cause_chain]]'s mechanism** — that incident
  required a marker-ID SWITCH (small→big) to trigger the misattribution explosion. Here there is
  NO switch, and the error is in the OPPOSITE direction (under-report, not explosive over-report).
- **Consistent with, but more modest than, the ORIGINAL under-conditioning hypothesis** raised
  early in the 2026-07-10 investigation (small marker's near-center, few/tightly-clustered
  corners give a poorly-conditioned flow estimate, biased toward under-reporting fast real
  motion) — real, GT-confirmed, but the effect size is smaller than the first (artifact-inflated)
  pass suggested. Don't over-weight this as a dominant mechanism without further validation.
- **Relevant to [[project_touchdown_detect_velocity_gate_gap]]** (the open "detects but not
  soft" issue), but with reduced urgency given the smaller confirmed magnitude — any future
  velocity-magnitude gate design should still account for SOME under-report, just not assume it's
  as large as first measured.

## Methodology notes for future use

- `tools/gt_optical_flow.py`'s depth regularization is `Z_REG=0.2` (fixed 2026-07-11, was 0.01)
  — matches `gt_feedback.py`'s precedent. If re-deriving or cross-checking against an OLDER
  cached result computed before this fix, expect the old numbers to be inflated near touchdown.
- `_main()` CLI path breaks on recordings where `Opt Flow Fused` is 1-D (e.g. any run with
  `FLOW_FUSE_RING=0`, now the default) — use the importable `compute_gt_flow(rep_dir)` function
  directly instead.
- `align(t_abs, y)` resamples a MEASURED signal (arbitrary absolute timestamps) ONTO the GT's own
  descent-relative timebase (`t_g`, 0-based from `start_time` = when the controlled descent
  begins) — call it as `align(measured_t, measured_y)`, not the reverse.
- `start_time` means the GT-covered window does NOT include the pre-descent
  IC-convergence/hover-align phase — a KLT-fallback event logged in the app log may have occurred
  entirely OUTSIDE what this tool tracks; check `t_img >= g['start_time']` before assuming a
  logged event is GT-comparable.
