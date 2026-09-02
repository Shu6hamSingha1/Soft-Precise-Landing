---
name: feedback_detector_offline_replay_gotchas
description: "Three gotchas when scoring cross-marker detector variants offline on recorded frames: (1) recorded PNGs carry the ring-flow debug OVERLAY (CROSS_RING_OVERLAY_DBG default ON) -- live detection is unaffected, but offline replay sees ~3-5% drawn pixels and 15-21% of the CROSS_ADAPT_GATE mask is overlay halo; measured NOT to explain adapt's accuracy deficit. (2) headline detOK% is inflated by post-touchdown ground frames -- read the per-altitude bands. (3) ad-hoc runs+--frames pairing scores frames past the end of GT against clamped values."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**2026-09-02.** Gotchas found while scoring detector front-end variants with
`tools/validate_detector_gt.py`. All three inflate or distort offline numbers; none of
them touch the live flight path. Companion to
[[feedback_cross_detector_contrast_not_darkness]] (the design principle + eval set).

## 1. Recorded frames carry a debug OVERLAY -- live is clean, REPLAY is not

`cross_marker_perception.py` (~line 2506) draws the currently-tracked flow points onto
the frame it records, gated by **`CROSS_RING_OVERLAY_DBG`, which DEFAULTS TO "1"**. It
is inside the `if self.RECORD and self.CONTROLLER_READY:` block and draws on a
`frame.copy()`, so:

- **The live detection path never sees it** (user-confirmed, verified in code). The
  overlay is a post-processing aid for failure analysis, not an online step.
- **But it is baked into the saved PNGs** (`imwrite` at ~line 2414), so every offline
  replay of those recordings -- `validate_detector_gt.py` included -- scores contaminated
  images.

Measured on all 6 `test_data/DetectorFrameset` sets: **2.8-5.0 % of pixels are drawn**
(scene is ~96 % greyscale, so the colour is unambiguous). Fraction of each gate's mask
that is overlay halo:

| gate | share of mask that is drawn overlay |
|---|---|
| legacy `inRange(V<=100)` | 0.8-3.7 % (bright dots fail the dark gate -- near-immune) |
| `CROSS_ADAPT_GATE` (CLAHE+adaptiveThreshold) | **15.5-20.9 %** |

So contrast-based front ends key on the artifact far harder than the legacy gate does --
expect any gradient/profile variant to be worse still.

**BUT it does NOT explain adapt's accuracy deficit.** Perturbation test (inpaint over a
dilated overlay mask, re-score): adapt's within-0.15 hit-rate stays ~7-8 pts BELOW
baseline wherever baseline works -- flat_IC1 87.6 vs 94.8, flat_IC2 65.1 vs 72.9,
rover_IC4 84.8 vs 92.5. The deficit is real and is the fitter picking wrong line pairs,
exactly as [[feedback_cross_detector_contrast_not_darkness]] says. Removing the overlay
DOES move absolute numbers ~3-8 pts (mostly on the clutter sets, both directions), so
treat DetectorFrameset figures as carrying a ~+/-5 pt band. (Caveat: inpainting is itself
a perturbation, not ground truth -- good enough to answer "is the overlay driving this?",
not to produce corrected reference numbers.)

**Record future eval sets with `CROSS_RING_OVERLAY_DBG=0`.** Cheap, removes the confound.
Do it before developing the contrast front end, not after.

Incidental datum: the overlay is green for ring-sampled points and yellow for the
unconstrained-sampler fallback. In ALL 6 recorded sets green is **0.00 %** -- the ring
sampler was in fallback the whole time. Unexamined; may or may not matter.

## 2. Headline detOK % is dominated by post-touchdown GROUND frames

The recordings keep rolling after touchdown, and a camera sitting on/near the marker
detects it ~99 % of the time. Those frames swamp the average:

- `rover_IC2`: 133 of 310 frames are below 0.7 m alt, at 98.5 % detOK -- while the
  **4-6 m acquisition band is 20 %**.
- `clutter_IC2`: 133 of 402 below 0.7 m at 99.2 %, vs **27 % at 4-6 m**.

**Always read the per-altitude bands, and judge on the 4-6 m acquisition band** -- that
is what decides whether the flight can start closing its lateral offset at all. A run
whose offset stays frozen (GT lateral 2.76 -> 2.31 m across a whole rover descent) is
one whose acquisition band failed, regardless of a healthy-looking headline number.

Corollary: low-altitude failures in a run that never acquired are a CONSEQUENCE of the
blind dive (the drone is 2.4 m off, staring at the platform edge), not independent
evidence about the terminal regime. Don't diagnose the terminal phase from a run that
failed acquisition.

## 3. Ad-hoc `runs` + `--frames` pairing scores frames past the end of GT

`_score` aligns recorded frames to the tail of `Img_Data` (`off = M - N`) and interpolates
GT with `np.interp`, which **clamps** outside `t_g` rather than rejecting. On hand-paired
`Landing_Test/<ts>` + `Test_Videos/<ts>_raw` dirs the image record often outlasts the GT
window, so a large tail is scored against frozen values: measured **117/308 frames (38 %)
past `t_g` end** on one rover run, all clamped to the touchdown alt/bearing. That inflated
its detOK from a true 18 % to a reported 49 %.

**Prefer `--set test_data/DetectorFrameset`** (curated, coverage checked) over hand-pairing.
If you must hand-pair, filter to `t_g[0] <= t <= t_g[-1]` before believing any aggregate.

## 4. Run-log messages: which ones actually mean anything (2026-09-02)

Per-rep logs live NEXT TO the rep dir, not inside it: `ICValidation/<ts>/IC3_rep2.log`
alongside `ICValidation/<ts>/IC3_rep2/Ground_Truth.npy`. Pair them by that name to correlate
log text with measured outcome. Done over **2396 paired runs** (landed = lowest altitude
above the LANDING SURFACE <= 0.20 m):

| log line | in LANDED runs | in AIRBORNE-ending runs | verdict |
|---|---|---|---|
| `Disarming denied: not landed` | **97.4 %** | 87.3 % | ⛔ ROUTINE NOISE — means nothing |
| `Impact detected (\|a\|>50)` | 55.9 % | **26.6 %** | real signal, but fires spuriously |

**⛔ Do not read `Disarming denied: not landed` as PX4 contradicting a touchdown.** It appears
in ~97 % of genuine landings too. (I briefly cited it as evidence that PX4 disagreed with an
impact latch — wrong, retracted.)

**`Impact detected` IS a spurious-latch source.** It ends the descent loop
(`FC_node.LANDED=True`) and fires in 26.6 % of runs that end airborne. Confirmed case: a
GT-feedback static `rover_cross` run latched on `|a|=65.6 m/s²` while 0.49 m above the
platform, still descending at -0.32 m/s with `B_T=-0.36` still commanding descent, and was
then scored SOFT+PRECISE. This is a THIRD truncation mechanism alongside the touchdown-detect
latches -- and it is mode-independent (fires under GT-feedback too). See
[[project_20260902_archive_rescore_false_precise]] and
[[project_20260901_rover_cross_perception_diagnosis]].
