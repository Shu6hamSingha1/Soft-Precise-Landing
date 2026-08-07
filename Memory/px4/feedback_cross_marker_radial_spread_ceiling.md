---
name: feedback_cross_marker_radial_spread_ceiling
description: "Cross-marker Hx/Hy/Wx/Wy underperformance vs ArUco traced to tracked-point radial spread (a software-fixable point-selection bias, NOT just marker size); Hz specifically does NOT respond to spread and remains unexplained"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 79646ad7-0ee0-45bb-bd3b-73e43c1470bf
  modified: 2026-08-07T06:00:39.210Z
---

The cross+stub marker's flow-Jacobian solve (`_fill_A` in
`PX4_Gazebo/src/cross_marker_perception.py`) needs tracked points spread far
from the image center to observe Hz/Wz (linear columns) and especially Wx/Wy
(quadratic columns: `x*y`, `1+x^2`, `1+y^2`). Measured spread only reaches
~35-50% of the frame's normalized radial half-extent in every phase — this is
the root cause of Hz/Wz lagging ArUco's own board cal and of Wx/Wy being
practically unrecoverable (near-zero, sign-flipping correlation with GT even
under dedicated rollexc/pitchexc excitation).

**Why:** Two candidate fixes were tried and BOTH ruled out as insufficient
(don't retry either in isolation):
1. `_restrict_to_center_roi(roi_frac_y=0.65)` ghost-defense crop — loosened
   to 0.90/1.00 via new `CROSS_ROI_FRAC_Y` env var. Ghost defense held fine
   (`_reject_blobby_components` is the real defense now, ROI is redundant
   backstop), but Wx/Wy correlation only moved to a noisy +0.1..+0.26, and
   x-spread (unaffected by roi_frac_y) barely moved at all — proving x was
   never ROI-limited in the first place.
2. Lowering `CALIB_TAKEOFF_HEIGHT` (2.7m -> 2.4m -> 2.0m) to force more
   angular subtense: 2.4m held ok-rate 100% but didn't move Wx/Wy
   correlation; 2.0m broke detection outright (ok-rate 25%, marker overflows
   frame / line-geometry fit fails) before flow signal could improve.

The real ceiling is the marker's PHYSICAL FOOTPRINT (currently 3.0m plate at
2.7m calibration altitude) — not a recorder or detector parameter. The only
untried lever is a bigger physical marker
(`~/PX4-Autopilot/Tools/simulation/gz/models/cross_marker/model.sdf` size),
so tracked points sit farther from center while the whole shape still fits
in frame.

**How to apply:** Don't re-attempt ROI loosening or altitude lowering alone
expecting a Wx/Wy fix — both were tested cleanly (with a temporary radial-
spread diagnostic, `Radial Diag Log`, left in place in
`cross_marker_perception.py`/wired into `record_cross_marker_calibration.py`)
and neither moved the needle enough to matter. If revisiting this, the
next real lever is marker plate size, not maneuver/detection parameters.
See [[project_cross_marker_pipeline_20260801]] for the full data table.

**2026-08-06 correction/confirmation:** a same-day investigation briefly
suspected a DIFFERENT cause for the Hz/Wz weakness specifically (2 of 6
calibration recordings' z-phase GT looking contaminated by dominant
yaw-rate) and added a purity gate to `derive_cross_marker_cal.py` to filter
it out. That "contamination" turned out to be a bug in an ad-hoc diagnostic
script (mis-aligned `gt['Phase']` truncation vs `compute_gt_signals`'s
internal duplicate-timestamp filter), not a real data problem — re-checked
with correct alignment and all 6 recordings' z/yaw/yawagg phases are
genuinely clean. The purity gate was reverted (it made Wz worse, consistent
with there being nothing real for it to remove). Net effect: **this
memory's radial-spread/footprint explanation stands, uncontradicted** — the
2026-08-06 detour ruled out a competing "data contamination" hypothesis and,
if anything, adds supporting mechanistic detail: Wz's fitted coefficients
are large specifically on the y-flow columns, the signature of
near-collinearity you'd expect from insufficient radial spread (columns of
the flow-Jacobian regressor becoming linearly dependent), not a sign that a
different, unrelated cause is at play. No numeric values in the deployed
`_sensor_cal_hw`/`_sensor_cal_s` changed as a result.

**2026-08-06 follow-up — a software-only spread fix WAS found (partially
correcting this memory's "only untried lever is a bigger marker" claim),
with a genuinely mixed result:** `_sample_flow_points`
(cross_marker_perception.py) was changed to exclude a central disk
(`CROSS_FLOW_CENTER_EXCLUDE_FRAC=0.35`× the mask's own half-extent) from the
`goodFeaturesToTrack` mask, biasing candidate corners toward the arm/stub
tips instead of letting the cross's central intersection (the shape's own
strongest corner) dominate every frame's point pool, plus a frame-boundary
exclusion margin (`CROSS_FLOW_BOUNDARY_MARGIN_PX=20`) so corners likely to
exit-frame mid-track (and be silently dropped, biasing the surviving pool
back toward center) are never selected. Falls back to the unbiased mask if
too few peripheral corners survive.

Re-recorded 4 fresh valid runs (95%+ ok-rate gate) and re-derived:
- Radial spread genuinely increased as measured (`Radial Diag Log`): mean
  p90 0.113→0.184, max p90 0.289→0.380 — confirms the point-selection bias,
  not marker size, was masking real headroom.
- **Hx/Hy improved substantially: 0.55→0.73, 0.63→0.79.**
- **Hz did NOT move: 0.22→0.22 to 2 decimal places**, despite the same
  spread increase that helped Hx/Hy. Wz got marginally worse (R² 0.57→0.53,
  inter-run STD worse on some columns) though its own coefficient magnitude
  dropped (12.9→9.4).

**Correction to this memory's original conclusion:** radial spread being
software-fixable (not solely a marker-size ceiling) is now established, and
it DOES explain Hx/Hy's weakness relative to ArUco. But **it does NOT
explain Hz specifically** — Hz's flat R² despite a confirmed, substantial
spread increase rules out radial spread as Hz's (sole) binding constraint.
The peripheral-bias code is a net win for Hx/Hy (and doesn't hurt detection
ok-rate). Leading untried suspects for Hz specifically: z-phase excitation
amplitude too small relative to noise, or `_getVirtualPts`'s
perspective-divide noise near grazing rays swamping the loom signal
regardless of point spread — neither investigated yet. See
[[project_cross_marker_pipeline_20260801]].

**2026-08-07 DEPLOYED + Wz mechanism confirmed.** Decision (no real tradeoff
to weigh, once measured cleanly): kept the peripheral-bias code and pasted
the freshly re-derived matrix into `CrossMarkerPerception.__init__`
(`_sensor_cal_hw`/`_sensor_cal_s`, R² Hx=0.73 Hy=0.79 Hz=0.22 Wz=0.53) — Hx/Hy
win, Hz/Wz genuinely flat (not a regression caused by this change). Also
directly confirmed the mechanism behind Wz's huge/cancelling coefficients
(the "signature of near-collinearity" noted above): raw correlation matrix
of the 6 `Opt Flow Ang Vel` columns from the live `calibration_data/
output_cross/` runs has two near-zero eigenvalues (0.0015, 0.0021) — raw
`h1`(Ty)/`w0`(Wx) correlate r=0.98–1.00 in EVERY phase alike (x/y/z/yaw/
yawagg/settle), not just where they're co-excited. Cause: `_fill_A`'s Wx
column is `-(1+y^2)`, which stays ~constant (≈-1) whenever `|y|` is small —
numerically indistinguishable from Ty's constant `+1` column at the radial
spread this marker achieves. This is a structural degeneracy in the
per-frame Jacobian solve itself, present regardless of excitation purity or
calibration-run count — confirms Wz's weakness is the same radial-spread
ceiling as Hx/Hy conceptually, but unlike Hx/Hy it did NOT respond to the
peripheral-bias fix (Wz 0.57→0.53, roughly flat) because the degeneracy
needs `y` spread specifically large enough to break `1+y^2≈const`, which the
peripheral bias's achieved spread (p90 0.18–0.38) still doesn't reach.
Reinforces: bigger physical marker remains the only lever left for Hz/Wz/Wx/Wy.

**2026-08-07 z-excitation-amplitude suspect TESTED AND RULED OUT for Hz.**
`CALIB_AMP_Z` (position-sine amplitude, m) had been stuck at 1.2 since the
cross-marker recorder was created — inherited verbatim from
`record_output_calibration.py`, where a same-named 21-day-old memory
(`reference_aggregate_calibration`) had already flagged 1.2 as aggressive
(~2.2g thrust, raises clip-saturation on flow_x/y/ω_x/y 0.03%→0.28%) and
recommended backing off to ~0.9 — a recommendation that was never applied to
either recorder's code. Single-run test at `CALIB_AMP_Z=2.2`: achieved
altitude swing 0.52m→0.80m, z-phase raw-h2/noise-floor SNR 0.16–0.60→1.98 (a
genuine ~4x signal improvement, confirmed via `UAV Pose` vs commanded-z
tracking — PX4 position-tracking attenuates the commanded sine ~2.3-2.7x at
0.5Hz, so achieved velocity is much smaller than the amplitude parameter
implies). Full n=4 batch (6 recorded, 1 empty/armable-timeout SITL flake, 1
below the 95% ok-rate gate) re-derived: **Hz 0.22→0.25, Wz 0.53→0.56 — both
within noise, NOT a real gain — while Hx 0.73→0.68 and Hy 0.79→0.75 both
dropped**, reproducing the exact ArUco-era tradeoff (over-driving z costs the
other axes) the stale memory had flagged but never validated. A ~4x SNR
improvement moving R² by only +0.03 is decisive: if noise-floor/SNR were
Hz's binding constraint, that much SNR gain should have moved R² well beyond
noise. It didn't — **this rules out z-excitation amplitude as Hz's
bottleneck** and leaves the raw h1(Ty)/w0(Wx) structural collinearity above
as the standing, amplitude-independent explanation for both Hz and Wz.
DECISION: did NOT deploy the AMP_Z=2.2 cal (costs Hx/Hy for no real Hz/Wz
benefit) — live cal stays the AMP_Z=1.2/peripheral-bias one from the entry
above. `calibration_data/output_cross_amp22/` recordings kept on disk as
negative-result evidence, not wired into any script. Both untried suspects
from this memory's earlier "leading untried suspect" line are now closed
(radial spread: fixed for Hx/Hy, doesn't touch Hz/Wz; amplitude: ruled out
for Hz) — only the marker-footprint-size lever and
`_getVirtualPts`-perspective-divide-noise idea (never investigated) remain.
