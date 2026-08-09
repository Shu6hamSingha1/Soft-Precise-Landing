# HANDOVER — cross-marker Hz sign-flip investigation (2026-08-09)

**Read this first, then `docs/PLASMC_TUNING_GUIDE.md` for general orientation
if needed.** This doc exists because a session ran out of context mid-
investigation; it's a full narrative, not just a status line, so a fresh
session can pick this up without re-deriving the last several hours of work.

## TL;DR — where things stand

The cross+stub marker's `Hz` (vertical optical-flow) channel gives
catastrophic negative R² (roughly -12 to -35, sometimes far worse) on
landing validation flights, specifically in the **0.5-2.0m altitude band**,
while every other band and every other channel (Hx, Hy) are fine or merely
weak. This has been investigated across several sessions. As of this
handover:

- It is **confirmed NOT** caused by: the low-altitude calibration phases
  contaminating the fit, the landing-gear collision extension, or the
  hi-res marker texture swap. All three were tested via proper matched
  multi-flight batches (not single-flight A/Bs — see the methodology
  warning below) and ruled out.
- The **actual root cause is identified but not yet fixed**: the raw
  (pre-calibration) `Tz` signal's correlation with true GT vertical
  velocity **flips sign around 2m altitude** — strongly positive above,
  strongly negative just below. A linear calibration fit necessarily gets
  this wrong in one region or the other.
- **What's NOT yet known**: *why* the sign flips. The image-Jacobian
  formula itself (`_fill_A`'s Tz column) has no altitude term, so the flip
  must come from the tracked point set or a timing/KF effect. This is the
  next thing to chase.

## How we got here (chronological, don't re-litigate ruled-out steps)

1. **Low-altitude calibration phases (`lowalt_x/y/yaw`) were added** in an
   earlier session to give the calibration fit coverage down to 0.7m
   (`CALIB_LOW_ALT` env in `apps/record_cross_marker_calibration.py`).
   Investigation found these phases never actually settle — the position
   controller is still descending through most of the phase window (e.g.
   5.3m → 1.0m over the first 6s of a ~12s window), and the fit's purity
   gate (`clean_axis_mask` in `tools/derive_cross_marker_cal.py`) only ever
   recognizes literal `x`/`y` phase labels, so this transient-contaminated
   data fed the joint LSTSQ fit completely ungated.

2. **User correction (important, load-bearing insight):** the calibration
   matrix is supposed to be **altitude/depth-invariant by construction** —
   both the raw columns and the GT targets are already Z-normalized
   (`h = v/(z+Z_REG)`, see `feedback_scale_free_depth_free` memory). So
   "add low-altitude coverage" was solving the wrong problem — a correct
   cal shouldn't need altitude-specific training data at all. The fix
   should be to derive from the **cleanest (highest-SNR) data**, not to
   chase coverage.

3. **Applied:** `tools/derive_cross_marker_cal.py` now excludes
   `lowalt_*` phase samples from the M-fit by default
   (`CROSS_CAL_EXCLUDE_LOWALT=1`). Re-derived cal is live in
   `src/cross_marker_perception.py`'s `_sensor_cal_hw`/`_sensor_cal_s`
   (see the 2026-08-09 comment block there). **Keep this default** — it's
   correct in principle even though it turned out not to be the fix for
   the specific Hz-collapse symptom (see next point).

4. **But excluding lowalt data barely moved the numbers** on the same
   validation flight (2.0-1.0m Hz: -12.15 → -10.89) — so this wasn't the
   cause of the catastrophic-Hz symptom, even though keeping the exclusion
   is still the right default going forward.

5. **Bisected leg-extension vs. texture-swap** using a flight recorded
   after the leg-gear collision extension but before the texture swap
   (`validation_data/cross_output_legext_test/`). Found:
   - The 1.0-0.5m catastrophic Hz (~-39) reproduces even with the OLD
     (pre-08-09) calibration on a flight from BEFORE any of this session's
     changes — i.e. it's not a regression, it's the **original problem
     task #7 was opened for**.
   - The 2.0-1.0m band looked like a new regression from a single-flight
     comparison (baseline -0.13 vs. this flight -2.62), but see the next
     point — that comparison methodology turned out to be unreliable.

6. **Methodology correction (important — internalize this before running
   any more A/Bs on this channel):** single-flight before/after
   comparisons on the cross-marker's Hz channel in the 0.5-2.0m band are
   **not reliable**. Ran a proper 5-flight batch under the CURRENT setup
   (leg-extension + hires-texture + lowalt-excluded cal):
   `validation_data/cross_output_landing_batch/` — 2.0-1.0m Hz clustered
   TIGHTLY at -12.3 to -18.8 (mean ≈-15), 1.0-0.5m at -20.5 to -35.0 (mean
   ≈-28), consistent across all 5 flights. Then ran a matched batch under
   the REVERTED setup (old collision geometry via
   `x500_base/model.sdf.bak_before_legext_20260809`, old texture via
   `cross_marker/cross_marker.png.bak_before_hirestex_20260809`, old cal):
   `validation_data/cross_output_landing_batch_reverted/` (3 of 5 flights
   valid — SITL flaked on the other 2, normal pattern for this project) —
   2.0-1.0m Hz = -2.75 / **-198** / -21.3, 1.0-0.5m Hz = -9.8 / -31.8 /
   -0.03. **The reverted setup shows the SAME catastrophic range with
   WORSE variance**, not a cleaner signal. Conclusion: none of the
   session's changes caused or worsened this — it's present, comparably
   bad, in both configurations. **Lesson: this channel's run-to-run
   scatter (-0.03 to -198 seen across just 3 same-setup flights) is bigger
   than most real effects worth chasing — always batch (n≥3, ideally 5)
   before concluding a change moved this metric.**

7. **Root cause found:** computed the raw pre-calibration `Tz` signal's
   correlation with true GT vertical velocity directly (no calibration
   involved at all — `img['h_V'][:,2]` from `Img_Data.npy` vs.
   `_compute_gt_flow_zreg`'s GT), altitude-binned, on 3 independent
   flights from `cross_output_landing_batch/`:

   | band | run1 | run2 | run3 |
   |---|---|---|---|
   | 9-2.0m | +0.65 | +0.68 | +0.70 |
   | 2.0-1.0m | **-0.74** | **-0.78** | **-0.61** |
   | 1.0-0.5m | -0.13 | -0.18 | -0.45 |

   This is a clean, systematic, repeatable sign inversion — not noise.
   Any single linear calibration coefficient fit mostly on abundant
   high-altitude data will pick a positive Tz coefficient, guaranteeing
   wrong-direction predictions below 2m. **This is the actual mechanism
   behind every catastrophic negative Hz R² seen across this whole
   multi-session investigation, under every tested
   geometry/texture/calibration configuration.**

   Checked `_fill_A` (`src/cross_marker_perception.py`, ~line 190): the Tz
   column is `-x, -y` (standard depth-normalized image-Jacobian form) with
   **no altitude term at all** — so the formula itself isn't the source of
   the flip. It must be coming from the tracked point SET
   (`_sample_flow_points`, which has altitude/extent-sensitive exclusion
   logic — `CROSS_FLOW_CENTER_EXCLUDE_FRAC`, `FLOW_BOUNDARY_MARGIN_PX`) or
   a timing/KF effect (dt behavior, `kf_filter_causal`) that changes
   character across that altitude threshold. **Not yet determined which.**

## Reframing task #7

Task #7 was filed as "cross-marker has no close-range fallback for FoV
overflow." That framing is now known to be wrong — the actual mechanism is
a sign-inverting per-frame Jacobian/point-tracking solve around 2m
altitude, unrelated to FoV overflow (which was investigated and ruled out
earlier — the cross-marker's continuous single-marker handover was
specifically designed to avoid the ArUco nested-board overflow problem,
and does; see the design-intent note earlier in
`project_cross_marker_pipeline_20260801` memory). Update task #7's
description/title to reflect this before continuing, or open a new task —
whichever the new session's user prefers.

## Concrete next steps (pick up here)

The natural next move is to find WHY the sign flips. Candidate approaches,
roughly in order of how directly they test the two live hypotheses:

1. **Point-set composition hypothesis.** Dump the actual tracked point set
   (`_sample_flow_points`'s output, or instrument `_solve_jacobian`) for
   frames just above vs. just below the 2m threshold on one of the
   `cross_output_landing_batch/` flights. Check whether the point
   distribution (count, radial spread, arm-vs-stub-vs-background mix)
   changes qualitatively there — e.g. does `CROSS_FLOW_CENTER_EXCLUDE_FRAC`
   (0.35, a FRACTION of the marker's on-screen extent) start excluding a
   materially different subset once the marker fills more of the frame at
   low altitude? A per-point breakdown of each point's contribution to the
   solved Tz component (row-by-row in the pseudoinverse) would show
   directly whether specific points are driving the sign flip.

2. **Timing/KF hypothesis.** Check whether `kf_filter_causal`'s effective
   lag/phase response could invert a *fast-changing* Tz signal's apparent
   correlation with GT if the true signal's frequency content changes
   with altitude (e.g., faster apparent rates near the ground due to
   parallax). Test by re-running the same altitude-binned raw-correlation
   check using the UNFILTERED raw signal (`img['h_V']` before any KF) —
   this data is likely already available in `Img_Data.npy` or easy to
   recompute; if the sign flip persists identically pre-KF, this
   hypothesis is ruled out.

3. **Sanity-check against `img_data.py`'s (ArUco) equivalent.** The ArUco
   board's `_fill_A`-based solve is the same math, applied to a
   differently-shaped point set. If ArUco's raw Hz-vs-GT correlation does
   NOT flip sign across the same altitude range, that's evidence the cause
   is cross-marker-specific (its point-sampling / marker geometry), not a
   general image-Jacobian issue with this camera/altitude regime.

4. Once the mechanism is found, the fix is likely either (a) an
   altitude-adaptive point-selection rule, (b) an altitude-adaptive
   calibration (two regimes above/below the flip point — inelegant but
   possible), or (c) whatever the root cause suggests once found. Don't
   guess at a fix before nailing the mechanism.

## Data/artifacts left behind

- `validation_data/cross_output_landing_batch/` — 5 flights, current
  (leg-ext + hires-tex + lowalt-excluded cal) setup. One of the 6 raw
  entries (`Sun Aug 9 16-52-20`) never descended below 2m — treat as a
  failed/aborted run, exclude from any re-analysis.
- `validation_data/cross_output_landing_batch_reverted/` — 3 valid
  flights (2 SITL-flaked, produced no data), reverted (old geometry/
  texture/cal) setup.
- `validation_data/cross_output_legext_test/` — single flight,
  leg-extension-only (no texture swap), used for the point-5 bisection.
- `tools/make_cross_marker_hirestex.py` — regenerates the cross-marker
  texture at higher native resolution (not the cause of anything here,
  kept as the adopted texture fix).
- PX4-Autopilot backups for both the leg-extension and texture changes
  exist as `.bak_before_legext_20260809` / `.bak_before_hirestex_20260809`
  (pre-change) and `.bak_current_combinedfix_20260809` (post-change, saved
  during the revert-and-restore A/B) under
  `~/PX4-Autopilot/Tools/simulation/gz/models/{x500_base,cross_marker}/`.
  **Live PX4-Autopilot state as of this handover is the CURRENT
  (post-fix) one** — leg extension and hires texture are both active; the
  revert was temporary and cleanly undone.
- `PX4_Gazebo/px4_autopilot_overrides/` (the git-tracked PX4-Autopilot
  backup) is **STALE** — does not yet include the hires-texture swap.
  Re-snapshot it before relying on it for a restore.

## Don't re-do

- Don't re-add a `lowalt_zyaw`-style purity gate or re-litigate whether
  low-altitude calibration coverage is needed — point 2/3 above already
  settled this (altitude-invariance is the right frame; excluding
  `lowalt_*` from the fit is the correct default, already applied).
- Don't re-run a single-flight A/B to test whether some change affected
  Hz in the 0.5-2.0m band — batch n≥3 minimum, per the methodology note
  in point 6.
- Don't chase texture resolution or landing-gear standoff further for
  THIS specific problem — both were real, valid fixes for other things
  (Hx/Hy quality and touchdown standoff respectively) but are confirmed
  unrelated to the Hz sign-flip.
