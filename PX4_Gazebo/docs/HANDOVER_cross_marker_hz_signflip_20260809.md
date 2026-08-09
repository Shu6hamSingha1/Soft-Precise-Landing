# HANDOVER — cross-marker Hz sign-flip investigation (2026-08-09)

**⚠⚠ RESOLVED (session 2, part 3): the "root cause found" claim below (point
7) was WRONG — it was a statistical artifact of one unlucky 5-flight batch,
not a real sign-inversion mechanism. Pooling all 12 available flights shows
the true correlation is POSITIVE (same sign as everywhere else), not
negative. Jump straight to the "RESOLVED: the sign flip is a noise artifact"
section near the bottom for the real explanation and what to chase instead
— everything about a "Tz sign flip below 2m" earlier in this doc (TL;DR,
point 7, "Reframing task #7", and the original "Concrete next steps") is
SUPERSEDED.**

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

## 2026-08-09 follow-up (session 2) — point-set hypothesis CONFIRMED, KF hypothesis ruled out

Tested both live hypotheses (point-set composition vs. timing/KF) against the
EXISTING `cross_output_landing_batch/` data — no new flight needed, since
`record_cross_marker_validation.py` already logs `Radial Diag Log`
(per-flow-solve `(t, maxx, maxy, meanx, meany)` — radial spread of the
tracked points, normalized image-plane coords) into `Ground_Truth.npy` (NOT
`Img_Data.npy` — easy to miss; `Img_Data.npy` only has `N Flow Corners`,
not the spread breakdown).

- **KF/timing hypothesis (candidate 2): RULED OUT.** The point-7 raw
  correlation check already used `img['h_V']` directly — that field is
  `self._perception._hw_log`, populated in `_compute_hw` BEFORE any KF
  smoothing (the KF is only applied downstream, in
  `aggregate_calibration_phased.kf_filter_causal`, by the validate/aggregate
  tools). So the sign flip was already measured pre-KF; no re-test needed.
- **Point-set composition hypothesis (candidate 1): CONFIRMED, altitude-binned
  on all 6 `cross_output_landing_batch/` flights** (script:
  `/tmp/.../scratchpad/probe_radial.py`, not checked in — trivial to
  reconstruct: join `Radial Diag Log` timestamps against GT altitude/vz via
  `compute_gt_signals`, bin by altitude):

  | band | raw-Tz-vs-GT corr (6-flight range) | mean radial spread (meanx/meany) |
  |---|---|---|
  | 2-9m | +0.03 to +0.57 | ~0.11-0.18 |
  | 1-2m | -0.34 to -0.70 | ~0.38-0.43 |

  The mean tracked-point radial spread roughly **doubles** right at the same
  altitude the Tz correlation flips sign — consistent across all 6 flights,
  not a coincidence on one. This is strong evidence the point SET (not the
  Jacobian formula, not filtering/timing) is the mechanism, matching
  candidate-1's framing in the "next steps" section below.

  **What's still open:** WHY larger spread flips the sign rather than just
  changing SNR. The saved `Radial Diag Log` only has aggregate max/mean
  spread per solve, not a per-point breakdown — can't yet tell whether the
  low-altitude point set becomes asymmetric (e.g. biased to one side of the
  marker as `CROSS_FLOW_CENTER_EXCLUDE_FRAC` pushes candidates outward and
  boundary rejection asymmetrically removes far points on one side) or
  whether it's a conditioning/aliasing effect between the enlarged-spread Tz
  column and something else in the reduced 4-unknown solve
  (`A_reduced = A[:, [0,1,2,5]]` in `_solve_jacobian`). Next step: instrument
  a per-point Tz-row contribution dump (row-by-row through the pseudoinverse,
  or just log each frame's kept point (x,y) set) on a NEW flight — the
  existing saved data doesn't have per-point resolution, only the aggregate
  spread stat used above.

## 2026-08-09 follow-up (session 2, part 2) — ⚠ CONTRADICTS the "root cause found" claim above, re-open

Built the per-point diagnostic instrumentation the point-set hypothesis needed
(`CrossMarkerPerception._point_diag_log` / `get_point_diag_log()` in
`src/cross_marker_perception.py`, wired to `Ground_Truth.npy`'s new `"Point
Diag Log"` key via `apps/record_cross_marker_validation.py` — same pattern as
`Radial Diag Log`, additive-only change, backups at
`*.bak_before_pointdiag_20260809`). Ran 3 fresh `VALIDATION_PROFILE=landing`
flights (HEADLESS=1) to test it: `validation_data/cross_output_landing/Sun
Aug  9 19-28-06 2026`, `19-30-21 2026`, `19-32-02 2026`.

**Unexpected result: none of the 3 new flights reproduce the sign flip.**
Raw Tz-vs-GT correlation in the 1-2m band was **+0.55, +0.59, +0.71** (all
POSITIVE, same sign as the >2m band) — verified two independent ways (the
new `Point Diag Log`'s solved Tz, AND the same raw-`h_V`-vs-GT method the
original point-7 finding used), both agree. This is the opposite sign from
the original 6-flight `cross_output_landing_batch/` result reported above
(-0.34 to -0.70 in the same band).

**This is not just "high run-to-run scatter" (the point-6 methodology
warning) — both batches are internally TIGHT** (new batch: +0.55/+0.59/+0.71,
spread ~0.16; original batch: -0.34/-0.61/-0.64/-0.70/-0.74/-0.78, spread
~0.4 but all negative) **but oppositely SIGNED between batches.** That
pattern — tight within a batch, flipped across batches — points at something
that differs systematically between recording SESSIONS (Gazebo/PX4 boot
instances), not per-flight noise. Point-set radial-spread doubling at the
altitude threshold (session-2-part-1's finding) still held qualitatively in
the new batch too (not re-quantified per-band here but visible in the raw
point dumps) — so that part may still be real, but it does NOT explain a
sign flip that isn't reproducing.

**Ruled out as the explanation:** code/config drift — `git diff` confirms
`src/cross_marker_perception.py` only gained the new diagnostic logging
(additive, 23 lines) between the original batch recording and these 3 new
flights; no solve-path logic changed.

**Live hypothesis for the cross-batch sign difference (untested):** something
in the PX4/Gazebo per-boot spawn/init state that the V-frame GT leveling
(`compute_gt_signals` in `tools/aggregate_calibration_phased.py`) is
sensitive to — e.g. initial spawn yaw/heading not being deterministic across
`bash scripts/run_aruco_landing.sh` launches, interacting with the
quaternion-based V-frame transform in a way that could bias vz sign
indirectly (untested — vz is nominally gravity-referenced and shouldn't
depend on yaw, so this needs checking, not assuming).

## 2026-08-09 follow-up (session 2, part 3) — RESOLVED: the "sign flip" is a noise artifact, not a real mechanism, root cause is elsewhere

Pooled ALL 12 available landing flights (5 valid from the original
`cross_output_landing_batch/` + 7 from `cross_output_landing/`, spanning
Aug 8 19:09 through Aug 9 19:32 — several independent SITL boot sessions),
per-flight demeaned (subtract each flight's own mean before pooling, so
per-flight offset differences don't contaminate the estimate) to isolate the
TRUE population-level relationship between raw Tz and GT vz in the 1-2m
altitude band:

```
n_flights=12, n_samples=873
pooled corr(Tz, vz) in 1-2m band = +0.226
pooled OLS slope                 = +0.172
```

**This is POSITIVE — same sign as the >2m band (+0.47 to +0.75 across all 12
flights, rock-solid).** There is no real sign inversion. The apparent flip
was a statistical artifact:

- The `VALIDATION_PROFILE=landing` maneuver is a **linear-ramp descent** —
  vz is close to a CONSTANT ~-0.25 m/s for the entire flight (by
  construction: `apps/record_cross_marker_validation.py`'s `build_profile()`,
  `PROFILE=="landing"` branch — straight linear altitude ramp, no
  oscillation). Restricting to a narrow 1m altitude slice leaves almost no
  genuine variance in the TRUE signal to correlate against:
  `std(vz_g) ≈ 0.04-0.05 m/s` in that band, only marginally larger than the
  raw Tz measurement's OWN noise floor (`std(raw_tz) ≈ 0.02-0.06`, same
  units after the shared depth-normalization). A Pearson correlation
  computed under those conditions is dominated by per-flight measurement
  noise, not signal — its SIGN is close to a coin flip from one flight to
  the next, confirmed empirically: individual-flight 1-2m corr across the
  12 flights ranges from -0.70 to +0.71, no stable value.
- The original "root cause" claim (point 7 above) was built from ONE batch
  of 5 flights launched back-to-back in immediate succession that happened
  to land 5/5 on the negative side of that noise distribution — under an
  independent-coin-flip model that's a ~3% event, unlucky but not
  extraordinary, especially with the benefit of hindsight/no correction for
  having looked at other batches. The follow-up 3-flight batch (session 2,
  part 2) landed mostly positive, and pooling everything resolves it: **no
  stable inversion exists.**
- **Ruled out, definitively this time:** any point-set-composition,
  timing/KF, or geometry/calibration mechanism causing a real sign flip
  around 2m altitude. None of those need to exist — the observation they
  were invented to explain wasn't real.

**What IS real and still needs an explanation:** the catastrophic negative
Hz R² near touchdown, documented consistently across many EARLIER sessions
using a real full-range validation flight (not this narrow-band linear-ramp
diagnostic). That symptom is not "a linear calibration guessing the wrong
sign in one regime" (no wrong sign exists) — it's more likely a **low
true-signal-variance artifact of R² itself**: R² is 1 minus a ratio of
residual variance to TRUE-signal variance, and near the ground the true Hz
signal's variance is small (as demonstrated by `std(vz_g)` above), so even
a modest, reasonably-signed prediction error or bias — anything at all, not
specifically a sign error — can drive R² deeply negative simply because the
denominator (true variance) is tiny. This reframes the whole investigation:
stop looking for a sign-flip mechanism to fix; instead check whether the
raw Tz measurement's BIAS/noise floor near the ground (not its sign) is the
thing degrading R², and whether R² is even the right metric to track in a
low-true-variance regime (an absolute error metric, e.g. RMSE against the
same true-signal-variance-normalized units, would not have this
pathology and might be more diagnostic going forward).

**Don't re-do:** don't re-derive a sign-flip mechanism from a single batch
again — this whole detour started from exactly that. Any future altitude-
banded correlation check on a near-constant-vz maneuver (like this "landing"
profile) needs either (a) pooling across several independent flights before
trusting the sign, or (b) a maneuver with genuine velocity variation within
the band being tested (e.g. add a small oscillatory z-component to the
descent, or use a full multisine/landing validation flight instead of the
narrow linear-ramp one) so the correlation has real signal to measure
against.

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
