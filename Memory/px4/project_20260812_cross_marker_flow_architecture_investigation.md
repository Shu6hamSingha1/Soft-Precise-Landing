---
name: project_20260812_cross_marker_flow_architecture_investigation
description: "Cross-marker hard-landing investigation. Implemented+live-validated fixes (getFPS() init; dt/frame-pairing staleness; moment-loom+MAD+origin-spread-gate for Tz; ring-style sampling w/ paired-opposite rejection, CROSS_RING_SAMPLING=1) -- all correct but NONE alone resolves the hard landing; flow/Tz point-scarcity was not the whole story. BIGGEST FINDING (2026-08-13): for 4/5 IC1-5 flights, detection goes to 96-100% 'miss' for the final 1.5-3m of descent because the MARKER EXITS THE CAMERA FRAME BOUNDARY due to lateral drift on off-center ICs, NOT extent/color-gate saturation -- root-caused to the FOV-CBF feeding cbf_corners a SINGLE bare center point (extent-blind), FIXED via a CLOSED-FORM radius (r=MARKER_EXTENT_PX/2, cbf_radius param threaded through controller.py + cbf_visibility.py -- NOT materialized 4-point circle, superseded same day per user request; proven numerically identical to the 4-point version) -- empirically gives 0.24-0.85s lead time before actual failure in IC2-5, no false triggers in IC1; live-tested IC2 detect rate 77.7%->95.6-100% across two reps (n=1 each, promising but not yet a validated sweep result). Fix is SOFT (informs existing graduated Phase-2-ramp/drift-off-pullback mechanisms, doesn't touch cbf2_filter's centroid-only hard Phase-1 bound) by design, distinct from ArUco's inherent hard 4-corner requirement; cbf_visibility.py is now FULLY DEDICATED to cross-marker (radius mandatory, no corner-array fallback) and cbf_visibility_aruco.py unconditionally serves ArUco (controller.py's import now routes by MARKER_TYPE, not the old CBF_PHASE2_FIX gate -- side effect: ArUco always runs the Phase-2 rewrite now, flagged, not yet flight-verified for ArUco specifically). Also: confirmed gyro-derotation structurally necessary, retracted a wrong claim about nested-textured-ArUco (real reason: ~2.5x lower correspondence noise, not board spread)."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-13T15:23:48.874Z
---

Follow-up to [[project_cross_marker_hz_regression_bisection_20260810]] and
[[reference_cross_marker_headless_flight_testing]] — that thread closed the
*calibration-derivation* Hz investigation; this one goes deeper into the
*architecture* of the flow computation itself, prompted by the first
closed-loop hard landing.

## 1. IMPLEMENTED: `getFPS()` init + sanity-clamp fix (`src/gz_subscriber.py`)

Two real, narrow bugs found and fixed (committed as source, smoke-tested
against both ArUco and cross-marker worlds, no regression):
- `self._fps` was never initialized in `Image_Node.__init__` — only ever
  assigned inside `image_callback`'s `if self._t0 != self._t1` branch.
  `getFPS()` itself had no guard (`return self._fps`) — calling it before
  that branch first fires raises `AttributeError`. Fixed: `self._fps = None`
  in `__init__`; `getFPS()`'s docstring now documents the `None` sentinel
  contract callers must handle.
- No floor on the interval before inverting (`1/(t1-t0)`) — a near-duplicate
  timestamp (ROS queue burst, sim-clock quantization) could spike `_fps`
  non-physically for one sample, un-smoothed. Fixed: only update `_fps` when
  `(t1-t0) > 1e-4`; on reject, keep the last good value.

## 2. THE REAL dt/frame-pairing ARCHITECTURE BUG (found, plan reviewed, NOT yet implemented)

**Root mechanism:** `CrossMarkerNode.run()` only passes `imgs[-1]` (newest
frame) to `process_frame()`, discarding `imgs[0]` (the true adjacent-
previous frame, always available from `gz_subscriber.Image_Node`'s rolling
2-frame deque). Internally, `_compute_hw` relies on `self._prev_gray`/
`self._prev_frame_t`, persisted across calls and only updated on a
**successful** solve. On a detection dropout (e.g. `centroid_mismatch` near
touchdown), `_compute_hw` isn't even called (the `det.ok=False` branch
returns early) — so both the timestamp AND the tracked point positions
freeze for the whole dropout streak. When detection recovers, `LK` is asked
to bridge the ENTIRE accumulated gap in one step (large real displacement,
degraded correspondence) instead of one native frame interval.

**Contrast with `img_data.py` (verified by reading its actual code, not
assumed):** its primary `h` computation re-decodes ArUco markers fresh in
BOTH `imgs[0]` and `imgs[1]` *within the same call*, and its `dt` comes from
`Y = (V_flow_norm[1]-V_flow_norm[0]) * self._fps`, where `self._fps` is
continuously maintained from the camera's own native frame interval
(`image_callback`'s `self._t0`/`self._t1`) — completely decoupled from how
fast or slow the consumer/decode loop runs. No persisted "prev" state is
needed for the primary path; even the optional 1-frame LK-carry fallback
(`_persist_extras`) explicitly anchors to `imgs[0]` (never `imgs[1]`) and is
self-limited to one frame.

**Reviewed fix plan (not yet applied — user wants to review the diff
first):** pass both `imgs[0]`/`imgs[1]` into `process_frame`, use
`dt=1/getFPS()` instead of `t - self._prev_frame_t`, and decouple the LK
point-ADVANCEMENT step (should run every frame, `imgs[0]→imgs[1]`,
regardless of `det.ok`) from the Jacobian SOLVE/validation (still gated on
`det.ok`, since that needs a fresh mask). `_fill_A`/pseudo-inverse math
itself stays untouched.

**Empirical evidence this staleness is real and matters:** in a captured
terminal-descent window, even "normal" (non-outlier) point-diag entries ran
`dt=0.064-0.128s` — 4-8x the nominal ~0.016s (62Hz) native interval — because
detection itself intermittently fails throughout the terminal window (not
just isolated spikes), forcing LK to bridge correspondingly larger real
gaps every time.

**IMPLEMENTED 2026-08-12** (`CrossMarkerNode.run()`, `process_frame`,
`_compute_hw`, `__init__` — all in `cross_marker_perception.py`). Validated:
3 real flights landed without crashing, `dt` now pinned to exactly 0.016s
(min=median=max=p95) across 800+ solves per flight, zero entries above
0.05s (previously routine during terminal descent); a targeted unit test
(24 steps mixing real frames + forced detection misses) confirmed the new
`det.ok=False` path (still-advance-LK-tracking with `mask=None`) degrades
and recovers gracefully with zero exceptions.

**⚠ BUT THIS DOES NOT FIX THE HARD LANDING — verified directly, don't
assume it does.** Ran a fresh flight WITH the fix live and re-did the same
GT-comparison check (`_compute_gt_flow_zreg`) that found the original
problem. Result: the flight STILL landed hard (`rel_vel=1.74 m/s`, not
soft/precise, `min_alt=1.26m` — cut short even higher than before) and the
same two problems are still present in the raw data:
- **Sign-flip: still happens, one instance actually WORSE than pre-fix**
  (perceived `+2.96` vs GT `-0.80` at t=39.044 -- roughly 10x larger error
  than any pre-fix flip magnitude). This proves dt-staleness was never the
  root cause of the LK correspondence failures themselves -- they can and
  do happen on a single clean native-rate step too, whenever the tracked
  points are poorly conditioned or genuinely mistrack.
- **Dynamic-range gap: still present.** Even non-flipped frames show
  perceived `h_z` around -0.5 to -1.0 while GT is already -0.6 to -0.85 and
  climbing in the same narrow window.

**Conclusion: the dt/frame-pairing fix is real, validated, worth keeping
(correct architecture regardless), but it was never the mechanism behind
the hard landing.** Don't report it as resolving the touchdown problem.
The two candidates that actually target the observed failure modes remain
open: moment-loom+MAD-rejection (§3 -- targets the sign-flip specifically,
does NOT fix the magnitude gap either, confirmed in that section's own
testing) and a depth-aware fallback/recalibration for the near-ground
dynamic-range ceiling (still untried).

## 3. Moment-loom + MAD outlier rejection for `Tz`: IMPLEMENTED, validated offline AND live -- real but partial improvement, NOT a fix

Ported concept from `img_data.py`'s ring-moment (`h_z = -0.5*d(ln M)/dt`,
`M`=mean squared distance of tracked points from their own centroid, in
V-frame — a closed-form scalar, no per-point Jacobian pseudo-inverse for
`Tz` specifically). Prototyped directly on logged `Point Diag Log` data
(3 real flights, extended point-diag logging added 2026-08-12 to
`cross_marker_perception.py`'s `_solve_jacobian`/`getLogData()` — now logs
`prev_n, curr_n, dt, sol` per solve, not just `prev_n`/`Tz`).

**Results, 3 real cases:**
- Flip 1 (t=38.468, n=14): pinv wrongly flipped (+0.109), moment-loom
  correctly stayed negative (-0.514), matching GT (-0.530).
- Flip 2 (t=39.492): pinv flipped (+0.121), moment-loom stayed correct sign
  (-0.111), still underestimating magnitude like everything else near
  touchdown (the separate, bigger dynamic-range problem — moment-loom
  doesn't fix that alone).
- Overshoot case (t=38.644): moment-loom WITHOUT outlier rejection was
  WORSE than pinv (-1.151 vs pinv's -0.511, GT=-0.567) — traced to 2 of 15
  points (13%) having genuinely mistracked LK correspondence (displacement
  15-40x every other point's) that the simple mean-based moment computation
  has no mechanism to downweight. Adding the SAME MAD-based rejection
  ArUco's ring-moment already uses (`median + 3.0*1.4826*MAD` on per-point
  flow magnitude) recovered a sane value: -0.617, between pinv and GT.
- In calm conditions elsewhere in both traces, pinv and moment-loom agree to
  2-3 decimal places — no regression.

**Gyro-derotation for the moment-loom path: tested, NOT needed.** Initial
theoretical worry (Wx/Wy's quadratic-in-(x,y) columns could distort the
point cloud's apparent spread) tested empirically at this dataset's highest
observed rotation rate (Wx=0.284 rad/s) — derotating only shifted the
moment-loom Tz estimate by +0.0040, negligible next to the actual problem
scale (0.5-1.5+). Mechanism: the position-dependent (spread-distorting) part
of Wx/Wy's flow contribution is small relative to its uniform-shift part
(which doesn't affect centroid-relative spread at all) at this marker's
achievable coordinate ranges (|x|,|y| mostly <1) and observed rotation
rates. Revisit only if a future flight shows meaningfully larger tilt rates.

**User directive: do NOT remove Tz's column from the joint pseudo-inverse
solve** (even though a direct SVD test showed doing so wouldn't change the
Tx/Ty/Wz conditioning at all, since Tz's and Wz's columns are exactly
orthogonal per point and contribute paired/equal singular values). Moment-
loom is IMPLEMENTED as an ADDITIONAL, separately-computed `Tz` that
OVERRIDES `sol[2]` in `_solve_jacobian`'s output — the joint solve keeps
producing all 4 unknowns exactly as before; `rel_resid` is computed BEFORE
the override so it stays a clean diagnostic of the original pinv fit.
Gated on `len(prev_n) >= CROSS_MOMENT_LOOM_MIN_PTS` (env, default 8) and
`n_valid_after_MAD >= max(4, min_pts//2)` — below that, holds the pinv Tz
unmodified (untested/unstable moment estimate at very low N).

**IMPLEMENTED 2026-08-12** (`_solve_jacobian` in `cross_marker_perception.py`).
Offline sanity check against the exact pre-implementation prototype numbers
matched exactly (-0.6172, both computed independently). Live-flight-tested
(2 flights, no crash, no exception, moment-loom values visibly appearing in
`Point Diag Log`'s `sol[2]`).

**⚠ Live-flight validation (2026-08-12): confirmed a real, partial
improvement — NOT a fix. Two distinct residual failure modes found by
re-running the exact same GT-comparison diagnostic on a fresh flight:**
- **Threshold-too-conservative miss** (t=38.644, this flight): `n_total=7`,
  below `CROSS_MOMENT_LOOM_MIN_PTS=8` — override never fired, fell back to
  the raw (wrong) pinv value `+0.248`. Had it fired, moment-loom would have
  given `-0.619`, correct sign, close to GT `-0.599`.
  **FIXED same day: default LOWERED 8→6.** Offline-verified this exact
  frame now correctly gives `-0.619` (matches the hand-computed value).
  Live-flight-tested (2 more flights; one hit unrelated SITL "descent
  stall" flakiness, not a code issue — same pattern seen repeatedly
  earlier this session; the other landed clean, no crash, 100% detection).
  In that clean flight, 29 solves had n=6-7 points — cases that would have
  silently fallen back to a possibly-wrong pinv value at the old threshold
  and now get the moment-loom correction applied. 6 is still 2 above
  `MIN_FLOW_POINTS_SOLVE=4` (some margin kept above the absolute pinv
  floor). NOT yet re-validated at scale for whether 6 introduces its own
  instability (a smaller N makes the mean itself noisier) — watch for this
  on future flights, don't assume 6 is definitively correctly tuned either.
- **CORRECTED same day (was wrongly diagnosed as "majority point corruption"
  — re-examined the actual per-point data, that explanation doesn't hold):**
  `t=39.444`, `n_total=8`, override fired, MAD kept 6/8 points, moment-loom
  computed `+1.430` (wrong sign; GT was `-0.80`). Direct per-point
  inspection shows all 8 points tracked CLEANLY and CONSISTENTLY (small
  displacements, no 15-40x anomaly like the earlier case) — not corrupted.
  **Real mechanism: all 8 points sit in a SINGLE quadrant, clustered far
  from the image origin** (`x∈[0.53,0.99], y∈[0.10,0.56]`, mean radius
  ≈0.89). `Tz`'s per-point contribution is `-Tz·(x,y)` — when every point
  sits at nearly the SAME `(x,y)` (because they're all clustered in one
  small region far from center), that contribution is nearly IDENTICAL for
  every point, which looks like a UNIFORM SHIFT of the whole cluster, not
  a spread change — and centroid-relative variance is insensitive to a
  uniform shift BY CONSTRUCTION (the same reason `Tx`/`Ty` don't affect it
  at all). So the real `Tz≈-0.85` signal (GT) got almost entirely absorbed
  into a centroid shift instead of showing up as a spread change, leaving
  `ln(M1/M0)` dominated by small residual non-uniform noise within the
  cluster (plausibly from the large simultaneous lateral motion this frame
  also shows, `Tx≈+1.99`) — not real Tz signal reaching the measurement at
  all. Gyro rotation checked and ruled out (`Wx≈0.09, Wy≈0.009`, steady
  across neighbors, well inside the range already found negligible).
  **This is a DISTINCT failure mode from the point-starvation/MAD-outlier
  story above: moment-loom needs points spread AROUND THE ORIGIN, not just
  spread among themselves relative to each other. A tightly-clustered-but-
  internally-consistent point set (exactly what MAD rejection is designed
  to preserve) can still starve moment-loom of real signal if the whole
  cluster sits far from center — no amount of outlier rejection fixes
  this, since nothing in that set is actually an outlier.**

Overall test result on this flight: still landed hard (not soft/precise);
3 of 10 samples below 2m altitude still showed a sign mismatch vs GT even
with moment-loom live (down from... not directly comparable to a pre-fix
baseline at the SAME flight, but the mechanism above explains why it isn't
zero).

**FIXED same day (2026-08-13, user-proposed): origin-spread gate.** Added
`origin_ratio = M0 / ||centroid||^2` (cluster's own internal spread vs. how
far it sits from the image origin) as a precondition for applying the
moment-loom override at all, `CROSS_MOMENT_LOOM_MIN_ORIGIN_RATIO=1.0`
(env). Quantified on the 4 documented real cases before implementing: the
KNOWN-FAILED case (t=39.444) measured ratio=0.028; all 3 KNOWN-WORKING
cases measured 5.3-8823 — a massive, unambiguous gap, threshold=1.0 sits
with huge margin on both sides. Offline-verified all 4 cases route
correctly (fails→held-pinv, all 3 working cases→still use moment-loom,
matching their previously-validated values exactly). Live-flight-tested:
no crash, landed; in that flight 4.2% of eligible solves correctly fell
below the gate (held pinv instead), median ratio a healthy 20.1 (min
observed 0.51, close to but below threshold — plausible, not obviously
mis-set).

**Important, deliberately NOT overclaimed:** falling back to pinv on a
gated-out frame does NOT mean that frame reads correctly — pinv can still
be wrong for its own, separate reasons (confirmed: the t=39.444 case's
held-pinv fallback is still `+1.430` vs GT `-0.80`, unchanged). The gate's
job is narrower and more honest than "fixing" that frame: it stops
moment-loom from being applied in a geometry where it structurally cannot
help (so it can't make an already-uncertain frame WORSE), it does not
manufacture a correct answer where the underlying raw data doesn't support
one. The dynamic-range/depth-aware-fallback gap (§1 of the original
handover doc) is still the real remaining lever for frames like this.

**Conclusion: moment-loom is real and worth keeping (helps when it has
enough points, isolated bad ones get rejected, AND the surviving points
have real radial diversity), but it is NOT a fix for the hard landing, and
neither is the dt/frame-pairing fix (§2).** Two DISTINCT failure modes
found, not one: (i) isolated mistracked points skewing the mean (what MAD
rejection targets, works when this is the actual problem) and (ii) a
clean, internally-consistent but off-center-clustered point set that
structurally starves moment-loom of real Tz signal regardless of how clean
the tracking is (no outlier-rejection variant fixes this — it's a
geometric limitation of the method itself, not a data-quality problem).
Both trace back to the SAME root structural issue this entire
investigation keeps landing on: too few, too poorly-distributed reliable
tracked points at extreme close range near touchdown. Neither fix (nor
moment-loom itself) ADDS information -- they only make better use of
whatever points already exist, and (ii) shows that "better use" has its
own geometric ceiling even when the data is clean. A real fix needs either
(a) more/better-DISTRIBUTED points at close range (the structural
corner-scarcity problem from the marker's thin-line geometry, see the
io-calibration skill's cross-marker Hz history), or (b) a fundamentally
different close-range signal source (a depth-aware fallback, still
untried) rather than continuing to refine how the existing sparse/noisy
point set is processed.

## 3b. Ring-style sampling (2026-08-13, user-proposed): IMPLEMENTED, offline+live tested, addresses (b) not (a)

§3's conclusion identified "too few, too poorly-distributed reliable tracked
points" as the root structural issue. This is a direct attempt at the
DISTRIBUTION half (not the count half): guarantee tracked points come from
multiple directions around the marker's own centroid, rather than letting
one strong local corner (or one lucky early GFT draw) dominate the pool —
i.e., proactively prevent the exact off-center-clustering geometry that
made §3(ii)'s origin-spread gate necessary reactively.

Adapted (not directly ported) from `img_data.py`'s ArUco fixed-ring-station
design, because the two markers' eligible sampling regions differ
fundamentally: ArUco samples arbitrary background/scene texture (fixed
absolute pixel ring positions work fine, texture is everywhere); the
cross-marker's eligible region is the thin `dilated_mask` band around the
drawn lines only (the "structural corner scarcity" finding, §3's closing
paragraph) — fixed absolute ring positions would mostly land on nothing.

**Design** (`src/cross_marker_perception.py`, `CROSS_RING_SAMPLING=1` env,
off by default): partition the SAME eligible boundary-margin mask into
`CROSS_RING_N_BANDS` (default 2) radius bands x `CROSS_RING_N_SECTORS`
(default 8, must stay even) angle sectors, normalized by the marker's OWN
current centroid/half-extent (same normalization `_sample_flow_points`
already used for its center-exclusion disk). Each (band, sector) cell gets
its own masked `goodFeaturesToTrack` call, up to `CROSS_RING_PTS_PER_CELL`
(default 2) candidates — draws from whatever eligible texture exists in
that direction specifically, instead of one unconstrained call over the
whole region that a single strong local corner can dominate. Falls back to
the pre-existing unconstrained sampler if literally no cell yields a point
(empty mask edge case).

**Paired-opposite rejection** (user's explicit second requirement): a
parallel `self._prev_flow_cell_id` array (band*N_SECTORS+sector per point)
persists alongside `self._prev_flow_pts`. In `_compute_hw`, right after the
LK step's `status` array comes back, any point whose LK status is False
also invalidates its diametric partner's cell (`sector +
RING_N_SECTORS//2`, mirrors ArUco's `PLASMC_RING_PAIRED`/`_ring_opp_idx`)
for THAT frame's `keep` mask — rationale: a ring pair exists specifically
to balance that direction's contribution to the centroid/moment; letting
the surviving partner alone through re-introduces the same one-sided bias
pairing was meant to prevent, silently (not caught by the origin-spread
gate, which only looks at the surviving set's own geometry, not at what
asymmetry a one-sided drop just introduced).

**Verified before/along with implementation** (not asserted):
- Synthetic mask unit test: 2 bands x 8 sectors → 16 unique cells
  populated, exactly the expected structure.
- Synthetic status-array test: killing cell 2's point correctly also drops
  cell 6 (its diametric opposite, `2+8//2=6`) — confirmed by hand
  computation, not just code inspection.
- `python3 -c "import ast; ast.parse(...)"` synta check + a full HEADLESS
  live flight (`WORLD=cross_marker MARKER_TYPE=cross CROSS_RING_SAMPLING=1`,
  2026-08-13): 775 `process_frame()` calls over 23.7s, 100% detect rate,
  zero exceptions/tracebacks in the run log. Landing outcome unchanged
  (xy_err=0.420m, rel_vel=1.711 m/s — still a hard landing, as expected:
  this addresses point DISTRIBUTION, not the separate dynamic-range/
  close-range point-COUNT gap that §3's conclusion also named as needed).

**CORRECTED same day (2026-08-13, user-caught): ring center was WRONG in the
first cut, fixed before it was trusted.** The initial implementation
centered the radius/angle partitioning on the MASK's own centroid
(mirroring `_sample_flow_points_unconstrained`'s center-exclusion-disk
normalization). User caught this by re-deriving the actual mechanism from
first principles: img_data.py's ring design (`_ring_pts0_V`, centered on
the V-frame nadir/image center; `_ring_opp_idx`, pairing angle i with
i+N/2 at the SAME radius) relies on translational flow CANCELING between a
diametrically-opposite pair measured from the IMAGE CENTER (equal
magnitude, opposite sign) while divergence/Tz-induced radial flow ADDS
(same sign at both) — verified this is really what the code does by
re-reading `img_data.py:329-341` directly (comment at line 330: "the ring
ALWAYS tracks the NADIR patch ... = tilt-invariant flow"). Centering on the
MARKER centroid instead breaks this cancellation whenever the marker isn't
at the image center (the normal case), which would have made the
paired-opposite-rejection logic (this section's core requirement) silently
meaningless — rejecting a "partner" cell with no actual
translation-cancellation relationship to the failed point. Re-centered on
`self.center` (image center, resolution-adaptive `r_max = min(h,w)/2`,
matching img_data.py's `_Rmax` convention) before any live test was
trusted. Re-verified after the fix: synthetic test with an intentionally
OFF-CENTER mask confirms (a) points are still found (8 pts, 4 cells, after
moving the mask far enough from the frame edge that
FLOW_BOUNDARY_MARGIN_PX didn't wipe it — an unrelated test-setup artifact
caught along the way, not a code bug) and (b) every returned point's sector
assignment matches independently-recomputed image-center-relative theta.
Live-flight-retested with the corrected center (2nd attempt succeeded; 1st
hit the known unrelated "descent stall" SITL flakiness — 694/1290
process_frame() calls respectively, 100% detect, zero code exceptions in
either): 694 frames, clean run, no crash.

**REFINED same day (2026-08-13, user's moving-target concern): center-
coverage guard + adaptive radius, min radius 20px.** Even correctly
image-centered, the original ring used a FIXED frame-relative radius
(`r_max = min(h,w)/2`) — on a frame where the marker has moved off-center
(the whole point of the upcoming moving-target/rover phase) or is small,
that would draw "opposite pair" points from whatever texture exists at
that fixed radius/angle, which may not be ON the marker/target at all,
silently breaking the translation-cancellation the pairing math assumes.
Two guards added:
- **(a) center-coverage guard:** ring sampling only proceeds if the marker
  mask has eligible pixels (beyond the min radius) in >=
  `CROSS_RING_MIN_COVERED_SECTORS` (default `RING_N_SECTORS//2` = 4 of 8)
  angular sectors around the image center — i.e. the marker actually
  surrounds the center in enough directions. Fails → falls back to the
  unconstrained sampler for that frame (same code path as the empty-mask
  case), not a forced/degenerate ring.
- **(b) adaptive radius:** ring band radii are capped at
  `r_px[eligible].max()` (the marker's own actual extent from center THAT
  frame), not the fixed frame bound — bands never reach past real marker
  pixels regardless of how far/close the target is.
- **min radius `CROSS_RING_CENTER_MIN_R_PX=20`** (px): excludes the
  near-center pixels from both the guard check and the sampling bands —
  for the cross-marker specifically the exact image center (when the
  marker IS well-centered) is the low-texture gap between the arms, plus
  small-radius angles are numerically noisy in general.

Verified with synthetic masks before trusting it live: a marker centered
on/surrounding the image center still ring-samples normally (32 pts, min
observed radius 20.1px, i.e. respects the 20px floor); a small marker
mask confined to one corner (not surrounding center) correctly triggers
the guard and falls back to the unconstrained sampler (`cell_ids is
None`). Live-flight-retested with the refined guard: clean run, no
exceptions.

**NOT yet done / open follow-up:** a direct before/after comparison of
`origin_ratio` (§3's metric) and n_kept-per-frame between
`CROSS_RING_SAMPLING=0` and `=1` on matched flights — the live tests above
only confirm the mechanism runs correctly end-to-end with the corrected
(image-center, guarded, adaptive-radius) geometry, not yet that it
measurably improves the distribution metric in practice. Do that
comparison (offline, from two flights' Point/Radial Diag Logs) before
considering this "validated" in the same sense §3's origin-spread gate
was; log the result here when done. Also untested so far: an actual
moving-target (rover world) flight, which is the scenario this guard was
specifically added for — the guard's logic has only been synthetic- and
stationary-marker-tested to date.

## 3c. "Alpha collapse" near touchdown: NOT an alpha bug -- marker exits the camera FRAME BOUNDARY due to lateral drift (2026-08-13)

Follow-up to an IC1-5 (n=1, `CROSS_RING_SAMPLING=1`) smoke-test batch
(`test_data/ICValidation/20260813-111935/`, all 5 landed, none precise/soft).
A GT comparison of perception s/h/alpha vs independently-computed,
Z_REG=0.1-regularized, V-framed GT (sync-verified: median gap 0-4ms, p95
4-8ms within GT's logged window) found alpha R^2 catastrophically negative
near touchdown (-30k to -119k in the 1-0.5m bin) in IC2-IC5. Root-caused
below -- it is NOT a defect in the alpha formula (`_unweighted_principal_
angle`/`_disambiguate_angle`), NOT a pi-wrap issue, and not specific to
alpha at all.

**Finding 1 -- total detection dropout, not an alpha-specific issue.**
`Img_Data.npy`'s `alpha(t)` is EXACTLY constant (a single unique value,
std=0.0) for the entire sub-2m window in IC2, IC3, IC4, IC5 (IC1's
apparent "collapse" was a red herring -- just GT-coverage sparsity, n=5
samples; IC1 in fact has 99.6% detect for the WHOLE flight and never
exhibits this). `s_V` (centroid) is frozen too (1-3 unique values in the
same window). Cross-referencing `Detection Status`: for these 4 flights,
detection is `'miss'` for 96-100% of frames from the point alpha stops
updating through touchdown (70-105 consecutive missed frames, i.e. the
LAST several meters of every one of these descents). This is `det.ok`
going false and STAYING false -- the whole detection pipeline (center, s,
alpha, and by extension h/w since `_compute_hw` needs a fresh mask) goes
stale simultaneously via the existing hold-last-good fallback, not an
alpha-only symptom. The "catastrophic" R^2 is just: a flat frozen value
vs. GT yaw that kept evolving underneath it for 2-3 seconds -- a real,
growing, structural bias, not sensor noise.

**Finding 2 -- root cause is the marker exiting the FRAME BOUNDARY, not
extent/color-gate saturation.** Initial hypothesis (marker fills frame ->
color-gate/Hough saturates) was directly checked against `MARKER_EXTENT_PX`
and `Center Px` and DISPROVEN: extent at the point of permanent failure
was only 189-230px (frame is 480w x 640h in the rotated working
convention -- see CLAUDE.md's `img_data.py`-derived resolution note),
well under half the frame width, and extent had reached much larger
values earlier in-flight without failing. What actually correlates:
**the marker's CENTER pixel position, tracked over its last ~10 OK
frames before permanent loss, is moving steadily toward one frame edge**
(IC2: cx 141->71, frame width 480, extent~228 -> left edge = cx-extent/2
already crosses 0 by the last OK frame; IC3: cy 419->599, frame
height 640, extent~220 -> bottom edge crosses 640; IC4: cx 105->57,
extent~199 -> left edge crosses 0; IC5: cy 287->560, extent~189-221 ->
bottom edge crosses 640). In all 4 failing flights, `center +/-
extent/2` crosses the true frame boundary in the direction of sustained
lateral drift right at/before the last successful detection -- the
marker physically leaves the camera's field of view, at a MODERATE
altitude (~3m) and a MODEST marker size, well before touchdown. IC1 (the
only flight with 99.6% whole-flight detect) has a centered target
(ENU 0,0,5 -- no lateral correction needed) vs. IC2-5's off-center
targets ((2,2),(-2,2),(2,2,7),(2,2,3)) that all require sustained lateral
correction during descent; IC1 does drift toward its own frame edge too
late in the flight (extent 500px, center approaching a boundary-crossing
position) but only right at/after touchdown, when it no longer matters.

**Open observation, not yet resolved:** the existing FOV-cone visibility
CBF (`Control_Data.npy`'s `theta_cone(t)`/`theta_current(t)`/`rho_fov(t)`/
`d_min_fov(t)`) does NOT trip at the moment of marker loss in IC2 (spot-
checked): `theta_current` stays comfortably below `theta_cone`
(0.019 vs 0.133 rad at the last OK frame before permanent miss) right
through the failure. Either this CBF's geometric visibility model doesn't
correspond to what the Hough/color-gate detector actually needs to keep
finding >=2 line clusters, or it's measuring the wrong quantity for this
failure mode. Not dug into further this session -- worth a dedicated
follow-up given it's the CBF specifically meant to prevent this class of
failure.

**Practical implication:** for 4/5 of these landings, the controller was
flying the final ~1.5-3m on frozen/stale perception (dead-reckoning),
independent of anything the ring-sampling (S3b) or moment-loom (S3) work
touches -- those only matter once detection succeeds at all. This is
likely a bigger, more direct contributor to why these particular landings
were hard than the flow/Tz-focused mechanisms investigated earlier in this
file. Root cause is lateral-tracking/FOV-margin, not a flow-computation or
alpha-computation defect. Ties into the project's known "lateral wall"
history (see MEMORY.md's standing correction on lateral-wall root causes)
but is a DISTINCT, newly-quantified mechanism: the marker leaving frame
entirely, not degraded-but-present flow signal.

**Not yet done:** an n>=5 sweep to confirm this reproduces reliably (this
session's batch was n=1/IC, per the project's own sweep-methodology rule);
a fix (tighter FOV-margin CBF gain, or investigating why theta_cone didn't
trip); and determining whether this same mechanism, not the flow/Tz
degradation this file otherwise focuses on, is the PRIMARY hard-landing
cause for off-center ICs specifically.

## 3d. FOV-CBF extent-blindness fix: circle-around-center (2026-08-13, user-proposed), IMPLEMENTED + live-tested

Follow-up to 3c's root-cause (marker exits the true frame boundary while
the CBF's margin check, fed only the bare center pixel, stays "healthy").
Traced WHY the CBF didn't trip: `controller.py`'s cross-marker branch
(pre-fix) set `cbf_corners = [[get_center_px()]]` -- a SINGLE point. This
was an explicit 2026-08-01 design decision ("the CBF's ONLY hard guarantee
is the marker center staying in FoV... alpha/h,w already degrade
gracefully... don't need CBF protection"). Traced the actual mechanics in
`cbf_visibility.py::cbf2_filter`: Phase-1 (successful decode) is
CENTROID-ONLY by design for BOTH marker types (`delta_eff=0`,
"deliberately allow the marker to grow and overflow" -- comment predates
this investigation and is unrelated to the cross/ArUco distinction). So a
symmetric multi-point circle around center does NOT change Phase-1's hard
bound (mean is unchanged). What a single point DOES silently break: it
makes `delta2 = 0.5*(corners.max(0)-corners.min(0))` (the per-axis spread
cbf2_filter tracks every frame) trivially [0,0] -- which starves TWO
EXISTING, already-graduated/soft mechanisms of any real extent signal:
(a) Phase-2's decode-fail fallback, which RAMPS `delta_eff =
delta_prev*phase2_alpha` over `CBF_PHASE2_RAMP_FRAMES` (not an instant
cutoff) -- was silently inert for cross-marker (delta_prev always zero);
(b) controller.py's own drift-off pullback (`p_10_eff *= (1-frac)` on the
breaching axis only, already fractional/soft, line ~2731) -- driven by
`d_min_fov`, itself computed from `cbf_corners`' per-axis extremes, also
extent-blind with one point.

**User's key distinction, honored in the implementation:** ArUco's 4 real
corners are a HARD requirement (every one must stay resolvable or decode
fails outright) -- that's why ArUco's `d_min_fov`/drift-off naturally
reflects true marker geometry, no special-casing needed. The cross-marker
only needs its center intersection point (alpha/h/w already degrade
gracefully via hold-last/zero-output), so its extent-awareness should be
SOFT, informing the two already-graduated mechanisms above, NOT a new hard
"circle must stay in frame" cutoff. Since symmetric circle points leave
Phase-1's hard QP bound untouched (proven above, not just intended), this
distinction falls out of the existing architecture for free -- no new
hard/soft branching logic was needed, just feeding the right points in.

**Implementation:**
- `CrossMarkerPerception.get_marker_radius_px()` (new): `0.5 *
  max(last_bbox_w, last_bbox_h)` -- i.e. `MARKER_EXTENT_PX/2` -- gated on
  the SAME `_center_fresh` contract as `get_center_px()` (both null
  together on a genuine miss, never a stale bbox paired with a fresh
  center). Passthrough added on `CrossMarkerNode`.
- `controller.py`'s cross-marker CBF branch (~line 2511): `cbf_corners`
  is now 4 points, `center +/- (r,0)` and `center +/- (0,r)`, `r =
  get_marker_radius_px()`. Falls back to the bare center point if the
  radius is unavailable on an otherwise-fresh frame (shouldn't happen
  given the shared freshness gate, but degrades rather than drops the
  frame).

**Radius choice, empirically grounded (not assumed):** tested `r =
MARKER_EXTENT_PX/2` (already-computed, no new tuning constant) against
the actual IC1-5 failure data from 3c by recomputing what `d_min_fov`
would have been with a circle of that radius vs. the original bare-point
margin:
- IC3: circle margin goes negative at t=36.82, actual detection failure
  at t=37.67 -- **0.85s lead time**.
- IC5: negative at t=36.92, failure at t=37.40 -- **0.48s lead time**.
- IC4: negative at t=41.68, failure at t=41.92 -- **0.24s lead time**
  (tightest case -- a fast 2-frame extent jump 172->204->222px).
- IC2: already negative at the start of the available trace (can't lower-
  bound the lead time from this data, but it's at least as long as the
  window).
- IC1 (the one flight that never fails): only 15/673 ok-frames show a
  negative circle margin, all clustered right at touchdown (extent~500px)
  -- no spurious mid-flight triggers.

**Verified before/after implementing:**
- `get_marker_radius_px()` unit-tested standalone: correct value from a
  synthetic bbox, `None` when not fresh, `None` when no bbox recorded yet.
- `python3 -c "import ast; ast.parse(...)"` on both edited files.
- Live HEADLESS flight, default (centered) IC: clean run (2nd attempt;
  1st hit the known unrelated SITL "descent stall" flakiness, zero code
  exceptions in either). `d_min_fov` now goes <=0 on 96/573 frames of that
  flight (vs. essentially never, pre-fix, until the very final frames) --
  confirms the circle margin is actively engaging through a meaningful
  portion of the descent, not just a no-op.
- Live HEADLESS flight, IC2 (`INITIAL_DRONE_ENU=2.0,2.0,5.0` -- the exact
  off-center IC that had 77.7% whole-flight detect / sustained miss
  through the final ~1.5m in the 3c investigation): **100% detect
  (519/519) for the entire flight** with this fix. Single rep (n=1) --
  a strong, directionally-correct signal on the exact scenario this fix
  targets, but NOT yet a validated result (needs n>=5 per the project's
  own sweep-methodology rule before treating as confirmed). `min_alt`
  reported unusually high (3.94m) for this rep -- flagged but not
  chased down this session, worth checking before drawing conclusions
  about landing QUALITY (xy_err/rel_vel) from this same rep, as opposed
  to the detect-rate finding, which is unambiguous.

**Not yet done:** n>=5 IC1-5 sweep to confirm the detect-rate improvement
reproduces and to check whether it actually improves SoftPrecise
landing outcomes (fixing detection doesn't automatically fix the
downstream control response to previously-missing data -- a real
possibility, not assumed away); investigate the `min_alt=3.94m` anomaly
on the IC2 test rep before trusting its xy_err/rel_vel numbers.

## 3e. 3d SUPERSEDED same day: closed-form circle equation, not 4 materialized points (2026-08-13, user correction)

User: "Instead of building a circle of 4 points... use the mathematically
equation of the circle to find d_min_fov," plus: `cbf_visibility_aruco.py`
now exists as ArUco's own isolated copy (a SEPARATE, unrelated 2026-08-13
change -- a Phase-2 signed-projection rewrite gated behind
`CBF_PHASE2_FIX=1`, tested against ArUco only, not touched by this
section), freeing `cbf_visibility.py` itself to be edited for the
cross-marker's needs without the old "must stay bit-identical for every
caller" constraint 3d operated under.

**Change:** reverted `controller.py`'s cross-marker `cbf_corners` to a
single point again (matching pre-3d), and added a separate scalar
`cbf_radius` (= `get_marker_radius_px()`, 0.0 when unavailable/ArUco)
carried alongside it. Applied analytically wherever a per-axis margin was
computed:
- `d_min_fov` (controller.py): `rho_fov - (|center_offset| + cbf_radius)`
  per axis, and the overflow/drift-off breach thresholds
  (`bx_neg`/`bx_pos`/`by_neg`/`by_pos`) similarly radius-adjusted. This is
  EXACT, not an approximation -- for a circle of radius r centered at
  (u0,v0) tested against an axis-aligned box, the closest-to-breaching
  point on the circle is always exactly u0+/-r (same for v), so this
  closed form gives the identical worst-case margin the 4-point version
  computed, proven by direct comparison (below), not just argued.
- `cbf2_filter` (`cbf_visibility.py`, NOW free to edit for cross-marker
  per the user's note above): added an optional `radius=0.0` param (px).
  `delta2` (the per-axis half-extent Phase-2's ramped fallback uses) is
  now `radius/foc` (px -> tangent units via focal length) when
  `radius` is nonzero, else falls back to the original
  `0.5*(ct.max(0)-ct.min(0))` corner-array-spread calc -- so every
  existing ArUco caller (which never passes `radius`, default 0.0) is
  bit-identical to before this parameter existed.

**Verified:**
- Direct numerical comparison: `cbf2_filter` called once with a single
  point + `radius=100`, once with the OLD 4-materialized-point circle
  (same center, same radius) + `radius=0` -- both produced IDENTICAL
  `state["delta_prev"]` (`[0.370, 0.370]`), confirming the closed form is
  exact, not an approximation with different edge-case behavior.
  `radius=0` with real 4-corner-style input also reproduces the same
  value (ArUco-style backward-compat check).
- `python3 -c "import ast; ast.parse(...)"` on all three edited files.
- Live HEADLESS flight, IC2 (the same previously-failing off-center case
  3d tested): clean run, no exceptions. Whole-flight detect 95.6%
  (476/498) -- consistent with 3d's 100% (519/519) given both
  implementations are proven mathematically identical, so the small
  difference is ordinary rep-to-rep noise (n=1 each), not an
  implementation regression. This rep's `min_alt=0.49m` (reached real
  touchdown) resolves 3d's flagged `min_alt=3.94m` anomaly as unremarkable
  run-to-run variance in THAT specific rep, not a fix-induced issue.

**Still not yet done** (same open items as 3d, now against the closed-form
version specifically): n>=5 IC1-5 sweep to confirm the detect-rate
improvement reproduces and check downstream SoftPrecise landing-outcome
impact, not just detection rate.

## 3f. 3e's `radius=0.0` OPTIONAL-with-fallback param REMOVED: cbf_visibility.py fully dedicated to cross-marker, ArUco gets its own file unconditionally (2026-08-13, user correction)

User: "With cross-marker, we don't need radius=0.0 param approach." Asked to
confirm the routing implication before touching it (`cbf_visibility.py` was,
by DEFAULT, still the file BOTH marker types imported -- `CBF_PHASE2_FIX=1`
only ever switched ArUco to `cbf_visibility_aruco.py`, an unrelated,
already-in-place 2026-08-13 change (Phase-2 signed-projection rewrite under
test, not made by me) whose own docstring already stated "`cbf_visibility.py`
is the module the CROSS-MARKER pipeline runs" as an aspiration that hadn't
actually been wired up yet).

User's clarification, the real conceptual distinction (not just a style
preference): **ArUco's real 4 corners naturally encode marker size at every
altitude via their own spread** (near-coincident/tiny spread at high
altitude, splayed out close-up) -- `delta2` there is correctly DERIVED from
the corner array itself; there is no meaningful "radius" input separate from
that. **The cross-marker's single tracked point has no implicit size
representation at all** -- its radius (`MARKER_EXTENT_PX/2`, from the
color-gated mask bbox) is a genuinely separate, always-needed measurement,
not an optional add-on with an "off" (0.0) state.

**Change:**
- `cbf_visibility.py` (`cbf2_filter`): `radius` is now a REQUIRED
  parameter (no default, no corner-array-spread fallback branch);
  `delta2 = radius/foc` unconditionally. Module + function docstrings
  rewritten to state this file is now CROSS-MARKER-DEDICATED.
- `cbf_visibility_aruco.py`: added an accepted-but-UNUSED `radius=0.0` param
  purely for shared-call-site compatibility (controller.py's single
  `cbf2_filter(...)` call site is generic over both marker types); its own
  `delta2` derivation (real corner-array spread) is untouched. Module
  docstring updated to state it's now ArUco's unconditional default, not a
  `CBF_PHASE2_FIX`-gated alternative.
- `controller.py`: import switch changed from `CBF_PHASE2_FIX`-gated to
  `MARKER_TYPE`-gated -- `MARKER_TYPE=="cross"` always imports
  `cbf_visibility.py`, else always `cbf_visibility_aruco.py`.
  **Side effect, flagged explicitly (not hidden):** `CBF_PHASE2_FIX` no
  longer selects between two cbf2_filter implementations for ArUco (that
  was its whole prior purpose) -- ArUco now ALWAYS runs
  `cbf_visibility_aruco.py`'s Phase-2 rewrite unconditionally, since there
  is no longer an ArUco-compatible `cbf_visibility.py` to fall back to.
  If the original (pre-rewrite) Phase-2 behavior still needs to be A/B-
  tested for ArUco, that toggle would need to move INSIDE
  `cbf_visibility_aruco.py` as an in-module branch -- not implemented,
  out of this session's scope, flagged in that file's own docstring too.

**Verified:**
- Direct numerical comparison (same methodology as 3e): cross-marker's
  single-point+radius=100 call and ArUco's real-4-corner call (radius
  ignored) both produce IDENTICAL `delta2` = `[0.370, 0.370]` for
  geometrically equivalent inputs -- confirms the two files agree exactly
  where their inputs actually correspond, despite the split.
- `python3 -c "import ast; ast.parse(...)"` on all three edited files.
- Direct import test: `MARKER_TYPE=aruco` resolves `controller.cbf2_filter`
  to `cbf_visibility_aruco` (module attribute check, not just no-crash);
  `MARKER_TYPE=cross` already covered by the live flight test below.
- Live HEADLESS flight, IC2 (cross-marker, mandatory-radius path): clean
  run, no exceptions (ArUco's live path not re-flown this round -- only
  import-resolution-verified, not flight-tested, since no ArUco-specific
  behavior changed beyond the file-selection switch itself).

**Not yet done:** an actual ArUco SITL flight to confirm the routing switch
doesn't regress the live ArUco pipeline (only verified via static import
resolution + the numerical delta2 cross-check so far); the CBF_PHASE2_FIX
in-module-toggle follow-up noted above, if still wanted; the still-open n>=5
IC1-5 cross-marker sweep from 3d/3e.

## 3g. UNRELATED (pre-existing, not CBF/cross-marker-caused) NaN self-latch bug: found + root-caused + FIXED (2026-08-13)

Found while checking 3f's Phase-2-defect question on a real flight log
(`test_data/Landing_Test/Thu Aug 13 15-34-16 2026`): `theta_cone(t)`,
`kappa(t)`, `a_u(t)`, `w_u(t)`, `B_T(t)` all go NaN and STAY NaN for the
rest of that flight, starting at t=37.040 -- while `Detection Status` was
still `'ok'` (Phase 1, not a decode-fail/Phase-2 event; this was NOT the
Phase-2 defect being investigated). Confirmed present in 3/6 of that day's
test flights, absent in a flight from before any of the day's CBF/radius
work -- correlation only, not proof of causation at the time.

**Root-caused precisely, step by step against the logs (not guessed):**
1. `G`, `p`, `S`, `sigma`, `dp` all checked and stayed perfectly finite
   through the transition -- ruled out the funnel/barrier-transform math
   and the CBF's own margin computation entirely.
2. `dh_d(t)` (SMC's desired-flow RATE, `controller.py`'s
   `_dh_d_deque.append((_hd_src[-1]-_hd_src[-2])/self._dt[-1])`,
   ~line 1918 pre-fix) showed `[50, 50, nan]` for EXACTLY one frame at
   t=37.040 -- x/y pinned at the `DH_D_MAX=50` clip (evidence of a genuine
   near-inf blowup), z a literal unclippable `nan` (`np.clip(nan,...)`
   stays nan). `_hd_src` (`h_d`/`h_d_noS`) itself was checked and was
   smooth/finite through the same window -- the corruption is the
   DIVISION, not the input data.
3. Traced the divisor: `self._last_loop_dt` (`controller.py` `run()`,
   `_now_loop - self._last_loop_t` from `time.perf_counter()`) had **NO
   minimum-dt floor at all** -- unlike `gz_subscriber.py`'s `getFPS()`,
   fixed EARLIER THIS SAME SESSION for the textbook-identical class of bug
   (a near-duplicate-timestamp read producing a momentary non-physical
   spike; see that fix's own comment). A `perf_counter()` read landing
   within the OS clock's resolution of the previous one -> `dt~=0` ->
   `nonzero/~0` -> inf (clipped to 50 on x/y) or, when the numerator also
   happened to be ~0 on z, literal `0/0=nan`.
4. `dh_d[-1]` feeds `c` (SMC known-dynamics term) directly -> `c` NaN ->
   `vector`/`theta_ctrl` AND `a_v` both NaN same frame (matches `a_u`'s
   NaN onset exactly) -> `theta_cone` (via `I_a`, inside `cbf2_filter`)
   NaN same frame -> `kappa`'s RK5 update ingests the NaN `theta_ctrl` ->
   `self._kappa[-1]` NaN from the NEXT step onward, PERMANENTLY --
   integrating an ODE forward from a NaN state can never recover. This is
   the self-latching mechanism; structurally analogous to (but a
   completely different code location from) the 3 Phase-2 defects
   `cbf_visibility_aruco.py` was built to fix -- unrelated to that
   investigation, a coincidental discovery while checking it.

**Confirmed NOT caused by, and NOT related to, any of this file's CBF/
radius/ring-sampling work** -- fires with detection `'ok'` (Phase 1),
entirely inside the SMC's own `h_d`-rate finite-difference, several steps
upstream of anything touched in sections 1-3f.

**Checked MATLAB and Hardware for the same bug (user-requested):**
- **MATLAB** (`visualControl_IBVS_adaptive.m`): does NOT have this bug --
  `dt` there is a FIXED simulation timestep constant, never derived from
  wall-clock reads, so the near-duplicate-`perf_counter()`-read failure
  mode is structurally impossible in MATLAB.
- **Hardware** (`Hardware/scripts/controller.py`): HAS THE IDENTICAL BUG,
  same unguarded pattern at the same relative lines (`_last_loop_dt`
  computation ~line 1195-1198; the vulnerable `_dh_d_deque.append(...)`
  division ~line 1825) -- confirms `PX4_Gazebo/src/controller.py` inherited
  it via the documented 2026-08-04 port from Hardware. This is a REAL
  pre-existing exposure on the actual hardware-flight pipeline, not
  SITL-only. NOT YET FIXED THERE -- only PX4_Gazebo/src/controller.py was
  edited this session (backup-discipline / explicit-confirmation norm for
  touching the hardware-flight codebase; ask before porting).

**FIXED (PX4_Gazebo/src/controller.py only, this session):**
- `run()`: `self._last_loop_dt` now only updates when `_now_loop -
  self._last_loop_t > 1e-4` (same floor value/convention as
  `gz_subscriber.py`'s fix); on reject, holds the last-good dt rather than
  accepting a near-zero read.
- `_dh_d_deque.append(...)`: independent second guard -- computes the
  raw ratio only if `self._dt[-1] > 1e-6` (matching the CV-KF sibling
  branch's existing threshold), AND explicitly checks
  `np.all(np.isfinite(...))` before appending; either failure holds the
  deque's last entry instead of pushing a non-finite value in. Belt-and-
  suspenders: the dt floor should prevent this from ever firing, but the
  finite-check catches it independently even if some other divisor/path
  produces non-finite output in the future.

**Verified:** unit-tested both guard branches directly (dt=0 -> holds
last; dt<=1e-6 -> holds last; normal case unaffected) before trusting live.
Two live HEADLESS flights post-fix (IC2 and IC3-equivalent off-center
ICs, the same class of flight that showed the bug pre-fix): zero NaN in
`dh_d(t)`, `kappa(t)`, `a_u(t)`, `w_u(t)`, `B_T(t)`, `theta_cone(t)` in
either. Given the bug was intermittent even pre-fix (didn't fire in every
flight), 2 clean flights is reassuring but not exhaustive proof -- no
contradicting evidence found, and the fix is verified correct at the
mechanism level (unit tests), not just by absence-of-symptom.

**Not yet done:** port the same fix to `Hardware/scripts/controller.py` if
wanted (real hardware flights are exposed to this same latent bug);
broader n>=5 confirmation that the intermittent trigger condition never
recurs post-fix.

## 4. Gyro-derotation: CONFIRMED structurally necessary for the cross-marker (empirically re-verified)

Direct SVD test on 5 real logged point sets: the FULL 6-column Jacobian
(no gyro subtraction) has condition number 9.3-103.2 across real frames,
even at the WIDEST achieved point spread (n=45, x reaching 1.10) only
getting to 9.3 — vs. the gyro-derotated reduced 4-column system's
consistent 1.7-4.6. `Wy`'s column `[1+x²,xy]` genuinely stays close enough
to `Tx`'s constant `[1,0]` at every point spread this marker achieves in
practice for the two to be numerically hard to distinguish from vision
alone — this is why the 2026-08-08 gyro-derotation fix exists, and it
should NOT be removed or considered redundant.

## 5. ⚠ RETRACTED: "ArUco avoids this via multi-marker spatial spread" — directly disproven, don't re-cite

Original claim (made in this same investigation, before verification):
ArUco doesn't need gyro-derotation because a nested/multi-marker board
gives it corners spread far enough across the frame to restore rank on the
Wx/Wy-vs-Tx/Ty columns geometrically.

**This was WRONG on two counts, both checked against real code/data.**
User clarification (load-bearing for future sessions): the intended
comparison target is specifically the **nested TEXTURED ArUco marker** —
the live `aruco.sdf` world's single `arucotag` model (small tag ID nested
inside a big tag ID, SAME central location, textured background baked in,
one physical plate) — NOT a spread-out multi-marker board, and not any
other/generic ArUco configuration. All comparisons in this memory use THIS
specific marker.
- Don't cite "ArUco board" or multi-marker-spread framing for this
  comparison — the nested textured ArUco marker is one physical tag, not
  several spread across the frame.
- Even setting that aside, ArUco's OWN actually-achieved corner spread
  (measured directly from `Virtual Feature Pts` in real flight data: median
  radius 0.052, max 0.55) is SMALLER than the cross-marker's own (up to
  ~1.1) — and ArUco's own full 6-column condition number, computed directly
  from 200 real frames, is 15.4-21.7 — comparable to or WORSE than several
  of the cross-marker's own measured frames. So ArUco's raw system is
  ALSO meaningfully ill-conditioned; geometry/spread does not explain why
  it gets away without gyro-derotation.

**CONFIRMED (2026-08-12, same session):** ArUco's corner correspondence is
EXACT (ID-based re-decode in both `imgs[0]`/`imgs[1]` each call, no tracking
ambiguity), while the cross-marker's is LK-TRACKED (demonstrably noisier).
Directly measured per-frame coefficient of variation of per-point flow
magnitude (how inconsistent individual points' measured motion is with each
other — a rigid-body flow field should be low-CV) across the FULL dataset,
not just the flagged outlier frames: ArUco (nested textured marker) median
CV=0.299, mean=0.332, p90=0.601 (476 real frames); cross-marker median
CV=0.747, mean=0.829, p90=1.274 (1581 real frames) — **~2.5x higher across
the board.** Since both systems' Wx/Wy-vs-Tx/Ty ill-conditioning is
comparable in magnitude (§4/§5), this is the real explanatory difference:
the same amplification factor applied to ArUco's much cleaner input stays
tolerable, applied to the cross-marker's noisier LK-tracked input produces
the damaging sign-flips/overshoots observed. This is now a confirmed
finding, not just a hypothesis — cite it directly.

## How to apply

- Don't re-propose "ArUco avoids gyro-derotation via marker/board spread" —
  it's disproven. The confirmed explanation (§5) is correspondence quality:
  ArUco's exact ID-based decode has ~2.5x lower per-point flow-magnitude CV
  than the cross-marker's LK-tracked correspondence (measured across full
  datasets, not just outlier frames) — cite this directly, it's settled,
  not a hypothesis needing re-verification.
- Don't re-derive whether gyro-derotation is needed for the cross-marker —
  settled, confirmed necessary, don't remove it.
- Don't drop Tz from the joint pseudo-inverse solve even though conditioning
  analysis says it's mathematically free to do — user directive, keep the
  joint solve's shape unchanged; moment-loom is an override, not a
  replacement of the fit structure.
- Before implementing moment-loom, decide the two still-open items from the
  reviewed plan: minimum point floor for a stable moment estimate (untested
  below ~9-10 points), and the NaN/hold-last convention when all points get
  MAD-rejected.
- The dt/frame-pairing architecture fix (§2) is reviewed but NOT
  implemented — don't assume it's live; check `cross_marker_perception.py`
  directly before citing its current behavior.
