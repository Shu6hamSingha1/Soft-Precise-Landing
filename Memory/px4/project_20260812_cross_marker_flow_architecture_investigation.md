---
name: project_20260812_cross_marker_flow_architecture_investigation
description: "CLOSED, honest negative result: implemented + live-validated TWO real fixes for the cross-marker hard landing (getFPS() init bug; dt/frame-pairing staleness fix, removes self._prev_gray/_prev_frame_t entirely; moment-loom+MAD-outlier-rejection for Tz, incl. origin-spread gate refinement) -- correct and validated, NEITHER resolves the hard landing. Root cause is structural point-scarcity/poor-distribution at close range, not staleness or per-point-solve quality. Also confirmed gyro-derotation still structurally necessary, retracted an earlier wrong claim about the nested textured ArUco marker (real reason: ~2.5x lower correspondence noise, not board spread), and added ring-style sampling + paired-opposite LK-failure rejection (CROSS_RING_SAMPLING=1) to proactively address point DISTRIBUTION -- implemented+live-tested (775-frame flight, no crash, 100% detect), before/after origin_ratio comparison still open."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-13T04:48:02.035Z
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

**NOT yet done / open follow-up:** a direct before/after comparison of
`origin_ratio` (§3's metric) and n_kept-per-frame between
`CROSS_RING_SAMPLING=0` and `=1` on matched flights — the live test above
only confirms the mechanism runs correctly end-to-end, not yet that it
measurably improves the distribution metric in practice. Do that comparison
(offline, from two flights' Point/Radial Diag Logs) before considering this
"validated" in the same sense §3's origin-spread gate was; log the result
here when done.

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
