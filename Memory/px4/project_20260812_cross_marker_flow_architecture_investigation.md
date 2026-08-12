---
name: project_20260812_cross_marker_flow_architecture_investigation
description: "Deep-dive into the cross-marker Hz sign-flip/hard-landing root cause: found and fixed a real getFPS() init bug, diagnosed a dt/frame-pairing architecture flaw (self._prev_gray staleness across detection dropouts, unlike img_data.py's always-adjacent-frame-pair design), validated moment-loom+MAD-outlier-rejection as a real Tz-quality improvement on live flight data, confirmed gyro-derotation is still structurally necessary for the cross-marker (NOT avoidable the way originally guessed), and retracted an earlier wrong claim about why ArUco doesn't need it."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-12T05:40:16.172Z
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

## 3. Moment-loom + MAD outlier rejection for `Tz`: VALIDATED on real flip data, NOT yet implemented

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
loom is to be implemented as an ADDITIONAL, separately-computed `Tz` that
OVERRIDES `sol[2]` in the final output — the joint solve keeps producing all
4 unknowns exactly as today.

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
