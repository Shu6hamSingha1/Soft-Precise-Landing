# HANDOVER — cross-marker FOV-CBF extent fix, ring sampling, and a live NaN-corruption bug (2026-08-13)

Read this before continuing cross-marker landing work. Full detail lives in
`Memory/px4/project_20260812_cross_marker_flow_architecture_investigation.md`
(sections referenced below); this doc is the entry point, not a replacement.

## What shipped this session (all committed + pushed to `main`)

1. **Ring-style point sampling** (`CROSS_RING_SAMPLING=1`, off by default) —
   `cross_marker_perception.py`. Partitions the eligible mask into radius-band
   x angle-sector cells around the **image center** (not the marker centroid
   — a first cut got this wrong, caught and fixed same day), with a min-radius
   floor (`CROSS_RING_CENTER_MIN_R_PX=20`) and a guard that only engages ring
   sampling when the marker actually surrounds the image center in enough
   sectors (else falls back to the unconstrained sampler — matters for a
   moving/off-center target). Includes paired-opposite LK-failure rejection
   (drop a point's diametric partner too, mirroring `img_data.py`'s
   `PLASMC_RING_PAIRED`). Memory §3b. **Still open:** an `origin_ratio`
   before/after comparison against `CROSS_RING_SAMPLING=0` on matched
   flights — only end-to-end "doesn't crash" has been verified, not that it
   measurably improves point distribution.

2. **FOV-CBF extent-blindness fix — the big one.** Root cause: for
   `MARKER_TYPE=cross`, the visibility CBF was fed only the marker's bare
   CENTER pixel, with zero awareness of the marker's own size. Diagnosed via
   an IC1-5 GT comparison (Memory §3c) that traced a catastrophic
   near-touchdown perception collapse (4/5 flights, 96-100% detection
   `'miss'` for the final 1.5-3m) to the marker's true edge exiting the
   camera frame while its center pixel stayed comfortably inside the CBF's
   margin. Fixed (Memory §3d-§3f) by:
   - `CrossMarkerPerception.get_marker_radius_px()` (new,
     `MARKER_EXTENT_PX/2`, freshness-gated like `get_center_px()`).
   - `controller.py`'s cross-marker CBF branch now carries this radius
     alongside the center point, applied via the **closed-form circle
     equation** (not materialized points — `rho_fov - (|offset|+radius)`,
     exact for an axis-aligned box).
   - `cbf_visibility.py` is now **fully dedicated to the cross-marker**
     (`radius` is a mandatory param, `delta2 = radius/foc` unconditionally —
     no corner-array fallback). `cbf_visibility_aruco.py` is now ArUco's
     **unconditional** default (import routes by `MARKER_TYPE`, not the old
     `CBF_PHASE2_FIX` flag — that flag no longer selects a file for ArUco,
     see `cbf_visibility_aruco.py`'s own docstring).
   - This is deliberately a SOFT constraint for cross-marker, not a hard
     one like ArUco's real-4-corner requirement — see Memory §3d/§3f for the
     full reasoning (it only informs two already-graduated mechanisms,
     Phase-2's ramp and the drift-off pullback; it doesn't touch Phase-1's
     centroid-only hard QP bound).
   - Live-tested: IC2 (the exact off-center case that was failing) detect
     rate went 77.7% -> 95.6-100% across reps. **n=1 each — not yet a
     validated sweep result.**
   - **Not yet done:** an ArUco SITL flight to confirm the `MARKER_TYPE`-
     based import routing change doesn't regress the live ArUco pipeline
     (only static-import-resolution + numerical cross-checks done so far);
     an n>=5 IC1-5 sweep to confirm the detect-rate gain reproduces and
     actually improves `SoftPrecise` landing outcomes, not just detection.

3. **A live, self-latching NaN-corruption bug — found, root-caused, and
   FIXED in `PX4_Gazebo/src/controller.py` (unrelated to items 1-2, found
   while checking them).** `self._last_loop_dt` (wall-clock loop timing in
   `run()`) had no minimum-dt floor; a near-duplicate `perf_counter()` read
   divided straight into the SMC's `dh_d` finite-difference, producing a
   literal NaN that self-latched through `kappa`'s RK5 state for the rest
   of the flight (commanded body-rate/thrust all going NaN). Fixed with a
   dt floor (matching `gz_subscriber.py`'s existing convention) plus an
   independent finite-check guard on the `dh_d` division itself. Verified
   via unit tests of both guards + 2 clean live flights post-fix. Memory
   §3g has the full trace.
   - **MATLAB does not have this bug** (fixed sim-timestep `dt`, no
     wall-clock involved — structurally immune).
   - **`Hardware/scripts/controller.py` HAS THE IDENTICAL, STILL-UNFIXED
     bug**, same unguarded pattern at the same relative lines. Confirmed
     `PX4_Gazebo`'s version inherited it via the documented 2026-08-04 port.
     **The user is porting this fix themselves from a separate (Windows)
     session — do not duplicate that work here unless asked.** If picking
     this back up, check first whether `Hardware/scripts/controller.py`
     already has the fix before touching it.

## Suggested next steps, roughly in priority order

1. Confirm the Hardware-side NaN fix has landed (ask, or check
   `Hardware/scripts/controller.py` for the same dt-floor pattern) before
   assuming it's still open.
2. n>=5 IC1-5 sweep with the CBF extent fix (`WORLD=cross_marker
   MARKER_TYPE=cross`, default env otherwise) to validate item 2's detect-
   rate gain and check actual landing-outcome impact — this is the most
   consequential open item, since everything else this session was smoke-
   tested at n=1.
3. A short ArUco SITL flight (`MARKER_TYPE=aruco`, default env) to confirm
   the `cbf_visibility.py`/`cbf_visibility_aruco.py` import-routing change
   didn't regress the ArUco pipeline.
4. If time permits: the `origin_ratio` before/after comparison for ring
   sampling (item 1), and a moving-target (rover world) test of the ring-
   sampling center-coverage guard, which was specifically designed for that
   case but has only been tested on a stationary target so far.

## Where to read more

- `Memory/px4/project_20260812_cross_marker_flow_architecture_investigation.md`
  — the full investigation, all findings, all verification detail. Sections
  1-2: earlier dt/frame-pairing + getFPS fixes (prior session). 3-3b: flow/
  moment-loom point-scarcity work (real but partial fixes, not the root
  cause). 3c: the frame-boundary-exit root cause. 3d-3f: the CBF fix and
  its iterations. 3g: the NaN bug. 4-5: gyro-derotation + ArUco-comparison
  retraction (earlier, still valid).
- `docs/PLASMC_TUNING_GUIDE.md` — general tuning/diagnosis entry point,
  auto-loaded each session.
- `.claude/skills/diagnose-flight-data/SKILL.md` — the GT-comparison
  discipline used throughout this investigation (always compare against
  independently-computed GT, never the controller's own reference; verify
  timestamp sync directly).
