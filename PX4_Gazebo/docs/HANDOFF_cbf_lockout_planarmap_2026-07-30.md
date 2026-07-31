# HANDOFF: CBF visibility-cone lockout & PlanarFeatureMap non-engagement (2026-07-30)

## UPDATE (later same session): a real fix WAS applied — see "The fix actually
## applied" section near the end. The root-cause investigation below remains
## accurate and should still be read first; the fix addresses the SYMPTOM
## (a degraded CBF going unnoticed) via an existing, validated safety
## mechanism, since the exact trigger inside `_feature_pts_fresh` was never
## 100% confirmed live (3 reproduction attempts with the plain launcher all
## failed to reproduce the fly-away — see "Dead ends"). If you get a
## reproduction with the `[cbf_corners]` debug print active, use it to close
## that remaining gap and consider whether a more targeted fix (upstream of
## the app-loop signal) is now possible.

**Read this before re-investigating why landings miss the marker, why theta_cone/I_a
collapse mid-flight, or why PlanarFeatureMap rescue doesn't seem to help.** This
session traced the mechanism in detail across both real hardware (Pi) and Gazebo
SITL. It is NOT fully resolved — the last link in the chain needs either a
reproduced fly-away with the new debug instrumentation, or code reading beyond
what this session had budget for. Read to the end before repeating any of this
work.

## TL;DR

1. Real Pi landings were missing the marker. Root-caused (with real telemetry) to
   a **CBF visibility-cone lockout**: `theta_cone` (the CBF's allowed tilt-cone
   magnitude) collapses toward zero once the marker drifts off-center, which
   clamps the actually-commanded correction (`I_a`) to near-nothing exactly when
   more correction is needed — a self-reinforcing lockout, not a one-off glitch.
2. **This is NOT hardware-specific.** An existing recorded Gazebo run
   (`test_data/Landing_Test/Wed Jul 29 16-03-41 2026`) shows the identical
   mechanism, more severely (full fly-away, `|s_e_n|` to 51, vs. 2-3.6x on
   hardware).
3. **The CBF's QP math itself is not the bug.** Independently re-solved the exact
   box-QP (via `scipy.optimize`) using the real corner/attitude data logged at the
   frozen frame — a large, genuine feasible correction (`‖theta‖≈1.2`) exists.
   The 10-iteration alternating-projection algorithm in `cbf2_filter` reproduces
   that same answer when given fresh state. So the live collapse is not a
   geometric dead-end and not an algorithm-convergence bug.
4. **The freeze is caused by something upstream feeding `cbf2_filter` stale/wrong
   corners**, not the CBF itself. A sequential, faithful replay of the real
   logged frames (same corners, same quaternion, same accumulating `state` dict,
   calling the REAL unmodified `cbf2_filter`) exactly reproduces the logged
   `theta_cone` trajectory through `t=6.0s`, then **diverges**: the replay shows
   `theta_cone` recovering healthily (0.05 → 1.20 by t=15s) while the real
   flight's logged value stays bit-exact frozen at `0.0072` for the remaining
   33+ seconds. Since the replay uses the same `cbf2_filter` code, the divergence
   means the REAL controller was feeding it different (stale) corner input than
   what's logged in `Img_Data.npy`'s `Image Feature Pts`.
5. **PlanarFeatureMap's small-marker-slot path is ruled out** as the source of
   that stale input: `Centroid Map Trust`/`Alpha Map Trust` are nonzero for
   exactly ONE ~5.3s window (`t≈18.5-23.8s`) in the entire 40s flight, and zero
   everywhere else — including the whole critical `t≈7s` freeze window, which
   happened *before* the map ever bootstrapped. The small-slot corner source
   requires ≥0.5 confidence (`CBF_SMALL_SLOT_CONF_MIN`) to ever activate, so it
   was never in play at the freeze point.
6. **This directly connects to a SEPARATE previously-open question — "why isn't
   PlanarFeatureMap rescue engaging"** (was tracked as task #5 this session).
   Both symptoms now look like manifestations of the same underlying problem:
   the map isn't providing usable confidence/rescue, and the CBF's raw-detection
   fallback path (`_feature_pts` / `FEATURE_PTS_FRESH`) has a plausible
   staleness-latching gap. **Fixing PlanarFeatureMap's engagement may fix the CBF
   lockout too** — don't treat them as two separate fixes without re-checking
   this connection first.
7. **The one link NOT yet closed**: whether `_feature_pts_fresh`
   (`img_data.py`) genuinely re-arms (`= True`) after real ArUco decode resumes
   post-gap, or stays latched `False`. `N Flow Corners` (used as a decode-recovery
   proxy this session) may track ring-flow/KLT count, not ArUco marker-decode
   success specifically — those could be different pipelines. This needs either
   direct instrumentation of `_feature_pts_fresh` over time (now added, see
   below) on a REPRODUCED fly-away, or careful code reading of exactly what
   condition gates line 3059's `self._feature_pts_fresh = True` vs the coast
   branch's `self._feature_pts_fresh = bool(_pm_rescue)`.

## Why you should NOT re-derive this from scratch

This took a full investigative session: pulling and cross-referencing hardware
`.ulg`/`Control_Data.npy` telemetry, extracting and decoding video frames,
independently re-solving the CBF's QP with `scipy`, and sequentially replaying
real recorded frames through the live, unmodified `cbf2_filter` function with
persistent state. All of that evidence is summarized above and the exact
commands/scripts used are in "Reproduction toolkit" below. Re-reading
`cbf_visibility.py` and `FUNNEL_CBF_DESIGN.md` cold and guessing at a fix
without this context is how the earlier `theta_floor` misdiagnosis happened
(see "Dead ends" below) — read this whole doc first.

## Evidence trail (chronological, this session, 2026-07-29/30)

### Part A — real hardware findings that started this
- Camera images were badly overexposed (mean brightness 199-251/255 across ALL
  10 recorded videos from the 2026-07-29 session) due to a **fixed 4x display
  gain** (`debayer_bayer_to_bgr`, `img_data.py`) tuned for indoor lighting
  (`~20/255 mean`) and applied unconditionally outdoors. **FIXED**: gain is now
  adaptive (`target_mean=128`, clamped `[0.15, 8.0]`), computed per-frame from
  the frame's own brightness. This is a SEPARATE issue from the CBF lockout —
  it affects whether the marker is *detected* at all, not whether the
  controller can *act* on a detection it has. Both matter; don't conflate them.
- Also found and fixed (unrelated to the above, found via direct source
  reading, all deployed and verified via sha256 hash-match Pi<->Windows mirror):
  - `focal[0]` crash when `focal` is a scalar (`controller.py`)
  - Buffer-desync `IndexError` on marker-loss->reacquisition
    (`_initialize_controller` reset `_t`/`_dt` but not `_s_raw`/`_s_good`/
    `_h_raw`/`_h_good`)
  - Unsafe `disarm()` on a (possibly false) touchdown detection mid-air ->
    replaced with `land()`
  - False touchdown-detector firings (loom-inversion armed/fired mid-air, no
    persistence/spike-rejection gate) -> detector disabled pending a proper
    fix
- Did a large architecture refactor: `img_data.py`/`imgstreamer.py` now work in
  ONE resolution (320x240, the ISP-scaled "main" stream) throughout, instead of
  juggling a 640x480 "raw"/320x240 "main" split with a `self._aruco_scale`
  conversion at ~15 call sites. `img_geometry.py`'s `CALIB_CX/CY`/`fx/fy` were
  halved to match (mathematically safe: `p_10 = center/focal` is a ratio,
  invariant under uniform scaling — confirmed no controller-behavior change).
  This was a code-clarity fix, not directly related to the CBF lockout.

### Part B — decomposing the real control cascade (why landings miss the marker)
Pulled `Control_Data.npy` from 3 hardware flights and traced the full control
chain (`s_e_n` -> outer PID/funnel -> `a_u` -> CBF -> `I_a` -> `B_T`/`w_u`) at
every sample. Found, consistently across all 3 flights:
- `s_e_n` (normalized lateral error) grows continuously, not from a sudden
  marker-loss event
- `a_u` (raw ASMC command) scales up correctly as error grows — the underlying
  control law is not the bug
- `I_a` (post-CBF, actually-commanded) stays small/bounded regardless of how
  large `a_u` gets
- `theta_cone` saturates low (down to fully 0.000 in one case) tracking exactly
  when `I_a` stops responding to `a_u`

**Isolated to the PURELY AUTONOMOUS window** (before any RC stick input, using
console's own visibility timestamps — same clock as `Control_Data.npy`'s `t`):
the lockout was ALREADY present before any pilot intervention in all 3 flights
checked (`theta_cone` down to 0.087 rad or fully 0.000 within the first 2.8-8.0s
of descent, purely autonomously). **This is not pilot-induced drift** — pilots
were catching an already-failing autonomous controller, not causing the failure.

Also checked GPS-derived pilot-intervention correlates across ALL 26
"Pilot took over using sticks" events in the 2026-07-29 session (cross-referenced
`.ulg` `logged_messages` timestamps against `vehicle_local_position`): found TWO
distinct intervention patterns — (1) high-altitude (5-13m) + large lateral drift
(3-10m), matching the CBF-lockout mechanism, and (2) near-ground (<0.7m)
landing-assist catches, mostly small drift. Don't conflate these when analyzing
future "pilot took over" events — check altitude/drift at the exact timestamp,
not just the event's existence.

### Part C — confirming Gazebo reproduces it, and localizing the mechanism
See "Reproduction toolkit" below for exact commands. Summary in the TL;DR above
(points 2-6).

## Dead ends / wrong turns (so you don't repeat them)

- **`theta_floor` bypass theory (WRONG, but plausible-looking)**: initially
  found that `theta_floor_deg=60°` is configured but the live `theta_cone`
  ignores it once `cbf2_filter`'s Phase-1 QP runs (`theta_cone =
  norm(th)` unconditionally overwrites the floored pre-CBF estimate). This is
  TRUE as a code-reading fact, but is NOT the actual root cause of the observed
  lockout — the QP's own output was independently verified correct (see point 3
  above) at the specific frame checked. Applying a floor here would have been
  the wrong fix; it would have masked bad input with an arbitrary tilt rather
  than fixing why the input was bad. **Do not re-propose a `theta_floor`-based
  fix without first re-confirming the QP's own answer is wrong at whatever
  frame you're looking at** (see "Reproduction toolkit" for how to check this
  in ~2 minutes via the exact-QP-solve script).
- Assumed `N Flow Corners == 0` -> `4` transition in `Img_Data.npy` meant "ArUco
  marker decode failed then recovered." This might be wrong — it may be a
  ring-flow/KLT tracking-point count, a different pipeline from ArUco marker
  decode. This unverified assumption is the reason point 7 (TL;DR) is still
  open. Check `img_data.py`'s actual `N Flow Corners`-logging call site against
  the ArUco-decode-success branch (~`img_data.py:3040-3060`) before trusting it
  as a decode-success proxy again.
- Tried reproducing the fly-away with the plain `run_aruco_landing.sh`/
  `_retry.sh` launcher (default IC, no extra env). **THREE separate
  full-duration attempts (one `timeout 280` cut short accidentally, two full
  `timeout 550` runs) all landed successfully/quickly** (did not reproduce) —
  this is a consistent result, not a fluke; the plain launcher's default
  scenario appears to just work reliably. Don't keep retrying this exact
  approach expecting a different outcome. The two ORIGINAL
  fly-away recordings this whole investigation is based on
  (`test_data/Landing_Test/Wed Jul 29 15-57-49 2026` and `16-03-41 2026`) were
  produced by a DIFFERENT Claude Code session on this same Ubuntu machine,
  believed to be attempting to replicate hardware-observed conditions — exact
  launch command/config unknown (not in `.bash_history`, no distinguishing env
  export found in `.bashrc`/`.profile`, no special config saved with the
  recordings beyond the standard `Control_Params.npy` dump, which matches the
  same baked-in non-default PLASMC params `run_aruco_landing.sh` already sets
  by default). **If you find that session's transcript or a way to identify
  its exact launch command, that would let you deterministically reproduce
  this instead of hoping for a lucky stochastic repeat.**

## Reproduction toolkit (all scripts written this session, in scratchpad —
## NOT committed to the repo; copy them somewhere permanent if you want to keep
## them long-term)

All of these were run via `scp` + `ssh ... python3 /tmp/<script>.py` against the
Ubuntu box (`shubham@10.176.60.132`), using `~/ws/scripts/env2025` (`source
~/ws/scripts/env2025/bin/activate`) for scipy/numpy, NOT the ROS2-dependent
`controller.py` import path (which needs `source /opt/ros/humble/setup.bash`
first, or a plain `import cbf_visibility` alone which has no ROS deps).

1. **Exact QP feasibility check** (proves the QP's math is/isn't the bug at a
   given frame): load `Control_Data.npy`/`Img_Data.npy` from a recorded run,
   pick a frame, extract real corners/quat/`I_a_raw`, build `center`/`focal`/
   `p_10`/`R`/`R33`/`yaw_c` exactly as `controller.py`'s call site does, then
   (a) call the REAL `cbf_visibility.cbf2_filter` fresh and (b) independently
   solve `min ||theta-theta_d||^2 s.t. |anchor + Lw2@theta| <= m2` via
   `scipy.optimize.minimize(method='SLSQP')`. Compare `‖theta‖` from both — if
   they roughly agree, the QP is not the bug at that frame.
2. **Sequential state-accumulating replay**: same setup, but loop over a real
   time range feeding real per-frame corners/quat/`I_a_raw`/`w_rp` through the
   REAL `cbf2_filter` with a PERSISTENT `state` dict (matching what the live
   controller does), printing `theta_cone`/`state['d']` alongside the logged
   value at each step. This is what caught the t=6.0s divergence point.
3. **PlanarFeatureMap confidence scan**: load `Img_Data.npy`'s `Centroid Map
   Trust`/`Alpha Map Trust`, find nonzero-index windows (`np.where(ct > 0)`) to
   see exactly when (if ever) the map was confident, across one or many
   recorded runs.

### Live debug instrumentation (added this session, present in the code NOW —
### both Gazebo `PX4_Gazebo/src/controller.py` and Pi `precise_landing/
### controller.py`, byte-identical, git-committed... — CHECK: see "Deployment
### status" below, may not be committed yet)

Both gated behind the EXISTING `PLANAR_MAP_DBG=1` env var (adds no new flag):
- `[planar_map] SELF-HEAL: ...` (pre-existing) — fires when the map does a full
  reset+re-bootstrap after sustained degenerate homography fits; reports how
  long it was degenerate.
- `[planar_map gate] confidence=... map_confidence=... rigid_ok=...
  gate_streak=... gate_on=...` (pre-existing, `controller.py`, every 30 frames)
- `[cbf_corners] src=... FEATURE_PTS_FRESH=... corners_is_none=...` (**NEW,
  2026-07-30**, `controller.py`, every 15 frames) — directly answers "is the
  CBF reading `'feature_pts'`, `'small_slot'`, or `'none'` right now, and is
  the freshness gate stuck?" This is the instrumentation needed to close
  point 7 (TL;DR) — run a flight with `PLANAR_MAP_DBG=1` until you catch (or
  reproduce) a fly-away, and read this print across the freeze transition.
  **Marked TEMP DIAGNOSTIC in a code comment — safe to delete once point 7 is
  resolved, doesn't change any control-path behavior.**

Launch command used this session (Gazebo, headless):
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
source /opt/ros/humble/setup.bash   # needed for ros2/rclpy; NOT sourced by non-interactive ssh
PLANAR_MAP_DBG=1 HEADLESS=1 MAX_ATTEMPTS=3 timeout 550 bash scripts/run_aruco_landing_retry.sh
```
(`timeout 550`, not shorter — a `timeout 280` run was cut off mid-flight before
this session realized it, producing a false "SUCCESS" that was actually just
the wrapper's retry-detection not triggering, not a real completed landing;
check the tail of the log for `gRPC error ... Socket closed` as the tell for
"got killed mid-flight, not a real outcome".)

Equivalent hardware command (Pi, `~/ws/scripts/precise_landing`):
```bash
PLANAR_MAP_DBG=1 python3 hardware_landing.py
```

## Deployment status as of end of this session (2026-07-30)

- Pi and Windows-repo-mirror (`Hardware/scripts/`) are byte-identical (sha256
  confirmed) for all touched files, INCLUDING the new `[cbf_corners]` debug
  print in `controller.py`.
- Gazebo copy (`PX4_Gazebo/src/`) has the SAME `[cbf_corners]` debug print
  added directly on Ubuntu (`shubham@10.176.60.132`) this session — **verify
  it's still present and re-sync with the Windows `PX4_Gazebo/src/` mirror if
  this repo location has since been edited elsewhere.**
- **None of this session's PX4_Gazebo-side or Hardware-side changes were
  committed to git during this session** (per user instruction, commits happen
  on request) — check `git status`/`git diff` on both the Windows repo and
  (if it's a separate checkout) the Ubuntu box before assuming any of this is
  persisted anywhere other than the live deployed files.
- `docs/FUNNEL_CBF_DESIGN.md` and `src/controller.py`/`src/cbf_visibility.py`'s
  stale `tools/replay_cbf.py` references (a tool that doesn't exist, only ever
  mentioned in comments) were corrected to `tools/validate_cbf.py` only, on the
  Ubuntu `PX4_Gazebo` copy. Same fix should be mirrored to the Windows repo
  copy if it has the same stale references.

## The fix actually applied (end of this session)

Ruled out `CBF_LW_ROT`/`CBF_RD3_DIRECT` as an explanation too: replayed the
same real frame data with BOTH variants (`Lw2` with vs without the
`[[0,1],[-1,0]]` rotation) — both produce healthy, recovering `theta_cone`,
neither reproduces the frozen live value. So this is not a CBF-coupling-
variant bug either; confirms (independently, a third way after the exact-QP
solve and the state-replay) that the bug is upstream of `cbf2_filter` itself.

**What was actually fixed**: not the (still not 100%-confirmed) trigger
inside `_feature_pts_fresh`, but the fact that NOTHING in the system could
detect a degraded CBF even once it happened. `landing_test.py`/
`hardware_landing.py`'s `feature_fresh` gate — which decides whether to keep
running closed-loop control or fall back to the validated
MARKER_LOSS_GRACE/open-loop-descent safety net — only watched
`TARGET_IS_VISIBLE`/`FEATURE_IS_STALE`/`RESCUE_ACTIVE`, none of which
reflect whether `cbf_corners` (what the CBF itself actually reads) is
available. So the CBF could run in a frozen, near-zero-authority Phase-2
fallback for 30+ seconds while the app loop kept believing everything was
fine and kept commanding based on the CBF's degraded output.

Applied, in both `PX4_Gazebo/src/controller.py` and the Pi's
`precise_landing/controller.py` (byte-identical across Pi / Windows
`Hardware/scripts/` mirror / Ubuntu `PX4_Gazebo/src/` / Windows
`PX4_Gazebo/src/` mirror — all sha256-confirmed):

1. A `self._cbf_corners_none_streak` counter, incremented every control-loop
   call where `cbf_corners is None`, reset to 0 whenever it's available
   (from either the small-slot or `feature_pts` source).
2. A new `CBF_CORNERS_STALE` property: `True` once that streak reaches
   `CBF_CORNERS_STALE_FRAMES` (default 30, override via env var).
3. `landing_test.py` (Gazebo) and `hardware_landing.py` (Pi)'s
   `feature_fresh` expression now ANDs in `not CBF_CORNERS_STALE` — so a
   sustained CBF corner-source outage forces the SAME already-validated
   marker-loss-grace fallback that a raw detection failure would, instead of
   silently continuing on a degraded barrier.

This is a safety improvement that's correct regardless of what the exact
`_feature_pts_fresh` trigger turns out to be — it makes a real degraded state
(confirmed present in the recorded fly-away data) visible to the system's
existing safety net, rather than leaving it undetected. It does NOT fix
whatever upstream condition causes `cbf_corners` to go stale in the first
place — that's still open (see "the one link not yet closed" above). Once
CBF_CORNERS_STALE fires in a real flight, the vehicle will fall back to
open-loop descent rather than continuing to fly on a frozen CBF for 30+
seconds — a bounded, understood failure mode instead of an unbounded one.

**Not yet done**: a live test confirming `CBF_CORNERS_STALE` actually fires
and triggers the fallback correctly in a real degraded scenario (still
blocked on reproducing the fly-away — see "Dead ends"). Recommend testing
this deliberately (e.g. temporarily lower `CBF_CORNERS_STALE_FRAMES` and
force `cbf_corners` to `None` for a controlled test) before trusting it in a
real flight.

## Suggested next steps, in order

1. If you have access to the other Claude Code session that produced the
   original two fly-away recordings, get its exact launch command first — much
   faster than re-guessing.
2. Otherwise, run more `PLANAR_MAP_DBG=1` attempts (headless, `timeout 550`+,
   `MAX_ATTEMPTS=3`+) until one reproduces a long-duration drift/fly-away, then
   read the `[cbf_corners]` prints across the transition to close point 7.
3. Once point 7 is closed (confirmed `FEATURE_PTS_FRESH` latches `False` and
   never re-arms, or confirmed it's something else), the actual fix is most
   likely in `img_data.py`'s freshness-gate/coast-branch logic (~line
   3395-3413) or in why PlanarFeatureMap takes ~18s to first bootstrap and
   loses confidence after only ~5s (separately worth understanding) — NOT in
   `cbf_visibility.py`'s QP, which is independently verified correct.
4. Re-run the img_process_freq check (separate open task, #2 from this
   session's tracked list) now that the Pi deployment is live — the resolution-
   consolidation refactor changed the hot path significantly.
