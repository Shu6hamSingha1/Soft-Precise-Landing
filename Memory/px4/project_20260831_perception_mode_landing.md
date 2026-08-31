---
name: project_20260831_perception_mode_landing
description: "2026-08-31 session: pushed real cross-marker perception-mode landing (GT-FB OFF) forward. Fixed the run() log-archive crash that blocked it, the origin-free arm-pixel selection in cross_marker_detector (recovers detection ~0.73->0.5m), the tdV2 flow-freeze false-fire at altitude, and — the key control finding — the terminal fly-away is _dtheta_correction (CBF visibility-conflict lift), NOT a spurious loom; removed it and folded the descent-rate/lateral-margin trade into the joint QP (CBF_AZ_COST_GAIN). Perception-mode landing still NOT working end-to-end; CBF change not yet SITL-validated."
metadata: 
  node_type: memory
  type: project
  originSessionId: 0c94ab6a-894a-4d39-9867-91dec8322965
  modified: 2026-08-31T00:14:35.954Z
---

# Perception-mode landing — 2026-08-31 session

Continues [[project_20260830_perception_touchdown_detect_broken]]. Goal: get a real
cross-marker perception-mode landing (`MARKER_TYPE=cross WORLD=cross_marker`, **no**
`PLASMC_GT_FEEDBACK`) to actually land. Not there yet, but four blockers removed and the
terminal fly-away root-caused.

## Committed this session (all pushed to main)

1. **`0a39d306` tdV2 perception touchdown detector** — `_touchdownDetectV2` in
   controller.py, cross-marker perception only, `PLASMC_TD_V2=1` default. NO `|s_e_n|`
   gate (touchdown is mechanical; off-target contact still disarms). 3 paths: OVERFILL
   (`N_corn` collapse + `extent>=0.55*frame_min`), BACKSTOP (extent+centroid frozen ~3s),
   FLOW-FREEZE (background median px-disp high→low transition — catches soft OFF-marker
   settles). All `PLASMC_TDV2_*` env-tunable. Replay-designed on 205 GT-FB landings.
2. **`b3243239` tdV2 flow-freeze fix** — now gated on `_td_ext_armed` (extent grew ≥2×
   from min). Without it, the "was-moving → now-frozen" pattern was satisfied by the
   IC-convergence reposition-then-settle → **false-fired LANDED at 4.87 m** on the first
   real perception run.
3. **`0bb18426` run() `list(int)` crash fix** — the marker-lost log-segment archive did
   `{k: list(v) …}` but `_buildLogDict` carries a bare scalar (`Fresh Gate Blocked N`);
   `list(0)` → `TypeError` → controller thread dies → drone hangs at ~0.9 m. GT-FB never
   hits this branch (its path keeps `TARGET_IS_VISIBLE` latched); real perception loses
   the marker near touchdown on nearly every landing. **This was the actual blocker.**
4. **`dd4ca9c6` origin-free arm-pixel selection** in `cross_marker_detector.py`. The old
   code selected each arm's mask pixels by ANGULAR BEARING FROM BBOX CENTRE (±10° of the
   Hough angle) — circular (uses a guessed centre to find the pixels whose fit gives the
   real centre) and bbox-centre ≠ junction once the cross is asymmetric / clipped /
   overfilling. Traced: detection died at alt 0.73 m, centroid `s` froze, ~0.5 m blind
   drift, 1.75 m miss. FIX: per angle cluster build a representative line from that
   cluster's Hough segments (direction = cluster-mean angle, point = member-endpoint
   centroid), select mask pixels by PERPENDICULAR DISTANCE to that line. Origin-free,
   works on- or off-frame (feeds the existing `in_fov=False` extrapolation path). Also
   ramped the downstream `centroid_mismatch` tolerance with marker fill. Offline replay
   on the recorded landing: approach 100%→100% (no regression), terminal blind-drift
   window (alt 0.94→0.51 m) **+22 frames recovered with a live tracking centroid**,
   pre-touchdown detect_ok 0.79→0.94. Below ~0.4 m the marker is genuinely gone
   (`hough_lt2_lines`/`color_gate_empty`). NOT yet SITL-validated.
5. **`e110b8a7` + `d3ba9311` — the KEY control finding + fix.** See next section.
6. Montage tooling: `f28d21a5` sim-time sync (drone+GT by real per-frame timestamps),
   `b231ccb0` chase mapped by descent-fraction (its mp4 FPS tag is `CHASE_FPS=30` but the
   chase sensor is 20 Hz; `record_chase.py` gate+stop-file guarantee it spans
   [descent-start, touchdown]). `overlay_image_features.py`: `s = (sx, sy, 1)` (homogeneous),
   h panel always draws `w=(wx,wy,wz)`.

## The terminal fly-away is `_dtheta_correction`, NOT a spurious loom

GT-ablation of the recorded perception landing (`gt_optical_flow.py` Z_REG tool):
- **Perception is FAITHFUL to GT down to alt ≈ 1.0 m** (Δloom < 0.15, centroid tracks,
  `det=ok`). No gradual divergence.
- **The break is at alt ≈ 1.0 m: `det_ok` → 0 for the whole last metre** — the whole-cross
  Hough detector cannot see a cross whose arms span more than the frame (`insufficient_fit_points`
  → `centroid_mismatch` → `hough_lt2_lines` → `color_gate_empty`). Origin-free selection
  pushes this down to ~0.5 m but doesn't eliminate it.
- **The loom `h_z` is NOT spurious.** Recomputed 3 ways (logged moment-loom, re-derived
  moment-loom, and a PAIRWISE-DISTANCE-RATIO loom immune to edge-clipping survivor bias)
  — all flip sign at the SAME instant (t≈8.83 s), coincident with the real GT `vz`
  zero-crossing. The loom TRACKS the ascent, it doesn't cause it.
- **The initiator: `_dtheta_correction`** (`controller.py`, `PLASMC_DTHETA_AZ_GAIN`
  default 10, cap 2.0, always-on, no enable flag). `_dtheta_norm = ‖θ_desired − θ_safe‖`
  = the lateral tilt the FoV box just suppressed; `I_a[2] -= 10·_dtheta_norm` adds
  vertical lift "to buy the lateral loop time." Trace: at alt 0.4 m, `s_e_n≈0.63`
  (unconverged), the lateral loop wants a hard tilt, the CBF says it loses the marker →
  `dtheta` 0→0.77→1.7, `I_a_z` −9.75→−12.04, `vz` reverses −0.2→+3.7, fly-away to 3 m.
  Then the loom-tracking z-SMC compounds it (`κ_z` 0.02→2.5). Same mechanism as the
  earlier IC5 fly-aways ([[project_20260825_dtheta_cbfmargin_session_wrapup]]).
- The joint CBF QP (`CBF_JOINT_QP`, default on for cross-marker) **was running but
  mostly-inert** here (`theta_cone` 0.1–0.3, box not binding hard). It does NOT subsume
  `_dtheta_correction` — that's a separate downstream `I_a[2] -=` with only a 2.0 cap,
  sequential to the QP, not an alternative.

## Fix: fold the trade into the joint QP; delete `_dtheta_correction`

- `cbf_visibility.py` **`CBF_AZ_COST_GAIN`** (default 5.0): inside the joint-QP outer
  loop, after each box projection, `Ia_z = min(Ia_z, max(I_a[2] − κ·‖y0−y*‖, −g))` when
  `I_a[2] > −g`. **Hard safety: `max(…, −g)` — the relief can only bring a descent toward
  hover, NEVER reverse it into a climb.** `min(Ia_z, …)` keeps deliverability/SMC lift.
  Lives inside the constrained solve → output `I_a` stays box- and sphere-consistent.
  `=0` disables → recovers the prior QP exactly.
- `controller.py`: `_dtheta_correction` compute + `I_a[2] -= _dtheta_correction` +
  crossfade + `_dtheta_xfade_tau`/`_dtheta_active_t` + the dead `dtheta_correction(t)` /
  `dtheta_xfade_w(t)` log fields all REMOVED. `dtheta_az(t)` (= `‖th_desired−th_safe‖`,
  now the drive signal for the QP relief) and `theta_desired(t)` KEPT as real
  diagnostics; `_dtheta_az_log` also still consumed by the upstream `h_ref_eff` channel.
  `PLASMC_DTHETA_HREF` still gates that separate upstream h_ref shaping.
- `validate_cbf.py` 12/12 (theta path, no `A_CAP` — unaffected). New joint-QP test 5/5
  (feasible passthrough / box-bind relieves to −g / extreme stays finite cone<π/2 /
  SMC-lift left alone / QP-off = theta path untouched).
- `docs/CBF_visibility.{tex,pdf}` updated: §3.1 "Descent-rate relief under an active box",
  eqs. for `Δθ` and the boxed relief, Notation + property-list entry.

## STATE / next steps

### 2026-08-31 (later) — FIX SITL-VALIDATED: fly-away gone, terminal now purely lateral-blind

First perception-mode IC1 run after the `_dtheta_correction` removal
(`HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross`, no GT-FB, IC1 `0,0,5`).
Bundle: `test_data/Landing_Test/Mon Aug 31 05-03-03 2026/`;
montage `test_data/Test_Videos/montage_perc_20260831_0503.mp4`.

- **Fly-away is GONE.** Descent is **perfectly monotone** — 0.0 m total upward travel
  over the whole descent (vs the prior run's climb to 3.06 m). Confirms the fly-away was
  `_dtheta_correction` and nothing else. This is the SITL validation of the removal.
- **`CBF_AZ_COST_GAIN` relief never engaged** — `az_joint_delta(t) ≡ 0` for all 1496
  samples (box never bound hard enough). So the relief term is validated only as
  "inert / no regression", not as an active mechanism yet. `dtheta_az(t)` peaked 5.5
  (conflict signal was live) but with `_dtheta_correction` gone it now only feeds the
  `h_ref_eff` channel.
- **Still a precision FAIL.** tdV2 `[overfill]` path fired and disarmed at alt 0.23 m,
  **0.59 m off** (honest @ min-alt), 1.07 m/s. Endpoint xy 0.83 m.
- **Root cause of the miss = terminal overfill blindness, exactly as flagged.** Trace:
  `|s_e_n|` grows monotonically once `MARKER_EXTENT_PX` crosses ~270 px (t≈37.8 s):
  0.16→0.27 (ext 318)→0.50→0.54, then at ext-collapse (ext 319→84, t≈41.9 s) a single
  `|a_u|` transient to 13.7 + `I_a_z` spike to −20.4 (brief, non-sustained) + `kappa`
  ratchet 0.12→0.50; ext then frozen at last-good 168 and `|s_e_n|` drifts 0.63→1.02
  blind. No ascent through any of it. CrossMarkerNode diag: detect ok 1516/1535 (99%)
  overall but the failures cluster in the last metre (`centroid_mismatch` ×10,
  `hough_lt2_lines` ×4, `color_gate_empty` ×2).
### 2026-08-31 (later still) — IC2–5 n=1 perception-mode: ALL FAIL, off-center lateral divergence

At user request ran IC2–5 n=1 (same env, cross-marker, no GT-FB). Bundles/montages:
- IC2 `2,2,5` — `test_data/Landing_Test/Mon Aug 31 05-14-32 2026/`, `montage_perc_20260831_IC2.mp4`
- IC3 `-2,2,5` — `…05-15-27 2026/`, `montage_perc_20260831_IC3.mp4` (no chase cam that run)
- IC4 `2,2,7` — `…05-16-52 2026/`, `montage_perc_20260831_IC4.mp4`
- IC5 `2,2,3` — `…05-17-45 2026/`, `montage_perc_20260831_IC5.mp4`

**All four: `TARGET_LOST` mid-flight, hard IMU-impact disarm, huge miss** — IC2 4.9 m,
IC3 3.2 m, IC4 4.0 m, **IC5 10.3 m** (rel_vel 4.3–4.7 m/s at min-alt on every one).

**Mechanism = off-center lateral OVERSHOOT / anti-restoring divergence (NOT the CBF, NOT
`_dtheta_correction`).** GT trajectory, IC2: start (1.9,2.0,5) → flies toward marker in
x, crosses x=0 at ~45–50% of descent → **keeps going**, overshoots to x=−4.7, y drifts
−1.1, ends 4.8 m off. IC5 identical shape but worse: crosses origin ~40%, runs away to
(−7.8,+6.6). `|s_e_n|` never converges (sits 0.5–0.8, peaks 0.9–1.4). As the marker
crosses to the far side of the FOV the centroid pins to the frame edge (`Center Px x`
goes negative / caps ~215 of 480) → detector collapses (`centroid_mismatch`,
`lt2_angle_clusters`, `color_gate_empty` pile up) → TARGET_LOST → ballistic to impact.

IC1 "works" (monotone descent) ONLY because it starts dead-centered with ~zero lateral
error; the lateral loop is not asked to null a real off-center error. This is the
long-known off-center lateral wall / anti-restoring `a_u` behaviour
([[feedback_lateral_wall_anti_restoring_au]], [[feedback_ic2_lateral_gain_chain]]),
re-confirmed here in real perception mode. **Perception-mode landing is IC1-only until
the off-center lateral loop is stabilized** — that is now the binding constraint, ahead
of the terminal-overfill hand-off.

- **IC1 n=5 NOT run yet** — not a converging landing even at IC1 (0.59 m terminal miss).

### 2026-08-31 — IC2 deep-dive root cause (data + GT + PPC)

Bundle `test_data/Landing_Test/Mon Aug 31 05-14-32 2026/`; figure
`test_data/Test_Videos/IC2_rootcause.png`; montage `montage_perc_20260831_IC2.mp4`.
Run is in **combined/blended-surface mode** (`PLASMC_HD_FUNNEL_REF=1` in the banner) →
outer PID `V_ds_d` is unused (`ds_d(t)≡0` is expected, not a bug); lateral demand flows
`s_e_n → funnel-ref backmap (p_r,zeta_r,S_r,dp_s) → h_d lat-rate → middle SMC → a_v →
a_u=−G⁻¹a_v → I_a`.

**Timeline (t = s since descent start; GT descent phase is only ~5.5 s):**
- **t 0–3.5 s, alt 5.0→2.9 m — perception PERFECT, y-axis diverges anyway.** `s_V` vs GT
  `V_s_g`: |err| < 0.02 on both axes the whole time; flow `h_V` tracks GT within ~0.1.
  `s_e_n_x` converges 0.35→0 cleanly. **`s_e_n_y` GROWS 0.35→0.85** — and GT `V_s_g_y`
  grows the same 0.35→0.76, so the divergence is REAL vehicle motion, not a perception
  error. The funnel-ref lateral demand `hd_rate_y` is correctly signed (opposes the
  error) but **capped at ≈0.45** (scale-free v/Z) and cannot null a 2.83 m offset in the
  ~3.5 s before the marker leaves frame, while Z (hence metric authority) shrinks.
  `a_u_y` is intermittently **anti-restoring** (same sign as `s_e_n_y`), partly
  cancelling the demand ([[feedback_lateral_wall_anti_restoring_au]]).
- **t ≈ 3.5 s — `|s_e_n_y|`≈0.85 breaks out of the `p_s` funnel (≈0.49).** `p_s` is
  **purely wall-clock**: `p_s(t) = (1.2−0.35)·e^{−0.5 t} + 0.35` to within 0.002, zero
  coupling to convergence. It squeezes to ~0.49 by 3.5 s regardless of error. Once
  outside, the PPC transform amplifies → `a_u` starts climbing (2→2.8→…).
- **t ≈ 3.7 s, alt 2.6 m — detection dies** (`det=ok`→`--`; `lt2_angle_clusters`,
  `centroid_mismatch`, `color_gate_empty`). Marker is oblique + near frame edge
  (`s_V_y`≈0.8) + motion-blurred by the `a_u` kick. `MARKER_EXTENT_PX`≈120 — **NOT
  overfill** (overfill ≈240); this is an off-axis/edge loss, a different failure from
  IC1's terminal overfill. `s_V` then **freezes at (0.144, 0.798) for the entire
  remaining ~2 s** while true GT bearing swings to (+32, −20). Controller flies the rest
  blind on a 2-s-stale centroid.
- **t 3.7–5.8 s — ~2 s control-log gap** (controller thread stalled / marker-lost log
  branch), drone kicked hard; resumes with `s_e_n` sign-flipped and `|a_u|`≈43.
- **t 5.8–8.5 s — funnel-reset limit cycle.** `p_s` is **re-armed to 1.2 every
  ~0.3–0.7 s** (7 resets logged) on the frozen/violating error; each re-squeeze
  re-violates → repeated `|a_u|` spikes 10–43. These kicks (not a kappa ratchet — `kappa`
  x/y sit at the 0.5 floor all run; the spikes come from the `−G⁻¹a_v` / zeta transform)
  throw the drone ballistically off. → `TARGET_LOST`, 4.9 m miss, 4.7 m/s IMU impact.

**PPC scorecard:**
- **`s_e_n` vs `p_s` (outer funnel): FAILS.** x stays inside; **y exits at t≈3.5 s** and
  is outside ~9 % of the run; the exit + the wall-clock re-arm cycle is the proximate
  explosion trigger. `p_s` shrink rate (`gamma_s=0.5`, ~2 s) is far faster than the
  loop's real y-convergence → funnel width is acting as a divergent gain.
- **`h_e` vs `p` (middle/flow funnel): OK.** `frac_outside ≈ 0` on all 3 axes; brief
  spikes to 13–17 but `|zeta|` peaks ~3.7 and ends ~0.25. The flow PPC is not the
  problem; the **outer `s_e_n` PPC + the lateral authority under it are.**

**Root-cause ranking for IC2:**
1. **Off-center y lateral loop has insufficient (and partly anti-restoring) authority** —
   diverges 0.35→0.85 with perfect vision. Binding constraint.
2. **`p_s` outer funnel is open-loop wall-clock** — squeezes past the un-converged error
   at ~3.5 s and the PPC transform turns the residual into an `a_u` spike; funnel width
   here = gain.
3. **Funnel wall-clock RE-ARM creates a limit cycle** of 10–43-magnitude `a_u` kicks once
   the error is stuck outside → the actual fly-off driver.
4. **Perception loss at alt ~2.6 m is a consequence** (off-axis + edge + kick-blur), then
   `s_V` frozen-hold for the last 2 s makes recovery impossible. NOT overfill (≠IC1).
5. Yaw/`alpha` perception is wrong all run (`aP`≈−1.1 rad vs GT swinging ±0.5) — secondary
   (rotates body lat axes), didn't corrupt `s_V`.

Fix directions (not yet chosen): (a) make `p_s` convergence-coupled or slow `gamma_s` so
it can't outrun the loop; (b) kill/limit the wall-clock funnel re-arm (one re-arm, or
freeze width on marker-loss instead of resetting); (c) raise off-center lateral authority
/ fix the anti-restoring `a_u` sign on y; (d) hold-last vs. widen-funnel on detection
loss rather than freezing `s_V` outright. (a)+(b) are the cheapest test.
- **Still open (flagged, not fixed):** the whole-cross detector still can't detect below
  ~0.4 m (marker overfills) → below there the loop is on last-good `s` + `h` + IMU. The
  real terminal fixes remain: (a) commit / hand off to flow once the marker overfills, or
  (b) a local X-junction / dual-scale marker so a fiducial stays in frame to touchdown.
- Recordings from this session (gitignored): `test_data/Test_Videos/montage_percfix_20260831.mp4`,
  `overlay_percfix_20260831_0259_{s_alpha,h}.mp4`, `Mon Aug 31 02-59-19 2026.mp4` +
  bundle `test_data/Landing_Test/Mon Aug 31 02-59-38 2026/`.
