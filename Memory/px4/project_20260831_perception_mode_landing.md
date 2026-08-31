---
name: project_20260831_perception_mode_landing
description: "2026-08-31 session: pushed real cross-marker perception-mode landing (GT-FB OFF) forward. Fixed the run() log-archive crash that blocked it, the origin-free arm-pixel selection in cross_marker_detector (recovers detection ~0.73->0.5m), the tdV2 flow-freeze false-fire at altitude, and — the key control finding — the terminal fly-away is _dtheta_correction (CBF visibility-conflict lift), NOT a spurious loom; removed it and folded the descent-rate/lateral-margin trade into the joint QP (CBF_AZ_COST_GAIN). Perception-mode landing still NOT working end-to-end; CBF change not yet SITL-validated."
metadata: 
  node_type: memory
  type: project
  originSessionId: 0c94ab6a-894a-4d39-9867-91dec8322965
  modified: 2026-08-31T00:30:24.524Z
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

### 2026-08-31 — WHY perception IC2 diverges but GT-FB IC2 lands (0.017 m)

Compared clean-phase (T<4 s, before detection death) of perc-IC2
(`…05-14-32 2026`) vs a GT-FB IC2 (`test_data/Multi_IC/20260828-110310/IC_2_rep_1`,
GT channels = all, 5/5 precise+soft). **Same control law, same params** (only `p_inf`
1.0 vs 2.5, a middle-funnel floor, differs). Both are in **combined/blended mode**
(`ds_d(t)≡0` in BOTH) → `h_d` lateral rate is built from **`s_dot_meas`**, the
**time-derivative of the centroid feature**, in both cases. So the position feature `s_V`
matching GT to <0.02 (which it does) is NOT what the lateral loop consumes. The two
perception defects that GT-FB doesn't have:

1. **`s_dot_meas` is 7–12× noisier under perception.** absmax x,y: GT-FB 0.12/0.10 vs
   perc **0.86/1.20**; std 4–6×; raw d/dt(logged `s`) 10–15× noisier
   (std 0.09 → 0.85). Differentiating even the savgol'd ~62 Hz centroid (sub-pixel
   quantization + V-frame reprojection jitter) injects large spurious feature-velocity.
   The blended surface feeds it straight into `h_d` → middle SMC chases a noise-corrupted
   flow target → `|a_u lat|` mean 0.48→**1.18**, max 1.5→**3.9** → attitude buffeting
   (`|e_R|` 3–7×). The marginal, ~0.45-capped y restoring demand can never net a
   convergence against that; and the buffet blurs/edges the marker → detection death at
   alt 2.6 m (GT-FB never "loses" its marker).
2. **The yaw feature `alpha` (= `s[3]`) is broken under perception for off-center ICs.**
   perc `alpha` mean **−1.26 rad (−72°)**, range −1.84..−0.81; GT-FB `alpha` mean 0.02
   rad. Drives a real yaw attitude error `e_R_z` → **1.0 rad** (GT-FB 0.14). A 72°-wrong
   yaw estimate rotates the controller's V-frame x/y ~72° from truth → the "restoring"
   push on `s_e_n` goes largely along the wrong axis → x still looks like it converges
   while **y diverges**. Likely cause: IC1 views the cross axis-aligned & fully in frame
   (clean stub); IC2 views it obliquely from 2.83 m off → stub intermittently not found →
   `cross_marker_perception.py` holds/recomputes `alpha` from bad points.

**Net:** perception-mode off-center failure is NOT bad centroid position — it's (i) the
blended surface consuming a 10×-noisier **differentiated** perception signal, (ii) a
genuinely **broken yaw/alpha** channel off-center, and (iii) perception **dropout** at
alt ~2.6 m where GT never drops. Any fix has to address the `s_dot_meas` noise path
(filter / use back-mapped analytic `ds_d` instead of measured derivative off-center) and
the `alpha` robustness, not just the `p_s` funnel.
- **Still open (flagged, not fixed):** the whole-cross detector still can't detect below
  ~0.4 m (marker overfills) → below there the loop is on last-good `s` + `h` + IMU. The
  real terminal fixes remain: (a) commit / hand off to flow once the marker overfills, or
  (b) a local X-junction / dual-scale marker so a fiducial stays in frame to touchdown.
- Recordings from this session (gitignored): `test_data/Test_Videos/montage_percfix_20260831.mp4`,
  `overlay_percfix_20260831_0259_{s_alpha,h}.mp4`, `Mon Aug 31 02-59-19 2026.mp4` +
  bundle `test_data/Landing_Test/Mon Aug 31 02-59-38 2026/`.

### 2026-08-31 — VDS s_dot_meas glitch gate applied; helps at the margin, doesn't fix off-center

`_vdsKFStep` had NO outlier rejection (unlike `_yawKFStep`). Added a per-axis gate on the
**raw inter-measurement step** `(z−z_prev)/dt` — NOT the KF innovation (with `q=10` kept,
the χ²-vs-S gate goes blind: S inflates until any jump scores d²<1; and gating a fixed
ref against the innovation clips normal frames because the q=10 filter's own predicted
position is noisy). `|raw_rate|` beyond `√gate·gate_rate` (≈6 u/s vs measured ~0.85 u/s
clean std, 5–7 u/s glitch spikes) inflates that axis' R by d²/gate → K→0 for the frame.
Within-band frame = bit-identical to pre-gate (verified `max|Δvx|` gate-on vs off on an
identical clean stream = 0 → zero added lag). Env `PLASMC_VDS_KF_GATE` (d², default 9;
0=off), `PLASMC_VDS_KF_GATE_RATE` (rate 1σ, default 2.0). Diag scalars `VDS Gate Hits N`
/ `VDS Gate Calls N` / `VDS d2 Max`. Commit on main.

**SITL results (perception mode, cross_marker, no GT-FB, n=1 each):**
- **IC1**: no regression — monotone descent, min_alt 0.20 m, no ascent. Two runs:
  honest xy 0.38 m / 0.77 m (n=1 variance; the earlier ungated IC1 was 0.59 m).
- **IC2** (`Mon Aug 31 11-25-08 2026`): **still TARGET_LOST**, but milder. Gate fired
  51/1848 axis-calls, d²max 2.9e4 (severe glitches caught).
  | metric | ungated IC2 | gated IC2 |
  |---|---|---|
  | s_dot_meas absmax x,y (T<5s) | 0.86 / 1.20 | 0.73 / **0.80** |
  | s_dot_meas std x,y | 0.217 / 0.184 | 0.219 / **0.162** |
  | final xy_err | **4.8 m** | **2.9 m** |
  | GT x overshoot | +1.9 → −4.7 (runaway) | +1.9 → −2.5 (milder) |
  | \|s_e_n\| end@5s | 0.86 | 0.90 (still diverging) |

**Verdict:** the gate does exactly what it was scoped to — clips the rate SPIKES (y absmax
1.20→0.80, worst d² 2.9e4 rejected) — and the IC2 miss roughly halved (n=1, partly noise).
But it does **not** touch the broadband `s_dot_meas` noise FLOOR (std only 0.184→0.162,
~12%), and the off-center failure MECHANISM is unchanged: y `s_e_n` still diverges
0.5→0.9, crosses origin ~50% then overshoots, TARGET_LOST. The dominant remaining terms
are (i) the broadband noise floor (needs a better upstream signal or accepted smoothing
lag — a gate can't fix a floor), (ii) the wall-clock `p_s` funnel breakout + re-arm, (iii)
the anti-restoring y `a_u`, (iv) the broken `alpha`/yaw channel (→ move `w_z` to gyro,
keep `w_x=w_y=0`), (v) perception dropout at alt ~2.6 m. Gate stays in as a cheap,
lag-free spike guard; it is not the off-center fix.

### 2026-08-31 — s_dot_meas NOISE FLOOR root-caused; sub-pixel junction refine tried, doesn't help

The gate (prev section) clips spikes, not the floor. Traced the floor on the gated IC2
run (clean approach, det_ok, alt 2.7–4.9 m; `gt_optical_flow` for GT):

| stage | HF noise (poly-detrended) | corr |
|---|---|---|
| raw pixel centroid `Center Px` | **0.69 px (x) / 0.41 px (y)** | — |
| calibrated V-centroid `s_V` | 0.0026 / 0.0049 u | **raw-px same-axis 0.78 / 0.50**; quat only 0.22–0.24 |
| implied `s_dot` from px jitter alone | **~0.13 u/s** | (measured `s_dot_meas` std 0.16–0.22; GT-FB 0.04) |

**→ ~70–80 % of the `s_dot_meas` floor is raw pixel-centroid jitter differentiated at
62 Hz.** Attitude/reprojection is minor (corr 0.22). And the jitter **grows with marker
fill**: alt 4–4.9 m (ext 87 px) 0.23/0.19 px → 3–4 m (ext 108) 0.69/0.46 → 2–3 m
(ext 118) **1.33/0.58** → dead below 2 m.

**Mechanism:** the cross center is `_line_intersection(line_i, line_j)` — analytic
intersection of two per-frame Huber line fits, **no sub-pixel junction refine, no
temporal filter**. As the arms reach the frame edges the mask-pixel selection grabs
off-line pixels (interior/opposite-arm/edge-aliasing — point counts balloon and go
lop-sided, seen 155 vs 523) → the two line SLOPES wobble frame-to-frame → the
intersection walks. GT-FB has zero jitter so the floor never appears there.

**Tried: sub-pixel junction refine** (`_refine_junction_subpix`, commit on main,
`CROSS_SUBPIX_JUNCTION`, **default OFF**). Local quadratic fit to the greyscale window
at the junction (X-junction of two dark strokes = intensity minimum → +def Hessian),
guarded shift cap, runs `in_fov` only, flows through the existing centroid_mismatch
checks. **Apples-to-apples offline replay (recorded IC2, same frames): NO jitter
reduction** from this OR `cv2.cornerSubPix`, at any shift guard (baseline 2.05/1.68 px
on the 2× half-res mp4; every variant within ±0.05; cornerSubPix made a clean ~0.9 px
sub-pixel correction and the jitter still didn't move). Confirms the jitter is line-SLOPE
noise, not junction localisation — a junction-region intensity fit has no leverage on it.
Kept as a knob (offline replay is on the downscaled H.264 IMG_RECORD mp4, not the live
raw-frame path) — needs a live `CROSS_SUBPIX_JUNCTION=1` A/B to fully rule out.

**Real levers for the floor (next):**
1. **Intensity-weighted sub-pixel line-centerline fit** — per scanline across each arm,
   take the intensity-weighted centroid of the dark band → sub-pixel centerline sample;
   fit the line to those instead of binary mask pixels. Attacks slope noise directly,
   ~zero lag.
2. **Temporal accumulation** — motion-compensate recent frames' mask/edge points by the
   KF velocity, fit each arm line once over the pooled set (√N slope-noise reduction),
   ~2–3 frames (40 ms) lag.
3. **Conditioning-aware KF R** — feed the line-fit condition number / inter-line angle /
   point-count imbalance into the VDS + `_kf_feat` measurement noise per frame instead
   of fixed `R=1e-3`. Zero lag.
4. **VDS KF `q` 10→~3** — now that the floor is known to be white px jitter (not model
   mismatch), more smoothing helps; q=1 was too far (110 ms, stalled off-center), q≈3
   with the gate is an A/B.
5. **Bounded-size fiducial** (X-junction / dual-scale) — the jitter blows up because the
   whole-cross overfills; a constant-size feature holds constant fit geometry to
   touchdown. The real terminal answer.

### 2026-08-31 — intensity-weighted sub-pixel arm-centerline fit: HALVES raw centroid jitter (live)

Implemented `_fit_arm_centerline_subpix` in `cross_marker_detector.py` (commit on main,
`CROSS_SUBPIX_CENTERLINE`, **default OFF**). Instead of `_robust_fit_line` on binary
mask pixels (whole anti-aliased rows toggle in/out of the colour gate each frame →
slope wobble), it marches along each arm and at every station takes the
intensity-weighted centroid of the dark band in the perpendicular greyscale strip → a
sub-pixel centerline sample (continuous response to a ½-px edge shift, not a toggled
row); TLS-fit through the stations. Both arms must succeed or it keeps the
`_robust_fit_line` pair. All downstream gates unchanged. Env knobs
`CROSS_CENTERLINE_STEP_PX` / `_MIN_STATIONS` / `_MIN_DARKNESS`. Diag `CENTERLINE_STATS`
printed by CrossMarkerNode at shutdown.

**Offline replay (recorded IC2 mp4): ~neutral** (2.05/1.68 → 2.04/1.61 px, y −4%) —
useless as a verdict because the IMG_RECORD mp4 is 2×-downscaled + H.264, which destroys
the anti-aliasing ramp this method reads. **LIVE SITL is where it works:**

| IC1 perception run | raw `Center Px` HF jitter x,y (alt 2.5–4.9 m) | `s_dot_meas` std x,y (T<6 s) |
|---|---|---|
| gate off (older, 05-03) | 0.686 / 0.847 px | 0.094 / 0.090 |
| gate on baseline (11-21) | 0.723 / 0.581 px | 0.061 / 0.083 |
| **centerline ON (12-04)** | **0.360 / 0.441 px** | 0.070 / 0.075 |

**Raw centroid jitter ~halved** (0.72/0.58 → 0.36/0.44 px) — the direct measurement of
what the method targets. `s_dot_meas` std stays ~flat because the `q=10` VDS KF + the
glitch gate already dominate that stage and n=1 run variance is ~this size; the win is a
**cleaner input**, which should matter more once VDS `q` is relaxed (lever #4). No
landing regression (honest 0.58 m, min_alt 0.25 m, no ascent).

**Next:** n≥5 IC1 A/B (`CROSS_SUBPIX_CENTERLINE=1` vs 0) per sweep methodology; if the
jitter win holds, flip the default AND drop VDS `q` 10→~3 (now safe — the floor is
px-jitter, and the input is now half as noisy). Then re-check IC2 off-center: cleaner
`s_dot_meas` reduces the buffeting that the funnel breakout + anti-restoring `a_u`
amplify — necessary but not sufficient (funnel + yaw/alpha still open).

### 2026-08-31 — n=5 IC1 A/B: centerline fit does NOT hold up; single-run halving was a fluke

Ran 10 headless IC1 perception landings interleaved (`CROSS_SUBPIX_CENTERLINE` 1 vs 0).
Manifest + per-run metrics computed from each bundle (Center Px HF jitter, alt 2.5–4.9 m
band; `s_dot_meas` std T<6 s).

| arm | raw Center Px HF jitter x / y (px) | `s_dot_meas` std x / y |
|---|---|---|
| OFF baseline (n=5, one bundle unreadable) | 0.88 ± 0.13 / 0.70 ± 0.22 | 0.064 / 0.077 |
| ON centerline (n=5) | 1.85 ± 2.21 / 2.55 ± 3.93 (one blowup run 6.3/10.4) | 0.068 / 0.083 |
| ON centerline, blowup run dropped (n=4) | ~0.75 / ~0.59 | — |

**Verdict: the 2026-08-31 single-run "0.72→0.36 px halving" did NOT replicate.** With
n=5 the centerline fit is **~10–15 % on jitter at best** (0.88/0.70 → 0.75/0.59 with the
blowup excluded), well inside the run-to-run scatter (±0.22 on y), and **zero effect on
`s_dot_meas` std** (0.064/0.077 → 0.068/0.083 — the `q=10` VDS KF + glitch gate fully
dominate that stage). **Leave `CROSS_SUBPIX_CENTERLINE=0`. It is not the floor fix.**

Meta-finding: **IC1 perception mode is not a stable baseline for a perception A/B** — of
the 10 runs, endpoint xy_err ranged 0.5–23 m and 2 flagged `ASCENDING`; the 4 "clean"
runs per arm sat at 0.47–0.78 m honest @ min-alt. A ~15 % perception-jitter change is
far below this noise. Any future perception A/B needs either a replay harness on RAW
frames (not the lossy mp4) or a much larger n.

**Intermittent terminal ASCENT is back (1 genuine in 10).** OFF-baseline rep1: 0.87 m of
upward travel, onset ~76 % of run at alt 0.56 m, endpoint 23 m. NOT the old
`_dtheta_correction` (removed) — `dtheta_az`=0 at onset, `az_joint_delta` never fired,
`kappa_z` flat at 0.03. `h_z` (loom) swung −2.95 → +3.78: looks like the terminal
loom-inversion driving the z-SMC to climb once the marker overfills at ~0.5 m. The other
"ASCENDING" flag (ON rep2) was a mislabel — GT shows 0.00 m upward travel, it was a 9.8 m
lateral TARGET_LOST. So: ~1/10 genuine terminal loom-driven ascent, distinct root from
the 2026-08-30 dtheta fly-away.

---

## SESSION FIXES — CONSOLIDATED (2026-08-31)

All committed + pushed to main. Two land-on-by-default, two knobs-default-off.

| # | change | file(s) | env / default | status |
|---|---|---|---|---|
| 1 | **`_dtheta_correction` removed**; descent-rate/lateral-margin trade folded into the joint CBF QP as `CBF_AZ_COST_GAIN` (clamped at −g, can only slow a descent) | `cbf_visibility.py`, `controller.py`, `docs/CBF_visibility.{tex,pdf}` | `CBF_AZ_COST_GAIN=5.0` (on) | **SITL-validated**: IC1 fly-away gone, descent monotone (0.0 m upward). `az_joint_delta` mostly never engages (box rarely binds hard). ⚠ intermittent terminal ASCENT still ~1/10 via a *different* root (terminal loom → z-SMC), see above. |
| 2 | **VDS `s_dot_meas` KF glitch gate** — per-axis test on the raw inter-measurement step `(z−z_prev)/dt` (χ²-vs-S goes blind at `q=10`); spike frames get `R∝d²/gate` → K→0, clean frames bit-identical (zero lag) | `controller.py` `_vdsKFStep` | `PLASMC_VDS_KF_GATE=9` (on), `PLASMC_VDS_KF_GATE_RATE=2.0` | **Applied, on by default.** Clips rate SPIKES (IC2 y absmax 1.20→0.80, worst d²≈2.9e4 rejected; IC2 miss 4.8→2.9 m n=1). Does NOT lower the broadband noise floor (a gate can't). No regression IC1. |
| 3 | **Sub-pixel junction refine** — local quadratic intensity fit at `_line_intersection`, +def-Hessian + shift guards | `cross_marker_detector.py` `_refine_junction_subpix` | `CROSS_SUBPIX_JUNCTION=0` (**off**) | **Doesn't help** — apples-to-apples offline: no jitter change vs analytic OR `cv2.cornerSubPix` at any guard. The jitter is line-SLOPE noise, not junction localisation. Kept as a knob only. |
| 4 | **Intensity-weighted sub-pixel arm-centerline fit** — per-station intensity-weighted dark-band centroid → TLS line fit, replaces the two `_robust_fit_line` binary-mask fits | `cross_marker_detector.py` `_fit_arm_centerline_subpix`, `_sample_bilinear` | `CROSS_SUBPIX_CENTERLINE=0` (**off**), `CROSS_CENTERLINE_STEP_PX=2.0`, `_MIN_STATIONS=8`, `_MIN_DARKNESS=20` | **Marginal / didn't replicate** — n=5 A/B: ~10–15 % jitter reduction at best (within scatter), zero `s_dot_meas` effect. Leave off. |
| 5 | **CrossMarkerNode shutdown diag** for `SUBPIX_STATS` / `CENTERLINE_STATS` | `cross_marker_perception.py` | — | cosmetic |

### Analysis findings recorded this session (no code)
- **`s_dot_meas` noise floor root cause** = raw pixel-centroid jitter (0.2 px at altitude → 1.3 px as the whole-cross overfills; line-SLOPE noise from which anti-aliased edge pixels clear the colour gate each frame), differentiated at 62 Hz → ~0.13 u/s. GT-FB has none.
- **Why GT-FB IC2 lands (0.017 m) but perception IC2 diverges**: SAME control law, both in blended/combined mode (`ds_d≡0`) → `h_d` lateral rate is built from **`s_dot_meas`** (the centroid *derivative*), which is 7–12× noisier under perception (+ `alpha` 75–100° wrong off-center, + perception dropout at alt ~2.6 m). Centroid POSITION `s_V` matches GT to <0.02.
- **IC3/IC4/IC5 = the same failure as IC2**, one mechanism four instances: off-center lateral loop crosses the target and diverges (GT-real, perception faithful) → wall-clock `p_s` funnel squeezes past the un-converged error → PPC transform → `\|a_u lat\|` 60–156 → wall-clock funnel RE-ARM limit cycle → ballistic fly-off; `alpha` 75–100° wrong on every off-center IC. **IC5 worst (10 m)** because its 3 m start = 43° initial bearing → `\|s_e_n\|` starts at 0.81, immediate funnel breakout, zero recovery window.
- **"w_z from gyro" is already the default** (`W_XY_DEROT=zero` + `PLASMC_CH_CLEAN=1`): the `ṡ`/`h_d`/`c` de-rotation already uses gyro `psi_dot_b`, not image `w`. The broken thing is the **`alpha` orientation feature** (relative heading, perception-side), which can't be gyro-sourced.

### Open, ranked (off-center is the wall; the floor is NOT the binding constraint)
1. **Wall-clock `p_s` outer funnel** — make it convergence-coupled / slow `gamma_s`; kill the wall-clock RE-ARM (one re-arm, or freeze width on marker-loss). Cheapest, attacks amplifiers 2+3.
2. **Anti-restoring / under-authority lateral `a_u` off-center** — the primary. `s_e_n` diverges with perfect vision.
3. **`alpha` off-center** — 75–100° bias at oblique view; perception fix in `cross_marker_perception.py` (`_getVirtualPts` / `CROSS_ALPHA_0` geometry dependence).
4. **Terminal overfill** — whole-cross detector dies below ~0.4–2.6 m (alt depends on offset); real fix = commit/hand-off to flow, or a bounded-size X-junction / dual-scale fiducial. Also the intermittent terminal loom→ascent (1/10) lives here.
5. **`s_dot_meas` broadband floor** — deprioritised: neither IC1 (terminal) nor IC2–5 (off-center funnel) is primarily driven by it. If revisited: VDS `q` 10→~3 with the gate, or conditioning-aware `R`.

### 2026-08-31 — CROSS_ALPHA_0 re-derivation: it's ~0°, not 90.23° — AND the slope flipped +1

Wrote `tools/derive_cross_alpha0.py` (commit). Recorded fresh phased cross-marker cal
flights with `CROSS_ALPHA_0=0` → `calibration_data/output_cross_alpha0_20260831/`
(99% detect-ok, full yaw+yawagg excitation, 500–615 yaw-excited samples/run).

**n=3 (n≥5 pending), extremely consistent:**
- raw disambiguated principal angle vs GT ENU relative yaw: **UNCONSTRAINED slope = +0.99 /
  +1.00 / +0.98, R² = 0.92–0.94**.
- slope **+1** offset (`alpha_0`): 0.29 / 0.60 / 0.46° → **mean 0.45°, inter-run std 0.13°**.
- slope **−1** (the 2026-08-08 assumption) offset: 2–7°, std 1.9°, **R² = −2.8** (garbage).

**So the deployed `CROSS_ALPHA_0 = radians(90.23)` is wrong two ways:** (1) it was derived
2026-08-08 by *forcing* `alpha_dot = −psi_dot` (slope −1, Jabbari Asl) — the current
pipeline tracks **slope +1**; (2) the offset is **~0.45°**, not 90.23°. Coherent story:
the 08-08 value baked in the 90° camera-mount yaw rotation (skill note: "suspiciously
close to exactly 90deg… plausibly a direct consequence of the 90deg camera-mount yaw"),
and since then the `[y,−x]` swap in `_getVirtualPts` took over that rotation ("REPLACED
here, not stacked" comment) + a `[y,−x]↔[x,y]` convention was churned 2026-08-10 (io-cal
skill) — `alpha_0` should now be ~0 and was never updated. This is exactly the ~−87°
staleness the landing analysis measured (perception alpha −82° at aligned hover; the yaw
loop drives an ~80° phantom correction → V-frame lateral command rotates ~90° → off-center
`s_e_n` never converges).

**Proposed fix (NOT yet applied — load-bearing, IC1-5 gate):**
`cross_marker_perception.py:504` → `CROSS_ALPHA_0` default `radians(90.23)` → **`radians(~0.5)`**
(final value after n≥5). With slope +1 (`alpha ≈ +psi_ENU`, the ArUco convention),
`BODY_YAW_ALPHA_K = −1.0` already maps it correctly to `yaw_c ≈ −psi_ENU` (NED, matching
the compass path) — **K stays −1, only the offset changes.** Then re-run perception IC1
+ IC2 (compass-yaw A/B no longer needed — this IS the yaw fix); if `s_e_n` converges
off-center, run the full IC1-5 gate.

### 2026-08-31 — CROSS_ALPHA_0 90.23°→0.58° APPLIED + SITL-VALIDATED: off-center blocker cleared

Final n=5 derive (`tools/derive_cross_alpha0.py`, `output_cross_alpha0_20260831/`):
unconstrained slope +0.96..+1.00 (mean **+0.98**, R² 0.90–0.94), offset per-run
0.29/0.60/0.46/0.61/0.94° → **circ mean 0.58°, std 0.22°**. Applied to
`cross_marker_perception.py:504` (`radians(90.23)` → `radians(0.58)`); comment rewritten;
`BODY_YAW_ALPHA_K=-1` unchanged (correct for a slope-+1 alpha). Backup:
`Obsolete/src/cross_marker_perception_v_pre_alpha0_rederive_20260831.py`. Commit `b963e207`.

**SITL (perception mode, cross_marker, no GT-FB, n=1 each) — dramatic:**
| | broken (α₀=90.23) | FIXED (α₀=0.58) |
|---|---|---|
| aligned-hover `board alpha` | **−82°** | **+3.7° / +8.4°** ✓ |
| IC1 | FAIL, honest 0.38–0.77 m | **PRECISE-only** — endpoint 0.090 m, **honest xy 0.017 m**, 0.094 m/s |
| IC2 | **TARGET_LOST, 4.9 m, fly-off** | **FAIL but LANDED** — endpoint 0.242 m, honest 0.237 m, 0.60 m/s, min_alt 0.07 m |
| IC2 perc α early | −71° | **−2°** |
| IC2 `\|s_e_n\|` start→4 s | 0.49 → **0.86 (diverging)** | 0.49 → **0.27 (converging)** |
| IC2 `angle(h_xy, h_d_xy)` T<4 s | 84° (⊥) | **38°** |
| IC2 `e_R_yaw` max T<4 s | 1.00 rad | **0.55 rad** |

**The re-derived `alpha_0` was THE primary off-center blocker**, exactly as the GT-FB vs
perception comparison predicted: alpha −72° → −2° → `yaw_c` correct → V-frame lateral
command no longer rotated ~90° → achieved flow tracks the demand → `s_e_n` converges.
**First PRECISE perception-mode cross-marker landing (IC1 0.017 m).** IC2 went from a
4.9 m fly-off to a controlled 0.24 m landing.

**Residual (not zero):** IC2 `angle(h_xy,h_d_xy)` 38° not 0, `e_R_yaw` 0.55 not 0,
terminal `|a_u|` still spikes to 75 — alpha still degrades somewhat at oblique view /
near overfill, and the wall-clock `p_s` funnel + its re-arm are still live amplifiers.
IC2 is now a *converging* landing that needs tightening, not a diverging one.

**Next:** full **IC1–5 n=5 gate** (`run_ic_validation.sh`, perception mode) — mandatory
now that a control-path (alpha) default changed. Then revisit the open list (funnel #1,
terminal overfill #4) against the new baseline.
