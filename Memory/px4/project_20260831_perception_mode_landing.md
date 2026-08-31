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

### 2026-08-31 — IC1-5 n=5 perception-mode GATE (post CROSS_ALPHA_0 fix): off-center DECISIVELY fixed

`run_ic_validation.sh` N_REPS=5, `WORLD=cross_marker MARKER_TYPE=cross`, no GT-FB.
Bundle `test_data/ICValidation/20260831-144626/`.

| IC | prec/5 | mean xy | max xy | mean vel | read |
|----|--------|---------|--------|----------|------|
| IC1 | 1/5 | 1.12 | 5.36 | 0.20 | 2 good (0.069 P, 0.169); 1 terminal ASCENT (rep1, deck-clean at 0.06 m then loom→z-SMC climb to 7 m, →5.4 m); **2 test-rig IC-convergence FALSE-POSITIVE** (rep3/4: "IC converged t=0.40s pos_err=0.404 m" + alpha std 29° → descent-stall hover-abort at 6 m, `SP={}`) |
| IC2 | 3/5 | **0.110** | 0.188 | 0.50 | **all 5 landed 0.06–0.19 m** (was 100 % TARGET_LOST 4.9 m) |
| IC3 | 3/5 | **0.100** | 0.165 | 0.39 | **all 5 landed 0.03–0.17 m** (was 100 % TARGET_LOST 3.2 m) |
| IC4 | 3/5 | **0.101** | 0.218 | 0.36 | **all 5 landed 0.01–0.22 m** (was 100 % TARGET_LOST 4.0 m) |
| IC5 | 3/5 | 0.406 | 1.65 | 1.24 | 4 good (0.06–0.14 m); 1 hard bounce (rep1: deck at 0.23 m, then rel_vel 4.2 m/s, →1.6 m — fast 3 m descent, tdV2 timing) |

**VERDICT: CROSS_ALPHA_0 90.23°→0.58° is decisively validated.** IC2/IC3/IC4 — every
single one a 3–5 m TARGET_LOST fly-off before — now land at **mean xy ~0.10 m, max ~0.2 m,
soft, 3/5 precise each**. First working off-center perception-mode landing, ever.

**Remaining failures are PRE-EXISTING, not regressions:**
- **IC1 rep3/4 (2/5): test-rig IC-convergence false-positive** ("converged" at t=0.40 s /
  0.40 m err, alpha std 29°) → drone not actually at the IC → descent-stall hover-abort.
  Class of [[feedback_yaw_compass_drift_ic_start]]; intermittent; NOT alpha_0, NOT PLASMC.
  Needs the IC rig's convergence gate hardened (min settle time + tighter pos/alpha-std bar).
- **IC1 rep1 + IC5 rep1 (terminal): the overfill blocker** (open item #4). IC1 rep1 is the
  intermittent loom→z-SMC ascent (~1/10 seen in the A/B) — now exposed because the drone
  reaches the deck *centered* (0.06 m). IC5 rep1 is a fast-descent hard bounce.

**Open list, re-ranked against the new baseline:**
1. **Terminal overfill** (was #4, now #1) — the whole-cross detector dies as the marker
   overfills; below there the loop coasts on last-good s/h/IMU → loom inversion → z-SMC
   ascent (IC1 rep1) or hard bounce (IC5 rep1). Real fix: commit/hand-off to flow at
   overfill, or a bounded-size X-junction / dual-scale fiducial. This now gates the last
   ~0.1 m on the ICs that otherwise land clean.
2. **Test-rig IC-convergence gate** — false "converged" at 0.4 s. Harden it (rig only).
3. **Wall-clock `p_s` funnel + re-arm** — still a live amplifier (IC2 terminal `|a_u|` ~75,
   `angle(h,h_d)` 38° not 0); matters more now that landings converge.
4. `alpha` residual at oblique/overfill (`e_R_yaw` 0.55 not 0) — smaller now, revisit later.
5. `s_dot_meas` broadband floor — still deprioritised.

### 2026-08-31 — terminal-overfill perception failure, characterized (GT-FB + perception-mode data)

Traced perception `s_V` / moment-loom `h_V[2]` / a hand-computed flow-divergence loom vs
GT, through the terminal, on 3 bundles:
- `Multi_IC/20260828-110310/IC_2_rep_1` — **GT-FB**, lands 0.017 m (reaches deck clean).
- `ICValidation/20260831-144626/IC1_rep2` — perception mode, **PRECISE 0.069 m**.
- `ICValidation/20260831-144626/IC1_rep1` — perception mode, deck-clean @ 0.06 m then **ASCENT**.

**The moment-loom `h_V[2]` is the single point of failure, and it fails at the
`MARKER_EXTENT_PX` DE-saturation boundary — not at first overfill.**

- **alt 1.6→0.2 m** (`ext` ramps to 318 = frame_min by ~0.6 m, then pinned): moment-loom
  is merely BIASED — reads ~−0.3 to −0.4 vs GT ~−0.45 to −0.55 (~30 % under-read), right
  sign. Survivable: **IC1 rep2 landed PRECISE with exactly this biased loom.** GT-FB IC2's
  OWN moment-loom is identically pinned (~−0.4 flat, never tracks the terminal
  acceleration) — it lands 0.017 m *only* because it consumes GT loom, not perception.
- **alt ~0.18 m**: real `vz` briefly crosses zero (terminal float); GT loom → +0.1..+0.25;
  both perception looms follow correctly. Not the problem.
- **alt ~0.35 m, `ext` collapses 318→238→155**: **moment-loom SPIKES +1.1 → +3.6 → +5.6**
  while GT loom is only +0.6 — an ~8× over-read the instant the arms start leaving the
  frame edges and the visible area changes fast/discontinuously (`d(ln area)/dt` blows
  up). z-SMC (already fighting a real +0.7 ascent) sees +5 → slams up-thrust → runaway.
- Then `ext`→76, LK `npts` 165→27, detection `--`, `s_V` HELD, fly-off to 3–4 m.

**Flow-divergence loom (hand-computed from `Flow Points Prev/Curr Px`, median radial
expansion rate) is BETTER but not clean:** stays sane through the spike (~+0.75 vs
moment +5), but under-reads ~3× at altitude (~−0.15 vs GT −0.5), is noisy ±0.3
frame-to-frame, and also goes garbage once `npts` < ~40. Not a drop-in loom replacement.

**Centroid `s_V` degrades GRACEFULLY** — biased toward 0 below ~0.15 m (line-intersection
pulled to frame center by clipped arms) but no wild spike until detection fully collapses
(which the ascent itself triggers). Lateral can coast on last-good `s_e_n` + commit.

**Hand-off design, now data-grounded:**
- **Trigger:** `MARKER_EXTENT_PX >= frame_min` sustained ~N frames. Fires ~alt 0.6 m,
  ~1.5 s before the loom spike. Scale-free (frame fraction, no Z).
- **z-axis:** on trigger, STOP closing the vertical loop on vision (neither moment nor
  flow-div loom is reliable). Commit a **fixed gentle open-loop descent** (`h_z_d` =
  small negative const ≈ funnel-floor rate) for the last ~0.3 m — drone is centered
  (`s_e_n` ~0.05) and ~0.6 m up; ~0.15 m/s lands in ~4 s with no loom feedback. tdV2
  OVERFILL / FLOW-FREEZE catches touchdown.
- **lateral:** freeze/commit `s_e_n` (already ~0), let `sigma_xy = zeta_h(h_e)` on
  measured `h` damp residual drift — the existing `_terminal_commit` machinery.
- i.e. **at overfill: stop closing the vertical loop on vision; commit descent +
  flow-damp lateral to touchdown.** Bounded-size fiducial remains the cleaner long-term fix.

### 2026-08-31 — CORRECTION + user design-rationale: the CENTROID is fine; the loom is the failure

(Amends the entry above — "moment-loom" wording was imprecise, and a nested-fiducial
suggestion was WRONG.)

**User (design rationale, agreed):** the cross+stub was chosen SO THAT even if the marker
drifts near touchdown and only *partial* segments of two arms are visible, their
intersection still gives the image centroid (incl. the off-frame `in_fov=False`
extrapolation path). A nested / dual-scale fiducial does NOT have that property — it
needs a whole sub-marker in view. **Do not propose replacing the cross marker.**

**The data confirms this:** perception `s_V` (centroid) degraded GRACEFULLY through the
whole terminal — biased toward 0 below ~0.15 m (clipped arms pull the LS line fit toward
frame center) but NO spike, tracking GT within ~0.1 until detection fully collapsed
(which the ascent itself caused). The cross's partial-intersection design did its job.

**The failure is the loom `h_z`** — and it is NOT a `d(ln area)/dt` moment loom; it's the
**LK image-Jacobian least-squares solve** (`_fill_A` Tz column, `_solve_jacobian`).
`_fill_A`'s Tz/Wz columns are POSITION-WEIGHTED (`[-x,-y]` / `[-y,x]`), so they need
tracked points spread across a range of radial positions. At overfill the points
cluster + `n_flow_corners` collapses (165→27 here) → the Tz column is ill-conditioned →
the lstsq `h_z` becomes a large-variance garbage number → the +5 spike (GT +0.6). Same
root cause as the documented Hz/Wz cal weakness (position-weighted columns, radial-spread
starved), pushed to its extreme. There is already a partial mitigation
(`CROSS_RING_SAMPLING`, `CROSS_MOMENT_LOOM_MIN_PTS=6`, 2026-08-14) that targeted the
`ext≈210-220 px` regime; it is not enough at `ext=318` (full overfill).

**Fixes that KEEP the cross marker:**
1. **Robust loom terminally** — replace/blend `h_z` at overfill with the MEDIAN radial
   expansion ratio of the tracked cloud (`−½·median(r_curr/r_prev)/dt`), which is robust
   to the conditioning collapse: hand-computed here it stayed +0.75 (vs lstsq +5) through
   the spike. Noisy ±0.3 and ~3× under-reading at altitude, so terminal-only.
2. **Or open-loop commit descent** at `ext`-saturation (`MARKER_EXTENT_PX >= frame_min`
   sustained), skip the vertical vision loop for the last ~0.3 m — drone is centered,
   ~0.6 m up, ~0.15 m/s commit lands in ~4 s, tdV2 catches touchdown.
3. **Or a pairwise-distance loom** on robustly-tracked cross features (junction ↔ arm
   tip), immune to one arm being clipped.
None need a marker change. (2) is the cheapest; (1)/(3) keep a real loom.

### 2026-08-31 — IC5 montage-run fly-off diagnosed: terminal LATERAL overshoot, NOT overfill

Bundle `test_data/Landing_Test/Mon Aug 31 19-19-48 2026/`; montage
`montage_IC5_alpha0_20260831.mp4`; onboard `Mon Aug 31 19-19-34 2026.mp4`. n=1 IC5
(2,2,3), post-α₀. TARGET_LOST, endpoint 6.4 m @ 7.8 m/s.

**It is a terminal lateral-tracking loss, not the h_z overfill spike.** Evidence:
- **GT: crosses the target then drifts away, monotonically.** pos (+2.1,+1.8) → (−0.1,+0.7)
  xy 0.76 m at alt 1.87 m (~converged) → (−0.6,+0.6) → (−0.8,+0.4) → (−1.1,−0.4) xy 1.18 m
  at alt 0.63 m → runs off in −y to (+0.5,−6.3). Never re-converges after the crossing.
- **Raw pixel centroid slides off the frame BOTTOM:** `Center Px` (rotated 240×320, centre
  (120,160)) y = 233 @ alt 1.5 m → 265 @ 1.1 m → 289 @ 0.8 m → (203,324) off-bottom @
  0.6 m → (257,333) fully off-frame @ 0.5 m (extrapolated `in_fov=False`).
- **Onboard frame @ alt 0.8 m = EMPTY** — uniform grey + the two leg "ghost" shapes at the
  side edges; no cross marker anywhere. Camera has fully drifted off the target.
- `MARKER_EXTENT_PX` only ~238 (NOT the 318 full-overfill) — the marker didn't overfill,
  it walked out of frame.
- `|s_e_n|` never nulls terminally: ~0.40 @ alt 2 m → 0.6 @ 0.9 m → 0.8 → 1.05 → 1.17 →
  **1.51** @ 0.59 m; `|a_u lat|` tracks it up 0.9 → 7.1 (wall-clock `p_s` funnel breakout).
- THEN: junction off-frame → `lt2_angle_clusters`/`centroid_mismatch` → `det=miss` @ alt
  0.55 m, `s_e_n` frozen ~1.2; `n_flow_corners` 125→50→14→0; `h_z` LK-Jacobian spike to
  +1.5; frozen-large `s_e_n` + up-loom → fly-off. (The h_z spike is real but it is
  *finishing an already-lost approach*, not initiating it — unlike IC1 rep1 which reached
  the deck centred at 0.06 m then ascended on a pure loom spike.)
- Descent was also aggressive for the 3 m budget: `h_d_z` pinned −0.30, achieved `vz`
  ~−1 to −1.5 m/s at alt 1–1.5 m (a GT `vz` −2.2 spike likely bridge jitter) — eats the
  altitude before the lateral residual closes.

**Root cause (IC5-specific severity):** the off-center lateral loop closes ~90 % of the
error (α₀ fix) but **does not null the terminal residual (~0.4 → diverges)**, and IC5's
3 m start compresses the descent so there is no recovery time — the drone lands onto
empty ground ~1.2 m off, marker exits frame, detection collapses, fly-off. Same wall-clock
`p_s` funnel + terminal-lateral-authority problem as open item #3, just most exposed at
IC5. NOT a new mechanism; NOT the α₀ fix; the h_z overfill spike (#1) is a downstream
compounding factor here, not the trigger.

**Implication for the fix order:** the terminal `p_s` funnel / lateral-authority fix
(#3) matters as much as the h_z hand-off (#1) — IC5 (and IC3's 0.30 m endpoint, IC2's
38° h/h_d angle) are all the un-nulled terminal residual, not overfill.

### 2026-08-31 — IC5 perception vs GT-FB: s_e_n diverges because the middle SMC gain κ_xy can't ramp

Compared PERC IC5 (`Mon Aug 31 19-19-48 2026`, fly-off) vs GT-FB IC5
(`Multi_IC/20260828-110310/IC_5_rep_1`, lands 0.017 m). Same lateral/CBF/funnel machinery
(GT-FB bundle is pre-`_dtheta_correction`-removal — minor).

| signal (alt 1.85 → 0.6 m) | GT-FB IC5 | PERC IC5 |
|---|---|---|
| `\|s_e_n\|` | 0.35 → 0.06, monotone | 0.38 → rebounds → **1.36** |
| `\|h_e\|` (flow error `h−h_d`) | 0.03–0.20 | 0.29 → **2.4** |
| measured `h_x` vs GT `h_x` | exact (injected) | ~matches GT (+0.3 vs +0.4) — **NOT ⊥ anymore**, α₀ fix worked |
| demand `h_d_x` | small, matched | −0.25 restoring, **correct but not delivered** |
| `kappa_xy` | ~0.2 (never needs to ramp) | **~0.2, DECLINING to 0.16** (can't ramp) |
| `theta_current` (tilt) | 2° → 0.1° | 3° → **24.5°** |
| CBF `theta_cone` | 1–2° (inactive) | 4° → 38° (engaging, not hard-clamping) |
| `theta_desired` vs `theta_current` | — | ≈ equal (~28°) — drone gets the tilt it asks |
| `az_joint_delta` (CBF_AZ relief) | — | **0.00, never engaged** |
| `S_r = s_e_n/p_r` | 0.01 | **0.16** (`p_r`≈10, never binds; `S_MARGIN` clip nowhere near) |

**Reason for `s_e_n` divergence:** the drone has a real ~+0.3 lateral optical flow that
needs reversing; `h_d` correctly demands −0.25; `h_e ≈ 0.55` and GROWING. Rejecting that
needs the middle adaptive-SMC gain `κ_xy` to ramp. **It doesn't** — it sits at ~0.2 (near
`κ_0`) the whole descent. Three reasons, all in the κ-ODE `dκ/dt = Θ·N·G·|σ| − N·P·κ`:
1. `τ_κ = 1/(N·P) = 1/(0.1·2.5) ≈ 4 s` vs IC5's ~4–5 s descent → **adaptation-rate-limited**
   (the controller.py:406 comment's own concern, for the 3 m profile).
2. `P_xy = 2.5` leakage (baked up 1.5→2.5, 2026-07-22) bounds `κ_eq = Θ·G·|σ|/P` LOW.
3. `p_inf_xy = 2.5` (middle funnel floor, rebaked 2026-08-28 from 1.0) keeps `zeta_h`
   small → weak κ growth drive. That rebake's comment says it was traced on an **IC2
   GT-FB n=5 A/B** — "s_e_n was small and still CONVERGING... the funnel getting tight,
   not the error growing, triggered every breach. 2.5 gave 5/5 CLEAN."

**So the middle SMC is deliberately detuned (slow N, high leakage P, loose funnel) to
avoid the terminal κ-ratchet / actuator-wall blowup — and that detuning was validated
under GT-FEEDBACK, where the lateral disturbance `h_e` is ~0.05 and κ never needs to
ramp.** Under perception `h_e` is ~0.5 (perception `h`/`s_dot_meas` noise + a sluggish
outer loop leaving a residual `s_e_n`≈0.38), and the detuned κ can't reject it in the IC5
time budget → `s_e_n` runs away → position-barrier transform blows `a_u` to 6+ too late →
marker off-frame → fly-off.

**Clamps/limits checked — what is NOT stopping convergence:** `kappa_max=30` (κ never
approaches it), `S_MARGIN=0.05` clip on `S_r`/`S_s` (`p_r`≈10, funnel never binds),
`PLASMC_DSD_LAT_MAX=100` (off), the CBF FoV cone (`theta_cone` engages but
`theta_current`≈`theta_desired`, ~28° delivered — not hard-clamped), `az_joint_delta`
(never fired), `PLASMC_AU_MAX_XY` (a_u reaches ~6, not near a cap). **What IS limiting:**
the κ-ODE leakage/rate tuning itself — a soft, structural limit on the adaptive gain,
not a hard clip.

**Implication:** the perception-mode terminal needs the middle κ-loop to be able to
develop lateral authority against a perception-scale `h_e` WITHOUT the terminal ratchet
blowup — i.e. the "one knob one job" P-vs-E / velocity-damping direction
([[feedback_dont_conclude_lag_floor]], [[feedback_terminal_smc_actuator_wall]]) has to be
revisited for the perception regime, not just GT-FB. This is the same family as open #3
(terminal lateral authority); the CBF and the funnels are NOT the blocker here.

### 2026-08-31 — IC5 CBF analysis CORRECTED + a_u component that fell short

**Earlier CBF read ("cone engages but delivers what's demanded") was WRONG.** Compared
`I_a_raw[:2]` (pre-`cbf2_filter`) vs `I_a[:2]` (post) — magnitude AND direction — for the
PERC IC5 fly-off (`Mon Aug 31 19-19-48 2026`) vs GT-FB IC5.

| alt | `\|Iar_xy\|` | `\|Ia_xy\|` | angle(Iar, Ia) | `dtheta_az` (rad) | `theta_safe` |
|---|---|---|---|---|---|
| **PERC**, 1.2 m | 0.63 | 1.23 | 14° | 0.71 | 43° |
| **PERC**, 1.0 m | 0.64 | 1.73 | **83°** | 0.15 | 8.5° |
| **PERC**, 0.97 m | 1.51 | 1.68 | **179°** | 0.01 | 7.9° |
| **PERC**, 0.9–0.6 m | 1.5–6.0 | 3.5–6.2 | **90–138°** | 0.36→0.85 | 19–38° |
| **GT-FB**, all alts | 0.2–0.47 | ≈same | **<10°** | ≈0 | 0.1–2.6° |

**The CBF joint QP does NOT merely cap the tilt — once `s_e_n` grows past ~0.5 and the
marker nears the FoV box (alt ~1.2 m), it ROTATES the lateral acceleration 90–180° away
from the SMC's restoring direction (to a keep-visible direction) and AMPLIFIES it 2–4×.**
The `I_a[:2]` actually delivered to the drone (magnitude 3–6) therefore does NOT reduce
`s_e_n` — it points the wrong way → `s_e_n` grows → CBF redirects harder (`dtheta_az`
0.7→0.85 rad = 40→49°) → runaway. A visibility-conflict feedback loop. In GT-FB the
CBF is inert (marker stays centred, `angle(Iar,Ia)` <10°, `dtheta_az` ≈0) — there is no
conflict to trigger it.

**Which a_u component fell short** (`AU_DECOMP_DBG=1` re-run of PERC IC5;
`a_u = reach + switch + equiv + drift`):

| phase (`\|s_e_n\|`) | reach (Γσ) | switch (κ) | equiv (c−Sṗ) | drift (Ωζ) | κ_xy |
|---|---|---|---|---|---|
| approach 0.59 | 0.6 | 0.2 | 0.6 | 0.2 | 0.70 |
| terminal 0.57–0.76 | 0.4 (flat) | 0.3 → **2.1** | 1.0 → **5.4** | 0.3 → 2.8 | 0.24 → 0.44 |
| GT-FB (s_e_n ≤0.35) | ~0.005 | ~0.005 | small | small | 0.2 (idle) |

- **`switch` (adaptive-κ switching) is the term that should grow to reject a disturbance
  — it stayed small (0.2 → 2.1)** because `κ_xy` never ramped (0.24 → 0.44 over the whole
  descent; max is 30). Root cause is the κ-ODE tuning from the prior entry
  (τ_κ≈4 s > IC5 descent, P_xy=2.5 leakage, p_inf_xy=2.5 loose funnel).
- **`reach` (fixed Γ_xy = 0.25 proportional) is structurally too small (~0.4)** to carry
  the restoring load alone once `switch` is starved.
- **`equiv` (feed-forward: loom, ḣ_d, cross(w,s)) ballooned to 5.4** — the *dominant*
  terminal component — but that is a SYMPTOM, not authority: `s_e_n` diverging → `hd_rate`
  thrashes → `dh_d`/`c` large → `equiv` large. It amplifies the instability.

**Net:** the error-driven feedback authority `reach + switch ≈ 0.4 + 1–2` was inadequate
to null a perception-scale `s_e_n`, because the one component that scales with a
disturbance (`switch`, via κ) is rate-/leakage-limited. Then the CBF QP took that
inadequate command and rotated it 90–180° off the restoring axis. GT-FB never exercises
either failure because `s_e_n` stays <0.35 and every term stays ~0.

### 2026-08-31 — CBF frame conventions validated: axes OK, but un-leveled cr2 + dft/L_ω regime is the fault

Checked the CBF (`cbf_visibility.py::cbf2_filter`) frame handling against
`_getVirtualPts` and the corrected α₀, on the IC5 fly-off state.

**1. Axis convention — CONSISTENT (no rotation/sign bug).** Computed `_getVirtualPts`'s
`s_V` and the CBF's `cr2` on the SAME raw centroid pixels through the IC5 descent. Both
use the `[y,−x]` swap; at altitude they agree to ~0.07 (`s_V` (+0.41,+0.18) vs `cr2`
(+0.48,+0.12) at alt 1.5 m). No 90° rotation, no mirror. `yaw_c` (from α₀, now correct)
feeds the CBF's `Rzm` consistently. `validate_cbf.py` 12/12, incl. test 6 "conventions
(Rz round-trip)".

**2. The CBF is UN-LEVELED; the SMC / `m2` are LEVELED — they disagree on the off-center
distance at high tilt.** `_getVirtualPts` applies `C_R_V` (removes roll/pitch); the CBF's
`cr2` is the raw `[y,−x]` centroid, un-leveled (by design — "real camera plane" FoV). At
5° tilt `s_V ≈ cr2`. At 24° tilt (IC5 terminal) **`s_V_y = −1.10` (leveled, what `s_e_n`
sees) vs `cr2_y = −0.55` (un-leveled, what the CBF sees)** — the CBF reads ~HALF the
off-center distance. Meanwhile `m2` (the FoV half-extent it compares against) IS the
leveled `p_10`. So the CBF constrains an un-leveled position against a leveled bound
while the SMC acts on a leveled error — three-way inconsistent once the drone tilts hard.

**3. `dft` (drift look-ahead) is what fires the QP; `L_ω` geometry is what rotates it.**
`anchor = cr2 − Lw2@th_curr + dft`, `dft = CBF_TAU(0.3) · EMA(d)`, `d` = centroid drift
rate. In the terminal the marker drifts fast (`Center Px` y 251→323 in ~0.6 s) → `d`
large → `dft_y ≈ −0.3+` → `f_y` pushed past `−m2_y` even though `cr2_y` alone is still
inside the frame. Once the box is "violated", the projection moves `Ia_lat` along
`M[k]` = k-th row of `L_ω@M_rot@P`; with the centroid at `x2 ≈ 1.0–1.2` the `−(1+x2²)`
term dominates `L_ω[0]` (norm 1.0 centred → ~3 here) → `M[0]` points ~85° off the radial
restoring direction and its large norm makes the projection OVERSHOOT and flip the
command sign (measured `angle(Iar, Ia)` 90–179°, `|Ia|` 2–4× `|Iar|`). Self-reinforcing:
flipped `Ia` → more drift → bigger `dft` → QP fires harder.

**4. The validator does NOT cover this regime.** `test_no_strangle` uses `th_des` = 1–7°
(IC5: 24–35°), `dt=None` → `dft ≡ 0` (IC5: large `dft`), single-step (no EMA `d` built
up), `R` clipped to 17° tilt. `test_lw_fidelity` is explicitly "near-hover tilt"
(±4.6°). So "12/12 pass" says nothing about off-center + high-tilt + drifting — exactly
where the CBF misbehaves.

**Verdict:** no clean frame/sign bug — the axes and `yaw_c` check out. But the CBF (a) is
frame-inconsistent with the SMC once the drone tilts (un-leveled `cr2` vs leveled
`s_e_n`/`m2`), and (b) in the off-center + fast-drift regime its `dft` + `L_ω`-geometry
combine to produce a ~90°-rotated, magnitude-amplifying "correction" that fights the
restoring command instead of gracefully degrading. Root cause is still upstream (marker
should never drift that far — the middle κ), but the CBF actively worsens it once there,
and its passing validator is misleadingly scoped.

### 2026-08-31 — CBF: `dft` drift-lookahead is the term that flips the command (isolated re-run)

User clarification accepted: SMC on the VIRTUAL (leveled) plane, CBF on the ACTUAL image
plane — by design, not a bug; the two planes are meant to differ under tilt. Re-ran
`cbf2_filter` in isolation on the reconstructed IC5 fly-off state, with real `dft` vs
`dft` forced to 0 (`CBF_TAU=0`):

| alt | `dft` (x,y) | `I_a` in | `I_a` out (real dft) | angle | `I_a` out (**dft=0**) | angle |
|---|---|---|---|---|---|---|
| 1.49–1.30 | ~0 | (0.3, 0.6) | ≈ unchanged | **0°** | ≈ unchanged | **0°** |
| 1.20 | **(+0.63, +0.17)** | (−0.12, +0.62) | (−1.17, +1.28) | **31°** | (−0.12, +0.62) | **0°** |
| 1.11 | (+0.57, +0.35) | (+0.42, +0.51) | (−0.62, +1.67) | **60°** | unchanged | **0°** |
| 0.96 | (+0.72, +0.22) | (+0.21, −1.82) | (−2.79, +1.64) | **127°** | (−0.01, −1.58) | **7°** |
| 0.85 | (+0.65, −0.11) | (+0.81, −0.27) | (−1.75, +2.76) | **141°** | (+0.52, +0.09) | 28° |
| 0.64 | (+0.80, **−2.34**) | (+4.57, −1.23) | (+3.12, +6.44) | 79° | (+2.83, +3.01) | 62° |

**`dft` is the primary cause of the CBF flipping the restoring command.** With `dft=0`
the CBF is a clean **passthrough (angle 0°) down to alt ~1 m** — the no-strangle property
holds, the QP + `L_ω` handling is fine for a restoring command. Only in the DEEP terminal
(alt <0.9 m, centroid genuinely at the FoV edge) does it rotate 20–60° — that residual is
the `(1+x²)` `L_ω`-geometry effect, and it's mild.

**Why `dft` breaks it:** `dft = CBF_TAU(0.3 s) · EMA(d)`, `d` = measured centroid drift
rate (de-rotated). As `s_e_n` diverges the drift rate grows to several units/s → `dft`
components reach 0.6–2.3, which ALONE exceed `p_10 ≈ [1.19, 0.89]` → the box is
"violated no matter what `I_a` does" → the QP does a maximal projection along the skewed
`L_ω` gradient `M[k]` → overshoots and flips the command. The flipped command increases
the drift → `d` bigger → `dft` bigger → **positive feedback**. `dft` linearly
extrapolates a drift rate that (a) the CBF's own action is supposed to reduce but instead
increases, and (b) is actually accelerating. (`CBF_HZ_AWARE_DRIFT`, default off, makes
`dft` LARGER — do not enable it here.)

**Fix candidates (CBF side):** cap `|dft|` at a fraction of `p_10`; clamp/rate-limit the
`d` estimate; shrink `CBF_TAU` terminally; or gate `dft` on "drift toward edge AND
`s_e_n` small" (trust the look-ahead only when the loop is otherwise healthy). Root cause
still upstream (middle κ lets the marker drift there), but `dft` converts a recoverable
drift into an unrecoverable fly-off.

### 2026-08-31 — CONFIRMED: 90° frame inconsistency between SMC and CBF (Rz_p90b)

The user was right. Validated with clean synthetic tests (level drone, yaw_c=0).

**SMC / `_getVirtualPts` frame:** raw pixel `(x,y)` → `[y,−x]` swap = **`Rz(−90°)`** →
gravity-leveled V. `s_e_n`, `a_u`, `I_a` all inherit this `Rz(−90°)`-swapped leveled
frame. The `[y,−x]` swap is validated (marker world-yaw=0; `s_e_n` diverged with the
`[−y,x]` version).

**CBF centroid `cr2`:** `ct = (x,y)` → `[ct[1], −ct[0]]` = `[y,−x]` = **`Rz(−90°)`** —
MATCHES the SMC.

**CBF tilt `th` / `th_desired`:** `I_a` → `Rzm` (de-yaw) → `Rz_p90b =
[[0,−1],[1,0]] =` **`Rz(+90°)`** — OPPOSITE rotational sense to the centroid swap.
Empirically **`angle(th_desired, −cr2) = 90°` EXACTLY** for every marker offset — the
CBF's tilt reading of the SMC's restoring `I_a` is 90° off from the direction that
re-centers the marker.

**Why the validator misses it:** the 90° largely CANCELS inside the box math —
`CBF_LW_ROT` right-multiplies `L_ω` by `[[0,1],[−1,0]] = Rz(−90°)`, undoing the `Rz(+90°)`
in `th`, so `Lw2@th` is ~correct (`validate_cbf.py` test 1 = 6.27% near-hover). `test 6
"conventions"` only checks `Rz(yaw)∘Rz(−yaw)=I`, never `th` vs `cr2`. So with `dft=0` the
CBF is a clean passthrough of a restoring command (synthetic test: ≤23° rotation even
with the marker past the edge — that residual is the `(1+x²)` `L_ω` geometry).

**Where the 90° leaks out: `dft`.** `dft = CBF_TAU·d` is measured/added in the `cr2`
(`Rz(−90°)`) frame; it enters `f = anchor + Lw2@Ia_lat` whose Jacobian carries the
mismatched-frame `L_ω`, so a `dft`-triggered box violation gets corrected ~90° off.
Confirmed: rotation scales linearly with `CBF_TAU` (0.0→0°, 0.15→17–34°, 0.3→42–77°).
Flipping `Rz_p90b`→`Rz(−90°)` roughly halves the rotation for diagonal markers
(33°→0°, 34°→4° at tau=0.15) — but not axis-aligned (breaks the `Lw2@th` cancellation:
`Rz(−90°)∘Rz(−90°) = Rz(−180°)` = sign flip on the coupling). So the fix is a COUPLED
convention rework (`Rz_p90b` + `CBF_LW_ROT` + possibly the `[y,−x]` corner swap in
lockstep), not a one-line flip, PLUS the `dft` over-aggression cap.

**Net:** genuine 90° SMC↔CBF frame inconsistency in `cbf_visibility.py`'s `Rz_p90b`,
masked in the box math by a compensating `CBF_LW_ROT` rotation (so validator-clean and
`dft=0`-clean), surfacing as the `dft`-triggered lateral-command rotation that turns an
off-center drift into a fly-off. Two coupled fixes needed: (1) make the CBF tilt frame
consistent with the `Rz(−90°)` `[y,−x]` convention used by `_getVirtualPts`/`cr2`
(rework `Rz_p90b`+`CBF_LW_ROT` together, re-validate `Lw2` fidelity); (2) cap/rate-limit
`dft`.
