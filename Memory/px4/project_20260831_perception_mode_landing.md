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
