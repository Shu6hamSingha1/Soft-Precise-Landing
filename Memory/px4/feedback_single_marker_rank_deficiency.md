---
name: feedback_single_marker_rank_deficiency
description: "⭐ (2026-06-22) SINGLE-MARKER architecture (PLASMC_SINGLE_MARKER, default-off) to kill nested-ArUco primary SWITCHING (root of terminal loom spikes → vz launches): lock to largest-spread marker (re-lock only when gone), dense flow, visibility-MARGIN ring-switch (flow→ring, centroid→KLT). CORRECTLY WIRED but DOES NOT WIN — bottlenecked by LATERAL RANK-DEFICIENCY. A single near-axial marker has cond 15-60 (p90 270-471) vs multi-marker ~11 (set by spatial SPREAD ~s_phys/Z, NOT point count). The MOMENT loom (FLOW_LOOM_DECOUPLE) sidesteps the pinv rank-deficiency for the LOOM (−½d(lnM)/dt = direct area-rate scalar, NO matrix inversion → loom continuous |Δh_z|@switch~0) — but lateral h_x,h_y is STILL pinv rank-deficient → drift → 1/Z → vz launch. n=5: single+moment 1/5 vert 0/5 sub vs momentsz (multi-marker moment loom) 0/5 vert 2/5 sub → MOMENTSZ WINS. The lateral disadvantage is a SCALE BIAS not jitter (single lat_noise 0.126 < momentsz 0.212; dense pts average noise). CONVERGENCE: multi-marker + size-norm moment loom (FLOW_LOOM_DECOUPLE) is the vertical-channel fix (conditioning AND switch-handling); single-marker trades a solved problem (switching) for a harder one (rank-deficiency). OPEN: might work on a DEDICATED single-LARGE-marker world (well-conditioned at close range) = Gazebo SDF change, untested."
metadata:
  node_type: memory
  type: feedback
  originSessionId: eba9fa95-5b93-4294-bbca-81468bb36670
---

**Motivation.** The nested-ArUco board's PRIMARY marker (min-ID) SWITCHES as concentric markers
overflow the FoV → the corner spread jumps → the pinv loom spikes (|Δh_z| up to 4.4 in one frame) →
terminal VERTICAL LAUNCH. Single-marker eliminates the switching by locking to ONE marker.

**Architecture (PLASMC_SINGLE_MARKER, default-off; all correctly wired + compiles).**
- **Lock** (no min-ID flicker): pick the LARGEST-spread decoded marker; re-lock ONLY when the locked
  one disappears (`_locked_marker_id`). On the nested board the locked marker still overflows at
  terminal → one re-lock (not per-frame flicker).
- **Dense flow**: GFT (texture corners inside the marker polygon — 2026-06-22, switched FROM the
  baseline scaled-quad `_scaled_quad_points` port, which sits on EDGES → aperture-problem LK noise;
  GFT corners are 2D-trackable, lower input noise). scaled-quad kept as GFT empty-fallback + KLT bridge.
- **Visibility-MARGIN ring-switch** (`PLASMC_MARKER_FOV_MARGIN=40` px): when any corner comes within
  40px of the FoV edge → `_marker_leaving` → FLOW routes to the RING (`corner_ok=False` via the EXISTING
  mechanism — `if corner_ok: …else ring carries ALL 6 components`), but the CENTROID KEEPS the corners
  (KLT) — the 1st moment is robust to near-edge corners, the derivative flow is not. Only the flow
  switches to rings, not the position.
- The detection-vs-visibility classifier IS the existing KLT `_in_bounds` check (DETECTED=decode /
  VISIBLE=decode-fail+KLT-in-bounds / NOT-VISIBLE=KLT-out-of-bounds). A separate `_marker_visible`
  flag was tried then REMOVED as REDUNDANT with `corner_ok` (corner_ok=False already → ring-all); the
  MARGIN is what makes visibility distinct (switch a few frames BEFORE full exit, off cleaner corners).

**THE RANK-DEFICIENCY (the binding limit; quantified).** A single near-axial marker's interaction
matrix `L_s` is degenerate: `cond ≈ 15-60` (p90 270-471) vs multi-marker `~11`. Set by the spatial
SPREAD (`≈ s_phys/Z`, the normalized corner extent), NOT the point count — the `vz` column `[−x,−y]`
AND the `wx/wy↔vx/vy` confounding both vanish at small `x,y`. Afflicts ALL flow columns (lateral
`h_x,h_y` AND loom `h_z`). Extending the spread is the ONLY fix; a single marker can't (bounded by its
own size). `loom_noise = input_LK_noise / σ_min²`.

**WHY THE BOARD IS NESTED (geometric).** No single marker fills the FoV across the 10m→0.4m descent
(>25×, ~4.5 octaves; image fraction = `f·s_phys/(Z·W)`). A 1m marker: ~4% FoV at 10m (rank-deficient),
42% at 1m (good), overflows at ~0.42m (gone) → well-conditioned only in ~1 octave. The nested
MULTI-SCALE markers TILE the range (big fills FoV at altitude, small near deck). That is the board's
purpose, and it's real.

**THE MOMENT LOOM SIDESTEPS THE RANK-DEFICIENCY (key insight, holds).** `FLOW_LOOM_DECOUPLE` computes
the loom as `−½·d(lnM)/dt` — a DIRECT area-rate scalar (fractional 2nd-moment expansion), NO
`pinv(L_s)`, no `σ_min`. So the rank-deficiency CANNOT appear in it. On a locked single marker it's
also switch-free + ID/size-independent (same marker → `s` cancels in `M₁/M₀`). VERIFIED: loom
continuous across switches `|Δh_z|@switch ≈ 0` (vs pinv loom 4.4). **BUT it only replaces `h_z`** —
the LATERAL flow `h_x,h_y` is still pinv rank-deficient.

**RESULTS (IC2, n=5 unless noted; user: n=5 enough).**
- pinv-loom single-marker (scaled-quad): vz launches PERSIST (vz_max 6-13, 1/8 sub) = rank-deficiency
  noise; dense engaged (Nfc~184), switch spikes reduced (lock works) — but outcome poor.
- single + MOMENT loom: loom now CONTINUOUS (moment loom works) but vz launches STILL persist
  (1/5 vert, 0/5 sub) — the failure MOVED from loom to LATERAL channel (drift → 1/Z → vz launch).
- **momentsz (multi-marker moment loom, current best): 0/5 vert, 0/5 fly, 2/5 sub → WINS.**
- The single-marker lateral disadvantage is a SCALE BIAS, NOT jitter: single lat_noise 0.126 < momentsz
  0.212 (the 180 dense points average noise DOWN) → dense averaging can't fix it; the rank-deficiency
  mis-attributes flow between confounded columns = systematic error.

**CONVERGENCE / VERDICT.** The multi-marker board + size-normalized corner MOMENT loom
(`FLOW_LOOM_DECOUPLE`) keeps conditioning (multi-marker spread) AND handles switches (`M/sz²`) —
achieving everything single-marker tried, without the rank-deficiency cost. Single-marker TRADES a
solved problem (switching, already fixed by `M/sz²`) for a HARDER one (lateral rank-deficiency).
**`FLOW_LOOM_DECOUPLE` is the bakeable vertical-channel fix** (vert 5/15→1/15 at n=15, 0/5 here, loom
continuous |Δh_z| 4.4→0.4) — pending the IC2-5 gate. Keep single-marker DEFAULT-OFF.

**RESOLVED 2026-06-23 — SINGLE-LARGE-MARKER WORLD WORKS (the nested-board verdict was BOARD-SPECIFIC,
not single-marker-fundamental).** Switched the world to a single ~1m ArUco (arucotag.png id 0, plane
1.0m; `~/PX4-Autopilot/.../models/arucotag/model.sdf`, backup .bak_board). RE-CALIBRATED in-world
(`PLASMC_SINGLE_MARKER=1`, 5 phased runs, derive_board_cal+derive_ring_cal): lateral fit **R² Hx 0.63
Hy 0.62** — the LARGE marker is WELL-CONDITIONED (vs the nested-board single-marker collapse 0.07),
because conditioning ∝ angular extent `s_phys/Z` and a 1m marker subtends a wide angle at close range
(the off-axis x²/y²/xy terms that separate vz/wx/wy from vx/vy are order-1). Cal pasted to img_data.py
(board cal preserved in .pre_singlemarker_cal_bak + commented; cal is now single-marker-specific).
**n=5 MATCHED-cal landing: 2/5 sub-meter (0.34, 0.38m), 100% decode, Nfc→184** — ESCAPES the
nested-board dead-end (0/5 sub) and reaches PARITY with multi-marker momentsz (also 2/5). The lateral
rank-deficiency WALL IS BROKEN. REMAINING (shared with multi-marker, NOT single-marker-specific): vz
launches 4/5 (vz_max 2.5-4.7, terminal vertical channel) + 1/5 stochastic catastrophic fly-away (19m,
std 7.16) = the session-long terminal ceiling. So single-marker is now a VIABLE architecture (was a
dead-end on the board). Smoke test (mismatched board cal) had already hinted it: 0.08/1.56m.
**REFINEMENT (optional):** high-altitude Nfc=0 (1m marker ~54px at 5m → 180 scaled-quad points pack
~3px apart → dense LK degenerates) — centroid carries the altitude phase, flow takes over at close
range (s_phys/Z grows); scale dense-point density to marker size to recover altitude flow.

**⛔ CENTROID-RATE OBSERVER = DEAD-END + ALTITUDE-STARVATION MISDIAGNOSIS CORRECTED (2026-06-23).**
Implemented a gyro-compensated centroid-rate observer (PLASMC_CENTROID_RATE, default-off) to fill the
supposed altitude velocity gap: h_x=ṡ_x+x0·h_z+y0·wz, h_y=ṡ_y+y0·h_z−x0·wz (ṡ from DECODED-corner
centroid, wz gyro, h_z moment loom; wx/wy dropped because the V-frame already levels roll/pitch — that
frame fix was correct but minor). REGRESSED: IC2 observer 1/5 sub vs baseline 3/5; the observer flow is
6.2x→5.3x INFLATED and corr 0.05 (NOISE) — differentiating the decoded centroid amplifies sub-pixel
decode jitter; fundamentally noisier than LK flow (which tracks actual displacement, not a difference of
two noisy detections). **The PREMISE was WRONG**: the baseline ALTITUDE FLOW IS FINE (alt>3m corr
0.39-0.59, ratio 1.1-1.4x) — the KF + flow-recovering-by-~3-4m bridge the top-of-descent Nfc=0; the
consumed velocity was NEVER starved. So Nfc=0 at the very top is real but INERT (KF bridges it). **The
IC2 fly-aways are STOCHASTIC/terminal (1-2/5 run-to-run), the SAME ceiling as the multi-marker board —
NOT altitude velocity starvation, NOT fixable by a velocity observer.** Observer stays default-off
(dead code, documented). DON'T re-chase: centroid-rate-as-velocity (decode-jitter noise), altitude-
starvation-as-fly-away-cause (baseline flow OK). Single-marker = ~3/5 sub at IC2 w/ stochastic terminal
fly-aways = PARITY w/ multi-marker; the binding limit is the shared terminal stochasticity.

**⛔ IC2-5 GATE FAILED (2026-06-23) — single-marker BAKE is PROVISIONAL, NOT committed.** Baked
PLASMC_SINGLE_MARKER=1 + FLOW_LOOM_DECOUPLE=1 default-on then ran run_ic_validation (IC2-5 ×3):
**2/12 sub-meter, 6/12 fly>3 (incl 25m IC3, 52m IC4), xy_med 2.88** — REGRESSES vs the multi-marker
baseline (IC2 4/5). All IC2-5 start OFF-CENTER (±2m) = the trigger. Flags left default-on (the world+cal
are single-marker; reverting → board-mode-on-single-marker = broken) but marked PROVISIONAL in code, NOT
git-committed. **ROOT (V-frame GT diagnostic, fly-away 00-30-49 vs clean 00-32-14):** the terminal
fly-away is LATERAL, off-center-driven, NOT loom (loom under-reports identically in the CLEAN 0.34m rep).
Mechanism: drone off-center at MID alt (s_e_n 1.0 breach vs clean 0.29) → the CORNER lstsq produces
SPURIOUS lateral-flow SPIKES (h_y max 8.6 vs GT 1.65; consumed/fused follows to 12.5) → over-reaction →
fly-away. VERIFIED spike ∝ off-center (corr +0.40, lever-arm 0.46 vs 0.37) NOT tilt (corr +0.02) → it's
OFF-CENTER ILL-CONDITIONING amplifying LK noise (L_s rotation cols wx/wy carry x²/xy terms that swell ∝
offset → confound with translation → σ_min collapses → pinv amplifies noise), + FoV-edge corner-set
clipping. The single LARGE marker fixed the SIZE/angular-extent conditioning term but NOT the POSITION
term (off-center). **The RING STAYS BOUNDED** (h_y max 1.6 ≈ GT) because it's concentric on the OPTICAL
AXIS (offset-INVARIANT geometry, ego-ground flow, robust radial-div loom) — but the FUSION over-trusts
the spiking corner (corner_conf keys on KLT-staleness NOT conditioning/spike). **THE FIX (next):
conditioning/innovation-aware fusion — down-weight the corner toward the BOUNDED RING when the corner
flow spikes / s_e_n breaches / corner goes off-axis.** This is the SAME off-center lateral wall as the
multi-marker board, now isolated to a specific fixable fusion-weighting mechanism. Records NC109(+gate).

**NOISE LEVERS (`loom_noise = input_noise/σ_min²`; all A/B'd 2026-06-22, n=5 — ALL lose to momentsz).**
- **GFT** (texture corners): NEGATIVE — REVERTED to scaled-quad. Predicted to cut input noise (2D
  corners, no aperture problem) but a clean ArUco marker has only ~4-8 resolvable cell corners → GFT
  STARVES the point count → less √N averaging → MORE noise (lat_noise 0.126→0.173, vz>3 5/5). Point
  COUNT beat point QUALITY; the 180 scaled-quad points average better despite the aperture problem.
- **rcond ↑** (`FLOW_LSTSQ_RCOND=3e-2` vs default 1e-3): textbook noise-vs-BIAS. Cut lat_noise
  0.232→0.051 (4.5×, truncates the weak modes → caps `1/σ_min²`) — HELPED (vz>3 5/5→3/5, fly 2/5→1/5,
  so the rank-deficiency JITTER was real & fixable) — BUT attenuated |h_lat| 0.437→0.179 (2.4×,
  under-reports velocity → sluggish) AND can't restore the CORRELATION collapse → STILL loses to
  momentsz (rcond 1/5 sub vs momentsz 2/5). Tikhonov/ridge = same trade; weighting inert (geometric).
- **moment loom** (`FLOW_LOOM_DECOUPLE`): avoids the lstsq ENTIRELY for the loom — no rank-deficiency.
  Clean, and THE vertical fix (multi-marker). Only fixes h_z though; lateral stays rank-deficient.

**GT-FLOW SEPARATION (the decisive diagnosis — cal-mismatch vs rank-deficiency vs frame).** Measured
CAL'd lateral flow (V-frame, via `validate_output_flow.prep`) vs GT V-frame flow (`gt_v_flow`), alt
0.4-3m: (1) SLOPE: single-marker slope differs from the BOARD reference (e.g. slope_hy 1.77/0.40 vs
board 0.50; raw_slope 2.08 vs 0.26) = a REAL CAL MISMATCH (the board-derived `_sensor_cal_hw` mis-scales
the single-marker point set) — but FIXABLE by re-cal. (2) R² COLLAPSE: single-marker h_y R² **0.74→0.07**
(both single arms 0.07/0.10) = a linear cal CANNOT restore correlation = the geometric rank-deficiency.
(3) FRAME CHECK (user-flagged — ensure V-frame): the yaw-invariant **magnitude |h_lat|** R² ALSO
collapses (board 0.76 vs single 0.34-0.48), and ≈ the per-axis R² → NOT a yaw-rotation artifact (a
rotation preserves magnitude). A global ~−14° best-fit V-frame yaw offset exists on ALL arms (board
included, tolerated at R² 0.76) — a separate small measured-vs-GT yaw discrepancy (EKF-yaw? convention),
NOT the single-marker driver. **VERDICT: the fatal effect is the CORRELATION COLLAPSE (cal-/frame-
independent); the scale mismatch is secondary & a re-cal would NOT rescue single-marker → re-cal NOT
worth running.** Tools: `validate_output_flow.prep`/`r2_slope`, `gt_optical_flow.compute_gt_flow`.

**DEAD-ENDS / DON'T RE-DO.** (a) Single-marker on the EXISTING nested board — rank-deficiency-bound,
loses to momentsz. (b) Scaled-quad dense points — sit on edges (aperture problem); GFT corners track
cleaner. (c) `_marker_visible` flag — redundant with `corner_ok`; use the MARGIN instead. (d) Expecting
interior points (any density) to fix the CONDITIONING — it's spread-set; they only cut NOISE (~√N).

**Knobs:** `PLASMC_SINGLE_MARKER` (default-off), `PLASMC_MARKER_FOV_MARGIN` (40px, flow→ring margin),
`FLOW_LSTSQ_RCOND` (1e-3), `FLOW_LOOM_DECOUPLE` (the moment loom = the actual fix). Backup:
`src/img_data.py.pre_singlemarker_bak`. See [[feedback_virtual_plane_loom_ring]] (terminal loom set-change
sign-flip = the same switch artifact), [[project_current_state]].
