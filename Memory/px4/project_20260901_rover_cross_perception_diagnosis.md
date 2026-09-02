---
name: project_20260901_rover_cross_perception_diagnosis
description: "2026-09-01/02 SESSION: diagnosed why perception-mode landing on the MOVING rover_cross platform fails (0/5 static IC1-5, all moving reps stall/fly-away). Root causes, one control-side + several scene-side, all traced to mechanism. NO code regression from rover integration (flat cross_marker IC1-5 still land on current HEAD). Key headline: the cross detector's fixed inRange(V<=100) gate is fragile to any dark object near the marker -- PROVEN in a controlled empty-world test. Committed 8ed004ad: CROSS_S_JUMP_GATE (default ON), CROSS_ADAPT_GATE (default OFF, has a 27% false-fit gap), tools/validate_detector_gt.py harness. Scene fix outside repo: rover_cross platform box lightened 0.25->0.6."
metadata:
  node_type: memory
  type: project
  originSessionId: 165e87c8-3181-4ae1-9945-1405e9e2021d
  modified: 2026-09-01T19:51:56.218Z
---

**Continues [[project_20260831_perception_mode_landing]] (stationary cross-marker works) →
started the MOVING rover_cross phase. Baseline = main @ `4d7bc210`; work committed on top
as `8ed004ad`.**

## Bottom line

Perception-mode landing on `rover_cross` (the +0.5 m platform + rover, cross marker):
**0/5 static IC1-5, every moving rep stalls or flies away.** NOT a code regression — the
identical detector+controller lands IC1-5 dead-centered on the flat `cross_marker` world
on current HEAD (control check: 86-100% detect, min_alt 0.03-0.17 m, ~0.05-0.10 m
centered). `controller.py` / `cross_marker_perception.py` / `cross_marker_detector.py`
have **zero `WORLD`/`rover` conditionals** — same code path both worlds. Pose indices
(`POSE_IDX_UAV=2`/`TARGET=1` for `pose/info`) verified against the commanded IC. The
failure is entirely what the shared code is FED.

## Failure decomposition (static IC1-5, legacy detector)

Two clusters:

| IC | flat | rover_cross | mode |
|----|------|-------------|------|
| IC1 centered | lands 0.03 | 70% desc-detect, GT-lat 0.4→3→9 m spike→recover→ground | intermittent detection loss |
| IC2/IC3 offset | lands 0.05-0.07 | ~3.4 s abort, 2-3% **descent** detect, GT-lat frozen ~2.4 m | **detector never locks → blind dive 2.4 m off** |
| IC4 offset+high | lands 0.10 | **85% desc-detect, GT-lat bounded, stalls @ 1.14 m** | **terminal overfill / loom collapse — NOT detection** |
| IC5 offset+low | lands 0.04 | ~2.9 s abort, 6% descent detect | detector never locks (worst, short runway) |

**Cluster A (IC2/IC3/IC5): oblique-view detector collapse.** Descent starts, drone tilts
to close its ~2.8 m lateral offset while descending → very oblique camera view. On the
flat world these same offset ICs detect 86-100%, so obliqueness alone isn't fatal — the
rover scene adds rover wheels + the 3 m marker's overhang edge (3 m marker floats over a
0.6 m platform) in the oblique FoV → wrong line-pair selection → `centroid_mismatch` /
`near_parallel_fit` / `insufficient_fit_points` from alt 5 m onward.

**Cluster B (IC1/IC4): terminal marker-overfill loom collapse.** Detection + lateral
tracking fine; the 3 m marker fills the 240 px frame at ~1.1 m altitude (over the 0.5 m
platform) → `h_z` loom collapses to ~0 → `B_T`→0 → descent stalls at ~1 m. Pure geometry,
detector-independent. Same mechanism as the stationary #1 open blocker
([[project_20260831_perception_mode_landing]] banner), just triggered ~0.6 m higher by
the platform so there's descent left to complete.

## ⭐ GT-FEEDBACK ABLATION (2026-09-02) — static rover_cross DOES land; the detector is the blocker

Ran the clean discriminator: `PLASMC_GT_FEEDBACK=1` on static `rover_cross` (airframe 4022,
`world rover_cross`) bypasses perception entirely.

| run | `min_alt` (rel-z) | above platform | verdict |
|---|---|---|---|
| `Rover_Static_IC1_Montage/rep1` | 0.51 | **0.01 m** | genuine landing, xy 0.007 |
| `Rover_Static_IC1_Montage/oldtex_rep1` | 0.51 | **0.01 m** | genuine landing, xy 0.010 |
| `Cross_Marker_Montage_Rover/Mon Aug 24 11-24-19` | 0.99 | 0.49 m | FALSE latch (`Impact detected`), see below |
| `Landing_Test/Thu Aug 27 00-21-14` (= `Rover_Static_IC1_Montage/rep1`) | 0.487 | **0.00 m** | genuine landing, xy 0.007 |

**With perfect state the rover platform is landable, dead-centred.** So the perception-mode
failure is the DETECTOR (already isolated by the flat-vs-clutter comparison: identical world
+ one dark box drops detOK 100%->60% at matched 20-30 deg obliquity), not the platform, not
the controller, and not the rover being a rover (this is the STATIC case).

**⛔ CORRECTION to "Cluster B = terminal overfill".** This file attributes the IC1/IC4 ~1.1 m
stalls to terminal marker-overfill loom collapse. GT-FB reaches 0.51 m with no trouble, and
under GT-FB the loom comes from truth so overfill CANNOT bite. Whatever stalls perception-mode
`rover_IC4` is therefore downstream of the detector, not an independent terminal-overfill
blocker. Treat the Cluster B attribution below as UNCONFIRMED.

⚠ Sample is 4 GT-FB static runs (3 genuine landings, 1 false latch), all centred/IC1-ish. Enough to show the platform is landable;
NOT enough for offset ICs -- which is exactly where the perception failure lives. A GT-FB
sweep at IC2/IC3/IC5 on `rover_cross` would close that gap and is cheap. **Not yet run.**

### Platform geometry — verified, use this constant

`rover_cross/model.sdf`: platform box 0.6x0.6x0.4 at pose z=0.30 -> **top at 0.50 m**, marker
plane at 0.501 (unchanged since before 2026-09-01, per the `.bak_before_platformcolor_20260901`
backup). Confirmed EMPIRICALLY, not just from the SDF: across the whole archive, rover runs
that genuinely settle (`|dz/dt| < 0.02`) rest at **rel-z 0.487, mode 0.5, n=12**. So a landed
rover drone reads `uav.z - target.z ~ 0.49`, NOT ~0 -- the target pose is the rover BASE
(z~0.02), not the landing surface. This is the same trap that made a first-pass archive audit
flag 34 genuine platform landings as failures; see
[[project_20260902_archive_rescore_false_precise]].

### ✅ The rover montage videos ARE correct (a retraction)

An earlier version of this section claimed `test_data/Test_Videos/montage_rover_static_ic1.mp4`
and `..._touchdowncrop.mp4` were mislabelled and showed a non-landing. **That was WRONG —
retracted (user challenge, 2026-09-02).**

They are built from `Landing_Test/Thu Aug 27 00-21-14 2026` (= `Rover_Static_IC1_Montage/rep1`),
which is a **genuine landing**: 10.3 s, rel-z 4.98 -> 0.49, `min_alt` **0.487 = 0.00 m above the
0.50 m platform**, xy 0.007, rel_vel 0.066, SOFT+PRECISE. Pairing confirmed by file times — the
montage was written 00:22:17, 61 s after `rep1.out` finished 00:21:16 — and by the montage's own
plot (x-axis to ~10 s, `|rel. position|` ending ~0.5), which matches this run and NOT the
`Cross_Marker_Montage_Rover` run (7.1 s, ending 0.99).

**Two mistakes worth not repeating:**
1. **Never assume a video<->run pairing from directory names.** I saw a rover montage, found a
   rover montage *directory*, and assumed — without checking file mtimes or matching the
   plotted trace against the data. The filename `rover_static_ic1` named the correct dir all along.
2. **⛔ On the rover, `|rel. position| ~ 0.5` MEANS LANDED**, not airborne — the trace is
   measured to the rover BASE and the platform is 0.50 m tall. This is the exact trap recorded
   two sections above (settled rover landings rest at rel-z 0.487, n=12); I wrote it down and
   then fell into it an hour later while reading a chart. When a rover plot bottoms out near
   0.5, that is the success case.

Separately and still true: the `Cross_Marker_Montage_Rover/Mon Aug 24 11-24-19` RUN is a false
latch — 7.1 s, monotonic 5.004 -> 0.990 rel-z, ending 0.49 m above the platform still descending
at -0.32 m/s with `B_T=-0.36`, terminated by `Impact detected (|a|=65.6)` and scored SOFT+PRECISE.
It is simply not the run the montage was built from. **So GT-FB static rover_cross is 3 of 3
genuine landings, not 2 of 3.**

## ⭐⭐ CLUSTER A ROOT CAUSE FOUND (2026-09-02): `centroid_mismatch` is 79 % of all failures

Traced the 9 STATIC + OFFSET perception-mode `rover_cross` runs (Sep 1-2, the "blind dive"
class) using the LIVE per-frame `Img_Data['Detection Status'] / ['Fail Reason']` — what the
pipeline actually produced, not an offline re-detection.

**The detector produces ZERO detections through the entire descent:**

| height above pad | frames | detOK |
|---|---|---|
| 3-5 m | 981 | **0.0 %** |
| 2-3 m | 365 | **0.0 %** |
| 1-2 m | 154 | **0.0 %** |
| 0-1 m | 107 | 13.1 % |
| below pad (on ground) | 256 | 55.5 % |

`FEATURE_IS_VISIBLE` tracks detOK exactly, so **the controller flies OPEN-LOOP the whole way
down**. That is why lateral never closes (2.9 -> 2.3 m) and the drone drops at 0.8-1.5 m/s
onto the ground BESIDE the pad. All 9 are honest `target_lost=True` aborts.

**Fail-reason histogram (pooled):** `centroid_mismatch` **1298**, `insufficient_fit_points`
219, `near_parallel_fit` 54, `lt2_angle_clusters` 20. One gate is 79 % of the problem.

### The gate is rejecting mostly-GOOD fits

Disabling ONLY the `centroid_mismatch` rejection (scratch copy of the module, descent band
alt > 1 m, ground frames excluded):

| set | detOK normal -> disabled | err med | within-0.15 |
|---|---|---|---|
| `rover_IC2` | **28.5 % -> 95.9 %** | 0.065 -> 0.055 | 95.9 % -> 76.2 % |
| `clutter_IC2` | **50.0 % -> 97.5 %** | 0.304 -> **0.034** | 46.7 % -> 59.7 % |
| `flat_IC2` | 100 % -> 100 % | 0.012 (identical) | 100 % -> 100 % |

- Detection triples while the MEDIAN centroid error barely moves (rover 0.065 -> 0.055) --
  the line fits underneath were fine; the gate threw them away.
- **Inert on clean scenes** (flat bit-identical) -- must-not-regress-clean holds.
- ⚠ **NOT free:** rover within-0.15 drops 95.9 -> 76.2 %, so ~24 % of recovered detections are
  >0.15 off. Do NOT simply disable the gate. But 0 % availability is worse than 76 % accuracy,
  because 0 % means open-loop.

### Mechanism, and why the two committed fixes miss it

`_detect_core` checks the fitted line intersection against the **MASK PIXEL CENTROID**
(`xs.mean(), ys.mean()`, tolerance `0.12 * max(bw,bh)` at small fill). When platform structure
merges into the SAME connected component, that centroid is dragged off the junction -- the fit
is right, the REFERENCE it is checked against is contaminated.

This is why neither existing fix helps: the `_isolate_marker_by_shape` fill band (`ee858086`)
and the compact-blob strip both key on the contaminant being a separate, COMPACT component.
On `rover_cross` it is an **elongated bar (aspect 3.9-5.1)** fused to the cross.

**PROPOSED FIX (untested at time of writing):** validate the intersection against the
**INLIER LINE POINTS of the two fitted arms**, not against all mask pixels -- contamination-
immune by construction. Score on {flat, clutter, rover} with the same harness.

### ⛔ Corrections to earlier statements in this same session

- "static offset reaches the platform 11/18" was WRONG. `h_min ~ -0.53` with 2.3 m lateral
  means it descended PAST the pad to the GROUND beside it. Re-classified with a footprint test
  (|lat| < 0.30 m at the lowest point): **0 of 18 static-offset runs landed on the pad.**
  Modes: 11 blind-dive-to-ground (this section), 2 stall centred above the pad, 5 stall-high /
  fly-away. Altitude alone never proves a landing on a raised pad -- check the FOOTPRINT too.
- The Mode-B stall (2 runs, 0.44/0.62 m above the pad, lateral closed to 3-10 cm) is **NOT a
  perception failure**: measured loom tracks the run's OWN GT at corr **+0.756 / +0.806** and
  slightly OVER-reports at the terminal, and the GT loom itself decays. Control-side; the
  commit-gate knobs (`PLASMC_COMMIT_*`) were all default 0 = OFF in those runs. See
  [[feedback_loom_commit_gate]].

## Three scene causes (rover_cross-specific, world/infra — NOT our source)

1. **Two-PX4-instance camera warm-up.** ~7 s frozen uniform-grey feed at start (byte-
   identical `mean 89.9` frames, `%V≤100 = 100`) → the `inRange` gate returns all 76800 px
   → `hough_lt2_lines` with constant bbox `(0,0,239,319)`. It's a ROS2-image-bridge /
   render-not-ready race on the two-instance stack (single-instance flat world doesn't
   have it). **NOT actually blocking any IC** — `landing_test.py`'s `track_target` waits
   on `TARGET_IS_VISIBLE` before takeoff/descent, so the warm-up is absorbed during the
   pre-descent hover; by descent-start detOK is ~100%. Was wrongly prioritised mid-session.
2. **Platform box dark side-walls.** `rover_cross/model.sdf` `platform_visual`
   `<diffuse>0.25 0.25 0.3</diffuse>` → HSV V≈76 < 100 → the 0.4 m box walls passed the
   dark gate, became long straight Hough segments, got fit as "cross arms" →
   `centroid_mismatch`. **FIX APPLIED (outside repo, backups alongside): diffuse →
   `0.6 0.6 0.6` (V≈153).** Result: IC1 `centroid_mismatch` 54→4, IC1/IC4 now descend
   dead-centered. Offset ICs still collapse on the *other* clutter (wheels, plate edge).
3. **Terminal overfill (= Cluster B above).** Open; needs the loom/commit work, not a
   scene change.

## Controlled proof of the detector weakness (the headline)

Empty `cross_marker` world + ONE dark visual-only box (0.6×0.6×0.3 m, V≈31, no collision)
on the marker plane at (1.2, 1.2), clear of the (0,0) landing point. Same legacy detector
that lands IC1-5 cleanly on the truly-empty world. Result: **IC2 detOK 100% → 10%; the
flight can't even start** (`TARGET_IS_VISIBLE` never fires, 6 retries time out, no
landing). Reproduced with zero rover, zero platform, zero two-instance stack. Signature
here was `color_gate_empty` (box blob dominates `_isolate_marker_by_shape`) rather than
`rover_cross`'s `centroid_mismatch` (box walls fit as arms) — different mechanism, same
root: **`inRange(V≤100)` cannot distinguish the marker from any dark object near it, and
collapses if the scene darkens globally.** Test world saved as `cross_marker_clutter.sdf`
(own world name) for the robustness work. See [[feedback_cross_detector_contrast_not_darkness]].

## Control-side findings (moving-platform terminal stall — traced earlier in the session)

The gated moving reps (GT-FB not; perception, Linear @0.47 m/s) descend clean+centered to
~1.5 m then stall at ~1.0-1.3 m. Mechanism, chased through Control_Data + offline recon:

- **The trigger is a single-frame perception centroid spike** `s_x`: 0.16 → **0.91** (one
  frame) → 0.16, recurring ~every 0.3 s once `MARKER_EXTENT_PX` saturates (~308-318 px).
  → `s_e_n` 0.13→0.77, `dh_d_x` → ±11 (feedforward-rate term, `DH_D_MAX=50` too loose),
  `theta_norm` → 27, `a_u_x` → ±30, `theta_desired = I_a[:2]/a_z` → **1.6 rad (92°)**.
- `cbf2_filter`'s FoV-visibility QP correctly clips that 92° lean → `dtheta_az` ≈ 1 rad.
- The joint QP's **`CBF_AZ_COST_GAIN = 5.0`** ("descent-rate relief per rad of box-
  suppressed tilt") folds **≈ 5 m/s² of upward accel into `I_a[2]`** → `B_T` collapses
  −0.73 → 0 → descent stops. On a moving target the paused descent lets the offset grow →
  more spikes → latched stall → fly-away.
- **CORRECTION to the first analysis:** the terminal `h_z` loom "inversion" is FAITHFUL,
  not causal. Against the run's OWN GT, `h_V[Tz]` tracks GT loom at **corr +0.89** through
  the overfill window; both go to ~0 as the descent stalls (and the drone bounces up ~1 cm).
  The loom is a *symptom*. (The earlier "loom inverts while true loom holds −0.28" compared
  against a *different, still-descending* GT-FB run.)
- **The raw reduced-lstsq `Tz` does NOT fail at overfill** in the `CROSS_BG_FLOW=1`
  vector-field regime — reconstructed offline from the saved `Flow Points`: `cond(A₄) ≈ 1.2`
  throughout, and reduced-4 `Tz` agrees with the full-6 solve, radial regression,
  moment-loom, AND the logged `h_V[Tz]` (all within ~0.05-0.1; corr +0.79-0.89 vs GT). It
  only blows up after point count collapses to ~66 (post-stall detection breakup). The
  moment-loom's `origin_ratio` gate also never trips (ratio 40-480, points near centre).
- **`rho_fov` is NOT the hard visibility bound** anymore — the QP uses `p_10 = center/focal`
  (fixed camera geometry). `rho_fov`/`d_min_fov`/`theta_cone` feed only the legacy (and
  `PLASMC_LFOV=0` → effectively inert) `d_min` path + the soft drift-off classification.
  So "recalibrate rho_fov for a new extent scale" is wrong; `MARKER_EXTENT_PX` still
  matters because it feeds touchdown-detect v2 / terminal-commit / ring-fusion.
- **Moment-loom is architecturally wrong for the cross marker** (needs persistent
  identifiable points; the cross has only churning texture/edge features) — but it was NOT
  the failed-IC5 cause: static IC5 had **0 `det.ok` frames the whole 2.9 s descent** (the
  moment-loom never ran on live data; `origin_ratio` gated it off anyway). IC5 failed
  because the detector never locked. See [[feedback_cross_detector_contrast_not_darkness]].

## Committed this session (`8ed004ad`)

- **`CROSS_S_JUMP_GATE` (0.35, DEFAULT ON)** in `cross_marker_perception.py` — single-frame
  centroid jump-outlier gate (rejects the `s_x` 0.16→0.91→0.16 spike class; bounded by
  `CROSS_S_JUMP_MAX_REJECT=2` consecutive rejects). Offline replay: catches every spike,
  0 false-positives on the healthy descent. In SITL it removed the *early* spike-stall
  (moving reps then descend centered to ~1.5 m) but does NOT fix the landing — 0/4 moving,
  the terminal overfill stall + rover tracking-lag remain.
- **`CROSS_ADAPT_GATE` (DEFAULT OFF)** in `cross_marker_detector.py` — CLAHE +
  `adaptiveThreshold` (+ global-Otsu fallback on a degenerate mask) + structureless-frame
  reject (`gray std < CROSS_ADAPT_MIN_STD` → `color_gate_empty`) + `CROSS_SEG_CAP=120`
  (longest Hough segments) + capped full-frame acquisition (only when the gate is on;
  keeps `process_frame` ~46 Hz, `detect()` 8.5 ms w/ tracking). **KNOWN GAP — do not bake:**
  on `rover_cross` IC2 offline it lifts detOK 49%→83% (descent bands 0%→72-100%) BUT
  only **73% of the extra detections are within 0.15 norm of GT** — confidently-wrong
  line-pair fits (median centroid disagreement vs legacy = 40 px). Needs a stroke-profile
  / geometry-first discriminant first.
- **`CROSS_EXTENT_FROM_LINES` (DEFAULT OFF)** — `MARKER_EXTENT_PX` / `get_marker_radius_px`
  from RANSAC cross-arm inliers instead of the mask bbox (front-end-stable scale).
- **`_isolate_marker_by_shape` fill-ratio band** (committed `ee858086`, DEFAULT ON) --
  user-measured root cause of Cluster A's `lt2_angle_clusters`: the selector's
  `score=-area` kept the largest square-ish component, and a diagonal platform-
  shadow-bar slice (fill 0.35, aspect 2.6) out-areas the thin cross (fill 0.07) ->
  cross discarded, 78/128 acquisition frames wrong (= the observed rate; 0/123
  flat). Now gated on fill `[0.02, 0.25]`. Eval set: flat unchanged, rover_IC4
  detOK 78->88 % (`lt2_angle_clusters` 102->33), rover_IC2 45->59 %; **clutter
  unchanged** -- box+shadow+cross MERGE into one component after the Hough dilate
  (`centroid_mismatch`), separate fix still open.
- **`tools/validate_detector_gt.py --set` + `test_data/DetectorFrameset/`** — offline GT-scored detector harness (analogue of
  `validate_bgflow_corr.py`): per recorded frame, run a candidate detector, compare
  leveled `det.center` to GT V-frame bearing (`gt_optical_flow.compute_gt_flow`), report
  detOK% + per-altitude-band, centroid-err median-px + within-0.15 hit-rate, top fails.
  Variants are env-dict entries in `VARIANTS`. Reproduces the SITL finding offline.
  ⚠ data-dir ↔ `_raw` dir pairing is manual (data dir lags the `_raw` dir ~13 s).

## Scene fixes OUTSIDE the repo

- `~/.gazebo/models/rover_cross/model.sdf` + PX4 copy: `platform_visual` `<diffuse>`
  `0.25 0.25 0.3` → `0.6 0.6 0.6`. Backups `*.bak_before_platformcolor_20260901`.
- `~/PX4-Autopilot/Tools/simulation/gz/worlds/cross_marker_clutter.sdf` — NEW, dedicated
  adversarial-clutter test world (clean `cross_marker` + one dark visual box near marker).
  Clean `cross_marker.sdf` restored (backup `*.bak_before_darkclutter_20260901`).

## NEXT

1. **Front-end robustness** (real work). The `_isolate_marker_by_shape` fill-band fix
   (`ee858086`) handles the separate-clutter-component case; still to do: the
   merged-component case (Hough dilate joins box+shadow+cross -> `centroid_mismatch`),
   then the contrast-based / polarity-agnostic / stroke-profile front end. Contrast-based,
   polarity-agnostic segmentation + perpendicular-profile stroke validation + geometry-
   first confirm. Validate on `tools/validate_detector_gt.py` across {flat clean, clutter,
   rover_cross} — must-not-regress-clean, fix-the-rest — then n≥5 SITL. Record fresh
   `IMG_RECORD=1` paired sets first. See [[feedback_cross_detector_contrast_not_darkness]].
2. **Terminal overfill / loom** (Cluster B, also stationary): line-width loom (survives
   overfill, geometry-anchored) OR terminal commit OR lower platform.
3. **Moving-target tracking lag** ([[project_rover_speed_sweep]] τ≈1 s) still there under
   perception once 1+2 are fixed.
4. `CBF_AZ_COST_GAIN=0` A/B — its "slow descent to buy lateral time" premise is backwards
   for a moving target; cheap test, may need a default change.
