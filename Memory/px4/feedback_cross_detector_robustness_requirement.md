---
name: feedback_cross_detector_robustness_requirement
description: "USER REQUIREMENT (2026-09-02): the perception pipeline must be robust to different LIGHTING conditions, different marker/background COLOUR combinations, TEXTURED backgrounds and general perception noise. This rules out patching gates one at a time -- the pipeline currently hard-codes two absolute assumptions (inRange(V<=100); the mask-centroid junction proxy) that both fail the requirement, and the eval set cannot even MEASURE it."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**Stated by the user, 2026-09-02:**

> "I need a robust perception pipeline which should work for different lighting conditions,
> different colour combinations of the marker and textured background and it should be robust
> to other perception related noise."

This is the ACCEPTANCE BAR for the cross-marker front end. It is stricter than "make
`rover_cross` land", and it retires the patch-a-gate-at-a-time approach.

## Why the current pipeline cannot meet it

Two hard-coded ABSOLUTE assumptions, both the same mistake -- *the marker is the only dark
thing*:

1. **`cv2.inRange(hsv, [0,0,0], [180,255,CROSS_COLOR_GATE_V_MAX=100])`** -- "the marker is dark
   in absolute terms." Breaks on: dim light (background falls below 100), bright light (marker
   rises above it), light-on-dark or coloured markers, and any dark object nearby.
   ([[feedback_cross_detector_contrast_not_darkness]] -- the locked design principle.)
2. **The `centroid_mismatch` gate** -- validates the fitted junction against the MASK PIXEL
   CENTROID, i.e. "the marker is the only thing in the mask." Breaks whenever foreign structure
   merges into the same connected component.
   ([[project_20260901_rover_cross_perception_diagnosis]])

Every fix attempted so far patches assumption 2 in one scene: the `ee858086` fill band, the
compact-blob strip, `CROSS_CENTROID_SPAN_RESCUE` + its fill ceiling. None touches lighting or
colour, and the last one made clutter flight WORSE
([[project_20260902_spanrescue_sitl_gate]]).

## What the requirement implies (the locked design, now load-bearing)

- **Contrast-based, POLARITY-AGNOSTIC segmentation** -- gradient magnitude / local deviation,
  relative not absolute. Identical response to dark-on-light and light-on-dark; indifferent to
  scene brightness. Drop the absolute V gate entirely.
- **Perpendicular intensity-profile STROKE validation** -- a marker stroke is a symmetric
  ridge/valley of ~constant width; a platform edge or shadow boundary is a STEP. This is the
  discriminant that rejects today's contaminant BY SHAPE rather than by darkness, which is what
  survives lighting and colour changes.
- **Geometry-first confirm** -- two profile-validated strokes, ~perpendicular, meeting in-frame.
  This also REMOVES the need for the mask-centroid proxy, so assumption 2 stops existing rather
  than being patched again.

## ⛔ Two constraints on how to validate it

1. **Score on ACCURACY and FLIGHT OUTCOME, never detect-rate alone.** Proven the hard way:
   clutter detOK 5-7 %->22-73 % while flight got WORSE (median xy ~1.9->~5.6 m, two 12 m
   fly-aways). `within-0.15` was the offline signal that predicted it; the headline rate was not.
2. **The eval set is the BOTTLENECK -- robustness is currently UNMEASURABLE.**
   `test_data/DetectorFrameset` has 6 sets but ALL are one lighting condition, one polarity
   (dark cross on light plate), one texture. Tuning against it = tuning to one scene again.
   The requirement needs sets spanning: sun angle + intensity, INVERTED polarity, a coloured
   marker, textured/cluttered backgrounds, and sensor noise -- recorded with
   **`CROSS_RING_OVERLAY_DBG=0`** so the frames are not overlay-contaminated
   ([[feedback_detector_offline_replay_gotchas]]).

**Build the eval set BEFORE the front end.** Otherwise there is no way to tell a robustness gain
from a scene-specific fit.

## ✅ EVAL SET BUILT + BASELINED 2026-09-03 — and it RESHAPES the target

`test_data/RobustnessFrameset/` (7 variants, 2743 frames, IC2, flat `cross_marker` geometry;
bulk gitignored, `MANIFEST.md` tracked). Assets: `tools/make_robustness_eval_assets.py`;
recording: `test_data/Rover_AB_harness/record_robustness_set.sh`. Recorded under
**`PLASMC_GT_FEEDBACK=1`** (the detector under test must NOT gate the flight, else the blind
variants never take off — and every variant gets the SAME trajectory so scores compare across
scenes) and **`CROSS_RING_OVERLAY_DBG=0`** (verified 0.00 % drawn pixels).

**Baseline, current detector @ `a53a5f63` — CORRECTED 2026-09-03 (the first one was
CONTAMINATED; see the ⛔ note below):**

| variant | detOK | err med | within-0.15 |
|---|---|---|---|
| base | 100.0 % | 0.014 | **97 %** |
| dim (sun 0.30) | 100.0 % | 0.017 | 81 % |
| bright (sun 2.50) | 95.0 % | 0.013 | 96 % |
| lowsun (grazing) | 100.0 % | 0.014 | 97 % |
| darkbg (0.18) | 99.6 % | 0.013 | **94 %** |
| **col** (chromatic, iso-V) | **62.4 %** | 0.013 | 83 % |
| **inv** (polarity flip) | **99.1 %** | **0.814 = 109.9 px** | **0 %** |

⛔ **The first published baseline was WRONG IN BOTH DIRECTIONS** and is superseded.
`validate_detector_gt.py` scored frames recorded AFTER the GT log ends against
`np.interp`-CLAMPED values; **16-42 % of frames per variant** fall outside that window
(`inv` worst at 42 %). That DEFLATED clean scenes (base 97->78 %, darkbg 94->76 %) and
FLATTERED `inv` (0->23 %). **Fixed in the scorer** with a hard GT-window guard
(`CROSS_GT_WINDOW_STRICT`, default on) that drops such frames and prints the count.
This is class 1 of [[feedback_recurring_analysis_mistakes]] — documented that morning,
walked into the same day. The guard now lives in the tool so no caller can repeat it.

### ⭐ LIGHTING IS NOT THE WEAKNESS — NOT predicted

8x sun-intensity range (0.30-2.50), grazing sun, dark ground: all **95-100 % detOK at
81-97 % within-0.15**. That half of the requirement is ALREADY MET. Do not spend effort
there without new evidence.

### ⭐ `inv` IS THE HEADLINE FAILURE — AND IT IS SILENT

**99.1 % detOK, 0 % usable, 109.9 px off, at EVERY altitude band.** `inRange(V<=100)`
keeps 94.8 % of pixels (the PLATE, not the cross), so the mask becomes the plate with the
cross as HOLES; Canny/Hough still find arm-like edges from the hole boundaries, so nothing
downstream objects. **A detector that fails loudly is recoverable; this one LIES** — any
detect-rate-based health metric scores this scene as perfect. `col` fails differently: only
62.4 % detOK but ACCURATE when it fires (83 %), because the gate keeps 0.0 % by construction
and detections come from the ROI/shape fallback.

### PASS CRITERIA for the front-end work (use these, not detOK)

- **`inv` within-0.15 must rise from 0 %** — hard pass/fail, the silent-lying case.
- **`col` detOK must rise from 62.4 %** while keeping within-0.15 >= 83 % — graded.
- **base/dim/bright/lowsun/darkbg must NOT regress.**
- Score on **within-0.15 AND flight outcome**, never detOK alone.

⚠ n=1 per variant, single IC/trajectory, SYNTHETIC textures (`col` is an adversarial
construction, not a photo of a real marker). Direction-of-effect only. Bulk is gitignored so
the set is NOT reproducible from git alone — regenerate + re-record if lost.


## FRONT-END ATTEMPTS 2026-09-03 — both FAILED, and why that is informative

Implemented behind env flags, both DEFAULT OFF, both kept in-tree with their measured results:

1. **`CROSS_GATE_MODE=contrast`** — polarity-agnostic, chroma-aware segmentation:
   `D = ||Lab(p) - blur(Lab)(p)||` with a chroma gain, thresholded at a percentile of D.
   No sign, no absolute level, illumination cancels. **REGRESSED**: base 100->89 %,
   dim 100->67 %, lowsun 100->66 %, and `inv` within-0.15 only 23->29 % (contaminated scale).
   **WHY: the PLATE BOUNDARY is a genuine high-contrast feature**, so contrast segmentation
   admits it exactly as strongly as the cross strokes and Hough still fits plate edges.
   Segmentation alone CANNOT do this job — which is precisely why the locked design has
   three stages.
2. **`CROSS_STROKE_VALIDATE=1`** — perpendicular-profile stroke validation: a stroke is a
   RIDGE/VALLEY (matching shoulders, differing core), an edge is a STEP (differing
   shoulders). Magnitudes only, so polarity- and colour-blind. **Also insufficient alone**:
   `inv` 23->26 % and centroid error WORSENED (53.9->80.0 px); cost `col` detOK 66.5->58.9 %.

**Timing is NOT the constraint** (user bar: >=30 Hz, i.e. 33.3 ms): `detect()` measures
2.6 ms legacy, 4.4 ms contrast, 5.8 ms +stroke, 9.0 ms both — 3.7-12x headroom at 320x240.

**What the `inv` visualisation actually showed** (and my mental model got wrong): the legacy
mask on `inv` is the PLATE AS A FILLED REGION WITH THE CROSS AS HOLES, and Canny/Hough DO
find the arms as hole boundaries — so detection succeeds and the geometry gates pass. The
centroid is still wrong everywhere. Neither a better mask nor a segment filter addresses
that, because the pipeline is fitting real structure; it is fitting the WRONG real structure.

**NEXT (untried):** stage 3 — geometry-first confirm. Do not add a fourth gate; the evidence
says the missing piece is a positive test that the two accepted strokes ARE the cross
(mutually ~perpendicular, intersecting IN-frame, comparable length/width, junction consistent
with both arms' inlier spans) rather than any two long edges that survived.
## ⛔ STAGE 3 TRIED 2026-09-03 — DEAD END. The "NEXT (untried)" line above is SUPERSEDED.

Both remaining pieces of the locked design were implemented and measured. Neither works,
and the reasons are specific enough to redirect the whole thread.

**Attempt 3 — candidate-mask ensemble** (`CROSS_GATE_MODE=ensemble`, `19a9ed62`, default OFF).
Changes the SELECTION, not the thresholding rule: legacy V-gate first, and only if that
yields no cross-shaped component, Otsu on Lab L/a/b in BOTH polarities, all scored by the
SAME `_best_cross_component` shape test. Legacy-first makes it inert by construction where
the detector already works (legacy short-circuits 73% of `base`, 90% of `col`, 1% of `inv`).
Result: base/bright/darkbg unchanged; dim 100.0/81 -> 99.2/**86**; lowsun 100.0/97 ->
98.4/**98**; col 62.4/83 -> 63.7/83; **inv 99.1/0 -> 42.6/0** (err 109.9 -> 78.1 px).
⭐ It does NOT meet the `inv` bar, but it converts inv from a SILENT liar into a partly loud
failure — 86 frames return `no_cross_shaped_component` and every band below 3 m refuses.
⭐⭐ **Key finding: on `inv` the ensemble picks the RIGHT channel and polarity on 60% of
frames (via `L+`), at fill 0.10-0.12 — genuinely cross-shaped. So segmentation is NO LONGER
the binding constraint on `inv`.**

**Attempt 4 — geometry-first confirm** (`CROSS_GEOM_CONFIRM=0|1|2`, `d371d744`, default OFF).
A positive test that the two accepted strokes ARE a cross (perpendicular / junction inside
both inlier spans / comparable width / comparable length).
**It regressed the clean scene and fixed nothing**: base detOK 100.0 -> 80.1% (55 rejects,
all `geom_not_perpendicular`) while base within-0.15 stayed 97% — it rejects ACCURATE
detections. inv stayed 0% usable in every mode.

⛔ **ROOT CAUSE — the perpendicularity premise is FALSE BY DESIGN for this marker.**
|fitted angle − 90| on clean `base` is strictly BIMODAL: 58% <15°, and **41% in a 35-55°
band centred on `STUB_REL_ANGLE_DEG = 45`**. The marker is a 3-armed cross WITH A 45° STUB,
and in over a third of CORRECT detections `_best_pair` legitimately selects a **stub+arm**
pair — which still gives the right junction, because the stub passes through the centre too.

⚠ **NOT a frame-convention bug** (user hypothesis, tested — do not re-try): a physically-90°
cross DOES skew in the RAW image plane under tilt, so the check arguably belongs in the
gravity-levelled VIRTUAL plane. Re-fitting `det.line_points_i/j` through
`_getVirtualPts` with each frame's own quaternion moves the distribution by **~0.1°** and
leaves the bimodality intact (base VIRT p50 1.0, 35-55° 41%). Tilt is only ~21-29° here,
which barely rotates a line DIRECTION near the image centre.

⛔ **And it would not separate anyway.** On `inv`, where EVERY detection is wrong, 62-67% of
frames are within 15° of perpendicular — **MORE cross-like than base's correct ones (58%)**.
Width ratio overlaps too (base p90 2.41 vs inv p90 2.49). The wrong structure `inv` locks
onto is geometrically a *better* cross than the real marker, so **no self-consistency test on
the two lines' relative geometry can separate them** — the distinguishing information is not
present in that comparison. Only sub-test 2 (junction inside both arms' spans) is sound and
`CROSS_CENTROID_SPAN_RESCUE` already implements it.

## Where that leaves `inv` — the hypothesis space has narrowed, not widened

`inv` is **not** a segmentation failure (the ensemble picks the right channel/polarity) and
**not** a geometry-confirm failure (its geometry is impeccable). Both stages of the locked
design are now spent. The next hypothesis must come from OUTSIDE the two-line model — the
open question is WHICH real structure the fit locks onto on a polarity-flipped scene and why
it is more cross-like than the cross. That has not been visualised yet, and should be, before
any fifth mechanism is written.

## ⭐⭐⭐ 2026-09-03 — WHAT `inv` ACTUALLY LOCKS ONTO (visualised, and it retires 4 attempts)

**The detector fits the PLATE'S OUTER EDGES and returns a plate CORNER as the junction.**
Rendered overlay (legacy gate mask + both arms' inliers + detected centre + GT centre) on
`inv` at 5/4/3/2 m: at 4 m the green arm lies along the plate's TOP edge, the blue along its
LEFT edge, and the detected centre sits on the TOP-LEFT CORNER while GT is mid-plate. Same at
3 m and 2 m. Only at 5 m -- plate still small, cross still dominant -- is the fit correct.

⭐ **This explains every measurement in the thread at once:**

| observation | why |
|---|---|
| 99.1% detOK, 0% usable | a corner is a perfectly legitimate intersection |
| ~110 px error | a corner is half a plate-width from the centre |
| perpendicularity non-discriminative | **a rectangle's corner is EXACTLY 90 deg** |
| width ratio overlaps | both "arms" are mask-boundary edges -- identical width by construction |
| span containment passes | the corner lies inside both edges' spans |
| ensemble helps only at altitude | as the plate fills the frame its boundary overwhelms the cross |

⛔ So all four stage-3 sub-tests pass on a plate corner BY CONSTRUCTION. Stage 3 could never
have worked, and the reason my model kept failing is that I was testing the RELATIONSHIP
BETWEEN THE TWO LINES when the discriminating information is in the NEIGHBOURHOOD of the
junction. **LESSON: four mechanisms were reasoned from a mental model of this failure and two
of those models were wrong when measured. A dozen annotated frames settled it in one pass --
VISUALISE THE FAILURE BEFORE WRITING THE FIFTH MECHANISM.**

## ✅ RING-TRANSITION CONFIRM — the first test in this thread that actually separates

`CROSS_RING_CONFIRM=1` (DEFAULT OFF), `CROSS_RING_MIN_TRANSITIONS` (6), `_RING_R_FRAC` (0.30).
    a cross junction has arms RADIATING from it;  a corner has two edges MEETING.
Sample a ring around the candidate junction, count MASK BOUNDARY CROSSINGS. Real junction
crosses every stroke twice (this marker: 4 arms + stub -> mode 10); a plate corner gives 2.
Polarity-agnostic (counts boundaries, not levels), scale-free (radius = fraction of extent),
no threshold of its own. Uses the CLOSE-STAGE mask deliberately -- the ring NEEDS the
surrounding context, and shape-isolation has stripped exactly the plate boundary that reveals
a corner. Skips (never vetoes) when the ring leaves the frame = the overfill regime.

**Per-frame separation** (reject when transitions < T; `base` min is 6, mode 10; 46% of `inv` <=2):

| T | base | bright | darkbg | dim | lowsun | col | **inv (catch)** |
|---|---|---|---|---|---|---|---|
| 4 | 0.0% | 1.2% | 0.0% | 0.3% | 0.0% | 1.7% | **45.8%** |
| 6 | 0.0% | 5.2% | 1.0% | 4.2% | 0.0% | 10.4% | **72.0%** |

**Full eval, detOK / within-0.15:**

| variant | base | bright | col | darkbg | dim | **inv** | lowsun |
|---|---|---|---|---|---|---|---|
| baseline | 100/97 | 95.0/96 | 62.4/83 | 99.6/94 | 100/81 | **99.1/0** | 100/97 |
| ring T=6 | 100/97 | 91.1/96 | 57.2/84 | 99.6/94 | 100/81 | **66.5/0** | 100/97 |
| ring T=4 | 100/97 | 94.1/96 | 62.0/84 | 99.6/94 | 100/81 | **83.0/1** | 100/97 |
| **ens_ring** | 100/97 | 94.1/96 | 61.7/84 | 99.6/94 | 99.2/**86** | **42.2/0** | 98.4/**98** |

⭐ **`ens_ring` (ensemble + ring) is the best combination**: removes 57% of `inv`'s silent lies
AND improves accuracy on `dim` (+5) and `lowsun` (+1), for <=1.6% detOK cost on good scenes.
`base` and `darkbg` are bit-unchanged. Lift is ~14-70x vs stage 3's ~0.02.

⛔ **BUT THE STATED PASS CRITERION IS STILL NOT MET.** `inv` within-0.15 stays **0%** under
every variant. This REJECTS wrong detections; it does not make them right -- the survivors are
still corners. The gain is converting `inv` from "99.1% detOK, silently wrong" into "mostly
refused", so the controller is not fed poisoned measurements and TARGET_LOST engages honestly.
**A failure-mode fix, not a detection fix. Judge it on that, and do not report it as solving
`inv`.** All flags DEFAULT OFF; no SITL gate run yet (offline only).

## Attempt scoreboard (all four in-tree, all DEFAULT OFF, each with its numbers)
1. `CROSS_GATE_MODE=contrast` -- FAILED, regressed clean scenes (plate boundary is genuinely high-contrast).
2. `CROSS_STROKE_VALIDATE=1` -- FAILED, `inv` 23->26%, error worsened.
3. `CROSS_GATE_MODE=ensemble` -- PARTIAL, `inv` 99.1/0 -> 42.6/0; picks the right channel/polarity on 60% of `inv` frames, so segmentation is NO LONGER the constraint there.
4. `CROSS_GEOM_CONFIRM=1|2` -- DEAD END, false by design (45 deg stub) and non-discriminative anyway.
5. `CROSS_RING_CONFIRM=1` -- BEST SO FAR, real asymmetric separation, but fixes the failure MODE only.

## ✅ SITL FLIGHT-OUTCOME GATE: `ens_ring` does NOT regress the ordinary scene (2026-09-04, n=5)

Must-not-regress check (offline eval already showed `base`/`darkbg` bit-unchanged; this asks
the same question live, since flight outcome — not detOK — is the actual pass criterion).
`test_data/EnsRing_AB_harness/ensring_ab.sh`, interleaved per rep, `HEADLESS=1
WORLD=cross_marker MARKER_TYPE=cross` (standing hard rule), IC2 (2,2,5), default legacy gate
vs `CROSS_GATE_MODE=ensemble CROSS_RING_CONFIRM=1`.

| arm | precise | median xy | detOK |
|---|---|---|---|
| off (default) | 4/5 | 0.086 | 100.0% (5/5) |
| on (ens_ring) | 3/5 | 0.077 | 100.0% (4/5), 99.6% (1/5) |

Both arms failed once at comparable magnitude (off 0.245; on 0.258, 0.701) — no directional
signal, consistent with this platform's known lateral stochasticity rather than anything
ensemble/ring-specific. detOK cost is negligible (one rep at 99.6%). **n=5 is too small to
prove equivalence, but there is no regression signal to chase, on the world this mode will
default to flying if ever turned on.**

⚠ This tests ONLY the must-not-regress arm. The actual claimed benefit (inv 99.1→42.2%
detOK, dim/lowsun accuracy up) needs its own SITL gate on the `cm_inv`/`cm_dim`/`cm_lowsun`
Gazebo world variants (they exist, `~/PX4-Autopilot/Tools/simulation/gz/worlds/cm_inv.sdf`
confirmed present) — NOT yet run. Still DEFAULT OFF pending that.

## ✅ SITL FLIGHT TEST ON `cm_inv`, PERCEPTION MODE (2026-09-04, n=5/arm)

Not the must-not-regress gate above — this flies the ACTUAL polarity-flip scene, detector
in the loop (unlike RobustnessFrameset, which was recorded under `PLASMC_GT_FEEDBACK=1` and
never let the detector control anything). `test_data/CmInv_AB_harness/cminv_ab.sh`, IC2
(2,2,5), `WORLD=cm_inv MARKER_TYPE=cross`, interleaved, default legacy gate vs
`CROSS_GATE_MODE=ensemble CROSS_RING_CONFIRM=1`.

**⚠ INFRA: 10 of 20 launches lost across two attempts** to a stale-process race after a
world-switch ("gazebo already running world: cross_marker" when `cm_inv` was requested,
traced to a `gz_bridge "Task already running"` failure whose leftover process desynced
MicroXRCEAgent's UDP port 8888 and cascaded through subsequent launches). `ko()`'s existing
`/clock`-topic check does not verify the port itself is released. **Fixed**: a `fuser -k
8888/udp` hard backstop added to `ko()`, confirmed working on the clean second pass. Not
proven fully robust from one clean run — watch for recurrence on future world-switching
harnesses.

**RESULT (8 valid reps of 10 flown, min_h = lowest altitude above the surface reached):**

| arm | min_h per rep | pattern |
|---|---|---|
| off (baseline) | 5.024, 5.062, 5.021, 5.039, 5.048 | **5/5 — never leave the ~5 m start altitude, σ≈0.01m** |
| on (ens_ring) | 3.155, 3.206, 0.116, 0.031, **5.005** | 4/5 make real descent progress (2 nearly touch down); 1/5 reproduces baseline exactly |

⭐ **Baseline is unambiguous: 5/5 reps get confidently-wrong plate-corner detections that
hold the aircraft at a stable-but-WRONG setpoint — it never even starts descending.** This is
the live confirmation of the offline root-cause finding (99.1% detOK, 0% usable): the wrong
detection isn't noisy, it's a STABLE wrong answer, so the controller settles on it rather
than diverging.

⭐ **`ens_ring` breaks that floor in 4/5 reps** — a qualitatively different outcome from
every single baseline rep, not a marginal shift. ⚠ It does NOT eliminate the mechanism:
`on/5` reproduced baseline's exact failure (min_h 5.005), so this reduces how often the
plate-corner lock-on happens, it does not close off the possibility.

⛔ **Neither arm actually LANDED** in any of the 10 reps (`DESCENT_ANOMALY` / `FAIL` /
`TARGET_LOST` throughout). This does NOT demonstrate `cm_inv` landing works — it demonstrates
the specific mechanism the ring test targets is real, reproducible, and mostly (not fully)
fixed by it. n=5/arm, one IC, one session — direction is solid, the ~80% break-out rate is
not a precise number.

Still DEFAULT OFF. What would make this bakeable: (a) re-run at higher n to firm up the 4/5
rate and characterize the `on/5`-type residual failure, (b) an actual landing attempt (this
test's IC/timeout wasn't tuned for descent-to-touchdown, only for "does it start descending"),
(c) the same test on `cm_dim`/`cm_lowsun`/`cm_col`.

## ✅ FIRM-UP: n=15/arm on `cm_inv`, combined across three batches (2026-09-04)

Pooled the 2026-09-04 SITL flight test with two follow-up batches (5+10 more reps, same
harness/IC/world). One batch ran mid-fix for a real concurrent-session-safety bug in the
harness (see [[feedback_recurring_analysis_mistakes]]-adjacent note below) — the fix itself
doesn't change flight physics, and results before/after look identical in character, so all
valid reps are pooled.

**min_h (lowest altitude reached), n=15/arm:**

| | stuck at ~5m floor | breaks below floor |
|---|---|---|
| off (baseline) | **13/15 (87%)** | 2/15 — but into OTHER bad outcomes, not progress: one `TARGET_LOST` descending nearly blind to 0.004m while ending 5.1m off-target, one partial `NOT_LANDED` at 3.6m |
| on (ens_ring) | 3/15 (20%) — reproduces baseline's floor exactly | **12/15 (80%)** — 5 moderate (3.2-3.8m), 4 substantial (1.5m), 3 near-touchdown (<0.12m) |

Confirms the n=5 finding at 3x the sample: **baseline gets stuck 87% of the time; `ens_ring`
breaks that in 80% of reps.** Neither arm landed in any rep across all 15.

⛔⛔ **SEPARATE, SERIOUS FINDING mid-investigation: the A/B harness's concurrent-session
guard was UNSAFE.** The guard checked ONCE before the loop; `ko()`, called before every one
of the 10-20 reps in a run, did an unconditional `kill -9` on every px4/gz/MicroXRCEAgent
process with NO re-check. If another session started SITL any time after the initial guard
passed, `ko()` would have killed it SILENTLY — and there is no way to prove after the fact
whether that happened (checked PX4's own boot logs for the original failure window: 13 boots,
all under one instance-0 dir, consistent with this script's OWN retry logic, but that does
NOT rule out a second session, since a second session's default launch hits the same
instance-0 ports). **User caught this by direct challenge, not something I surfaced myself.**
Fixed in all three harnesses (`cminv_ab.sh`/`loomr_ab.sh`/`ensring_ab.sh`, commit `9bd79574`):
`ko()` now re-verifies after its own cleanup, and if SITL processes are STILL present — which
cannot be something `ko()` itself spawned — the script ABORTS instead of killing again and
launching on top of it. Applies to ALL future `*_ab.sh` harnesses of this shape.

Still DEFAULT OFF.

## ✅ FIFTH ATTEMPT: span-balance confirm + a REAL upstream bug found via user challenge (2026-09-04)

User proposed a 4th discriminator: a cross's true centre sits INSIDE each matched arm's own
point span (points on both sides); a plate corner is the ENDPOINT of both edges meeting there
(points almost entirely on one side). Asked why not score BOTH matched arms, since all lines
of a real cross intersect at the same centre.

**First implementation (score both arms) genuinely failed**: 40%+ false-positive cost on
clean `base` -- barely better than random. I (WRONGLY) attributed this to the marker's
one-sided STUB (`STUB_REL_ANGLE_DEG=45`) getting matched against a long arm, "fixed" it by
scoring only the longer/more-supported arm, and it worked (FP 0-4% on 5/6 scenes). **That
explanation was checked with a FLAWED verification** (overlap against `det.stub_points` --
which is `None` in EXACTLY the frames where the stub got consumed into the main pair, since
the separate stub-ID step explicitly excludes `k in (i,j)` -- so the check silently
undercounted). User challenged this directly ("why can't you score both, they all intersect
at the same center") and was right to.

⭐⭐⭐ **CORRECT ROOT CAUSE, found via the arms' fitted ANGLE instead**: 100% of the
both-arms-scored false positives are frames where `_best_pair` matched the STUB as one of the
two "main" arms (verified: rel_angle of the picked pair ~45deg, not ~90deg). And this is not
rare: **41.6% of ALL clean-scene frames** have the stub picked into (i,j) instead of a real
diagonal. Root cause in `_best_pair` itself (a pre-existing bug, 2026-08-01 code, unrelated to
any of this session's work until now): a HARD two-phase gate --
`_best_pair(require_support=True) or _best_pair(require_support=False)` -- requires BOTH
clusters in a pair to individually clear `MIN_CLUSTER_SUPPORT`, and returns the FIRST
support-passing pair found, however bad its angle, WITHOUT EVER comparing it against a much
better pair that only narrowly missed the support bar. Quantified: **30.7% of stub-picks
(12.5% of ALL frames) are this exact bug** -- a near-perfect diagonal pair (`|rel-90|~0.6deg`)
exists but loses to a fully-supported stub pair (`|rel-90|~45deg`) purely because one
diagonal's Hough segments landed one below the support floor. The other 69.3% genuinely have
no near-90 pair available that frame (Hough missed a whole arm) -- a harder, separate,
still-open problem (recall, not selection logic).

## ✅ FIX LANDED, UNCONDITIONAL (not a flag -- a correctness bug, like the alpha_0/cell-ID fixes)

`_best_pair` replaced with a single SOFT-PENALIZED search across every pair (no hard gate, no
two-phase fallback): `score = |rel_angle-90| + PAIR_SUPPORT_PENALTY * support_deficit`
(`CROSS_PAIR_SUPPORT_PENALTY`, default 8.0deg per missing support point). Support becomes a
TIEBREAKER, not a veto -- preserves the original 2026-08-01 fix's intent (stop a spurious
near-90 noise pair beating a real, under-supported arm) while no longer letting a support
advantage override a vastly better angle fit.

**Verified (RobustnessFrameset, replaying every captured candidate pair per frame):**
- Eliminates the bug's stub-picks 100% on base/bright/darkbg/lowsun, partially on dim/col/inv
  (residual = the genuinely-no-pair-available case, unfixable by scoring alone).
- **ZERO regressions** across all 7 scenes -- no frame where a previously-good pick became a
  stub pick.
- Insensitive to the exact penalty value (tested 5.0-15.0, byte-identical outcome) -- not a
  fragile hand-tune.
- **Centroid position accuracy: FLAT** (base/etc within-0.15 unchanged to within 1pt). Traced
  why: the stub ALSO passes through the true centre (it's a real feature of the marker), so
  even the "wrong" pair intersects near the right point -- just less precisely-conditioned.
  ⚠ Don't expect this fix to move accuracy% headlines.
- **`heading_deg` (alpha/orientation) AVAILABILITY: the real, measured effect.** +11-13
  points on every clean scene (base 45.6%->59.2%, darkbg 61.0%->72.8%, lowsun 41.2%->54.0%,
  bright 45.4%->56.5%). Mechanism: `heading_deg` is the STUB's own direction, found by a
  SEPARATE step that excludes whatever `(i,j)` already consumed -- so whenever the buggy
  `_best_pair` ate the stub, heading became UNAVAILABLE entirely (not noisy), falling back to
  hold-last-good. `col`/`inv` barely move (upstream, segmentation-level failures).
- Diagnostic `CROSS_DIAG_PAIR_SELECT=1` -> `PAIR_SELECT_DIAG` kept in-tree (matches
  `HOUGH_DIAG_LOG`'s pattern) for future debugging of the still-open 69.3% (Hough-recall) case.

## ✅ SPAN-BALANCE CONFIRM landed too, `CROSS_BALANCE_CONFIRM=1` (DEFAULT OFF)

Once corrected to score only the longer/more-supported arm (now a REASONABLE heuristic given
the pair-select fix removes most stub-mismatches upstream, though not a substitute for fixing
`_best_pair` itself -- both were landed): `CROSS_BALANCE_MAX_DIST=0.30` default.

**Full eval, on top of the pair-select fix (detOK / within-0.15):**

| variant | base | bright | col | darkbg | dim | **inv** | lowsun |
|---|---|---|---|---|---|---|---|
| baseline (fix only) | 100/97 | 95.0/96 | 63.0/82 | 99.6/94 | 100/82 | 99.1/0 | 100/97 |
| balance | 100/97 | 95.0/96 | 51.3/84 | 99.6/96 | 99.2/83 | 41.3/0 | 100/97 |
| ring+balance | 100/97 | 91.1/96 | 48.0/84 | 99.6/96 | 99.2/83 | 31.3/0 | 100/97 |
| **ens_ring_balance** | 100/97 | 94.1/96 | 53.9/84 | 99.6/96 | **95.1/88** | 33.0/0 | 98.4/98 |

⭐ Complementary to ring, not redundant (different information: span geometry vs neighbourhood
topology) -- combined catches MORE of inv/col's silent lies (detOK down further, i.e. more
honest refusals) with a real accuracy GAIN on `dim` (82->88, the only variant to move it) and
no new cost on base/bright/darkbg/lowsun. `inv` within-0.15 is STILL 0% under every
combination -- none of these fix `inv`'s actual accuracy, only its honesty.

⛔ Still DEFAULT OFF (the confirm stage). The `_best_pair` fix is DEFAULT ON (unconditional).
NOT yet SITL-flight-tested -- unlike ens_ring's must-not-regress gate, this matters MORE to
validate live since it is now on by default on every flight, not opt-in.


## ✅ SITL MUST-NOT-REGRESS GATE for the _best_pair fix (2026-09-05, n=5)

`test_data/PairFix_AB_harness/pairfix_ab.sh`, interleaved, `WORLD=cross_marker
MARKER_TYPE=cross` IC2 (2,2,5), fixed (new default) vs `CROSS_LEGACY_PAIR_SELECT=1`
(exact pre-fix behavior, rollback switch added alongside the fix for this A/B).

| arm | precise | median xy | detOK |
|---|---|---|---|
| fixed (default) | 4/5 | 0.061 | 100.0% (5/5) |
| legacy (pre-fix) | 1/5 | 0.212 | 100.0% (5/5) |

No regression -- the fix comes out ahead on this small sample (legacy's one `NOT_LANDED`,
min_h 2.413, was a stall rather than a lateral miss). n=5 is too small to prove the fix is
*better* (this platform's lateral outcome is known-stochastic at low n), but there is nothing
here to hold the fix back on. `CROSS_LEGACY_PAIR_SELECT` kept as a permanent rollback switch
(matches this module's other always-available knobs) in case the soft-penalty scoring
misbehaves in a regime this offline eval + n=5 gate didn't cover.

## ⚠ SITL FLIGHT TEST on the actual benefit scenes: cm_dim / cm_lowsun / cm_col (2026-09-05, n=5/arm)

`test_data/SceneFix_AB_harness/scenefix_ab.sh`, PERCEPTION mode (detector actually flying),
IC2 (2,2,5), off (baseline) vs `ens_ring_balance` (`CROSS_GATE_MODE=ensemble
CROSS_RING_CONFIRM=1 CROSS_BALANCE_CONFIRM=1`, the single best offline combination). First
live test of these confirm stages on the scenes they were actually built for -- only `cm_inv`
had been flight-tested before this (project_20260903_cm_inv... entries above), `cm_dim`/
`cm_lowsun`/`cm_col` were offline-only.

**`cm_dim` -- INCONCLUSIVE, confounded by a SEPARATE issue.** 0/5 landed in EITHER arm.
`min_h` scattered similarly in both arms (several near-zero, several not) -- looks like both
arms hit the already-documented terminal-overfill control-side stall (project's #1 open
blocker), which swamps whatever detector-side signal this test was meant to isolate. Two bad
outliers: `on/3` TARGET_LOST xy=8.6 (lost the marker, dropped nearly to ground blind);
`off/5` FAIL xy=12.1 (severe fly-away). Neither arm is clean here; this scene doesn't cleanly
test the fix.

**`cm_lowsun` -- NO MEANINGFUL DIFFERENCE, but corrects a misreading.** ⚠ Initially misread the
`FAIL` classification (10/10 reps, both arms) as a systemic problem -- checked the raw log
and it is NOT: impact detector fires normally (`|a|=58.4 > 50.0 -> LANDED=True`), 100% detOK
both arms, real controlled landings. `FAIL` here means `xy_err` narrowly missed the strict
`<=0.1m` PRECISE/SOFT threshold, nothing more.
  xy_err  off: 0.147 0.191 0.239 0.136 1.244  (median 0.191, one outlier)
          on:  0.385 0.167 0.129 0.396 0.335  (median 0.335, no outliers)
Roughly a wash -- off's median is tighter but carries the only outlier; on is more consistent
but typically looser. Both arms land reliably here, unlike cm_dim.

**`cm_col` -- GENUINELY MIXED: real median gain, real tail-risk cost.**
  xy_err  off: 1.094  3.657 1.285 1.991 2.145   (median 1.991)
          on:  0.428 15.385 0.453 1.031 1.381   (median 1.031)
  detOK   off: 97.2   58.1  98.9 100.0  95.0    (worst 58.1)
          on:  97.4   34.0  97.7  98.3  89.3    (worst 34.0)
`on` wins 3/5 reps clearly (median ~half of off's) but its WORST rep is a genuine outlier --
15.4 m fly-away with detOK collapsing to 34%, meaningfully worse than off's worst (3.7 m,
58%). Plausible mechanism: being MORE selective about rejecting bad detections can
occasionally leave the pipeline with NO usable measurement during a rough patch, which is
worse than a lower-quality-but-PRESENT measurement (open-loop drift vs noisy-but-closed-loop
tracking). This is a real trade-off (typical-case accuracy vs tail-risk), not a clean win.

## ⛔ VERDICT: none of the three scenes support flipping the confirm stages on by default

`cm_dim` uninterpretable (confounded by a separate, already-known issue), `cm_lowsun` shows no
benefit, `cm_col` shows a real accuracy gain in the typical case traded against a worse
tail-risk failure -- a judgment call, not evidence to act on at n=5. `ens_ring_balance` (and
its component flags) all remain DEFAULT OFF.

**What would move this forward:** (a) fix or route around the cm_dim terminal-overfill
confound before re-testing there; (b) higher n on cm_col specifically to see whether the
15.4 m outlier is representative of a real tail-risk rate or a one-off; (c) investigate WHY
the confirm stages can occasionally starve col of all detections during a bad patch -- a
targeted fix for THAT failure mode (e.g. a rescue path when EVERY candidate gets rejected)
might convert the median gain into a clean win without the current tail risk.

## ✅ MARGIN-GATED RESCUE for ring+balance TOGETHER (2026-09-05) -- targets col's tail risk

Direct follow-up to the SITL finding above: `on/2` col's fly-away recording (detOK 34%,
xy=15.4m) showed `balance_not_centered` alone caused 106 of 657 total rejections that flight
(the largest single new-stage contributor, more than `ring_not_a_junction`'s 26) -- and
neither new stage had a rescue path, unlike the existing `centroid_mismatch` check
(`CROSS_CENTROID_SPAN_RESCUE`).

**Measured the mechanism precisely** (RobustnessFrameset, disagreement frames = one stage
rejects, the other passes): on `col`, frames where the PASSING stage passed only narrowly had
median GT error 0.154 (borderline); frames where it passed COMFORTABLY (>=50% headroom past
its own threshold) had median error 0.019-0.010 -- genuine detections that only tripped one
gate's precise cutoff, not corners. **Checked `inv` does NOT show the same pattern before
landing anything**: `inv`'s disagreement frames stay bad even at a comfortable margin (median
error 0.433 at margin 0.5-1.0, vs col's 0.019) -- `inv`'s wrong detections are confidently
wrong in a way both independent geometric tests correctly smell out, margin or not.

**Landed**: when RING_CONFIRM and BALANCE_CONFIRM run TOGETHER, replace independent hard
gates with a margin-gated rescue -- if one stage rejects but the other passes with >=50%
headroom past ITS OWN threshold (`CROSS_RING_BALANCE_RESCUE_MARGIN`, default 0.5), accept
anyway. Single-flag mode (only one of the two enabled) is UNCHANGED -- verified byte-identical
to before via the same offline eval.

**Full eval (detOK / within-0.15):**

| variant | col before -> after | inv before -> after | dim | bright | others |
|---|---|---|---|---|---|
| ring_balance | 48.0/84 -> 57.0/86 | 31.3/0 -> 37.8/0 | 99.2/83->99.6/83 | 91.1/96->95.0/96 | unchanged |
| ens_ring_balance | 53.9/84 -> 59.1/84 | 33.0/0 -> 34.3/0 | 95.1/88->95.4/88 | 94.1/96->95.0/96 | unchanged |

⭐ col detOK +9.0pts (ring_balance) / +5.2pts (ens_ring_balance), within-0.15 +2pt or flat --
a real, targeted gain with no accuracy cost. ⚠ inv's `within-0.15` STAYS EXACTLY 0% either way
(the safety property that matters is unaffected) but detOK does rise some (+6.5pt / +1.3pt) --
a partial, measured give-back of inv's honesty gain, smaller than the naive "require both
stages to reject" alternative (tested and rejected: that gave inv detOK 24.1%->74.6%, far too
much reversion). This margin-gated version is the smallest rescue that still fixes col.

**NEXT: re-run the col SITL A/B (5 reps, same design as the earlier flight test) to check
whether the rescue actually resolves the 15.4m fly-away outlier live, not just offline.**

## ⚠ SITL RE-TEST of col with the rescue: severity reduced, NOT eliminated (2026-09-05, n=5)

`test_data/SceneFix_AB_harness/scenefix_cm_col_rescue.tsv` vs the earlier `scenefix_cm_col.tsv`
(pre-rescue), same design (n=5 interleaved, off vs `ens_ring_balance`, IC2, perception mode).

**Within this run** (the valid comparison -- off/on interleaved, temporal drift shared):
off was clean throughout (detOK never below 98.7%, no `TARGET_LOST`); on still had one clear
collapse (`on/2`: detOK 70.5%, xy 5.757) and one moderate dip (`on/5`: 84.3%). **The tendency
for `on` to occasionally detect worse than `off` on this scene is still present.**

**But severity dropped substantially** vs the pre-rescue run: worst-case detOK 34.0%->70.5%,
worst-case xy 15.385m->5.757m -- roughly half as bad on both counts. Consistent with the
rescue mechanism working as designed (catching SOME of the false-rejections that would
otherwise cascade into total measurement blackout), just not all of them.

⚠ **Honest confound**: `off`'s own results were also notably cleaner in this run than the
earlier one (worst detOK 58.1%->99.1%, no TARGET_LOST at all this time) despite off's code
being byte-identical between runs -- session/scene variability, not attributable to the fix.
Part of the apparent improvement could be a generally easier run, not purely the rescue. The
WITHIN-run interleaved comparison (which controls for this) still shows on > off risk, just
less severely.

**n=5 (effectively n=1 "bad" data point per arm per run) cannot resolve "reduced severity" from
"noise."** Verdict: REAL, MEASURED, PARTIAL improvement -- not a fix. `ens_ring_balance` stays
DEFAULT OFF. Would need a substantially larger n (10-15+) specifically targeting this collapse
mode to say anything more precise about the residual rate/severity.

## ✅✅ MUCH STRONGER VALIDATION: 5 independent GT-FB flights, rescue improves EVERY SINGLE REP

Addresses the confound in the perception-mode col SITL re-test above (small n, off's own
results shifting between runs, closed-loop trajectory divergence muddying comparison). Under
GT-feedback the trajectory is fixed by the reference regardless of what the detector sees, so
every variant can be scored offline against the SAME real flight -- no closed-loop confound,
and only N flights needed (not N x arms) since every variant is a deterministic re-score of
the same recorded frames.

`test_data/GtfbMulti_col/` -- 5 independent `PLASMC_GT_FEEDBACK=1` flights on `cm_col`, IC2
(2,2,5), `CROSS_RING_OVERLAY_DBG=0` (clean frames), raw PNGs saved
(`tools/GtfbMulti_col/record_col_gtfb_multi.sh`, pattern copied from
`record_robustness_set.sh`). ~420-460 frames/rep, ~2200 frames total -- ~5x the single
RobustnessFrameset `col` recording this whole thread has relied on until now.

**detOK, no-rescue (`CROSS_RING_BALANCE_RESCUE_MARGIN=1.0`, unreachable thresholds) vs the
landed default (margin=0.5), per independent flight:**

| variant | rep1 | rep2 | rep3 | rep4 | rep5 | mean delta |
|---|---|---|---|---|---|---|
| ring_balance: no-rescue | 56.3 | 52.5 | 51.9 | 55.6 | 55.0 | |
| ring_balance: rescued | 59.7 | 60.9 | 58.4 | 61.6 | 63.2 | **+6.5pt, every rep +** |
| ens_ring_balance: no-rescue | 59.2 | 54.5 | 53.0 | 56.5 | 54.3 | |
| ens_ring_balance: rescued | 62.1 | 61.2 | 57.9 | 61.4 | 58.7 | **+4.8pt, every rep +** |

within-0.15 (accuracy): flat to +1pt mean on both variants, **no regression in any of the 10
rep-comparisons** (5 reps x 2 variants).

⭐⭐ **This is a materially stronger result than the perception-mode SITL A/B could give**: the
rescue improves detection rate on EVERY SINGLE one of 5 independently-recorded flights, for
both variant combinations, with zero accuracy cost anywhere. No outliers either direction --
this is not noise. The margin-gated rescue design is validated.

⚠ What this does NOT show: whether the improved detOK actually converts to better CLOSED-LOOP
landing outcomes in perception mode (that's what the earlier small-n SITL A/B measured, with
its own confounds). The two results are complementary, not contradictory: this shows the
rescue reliably improves what the detector reports; the earlier test showed a real but partial
reduction in closed-loop tail-risk severity. Together they support keeping the rescue landed
(a893a77e) while `ens_ring_balance` overall stays DEFAULT OFF pending more
closed-loop evidence.

## ✅ INNOVATION-GATED ADAPT FALLBACK — Kalman-filter-style fusion (2026-09-05)

User asked directly: "Can we fuse estimates from multiple approaches using a Kalman-filter
method?" Distinguished two different things bundled under "multiple approaches": `ring`/
`balance`/`geom` are GATES on the SAME estimate the primary pipeline produces (nothing to fuse,
only a confidence weight -- already done via the ring+balance rescue above); `baseline`/`adapt`
are genuinely INDEPENDENT front-ends (different pixels -> different Hough fit -> different
center estimates) -- these are the real fusion candidates.

**Measured**: `adapt` detects far more than baseline on `col` (90.5% vs 65.2% mean detOK) but
is far less accurate when it fires (77.0% vs 83.4% within-0.15) -- two sensors with different
bias/variance. A literal inverse-variance-weighted fusion on frames where BOTH fire does almost
nothing (baseline's variance ~33x smaller, so the average is ~97% baseline by construction).
Naively ACCEPTING adapt whenever baseline is silent is a WASH (3/5 col recordings less
accurate, 2/5 more, as coverage rose ~30-40%).

**Fix, mirroring the ring+balance rescue's own logic**: gate the fallback on agreement with a
prediction, not accept-any-estimate. This is a Kalman innovation test in its simplest form --
predict (hold-last-good leveled position), measure (adapt's candidate), gate on residual
distance. At `CROSS_FALLBACK_GATE_DIST=0.15`, offline analysis showed EVERY one of 5
independent col recordings improving simultaneously in BOTH coverage (+21-36%) and accuracy
(+1.4 to +6.5pt) -- no tradeoff anywhere, unlike anything else measured this week.

⚠⚠ **HONEST GAP between that idealized offline estimate and the verified shipped behavior.**
The offline estimate assumed `adapt` runs CONTINUOUSLY (its own track_state warm every frame).
The shipped fallback only invokes `adapt` SPORADICALLY (on primary misses), so its own ROI-
tracking never locks on the way a continuously-run detector would. **Verified by driving the
REAL `process_frame()` end-to-end** (not a standalone re-implementation) on the same 5
recordings: pooled detOK 79.0%->82.2% (+3.2pt, +41% coverage) -- net positive, real, but 2 of 5
reps mildly negative (-0.7, -1.8) against 3 clearly positive (+3.1, +9.4, +5.5). **Report the
verified number, not the idealized one.**

Also found and fixed a real bug during this verification: the FIRST implementation updated the
held-reference from ANY accepted detection (primary or fallback), letting an accepted,
less-accurate `adapt` estimate become next frame's gate reference -- compounding. One live test
showed a real regression (78.2%->75.2%) the offline validation never had. Fixed by updating the
held reference from the PRIMARY pipeline's own detections ONLY.

**Generalization check (user asked): rover data.** Tested on `DetectorFrameset`'s
`rover_IC2`/`rover_IC4` (oblique-view geometric clutter, a structurally different challenge
from col's lighting/colour) via the same real `process_frame()` path. Both mildly positive,
zero regressions: rover_IC2 within-0.15 75.5%->77.3% (n 159->176), rover_IC4 88.9%->89.5%
(n 487->494). Smaller effect than col (rover's baseline detOK is already 87-99%, leaving less
room), but the mechanism does not hurt on a structurally different failure mode.

Landed as `CROSS_FALLBACK_ADAPT_GATE=1` (DEFAULT OFF), `CROSS_FALLBACK_GATE_DIST=0.15`.
`_adapt_gate_override` context manager in `cross_marker_detector.py` lets the fallback force
the adapt gate for one call without touching the primary pipeline's global config. Uses
HOLD-LAST-GOOD, not a real velocity-aware KF prediction (the existing `_scen_kf_x` centroid KF
would predict better across a longer gap) -- validated first version; upgrading the reference
is a natural follow-on if hold-last-good's staleness ever shows up as a problem.

NOT yet SITL-flight-tested. Offline/replay-verified only (both col GT-FB and rover
DetectorFrameset), matching this thread's now-standard practice of full end-to-end code-path
verification before any flight test.

## ✅ FIRST SITL FLIGHT TEST of the innovation-gated adapt fallback (2026-09-05, n=5/scene)

`test_data/FallbackAdapt_AB_harness/fallback_ab.sh`, interleaved, off vs
`CROSS_FALLBACK_ADAPT_GATE=1`, IC2 (2,2,5), perception mode.

**Ordinary `cross_marker` scene (must-not-regress) -- PASSED, with a bonus improvement:**

| arm | precise | median xy | detOK |
|---|---|---|---|
| off (default) | 1/5 | 0.105 | 100.0% (5/5) |
| on | 3/5 | 0.081 | 100.0% (5/5) |

No detection collapse in either arm. `on` outperforms `off` even on the ordinary scene where
the fallback should rarely need to trigger (baseline usually succeeds) -- a genuinely good
sign, though n=5 doesn't prove it's not noise. This mattered more to check than for the
confirm-stage flags (ring/balance/ensemble), since this fix is a real default-on candidate,
not just an opt-in experiment.

**`cm_col` (the target scene) -- neither arm landed, but `on` is measurably better where it
matters:**

| arm | median xy | detOK median | detOK worst |
|---|---|---|---|
| off | 2.314 | 99.4% | 33.5% (one severe collapse) |
| on | 1.586 | 100.0% | 66.0% (no collapse as severe as off's worst) |

Confirms the ALREADY-DOCUMENTED terminal-overfill stall affects both arms on this scene
regardless of the detector fix (consistent with `SceneFix_AB_harness`'s earlier `cm_dim`/`col`
findings) -- neither arm actually touches down. But comparing the two arms directly: `on`'s
median xy is ~31% tighter, and its worst-case detection collapse is far less severe than
`off`'s. Not every rep favoured `on` (rep4 roughly a wash; rep5 `on` reached much closer to
the ground, min_h 0.570 vs `off`'s 5.015, before losing the target with a worse final xy).
Direction matches the offline validation; n=5 with zero landings on either arm cannot prove
the fix works on `col`, only that it doesn't regress and moves the right way.

**Verdict**: real, positive first flight evidence, no regressions found. `CROSS_FALLBACK_ADAPT_GATE`
stays DEFAULT OFF pending more data (higher n, and ideally a scene where `col`'s separate
terminal-overfill confound doesn't mask whether landings actually improve).
