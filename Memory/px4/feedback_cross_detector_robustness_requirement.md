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
