---
name: project_20260902_spanrescue_sitl_gate
description: "SITL gate of CROSS_CENTROID_SPAN_RESCUE (f49f567f) on rover_cross/flat/clutter. HEADLINE NEGATIVE RESULT: the offline detOK gain does NOT translate to flight -- clutter detOK 5-7%->22-73% while outcomes got WORSE (median xy ~1.9->~5.6 m, two 12 m fly-aways). On rover_cross it fixes acquisition decisively (detOK 48->98%, TARGET_LOST 3/5->1/5) and converts blind dives into stalls above the pad, but 0/5 land either way and fly-aways persist. A fill ceiling (CROSS_CENTROID_SPAN_FILL_MAX=0.6) removes every overfill rescue as designed but does NOT fix the fly-aways. KEEP DEFAULT OFF."
metadata:
  node_type: memory
  type: project
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**2026-09-02.** First SITL gate of the span rescue landed in `f49f567f` (see
[[project_20260901_rover_cross_perception_diagnosis]] for the offline case). Harness:
`test_data/Rover_AB_harness/spanrescue_ab.sh` (interleaved arms, `WORLD_KIND=rover|flat|clutter`).

## ⭐ HEADLINE: an offline detector gain that did NOT survive contact with the controller

**`clutter` n=5 (flat world + one dark box), no fill ceiling:**

| arm | detOK | xy_err | outcome |
|---|---|---|---|
| off (n=2 valid) | 5.2 %, 7.1 % | 1.125, 2.724 | both TARGET_LOST |
| on (n=5) | 21.9-72.9 % | 5.601, 1.120, **12.308**, **12.601**, 5.094 | 3 TARGET_LOST, 1 FAIL, 1 NOT_LANDED |

Detection improved 5-7 % -> 22-73 % and **flight outcomes got WORSE** (median xy ~1.9 -> ~5.6 m,
two 12 m fly-aways). The legacy centroid check fails on the MAJORITY of clutter frames
(620-804/run) and the rescue admitted ~half (309-454), including 13-41 at overfill.

**METHODOLOGY LESSON (the durable part): score perception changes on ACCURACY and FLIGHT
OUTCOME, never on detect-rate alone.** The offline harness predicted clutter 50->87.7 % detOK
with median centroid err 0.304->0.034 and that looked like a clear win; in flight the admitted
detections were wrong often enough to actively steer the aircraft away. The offline signal that
DID predict this was `within-0.15` (rover 95.9 %->75.9 %) -- the recall/precision trade, not the
headline rate.

## rover_cross IC2, static, perception mode, n=5/arm

| | detOK median | TARGET_LOST | landed ON pad | fly-aways |
|---|---|---|---|---|
| off | 6.7 % (no ceiling) / 48.1 % (ceiling batch) | 3/4, 3/5 | 0 | 0 |
| on  | **100 %** | **0/5**, 1/5 | 0 | 1 (49.8 m) |

Acquisition is decisively fixed and the FAILURE MODE CHANGES QUALITATIVELY: `off` runs end
BELOW the pad (-0.19..-0.54 m = blind dive to the ground beside it), `on` runs end ABOVE it
(+0.26..+0.48 m = tracking and holding). That is the Mode A -> Mode B conversion -- the
perception blocker clears and the known control-side terminal stall then binds. **But 0/5 land,
and lateral is highly variable (0.13-2.71 m).** 100 % detection is not 100 % CORRECT detection.

## Fill ceiling `CROSS_CENTROID_SPAN_FILL_MAX` (default 0.6) -- works, but is NOT the fix

Rationale: the legacy tolerance already ramps 0.12->0.72 as the marker fills the frame, so a
frame failing even that is badly wrong and admitting it at touchdown is worst-case. Evidence
that motivated it: the ONE flat fly-away (27.8 m) was the ONLY flat run with overfill rescues
(2), and its approach was healthy (lateral 2.87->0.11 m, centroid err 2-20 mm) until alt<0.3 m
where the error exploded 0.077->0.86 in <1 s.

**Result: overfill rescues -> 0 in every run (from 2 flat / 13-41 clutter), as designed. But
rover rep4 still flew away 49.8 m WITH ZERO overfill rescues.** ⛔ So the overfill hypothesis
explained the *timing* of one outlier, not the fly-away MECHANISM. Do not treat the ceiling as
having fixed that class.

## VERDICT

**Keep `CROSS_CENTROID_SPAN_RESCUE` DEFAULT OFF.** It correctly repairs the acquisition failure
it was built for and is the right diagnosis of `centroid_mismatch`, but it admits enough wrong
centroids to cause fly-aways, and it does not serve the robustness requirement
([[feedback_cross_detector_robustness_requirement]]) at all -- it is a patch on one validation
gate, not lighting/colour/texture robustness.

## ⛔⛔ SITL IS DOWN (2026-09-02 ~21:41) — blocks ALL further gating

Every launch now fails, in BOTH worlds, with PX4 `ERROR [init] Timed out waiting for Gazebo
world` / `Startup script returned with return value: 256`. A DIRECT manual
`run_aruco_landing.sh` fails identically, so **it is not the harness**. `gz sim` CLI
additionally throws a Qt5 ABI error:

```
libgz-sim8-gz.so.8.15.0: /lib/x86_64-linux-gnu/libQt5Quick.so.5:
undefined symbol: _ZNK16QDoubleValidator8validateER7QStringRi, version Qt_5
```

(The Qt5 libs are months old, so that specific error may be long-standing and NOT the PX4
path's cause — but nothing brings a world up.) It broke MID-SESSION: rover ran fine 21:26-21:38
and flat at 21:39-21:40, then nothing worked. **Needs machine-level repair (gz/Qt package fix
or a reboot); do not `apt`/reboot without asking the user.**

### ⛔ RETRACTION: the "stale gz server on world switch" theory was WRONG

I recorded flat-after-flat 3/10 launch failures vs flat-after-ROVER 7/9 and concluded it was
the `SH_REFERENCE` §10 stale-server-on-world-switch problem, then hardened `ko()` to kill every
`gz-sim` variant and wait for `/clock` to clear. **That did not fix it, and the rover probe
afterwards failed the SAME way** — so the failure is GLOBAL, not world-specific. The
"flat-after-rover" pattern was just ORDERING: the rover batch ran BEFORE the breakage, the flat
batch after. Classic confounding-of-time-with-condition; the fix is to interleave conditions,
which would have exposed it immediately. (The hardened `ko()` is harmless and worth keeping,
but it is not the fix for anything observed.)

## ✅ RESOLVED 2026-09-03 — DEFAULT FLIPPED TO **ON** (`a53a5f63`), SITL-gated

Interleaved n=5/arm gate on flat + clutter WITH the fill ceiling, zero launch flakes
(worlds interleaved so a world-specific failure shows immediately -- the ordering lesson):

| world | metric | off | on |
|---|---|---|---|
| flat | PRECISE | **0/5** | **3/5** |
| flat | xy median | 0.143 | **0.046** |
| clutter | TARGET_LOST | **5/5** | **1/5** |
| clutter | detOK median | ~11 % | **~53 %** |
| rover (earlier) | detOK median / TARGET_LOST | 48 % / 3-5 | **98 % / 0-5** |

**Overfill rescues: 0 in EVERY run of both worlds** (24-120 consulted, all refused). The
ceiling holds, and the pre-ceiling clutter fly-aways (12.308 / 12.601 m) did not recur.

Raw data preserved: `test_data/SpanRescue_Gate/`. Revert: `CROSS_CENTROID_SPAN_RESCUE=0`.

### ⭐ MECHANISM (the part worth carrying forward)

**The rescue does not improve the fit -- it stops DISCARDING good fits.** The chain:

1. `centroid_mismatch` validates the fitted line intersection against the MASK PIXEL
   CENTROID. Foreign structure fused into the SAME connected component (rover platform edge,
   the clutter box) drags that centroid off the junction -- the fit is right, the REFERENCE
   is contaminated.
2. Rejecting the frame leaves the controller with **no measurement**: `FEATURE_IS_VISIBLE`
   goes false, `s` goes stale, lateral error cannot close.
3. The drone then either declares `TARGET_LOST` or drifts on a stale centroid. **The
   "fly-aways" were never control instability -- they were the controller flying BLIND.**
   (Mode A: 0 % detection 5 m -> 1 m, lateral frozen 2.9 -> 2.3 m, 0.8-1.5 m/s drop onto the
   ground BESIDE the pad.)
4. Restoring those frames restores closed-loop feedback -> the blind dive becomes a held
   station.

**The ceiling covers the OPPOSITE failure.** At overfill the mask centroid legitimately
drifts (which is why the legacy tolerance already ramps 0.12 -> 0.72), so a frame failing even
that loosened check is badly wrong, and admitting it AT TOUCHDOWN is worst-case. Hence:
rescue = restore feedback during acquisition/approach; ceiling = refuse to inject garbage at
touchdown. Two halves, opposite jobs.

### ⭐ The unexpected result: it helps on CLEAN scenes too

flat has **100 % detOK in BOTH arms**, yet PRECISE goes 0/5 -> 3/5. Since detOK (measured
above 1 m) is identical, the gain is BELOW that band -- in the LATE APPROACH, after the marker
grows enough that the mask centroid starts drifting but before the ceiling cuts in. Only 1-3
rescues/run, and that is enough to keep lateral feedback live through final approach. So this
was never a rover-only fix.

### ⛔ NOT explained by either mechanism (open)

- The rover **49.8 m fly-away occurred with ZERO overfill rescues** -- unaccounted for.
- **Clutter is NOT fixed**: neither arm lands. `on` holds station at min_h 1.3-4.9 m instead
  of diving blind. Better failure MODE, not a success. Detection is still intermittent
  (34-65 %).
- This gate says NOTHING about [[feedback_cross_detector_robustness_requirement]] -- one
  lighting condition, one polarity, one texture.

## ⛔⛔ SITL was down 2026-09-02 -> restored 2026-09-03 by a Qt5 LD_PRELOAD WORKAROUND

Every launch failed with PX4 `Timed out waiting for Gazebo world`; PX4's own log showed the
cause -- it spawns Gazebo via the `gz sim` CLI, which aborts:

```
libgz-sim8-gz.so.8.15.0: /lib/x86_64-linux-gnu/libQt5Quick.so.5:
undefined symbol: _ZNK16QDoubleValidator8validateER7QStringRi, version Qt_5
```

System `libqt5gui5` is `5.15.3+dfsg-2ubuntu0.2+esm3`; `libqt5quick5` is
`5.15.3+dfsg-1ubuntu0.1~esm1` -- a split ESM stream, so qtdeclarative references a symbol
qtbase no longer exports. **`apt install --only-upgrade` CANNOT fix it** (qtdeclarative is
already newest at esm1). Real fix = downgrade qtbase to match, or wait for an ESM
qtdeclarative rebuild.

**WORKAROUND (in the harness):** preload the PyQt5-bundled Qt5Gui, which DOES export the
symbol:

```
LD_PRELOAD=/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5
```

⚠ **Anything launching SITL outside `spanrescue_ab.sh` still needs this preload** until the
packages are fixed.

### ⛔ Two of my own diagnoses here were WRONG -- recorded so they are not reused

1. **"stale gz server on world switch"** -- from flat-after-flat 3/10 launch failures vs
   flat-after-rover 7/9. A hardened `ko()` did not fix it and a rover probe then failed
   identically, so the failure was GLOBAL. The pattern was pure ORDERING (rover ran before the
   breakage, flat after) = time confounded with condition. **Interleave conditions.**
2. **"it broke at ~21:41"** -- wrong in mechanism. `dpkg.log` shows NO package activity that
   day and the libs date from Feb/May, so nothing broke; an accidental workaround (a PyQt5 Qt5
   path on the loader path) most likely stopped being present. Also: the user's "maybe it
   worked headless" hypothesis was tested directly and REFUTED -- `QT_QPA_PLATFORM=offscreen`
   gives the identical error.
