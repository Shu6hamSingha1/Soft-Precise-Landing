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

## ⚠ UNMEASURED + an UNRESOLVED infra bug

`flat` and `clutter` WITH the ceiling were never measured: the launcher failed to start on
nearly every attempt -- `PX4 SITL did not bring up Gazebo's /world/cross_marker/clock in 60s`
with PX4 stuck in "Waiting for Gazebo world...".

Pattern that identified it: **flat-after-flat 3/10 launch failures, flat-after-ROVER 7/9** ->
looked like the `SH_REFERENCE` §10 stale-gz-server-on-world-switch problem (the rover's
two-instance server outliving cleanup and holding the old world; the plain `gz sim` CLI pattern
does not match the server process). **A hardened `ko()` (kill every `gz-sim` variant + block
until no `/clock` topic is advertised) did NOT fix it** -- a fresh flat batch still failed 3/3.
So the world-switch theory is UNCONFIRMED and the root cause is still open. Until it is fixed,
**interleave worlds rather than running them in blocks**, so a world-specific launch failure
shows up immediately instead of after a wasted batch.

Process note: `pkill -f spanrescue_ab.sh` issued from a shell whose own command line contained
that string killed the shell mid-edit -- pitfall #1 in the sh-script-patterns skill, committed
anyway. Kill by PID read from a file.
