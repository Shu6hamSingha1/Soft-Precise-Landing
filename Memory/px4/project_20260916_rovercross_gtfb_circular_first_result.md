---
name: project_20260916_rovercross_gtfb_circular_first_result
description: "FIRST moving-target GT-FB result on the cross-marker (rover_cross) world (2026-09-16): baseline Circular, IC2, PLASMC_GT_FEEDBACK=1, WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross, n=3. 3/3 PRECISE (xy_err 0.043-0.081m), 0 target_lost, all landed on platform (min_alt 0.51m). Much better than the July 2026-07-02/03 ArUco-rover Circular baseline (0/4, 1.0-1.7m miss, yaw-ramp windup) -- attributed to PLASMC_YAW_RATE_LAW baked default-ON 2026-09-09, a week after that campaign, which directly fixes the yaw-ramp-windup mechanism the old baseline hit."
metadata: 
  node_type: memory
  type: project
  originSessionId: f78fde4c-2847-4689-96f6-0693b2bc0c15
  modified: 2026-09-15T18:46:57.380Z
---

**First moving-target GT-FB test on the cross-marker world (2026-09-16) — fills the gap identified
same session, and lands far better than the stale ArUco-rover baseline.**

**Why this was missing:** every prior moving-target GT-FB result — Linear 3/3 on-platform, Circular
baseline 0/4, AU_LEAD-tuned Circular 2/3 — was on `rover_aruco` (`project_moving_rover_landing_works`,
`project_rover_turning_open`, 2026-07-02/03). Verified by file provenance, not just labels: the
`cross_marker` model file dates to 2026-08-09, `rover_cross.sdf` to 2026-08-26 — over a month after
that campaign, so it could not have used cross-marker. The only prior GT-FB + `rover_cross` data was
STATIC ([[project_20260901_rover_cross_perception_diagnosis]], 3/3 landed dead-centered, rover not
moving). My own `XirXi2_RoverGTFB` sweep earlier this session also defaulted to `rover_aruco` (never
set `WORLD`/`ROVER_MODEL`) — also not cross-marker.

**The test.** `test_data/RoverCross_GTFB_Circular/harness.sh`: baseline config (no `AU_LEAD`, no
tuning — yaw ASMC/rate-law whatever is currently default), IC2 (2,2,5), `ROVER_TRAJ=Circular`,
`ROVER_MOTION=1`, `PLASMC_GT_FEEDBACK=1`, `WORLD=rover_cross ROVER_MODEL=rover_cross MARKER_TYPE=cross`.
n=3, no retries needed (3 valid in 3 launches).

**Result: 3/3 PRECISE.**

| rep | xy_err | rel_vel | target_lost | min_alt | target motion span |
|---|---|---|---|---|---|
| 00-04-14 | 0.043 m | 0.47 m/s | False | 0.516 m | 4.64 m |
| 00-06-51 | 0.081 m | 0.43 m/s | False | 0.508 m | 4.51 m |
| 00-08-00 | 0.054 m | 0.63 m/s | False | 0.513 m | 4.64 m |

All landed on the platform (min_alt ~0.51 m = 0-0.02 m above surface), confirmed real ~4.5 m Circular
motion span (not a stationary rover). `soft=False` on all — not meaningful here: relative velocity to
a moving target isn't near-zero at touchdown by definition, same caveat as every other moving-target
SoftPrecise reading this session.

**Why so much better than the July baseline (0/4, 1.0-1.7m miss, yaw-ramp windup — see
[[project_rover_turning_open]] mechanism 1):** `PLASMC_YAW_RATE_LAW` was baked default-ON
**2026-09-09** ([[project_yaw_rate_law_sign_bug_and_validation]]), a full week after the July
campaign — it replaces the windup-prone `kappa_a` ASMC/`psi_d` double-integrator (the exact mechanism
that detonated the old Circular baseline) with a direct rate law that doesn't saturate on a turning
target. This result is NOT primarily "cross-marker transfers cleanly from ArUco under GT-FB" (though
that assumption held, since GT-FB bypasses the image pipeline and marker geometry shouldn't matter to
control) — it's landing on top of a since-fixed controller bug the July baseline never benefited from.
The old AU_LEAD tuning work in [[project_rover_turning_open]] may now be unnecessary for Circular at
this IC/speed; not re-tested here.

**How to apply:** this is n=3, single IC (IC2), one trajectory (Circular @ default speed) — don't
over-generalize to Sinusoidal/Lissajous/EightShape or other ICs without testing them. No video was
recorded (`CHASE_CAM` not set in the harness) — add it for a follow-up rep if footage is wanted.
Cheap next steps: rerun the OLD ArUco-rover baseline under current code (isolate whether the
improvement really is YAW_RATE_LAW and not something else that changed since July) before treating
this as a validated fix for the whole turning-target thread.
