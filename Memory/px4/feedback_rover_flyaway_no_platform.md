---
name: feedback_rover_flyaway_no_platform
description: "Controlled n=3 A/B diagnosing the rover terminal fly-away: it is ROVER-SPECIFIC (aruco 0/3 fly vs rover 2/3, identical GT-FB config). Root cause = the marker is a floating visual-only plane 0.5 m above the rover with NO landing surface, so the terminal 1/Z danger-zone sits in open air instead of coinciding with ground contact. Fix = give the rover a real landing platform."
metadata:
  node_type: memory
  type: feedback
---

**Controlled A/B (2026-07-02), GT-FB baked config, n=3, ONLY the world differs:**
| arm | fly-aways | outcome |
|-----|-----------|---------|
| stationary `aruco` | **0/3** | all land dead-centered (lat 0.08/0.19/0.20 m), min-alt at ground (−0.01 m) |
| `rover` (stationary rover) | **2/3** | peaks 270–299 m / lateral 827–960 m; 1 clean at 0.06 m |

→ The violent terminal fly-away is **ROVER-SPECIFIC, not a control-law regression**
(the identical GT-FB config lands 3/3 dead-centered on the stationary marker).

**RULED OUT** (both cheap, decisive):
- Target-pose JITTER: both GT targets have ~0 position noise (rover pos_std ~1e-20 mm);
  GT-FB gets equally clean inputs in both worlds.
- Camera/marker MOUNT-OFFSET bias: fixed (marker−camera in V-frame) — fly-away persists.

**ROOT CAUSE = no landing surface at the marker height.** The rover `arucotag` is a
1 m **visual-only `<plane>`** mounted +0.50 m above the rover, with NO collision. The
highest rover collision is the body box (~0.1 m). So:
1. GT-FB drives camera→marker, i.e. to a 0.5 m-high point with nothing to touch down on.
2. Because the marker is 0.5 m up, the terminal small-depth / high-1/Z regime (where the
   LATENT terminal kick fires) is reached while the drone is still ~0.8 m base altitude
   (camera-marker depth ≈0.49 m at base 0.8 m) — i.e. in OPEN AIR, no contact below to
   arrest it. The kick develops and stochastically launches the drone (2/3). When it
   happens to stay centered it descends past to the body/ground and "lands" (1/3, 0.06 m).
In the stationary `aruco` case the marker is at ground level, so the small-depth danger
zone coincides with immediate GROUND CONTACT → the same latent kick is harmlessly
arrested → 0/3 fly. The terminal 1/Z kick (whole [[project_residual_cycle_wumax_bake]]
campaign) is the SAME latent instability; the rover geometry just UNMASKS it.

**FIX (setup, and physically correct for a moving-PLATFORM landing):** give the rover a
solid landing PLATFORM (collision box) at the marker height so the drone lands ON it at
~0.5 m — restoring the contact-arrest the aruco case has — or lower the marker onto the
rover body top. Optionally a terminal commit that fires at platform height. Then re-run
the A/B; the rover should match the aruco arm.

Harness: scratchpad/ab_flyaway.sh + ab_analyze.py (world A/B, per-arm autosave, min/peak
alt + fly-away metric). Stochastic — judge by fly-away RATE over n≥3, not single runs
(the ±5–7 noise floor + terminal-1/Z amplification, [[project_why_sp_achieved]]).
See [[feedback_rover_spawn_infra_fixes]], docs/MOVING_TARGET_PREP.md.
