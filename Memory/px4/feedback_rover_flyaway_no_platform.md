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
~0.5 m — restoring the contact-arrest the aruco case has.

**✅ FIX APPLIED + CONFIRMED (2026-07-02).** Added a `landing_platform` link to the
`rover_aruco` model.sdf: a 0.6×0.6 pedestal (0.1→0.5 m), fixed-jointed to `base_link`,
with the 1 m ArUco marker as a VISUAL on its top face (top at 0.5 m). ⚠ The marker must
be a visual ON the platform link, NOT the old separate `<include model://arucotag>` —
that static/massless link DETACHED and fell to z=−48 m once the model's link/joint tree
changed. Model files are OUTSIDE the repo: `~/.gazebo/models/rover_aruco/model.sdf` AND
`~/PX4-Autopilot/Tools/simulation/gz/models/rover_aruco/model.sdf` (keep both in sync;
gz reads ~/.gazebo first). Re-run (GT-FB, stationary, same config): the drone **lands ON
the platform at min-alt 0.51 m, dead-centered (lat 0.05 m), BOUNDED, no fly-away** — vs
2/3 fly-aways without the platform. Confirms the root cause + fix. (A/B rover+platform
arm was flaky on the frozen-pose infra flake so n was small, but the 0.51 m bounded
landing is decisive and matches the aruco parity.)

Harness: scratchpad/ab_flyaway.sh + ab_analyze.py (world A/B, per-arm autosave, min/peak
alt + fly-away metric). Stochastic — judge by fly-away RATE over n≥3, not single runs
(the ±5–7 noise floor + terminal-1/Z amplification, [[project_why_sp_achieved]]).
See [[feedback_rover_spawn_infra_fixes]], docs/MOVING_TARGET_PREP.md.

## ⭐ TERMINAL LATERAL-DRIFT IS RESOLVED — baseline 5/5 CLEAN (2026-07-02)
Attempted to "tune the terminal lateral-drift to improve the landing rate", but the
premise was CONTAMINATED. A clean landing-rate batch (rover+platform, GT-FB, CORRECT
config — pose-index fixed, chase cam on ground_plane so no index shift) gave **baseline
5/5 CLEAN**, every rep dead-centered on the platform (min 0.51 m = platform top, lateral
0.02–0.05 m, 0 fly). So the terminal lateral-drift the task targeted is ALREADY resolved
by (1) the landing PLATFORM (contact arrests before the open-air terminal-1/Z regime) and
(2) the POSE-INDEX fixes. The earlier "~1/3 clean / drifts / flys" were the CHASE-CAM
POSE-INDEX BUG (separate chase_cam model shifted target/UAV indices → wrong poses → garbage
control, see [[feedback_rover_spawn_infra_fixes]]) + no-platform geometry — NOT a control
deficit. My interim s_e_n-divergence / ζ_r-saturation diagnosis was run on index-bug-era
data (03-51/03-52) → also contaminated. **No GT-FB control tuning is warranted (baseline at
ceiling).** Candidate levers (P2INF_xy=2.0, PRINF=1.0) NOT run — no upside vs a 5/5 ceiling.
Next real frontier = perception-ON (GT-FB control is clean; perception may reintroduce
failures) and yaw-cal for a MOVING (turning) rover. Batch: scratchpad/batch_landrate.sh.
⚠ SITL still flaky: a frozen-pose startup race (huge pos_err + speed 0) costs ~2 retry
attempts/rep — self-recovers, but makes n≥6 batches slow (~5 min/rep).
