---
name: project_flowlatreduced_targetlost_gate_20260726
description: "2026-07-26: three fixes committed -- decode-staleness confidence decay (map_confidence, reconciled with Pi's independent same-day fix), a runtime attitude-rate gate for FLOW_LAT_REDUCED (fixes the long-flagged-not-implemented 2026-07-10/11 gap), and TARGET_LOST leveling + bounded go-around recovery in landing_test.py. Commits 5341e14/dcec397/ce513dc."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-26T17:27:41.987Z
---

**1. Decode-staleness confidence decay (commit 5341e14).** See
[[feedback_planar_map_confidence_lockup]] for the fuller history -- this closes it out.
Decays `map_confidence` (not `confidence`) as elapsed time since the last genuine
`loop_closure_correct()` grows, so a long KLT-only coast can't keep reporting high
`map_confidence` on self-consistency alone. Time-based from the start (learned from the
self-heal fix's own frame-count->time-based correction). Gazebo and Pi versions now
reconciled/identical in mechanism (Pi target field choice, `map_confidence`, was correct;
Gazebo's first draft wrongly targeted `confidence`, corrected before commit).

**2. `FLOW_LAT_REDUCED` runtime attitude-rate gate (commit dcec397).** Closes the gap
flagged since 2026-07-10/11 (`project_ic1_terminal_kick_root_cause_chain`): "the
FLOW_LAT_REDUCED model-misspecification itself is NOT fixed... needs runtime gating on
real attitude rate." Root mechanism (independently re-confirmed twice this session,
`IC1_rep2` and `IC2_rep3`): the reduced 4-DOF flow solve's `w_xy≈0` premise is about the
CAMERA's own roll/pitch RATE, not the target's static tilt (`FLOW_TARGET_LEVEL` only
checks the latter) -- a real, fast attitude excursion (often ignited by an earlier
marker-switch/handover misattribution event) gets silently folded into the lateral flow
measurement instead of being attributed to rotation, feeding a spurious velocity command
that can ignite/sustain a real physical tumble. Fix reads the FC gyro every frame
(`angvels`, always available) via the same `_vframe_w` transform the centroid-rate
observer uses; if real `|w_x|,|w_y|` exceeds `FLOW_LAT_REDUCED_WMAX_RADS` (default
0.3 rad/s), falls through to the full 6-DOF solve for that frame. Validated n=20
(IC1/IC2/IC3) with the gate confirmed firing routinely (2-442x/rep) and zero fly-aways;
A/B on IC5 (n=8/n=8) found no significant difference and no firing-frequency correlation
with severity -- the gate is not implicated in IC5's separate pre-existing variance.

**3. `TARGET_LOST` leveling + bounded go-around (commit ce513dc).** Root cause
(traced live, `IC5_rep3`, `ICValidation/20260726-165649`): the marker-lost open-loop
fallback commanded zero body *rate* (`send_attitude_rate(0,0,0,thrust)`), freezing
whatever tilt the vehicle had at the exact TARGET_LOST instant (measured
roll=-23deg/pitch=-12deg) rather than leveling it -- fixed thrust at a frozen bank angle
produced an accelerating ballistic ejection (climbing altitude mid-flight before falling,
62m/15.7m/s). This behavior was appropriate for its OTHER trigger (the separate,
already-validated `terminal_perception_loss`/loom-commit stationary-target path, which
only fires once already converged/near-level -- see
[[feedback_terminal_kick_commit_vs_live]], "open-loop freeze-and-fall is dumb-but-robust
for a STATIONARY target only") but was being reused, unmodified, for the general
TARGET_LOST failure path that can strike at any attitude. Fix (1): actively command
roll/pitch rate proportional to current measured tilt (stays on `send_attitude_rate`, not
the MAVSDK-only `send_attitude`, to avoid a mid-descent transport switch if
`CMD_TRANSPORT=dds` is active). Validated: worst case 62m->13m, the climb-then-eject
pattern eliminated, though residual lateral VELOCITY built up before disengagement isn't
retroactively cancelled by leveling attitude alone (a further refinement, not implemented).
Fix (2), per explicit user directive: on marker-loss-beyond-grace, climb to 5m (holding
current lateral position/yaw), reset perception/control state (reusing the existing
`CONTROLLER_READY` rising-edge reset), and resume closed-loop control -- bounded by
`LANDING_GO_AROUND_MAX_ATTEMPTS` (default 2) before falling to the open-loop descent.
**`target_lost` is STICKY, set on the FIRST genuine loss regardless of whether go-around
later recovers** (user: "the go-around approach is still a failure, I am just doing this
so we have additional data" -- corrects an earlier draft that wrongly deferred the
failure tag until attempts were exhausted). Verified end-to-end via live log trace
(`IC5_rep5`, `ICValidation/20260726-211545`): two go-around attempts fired, each reached
5.0m, marker re-lost both times, correctly fell through to open-loop, and the final
`Ground_Truth.npy` `SoftPrecise` dict shows `target_lost=True` as intended.

**Cross-cutting note:** items 2 and 3 both trace back to (or interact with) the same
family of `IC5`/aggressive-offset-IC failures characterized repeatedly this session
(marker-switch-triggered tumbles, large-offset+short-runway divergence) -- none of these
three fixes claims to fully solve IC5; each closes a specific, previously-open mechanism
while leaving IC5's fundamental large-offset/short-runway/velocity-momentum tension as
still-open, already-catalogued residual work.

**Process note (self-correction):** mid-session, launched a second SITL validation run
without confirming the first had finished, causing ~15 min of resource contention between
two concurrent Gazebo/PX4 instances (stale pre-fix job vs. the corrected-code job) before
being caught and killed. No data corruption resulted (the stale job's results were simply
discarded), but a reminder to always verify a prior background job's completion status
before launching a new one, not just check that a monitor was set up for it.

**Perception migration handover** (not a memory-tracked artifact, written for the
Windows/Pi-side Claude session, scratchpad-only): explains the perception pipeline
architecture, platform-agnostic vs. platform-specific component classification, and this
session's full Gazebo/Pi divergence-and-reconciliation history, for migrating this
pipeline to real hardware validation.
