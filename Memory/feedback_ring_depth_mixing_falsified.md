---
name: ring-depth-mixing-falsified
description: "The 'ring lstsq is depth-mixed, so a fixed ring->corner cal does not generalize' rationale (was in _compute_ring_flow docstring + my own reasoning) is FALSE for this scene. Falsified 2026-06-06 by theory + r~0.95 data. Ring CAN be calibrated/generalized; its real limits are noise + no-position + ground-relative."
metadata:
  node_type: memory
  type: feedback
  originSessionId: ring-cal-2026-06-06
---

**Claim (FALSIFIED):** "the lstsq V_v_ring is depth-MIXED (rings span ground+board),
so a fixed ring->corner cal does not generalize; the loom vz/z is depth-free and does."

**Why it's wrong:** `_fill_A` builds the IBVS interaction matrix with **perpendicular
depth Z (camera-frame z), normalized to 1** — NOT ray length. The ArUco board is
**coplanar with the flat ground**, and the ring lstsq runs in the **gravity-leveled
V-frame** (`_getVirtualPts`), so the plane is frontoparallel → **every pixel (corners
AND ring stations) shares one depth Z = altitude.** No depth spread → not depth-mixed.
(My earlier "Z = h/cosθ varies across the FoV" was ray length, irrelevant to the L-matrix.)
The "spans ground+board" premise only bites if the board is physically RAISED; it isn't.

**Evidence (GT-free, ring h_z vs corner h_z, 5 RingFlow descents):** r = **0.92–0.97 at
ALL altitudes** (5m→0.1m); bias small and ∝ signal magnitude (a fixed multiplicative
offset a fixed cal removes), NOT ∝ altitude. Depth-mixing would force divergence at
altitude; they don't diverge. (My first slope-vs-GT test showed ring slope "less stable"
but that was noise in R²≈0 bins — signal floor above 0.25m — and I retracted it.)

**Corrected mechanism:** the loom's value is **ROBUSTNESS** (texture-free median,
survives marker death), NOT depth-invariance — on this uniform-depth scene the lstsq h_z
already gives vz/Z, same as the loom. So a fixed **M_ring generalizes** and is derived
like the corner cal (see [[ring-flow-calibration]]).

**The real limits on generalizing ring flow** (none is depth): (1) flow is differential —
**no absolute target POSITION/centroid** (rings give velocity, not where the pad is) →
can't close the outer loop alone; one-time absolute acquisition needed, then handoff;
(2) rings are **ground-anchored, not target-anchored** — coincide only for a static
coplanar pad (hence r~0.95); for a MOVING target `target_motion = corner_flow − ring_flow`
(rings DECOMPOSE ego vs target — a generalization, not a failure); (3) **texture/planarity**
(low-texture white cells → noisy LK; engineerable via board texturing).

**Why:** a documented design rationale (code docstring) was geometrically wrong; it had
been about to drive "rings can't be a fixed cal / can only be a safety net" conclusions.
**How to apply:** treat ring flow as a calibratable, generalizable scale-free flow signal
(divergence + FoE lateral). Don't re-derive the depth-mixing story. The `_compute_ring_flow`
docstring + inline comment were corrected 2026-06-06. See [[outputcal-flow-validation-vframe]]
(V-frame leveling), [[wxy-unobservable-imu-fusion]] (corner-clustering degeneracy broken by the spread board; "genuine rank deficiency" OVERTURNED 2026-06-07, unrelated to ring depth).
