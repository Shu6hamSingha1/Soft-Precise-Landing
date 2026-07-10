---
name: project_partial_marker_parallelogram_recovery
description: "Next-step candidate (2026-07-09, not started) - extend parallelogram-completion corner recovery from the KLT-fallback tier (transient single-corner LK dropout) to genuinely partial/occluded marker detection (never fully decoded), for the terminal marker-overflow failure mode."
metadata: 
  node_type: memory
  type: project
  originSessionId: 32b3f473-5219-48ce-b070-323ef9cc309a
---

**Idea (user-proposed, 2026-07-09):** parallelogram completion (`missing = other1 + other2 - diagonal_partner`, pure 2D pixel-space, no depth/scale/intrinsics needed — see [[feedback_klt_relax_gate_parallelogram]] for the exact math) is a good general primitive for extracting feature points from partially-visible markers, not just the narrow case it was built for.

**Current scope (as shipped 2026-07-09, commit 9346aac-adjacent):** parallelogram completion only fires inside the KLT-fallback tier, which requires `self._prev_aruco_pts` — corners from a PRIOR successful full 4-corner ArUco decode, now being LK-*tracked* forward. It rescues a transient single-corner tracking dropout on an already-locked marker. It does NOT help a marker that's never been fully decoded in the first place (enters frame already partially occluded, or clipped by the frame edge from the start) — ArUco's decoder returns nothing at all in that case (no ID, no corners), so there's no seed to LK-track or complete from.

**Why: How to apply:** to extend this to true partial-visibility recovery (relevant to the terminal marker-OVERFLOW failure mode, [[feedback_terminal_overflow_deck_flyaway]]), need a DIFFERENT front-end that can identify >=3 marker-boundary candidate points WITHOUT a full ArUco decode. Candidates already in the codebase: `PLASMC_DENSE_RECOVER` (RANSAC homography over dense tracked points, default-off, older validation was "MIXED" per project_dense_recovery_and_failure_tagging — predates this session), or a dedicated corner/edge detector on the visible marker fragment. Either could feed into the SAME parallelogram-completion math once it produces >=3 candidates.

**Important caveat (from this session's investigation):** the parallelogram identity assumes weak-perspective/affine viewing (planar square -> parallelogram in image); it's only exact under mild perspective (small marker-apparent-size-to-distance ratio, near-fronto-parallel viewing) and degrades under strong perspective (keystoning) — i.e. large apparent marker size (close range) + elevated tilt. The regime where partial-visibility recovery would help MOST (marker overflow near touchdown) is EXACTLY the regime where this approximation is WEAKEST. Don't assume it transfers cleanly to the overflow case just because it validated for the momentary-flicker case (mid-descent, moderate tilt, small marker) — the accuracy there needs separate validation before relying on it terminally.

**Status: NOT STARTED.** Scoped as a follow-on after the current KLT-fallback fix ([[feedback_klt_relax_gate_parallelogram]]) is validated via IC1-5 n>=5 (in progress 2026-07-09). Revisit once we see whether overflow/drift-off failures still dominate the remaining failure modes after the current fix lands — if they don't, this may be lower priority than expected.
