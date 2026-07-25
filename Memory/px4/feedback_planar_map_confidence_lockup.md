---
name: feedback_planar_map_confidence_lockup
description: "2026-07-25: PlanarFeatureMap confidence-lockup bug found INDEPENDENTLY on Pi hardware AND in Gazebo SITL data the same day. Once a homography fit fails (resid_px=inf), frame_to_map freezes and refill keeps poisoning new points through it -- self-reinforcing loop that can pin confidence/map_confidence at exactly 0.0 permanently, even after the marker is cleanly re-tracked. Fixed on Pi (self-heal: track degenerate-fit streak, full reset + re-bootstrap past a time threshold) and ported into Gazebo (commit b420d3a) after independent code review confirmed correctness. Explains 2+ prior catastrophic fly-aways including the previously-unresolved 55.78m IC1_rep5."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-25T17:16:18.068Z
---

**Discovery.** User found this live on Pi hardware (`PLANAR_MAP_DBG=1`): after one bad
homography event, `confidence`/`map_confidence` pinned at exactly 0.000 for 14+s straight
while the ArUco marker was cleanly re-tracked (`roi_hits=30/30`) for long stretches
afterward. Independently, scanning all 1788 past ICValidation `Img_Data.npy` files in
Gazebo (this session) for the same signature (both confidence fields pinned at 0.0 for a
long consecutive stretch, with `fresh_decode=True` + low `err_px` reappearing inside that
stretch) found **16 hits (~0.9% of reps)**, 8 of which coincided with `TARGET_LOST`,
including **two genuine catastrophic fly-aways**: `IC1_rep5` (`ICValidation/20260722-
152133`, 32.4m/15.1m/s DRIFT_OFF) and `IC1_rep5` (`ICValidation/20260721-141516`,
55.78m/8.34m/s) -- the LATTER is the exact rep already sitting in
[[project_ic1_kappa_leakage_drift_20260721]] as an unresolved "kappa leakage" mechanism;
this lockup may be the actual (or an additional) root cause.

**Mechanism** (`src/planar_map.py::update()`): the homography refit needs
`len(common_ids) >= 4`; on failure (`cv2.findHomography` returns `None`, or the
near-collinear-spread rejection), `self.resid_px = np.inf` and `self.frame_to_map` is
**not updated** -- frozen at its last value. `resid_conf = max(0, 1 - resid_px/ceiling)`
is therefore exactly 0.0, hard-zeroing both `confidence` and `map_confidence` (both
multiply `resid_conf`) regardless of `track_conf`. Critically, `loop_closure_correct()`
(called on every fresh ArUco decode) only re-anchors the marker's OWN `map_pts` by
projecting *through* the current (possibly frozen/bad) `frame_to_map` -- it never
validates or repairs the transform itself. So a fresh, accurate decode makes the marker's
map points *consistent with* a frozen/bad transform without ever feeding new truth back
into it -- a self-reinforcing, circular poisoning loop with no existing recovery path.
Confirmed live in Gazebo (`IC1_rep5`, `20260722-152133`): `rigid=False`/`conf=mapconf=
0.000` pinned for the full remaining ~7s of flight while `fresh_decode=True` with
0.1-3px error continued for nearly 4 straight seconds -- the marker was tracked
perfectly, the map never recovered anyway, and tracking quality then genuinely
degraded later too (err_px ballooning to 300-700px), consistent with the poisoning
feedback loop actively getting worse, not just a stale confidence readout.

**Fix (Pi hardware, ported to Gazebo commit b420d3a; NOT applied to the Pi's copy by
this session -- left untouched at user's request).** `PlanarFeatureMap` tracks
consecutive degenerate (`resid_px=inf`) frames via `_degenerate_since_t`
(TIME-based, not frame-count -- Pi and Gazebo run at very different effective rates,
so frame-count is a poor cross-platform proxy for wall-clock time; the Pi fix went
through exactly this frame-count -> time-based revision mid-session). Past
`degenerate_reset_seconds` (default 0.4s, comfortably under `MARKER_LOSS_GRACE`'s
1.0s), calls `_full_reset()` (clears `map_pts`/`next_id`/`tracked_px`/`frame_to_map`/
`marker_slots`/`_next_slot_id` -- NOT just `bootstrap()` alone, which would leave stale
slot/feature-id references dangling) then `bootstrap(gray, quat_R=quat_R)` fresh from
the current frame.

**Code review verdict (independent, before learning the mechanism matched what was
already traced in Gazebo): correct.** Verified `bootstrap()` is safe to call mid-flight
(self-contained, no first-call-only assumptions). Verified `_full_reset()`'s coverage
against every consumer of the cleared fields. Trigger condition (`not
np.isfinite(resid_px)`) is defensively correct (also catches NaN). Two accompanying
Pi-side finding also ported: RANSAC bounds (`ransac_max_iters=200,
ransac_confidence=0.98` vs OpenCV's 2000/0.995 defaults) -- found via live Pi profiling
that RANSAC was 76% of per-frame cost (38-70ms), and bounding it measured BETTER
`map_confidence` AND lower `resid_px` at every tested bound, not just faster (not a
speed/accuracy tradeoff).

**Validated in Gazebo:** n=15 (IC1/IC2/IC4, mixed with a `PLANAR_MAP_DBG=1` probe) +
n=3 IC3/IC4/IC5 regression -- **zero fly-aways across all 5 ICs**, the cleanest full-IC
sweep of the whole session (IC1 2/3 SP, IC4 2/3 SP). `SELF-HEAL` itself did not fire in
this Gazebo sample (matches its ~0.9% measured historical rate -- rare-trigger safety
net, absence in a small sample is not a sign it's broken). The mechanism is separately
validated correct via code review and live Pi hardware testing (per user: "We have
validated the fix with hardware").

**Process note:** files had diverged three independent, unmerged ways before this
reconciliation -- Pi had the RANSAC-bounds fix (07-23) and the self-heal fix (07-25);
Gazebo had the `primary_zero_corners` rescue-gate fix (07-25, [[feedback_rescue_gate_zero_corner]]).
All three are now reconciled into the Gazebo-tracked copy. **The Pi's copy has NOT
been updated with `primary_zero_corners`** -- if working on the Pi side in a future
session, port that fix there too (or accept the two copies will keep diverging).
