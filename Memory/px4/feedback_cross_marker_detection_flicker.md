---
name: feedback_cross_marker_detection_flicker
description: "cross_marker_detector.detect() is a from-scratch Hough-line detector with zero temporal memory, structurally more flicker-prone than ArUco's cascade; CBF_CORNERS_STALE windowed-fraction fix (marker-agnostic) + an opt-in KLT center-bridge (cross-marker-specific, default off) both landed 2026-08-23 in commit 0431eab"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-24T05:10:01.749Z
---

Root-caused 2026-08-23 while investigating a terminal kappa-ratchet/actuator blow-up (see [[project_20260823_kappa_ratchet_detection_flicker]], found on ArUco but the underlying vulnerability generalizes).

**Root cause:** `cross_marker_detector.detect()` re-runs a full geometric pipeline from scratch every single frame — color-gate mask → Hough line detection → angle clustering → line-pair intersection fitting → centroid/bbox plausibility checks (~10 sequential hard-gated failure points: `color_gate_empty`, `hough_lt2_lines`, `lt2_angle_clusters`, `no_pair_found`, `near_parallel_pair`, `insufficient_fit_points`, `near_parallel_fit`, `ill_conditioned_intersection`, `centroid_mismatch`, `center_outside_bbox_margin`, `extrapolation_too_far`). `track_state` only crops the search ROI (`last_bbox`/`miss_count`) — it does not track points. Any single-frame perturbation (motion blur, a Hough vote falling just short, a borderline angle cluster) flips `det.ok` for that one frame with zero memory of the previous frame's success.

**`_center_fresh`** (the flag driving `cbf_corners` under `MARKER_TYPE=cross`) is a raw per-frame boolean with **no hysteresis at all** — structurally *more* exposed to this than ArUco's `small_slot` mechanism, which at least has a 5-frame accumulation before switching ON.

**Fix 1 (marker-agnostic, `controller.py`):** `CBF_CORNERS_STALE`'s consecutive-streak counter is defeated by flicker (any single "found" frame resets it to 0, even mid-sustained-loss). Added a windowed-fraction check (default 40-frame window, trips at ≥50% `None`), ORed with the existing streak — never weakens the fast sustained-loss case, closes the flicker gap. Deliberately **not** applied to `CBF_CORNERS_STALE_ABORT` (that property's whole design point is tolerating long normal coast bursts for the mission-abort decision).

**Fix 2 (cross-marker-specific, `cross_marker_perception.py`):** a bounded, plausibility-gated **KLT center-bridge**. `_compute_hw` already runs real KLT point tracking every frame regardless of `det.ok` (previously only feeding the h,w Jacobian), and `_prev_flow_cell_id` persists (subsetted, never resampled) across consecutive miss frames. New: match tracked points by `cell_id` across a miss streak, use their median displacement to bridge `center_px`/`last_bbox` from the last confirmed detection instead of dropping straight to `center_fresh=False` on any single Hough miss. Rejects (does not clip) an implausible displacement relative to the marker's last known size — same "reject, don't clip" philosophy as the ArUco PlanarFeatureMap rescue's own gate. **Default OFF** (`CROSS_CENTER_BRIDGE_FRAMES=0`) — new mechanism, not yet flight-tested. Enabling/validating it is cross-marker+GT-feedback-track work per the roadmap ([[project_marker_roadmap_gt_ablation]]).

**Scope note (2026-08-24):** confirmed this flicker mechanism is a DIFFERENT issue from [[project_20260824_crossmarker_offcenter_convergence_wall]] — that IC2 off-center failure happened with `Detection Status='ok'` on 450/450 frames (clean, continuous detection), so it's a pure control-law (kappa-leakage) issue, not caused by this flicker problem. Don't conflate the two when deciding what to fix next.

**Not done:** empirically measuring which specific Hough-pipeline failure_reason actually dominates in practice (the `_diag_log`/`Detection Status` field is exposed and recorded, just not yet analyzed across a real flight with actual misses — the one flight checked so far had zero raw misses); validating the KLT bridge; other robustness options considered but not built (temporal mask smoothing at the color-gate stage, carrying forward Hough angle-bin priors, a relaxed-gate partial-match analogous to ArUco's "3/4 corner" tier) — see the corresponding conversation for the fuller list of untried ideas.
