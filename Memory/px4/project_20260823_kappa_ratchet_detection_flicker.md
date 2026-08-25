---
name: project_20260823_kappa_ratchet_detection_flicker
description: "Terminal kappa-ratchet/actuator blow-up found on an off-center IC2 ArUco flight — CBF_CORNERS_STALE's 30-frame-streak guard doesn't trip against intermittent detection flicker on top of a sustained raw-detector freeze, so kappa keeps integrating against a frozen s_e_n"
metadata: 
  node_type: memory
  type: project
  originSessionId: db521dfb-a6f1-44ca-b3f6-432e98fc4866
  modified: 2026-08-23T12:41:51.087Z
---

Found 2026-08-23 while confirming the az-visibility-CBF work (unrelated — this is a pre-existing bug, not introduced by that work). GT-verified per the diagnose-flight-data skill (compared against raw `Ground_Truth.npy` poses, not controller-internal signals).

**Symptom:** IC2 (off-center) ArUco perception-mode flight — controller converged normally (`xy_err` 2.78m→1.16m over the descent) then suddenly kicked laterally ~1.8m in the final 0.5s before touchdown (`rel_vel=5.74 m/s`, `target_lost=True`).

**Mechanism, traced end-to-end:**
1. `Img_Data.npy`'s raw `Image Feature Pts` (the true per-frame ArUco decode, not the controller's derived signal) freezes — byte-identical corners for 43 consecutive frames, `t=39.604s` through touchdown at `t=40.532s` (~0.93s with zero new decodes). Root physical cause of the raw freeze itself not yet traced (candidate: marker too large/close for reliable decode at extreme close range — a "point-blank decode death" class already seen elsewhere in this project, e.g. `feedback_cross_marker_radial_spread_ceiling`).
2. `controller.py`'s `s`/`s_e_n` freeze in lockstep (`s_e_n` stuck at exactly `(-0.417, 0.722)` for the same window) — expected, they derive from the same raw detection.
3. **But `d_min_fov` (i.e. `cbf_corners`, fed through a SEPARATE small_slot/feature_pts/coast_hold selection cascade) flickers `0 ↔ real value` almost every tick during this same window**, even though the underlying raw detector is fully dead. Some tier of that cascade (likely the coast-hold's own grace-frame budget) intermittently re-validates the same stale corners.
4. `kappa`'s RK5 update is already gated — frozen when `CBF_CORNERS_STALE` (`_cbf_corners_none_streak >= 30` consecutive frames) is true. This guard exists specifically to prevent kappa-ratchet during feature loss (added 2026-07-30, ported from Hardware). **It never trips here** because the flicker keeps resetting `_cbf_corners_none_streak` back toward zero before it reaches 30 — the guard was designed for sustained loss, not intermittent flicker layered on top of a sustained underlying freeze.
5. `kappa[0]` climbs `0.78→2.07` over the window (integrating every tick against a `sigma` built from the frozen `s_e_n`), and `a_u` explodes 4 orders of magnitude (`~(-1,0.9) → (27,24) → (51,29) → (-13559,-346) → (1743,4430)`) — the lateral kick.

**Not yet done:** why the raw detector froze for 0.93s at this specific close-range/off-center moment (traced the freeze, not its physical trigger); which specific tier of the `cbf_corners` selection cascade was flip-flopping; a fix.

**Candidate fix direction (not implemented):** gate `kappa`'s freeze condition on something that isn't defeated by flicker — either the raw detector's own freshness (not `cbf_corners`' post-cascade validity) or a streak counter that isn't reset by a stale coast-hold re-validation.

**Scope note:** per [[feedback_aruco_perception_scope]] / [[project_marker_roadmap_gt_ablation]], this was found on ArUco (now comparison-only) — if pursued, the fix should target the generic mechanism (kappa's freshness gate), not be chased via further ArUco-specific test flights.
