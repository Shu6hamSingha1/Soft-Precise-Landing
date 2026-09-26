---
name: project_20260925_pending_perception_s_tasks
description: 2026-09-25 CLOSE-OUT — pending tasks after the perception-s-vs-GT-s investigation (stroke detector + s-loss fade + touchdown settle all baked as defaults). Read this first when resuming.
metadata:
  type: project
---

State at close: defaults `CROSS_DETECTOR=stroke`, `PLASMC_S_LOSS_FADE=1`, `PLASMC_TD_SETTLE_S=1.0` are live and committed (11a3d741 + results commit). One configuration for stationary AND moving targets — user requirement: never branch on target type. Full findings: `PX4_Gazebo/test_data/SPercGTFB_results/FINDINGS.md`; story in [[project_20260924_stroke_detector_rewrite]] and [[project_20260924_terminal_perception_rate_drop]].

**Pending (priority order):**
1. **n=5 pure-perception IC1-5 gate** with the baked defaults (only n=1 done: 4/5 SP, 5/5 soft, IC4 xy 0.090 vs 0.08). Set WORLD=cross_marker MARKER_TYPE=cross EXPLICITLY (launcher defaults to ArUco). Compare vs legacy (CROSS_DETECTOR=legacy PLASMC_S_LOSS_FADE=0 PLASMC_TD_SETTLE_S=0).
2. **Moving targets in PURE perception** (rover_cross Sinusoidal, rover_cross_deck Circular, no GT-FB) — settle + fade never tested there (GT-FB rover runs use GT touchdown depth).
3. **Moving-target gap to GT** (Sinusoidal 0.14-0.17 vs 0.04 m; Circular 0.17-0.26 vs 0.13): live perception ~3x slower than offline (GIL/threads with the 83 Hz controller). Next lever = perception in its OWN PROCESS. CROSS_S_PREDICT (KF-rate extrapolation) helps >1 m, hurts <0.3 m — off; a short-LS-rate version gated to >0.3 m is the cheaper alternative.
4. **Re-record manuscript GT-FB Linear/Circular deck videos** (Final/) — recorded with marker_dz 0.5 (true 0.201). User's call.
5. **Linear deck**: rover speed erratic (median 0.18-1.32 m/s per rep) — separate stimulus issue, unexplained.
6. Stroke terminal (<0.3 m) detOK still 60-96% (ring-topology confirm helps; a peer session has notes in project_20260924_stroke_terminal_ring_radius). Hardware/Pi port of the stroke detector was already synced by the peer (f334f068).
7. ArUco path not re-gated with the new settle default (comparison-only).

**Traps hit this thread (don't repeat):** editing a .sh while it runs; stamp-SET membership is not a valid frame<->log pairing (use contiguous run); scoring perception vs regularised x/(z+0.2) GT; deck-world GT marker_dz; stating a mechanism from magnitude alone (deck "teleport lag" — wrong); first fade version on V_ds_d never engaged (combined barrier).
