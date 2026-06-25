---
name: project_obsolete_cleanup
description: "⭐ IN-PROGRESS handoff: removing obsolete test_data (combined-barrier gain-parity-bug era + NC-falsified). Phase-1 manifest+executor READY (101 dirs/2.46GB, NOTHING deleted yet); Phase-2 Landing_Test rep-prune TODO. Full plan in PX4_Gazebo/docs/OBSOLETE_CLEANUP_HANDOFF.md"
metadata: 
  node_type: memory
  type: project
  originSessionId: 5919ddc2-a345-4241-aa4a-569e274e340e
---

**Obsolete test-data cleanup — started 2026-06-26, handed to a NEW CHAT. NOTHING DELETED YET.**
⭐ READ FIRST: `PX4_Gazebo/docs/OBSOLETE_CLEANUP_HANDOFF.md` (full decisions + exact commands).

**Goal:** remove test_data tied to a "false config later corrected" + their records.
**Scope (user-approved):** (A) combined-barrier GAIN-PARITY-BUG era — runs before fix
`22cc732` (`2026-06-20 20:41:48 +0530`); the lateral wall was THIS bug not a fundamental
limit ([[project_current_state]]). (B) NC-FALSIFIED bundles (Dead-end/FALSIFIED/REVERTED).
**Method:** archive→tarball (`~/spl_obsolete_archive/`, OUTSIDE repo — 276/281 dirs UNTRACKED,
no git undo)→verify→delete→re-scan records.

**Phase 1 (✅ DONE 2026-06-26, commit 0dbbd51 pushed):** 101 dirs/2.46GB/489 reps
(76 bug-era + 25 falsified) archived → `~/spl_obsolete_archive/obsolete_bugera_nc_falsified.tar.gz`
(980MB, 101 roots verified) then deleted; records re-scanned (configs 175, genuine SP unchanged
16+2 frozen) + committed+pushed. KEPT (fix-validation): `VdfGains_IC2_manuscript`,
`VdfBake_IC2_combined` (confirmed surviving on disk).
**Phase 2 (TODO):** Landing_Test (6.3GB autosave mega-dir, 1177 mixed-era reps) — REP-level
prune reps dated < the fix cutoff; build a rep manifest, REVIEW, archive+delete, re-scan.
Most reclaimable GB. Its lone "SP" is frozen-GT false ([[feedback_false_sp_frozen_gt]]).

**PROTECTED (never delete):** CoordDescent (genuine SP), SPCampaign, ICValidation (gates),
Test_Videos, RingFlow/SenFunnel, everything ≥06-21 (live single-marker/moment-loom/GT-feedback
thread [[project_gt_feedback_control_tuning]]). Tooling = [[reference_test_record_system]].
