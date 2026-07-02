---
name: project_obsolete_cleanup
description: "✅ Obsolete test_data cleanup COMPLETE 2026-06-26 (combined-barrier gain-parity-bug era + NC-falsified). Phase-1 DONE (101 dirs/2.46GB, 0dbbd51); Phase-2 DONE (Landing_Test 843 reps/4.54GB, 6.3GB→2.2GB, a428b9d+abb74a6). 3.27GB archived to ~/spl_obsolete_archive/, ~5GB reclaimed, genuine SP unchanged (16). Plan in PX4_Gazebo/docs/OBSOLETE_CLEANUP_HANDOFF.md"
metadata: 
  node_type: memory
  type: project
  originSessionId: 5919ddc2-a345-4241-aa4a-569e274e340e
---

**Obsolete test-data cleanup — ✅ COMPLETE 2026-06-26 (both phases executed the same night the
handoff was written; the "NOTHING DELETED YET" header was stale until 2026-07-02 — a dry-run of
`execute_obsolete_cleanup.py` then confirmed present:0 / already-gone:101. ⚠ Do NOT re-run
`--execute`: it would tar an empty set over the real 980 MB archive).**
⭐ Full decisions + history: `PX4_Gazebo/docs/OBSOLETE_CLEANUP_HANDOFF.md`.

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
**Phase 2 (✅ DONE 2026-06-26, commits a428b9d+abb74a6 pushed):** Landing_Test rep-level prune
— 843 reps dated < cutoff (Jun 5-16, bug-era/cal-contaminated) archived →
`~/spl_obsolete_archive/obsolete_landing_test_precutoff.tar.gz` (1.7GB, 843 roots verified) then
deleted; 381 KEEP reps (Jun 23-26 live thread) retained. **Landing_Test 6.3GB→2.2GB.** Clean
date gap (no reps Jun 16-23 → no ambiguous-cutoff reps). Genuine SP unchanged (16); removed lone
SP was frozen-GT false ([[feedback_false_sp_frozen_gt]]). Tools (committed):
`build_landing_test_manifest.py`, `execute_landing_test_cleanup.py`; manifest force-tracked
`Landing_Test/LANDING_TEST_MANIFEST.tsv`. **Whole cleanup COMPLETE** (3.27GB archived, ~5GB
test_data reclaimed).

**PROTECTED (never delete):** CoordDescent (genuine SP), SPCampaign, ICValidation (gates),
Test_Videos, RingFlow/SenFunnel, everything ≥06-21 (live single-marker/moment-loom/GT-feedback
thread [[project_gt_feedback_control_tuning]]). Tooling = [[reference_test_record_system]].
