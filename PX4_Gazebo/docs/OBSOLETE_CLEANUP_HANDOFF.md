# Obsolete test-data cleanup — HANDOFF (2026-06-26)

**Status: manifest built + execution script ready (dry-run verified). NOTHING DELETED YET.**
Pick up here in the new chat.

## Decisions (user-approved 2026-06-26)
- **Scope = obsolete data tied to a "false config later corrected":**
  - **A. Combined-barrier GAIN-PARITY-BUG era** — runs before the fix `22cc732`
    (`2026-06-20 20:41:48 +0530`) / re-bake `edc28c4`. Combined-barrier was default-on
    with the wrong *hot back-mapped* gains → fly-aways. The "lateral wall" was this bug,
    not a fundamental limit (`project_current_state`).
  - **B. NC-FALSIFIED** — bundles marked Dead-end/Debunked/FALSIFIED/REVERTED in
    `PX4_NewCal_Record`.
- **Method = archive to tarball → verify → delete → re-scan records.** Archive lands in
  `~/spl_obsolete_archive/` (outside repo). 276/281 test_data dirs are UNTRACKED → no git
  undo, hence the tarball.
- **REVIEW set** (06-19/20 baselines + gain-comparison arms): promote all to DELETE
  EXCEPT keep `VdfGains_IC2_manuscript` + `VdfBake_IC2_combined` (fix-validation refs).
- **Landing_Test (6.3 GB):** ALSO rep-prune reps dated **before** the fix cutoff. Needs
  its own manifest review (Phase 2).
- **Protected (never delete):** CoordDescent (genuine SP), SPCampaign, ICValidation
  (gates), Test_Videos, RingFlow/SenFunnel, anything ≥06-21 (live single-marker /
  moment-loom / GT-feedback thread), + the 2 fix-validation dirs above.

## Phase 1 — dir-level (✅ DONE 2026-06-26, commit 0dbbd51 pushed)
101 dirs / 2.46 GB / 489 reps (76 bug-era + 25 falsified) archived to
`~/spl_obsolete_archive/obsolete_bugera_nc_falsified.tar.gz` (980 MB, 101 roots verified)
then deleted; records re-scanned (configs 175, genuine SP unchanged 16+2 frozen) and pushed.
KEPT `VdfGains_IC2_manuscript` + `VdfBake_IC2_combined` (confirmed on disk). Original plan below.

### (original Phase 1 plan — 101 dirs / 2.46 GB / 489 reps: 76 bug-era + 25 falsified)
Manifest: `test_data/OBSOLETE_MANIFEST.tsv` (rebuild via `tools/build_obsolete_manifest.py`).
```bash
cd ~/Soft-Precise-Landing/PX4_Gazebo
~/ws/scripts/env2025/bin/python3 tools/execute_obsolete_cleanup.py            # dry-run (re-confirm)
~/ws/scripts/env2025/bin/python3 tools/execute_obsolete_cleanup.py --execute  # archive+delete+rescan
# then: git add -f records, commit, push (carve-out convention)
```
`execute_obsolete_cleanup.py`: tars all DELETE dirs → verifies listing → `rm -rf` →
re-runs `build_test_record.py` + `refresh_scan_sheets.py` (records auto-drop them).

## Phase 2 — Landing_Test rep-prune (TODO, ~most of the 6.3 GB)
`Landing_Test/` is the autosave mega-dir (1177 reps) mixing eras; can't dir-delete.
Build a **rep-level** manifest + execute, parallel to Phase 1:
1. New script: walk `test_data/Landing_Test/**/Ground_Truth.npy`; get each rep's datetime
   from its timestamp parent dir (e.g. `Mon Jun 22 03-07-06 2026/` or `20260605-...`),
   fall back to file mtime.
2. Flag reps with datetime **< `2026-06-20 20:41:48 +0530`** (the fix cutoff) as obsolete
   (bug-era + back-mapped + cal-contaminated). Emit a manifest (rep path, date, size).
3. **Present for review** (user asked for its own review), then archive+delete the flagged
   reps, re-scan. Expect to reclaim multiple GB.
Caveat: the lone Landing_Test "SP" is a frozen-GT false-SP (`feedback_false_sp_frozen_gt`),
so no genuine SP is lost.

## Tools (all in PX4_Gazebo/tools/, committed)
- `build_test_record.py` — scan test_data/ → json/tsv/md (run after any deletion).
- `refresh_scan_sheets.py` — idempotent: rebuild All_Test_Runs + Genuine_SP_Reps in the ods.
- `build_obsolete_manifest.py` — (re)build OBSOLETE_MANIFEST.tsv (Phase 1 scope).
- `execute_obsolete_cleanup.py` — dry-run/--execute archive+delete+rescan (Phase 1).
- `restructure_parameter_record.py` — ONE-SHOT, already done (don't re-run).
