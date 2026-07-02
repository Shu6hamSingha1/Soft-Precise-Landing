---
name: feedback_concurrent_session_lanes
description: "When two Claude sessions work the repo concurrently, the session that PRODUCED a finding owns its write-up surfaces (guide STATUS entry, its own memory topic files); a cross-cutting session (audit/records) must not pre-empt them — it may append records (NC rows, scans), preserve data, and fix verified errors, but leaves the finding's STATUS narration to the owner."
metadata:
  type: feedback
---

**Why:** on 2026-07-02 an audit session and a rover-campaign session ran concurrently in the
same working tree. The audit session wrote the campaign's speed-sweep results into the tuning-guide
STATUS; the user corrected: the campaign session will write its own STATUS update. Two sessions
narrating the same finding produces duplicate/conflicting STATUS blocks and "file modified since
read" churn (both hit it that day).

**How to apply:**
- The FINDING-OWNER session writes: its guide STATUS entry, its memory topic files, its index entry.
- A cross-cutting session (audit, record-keeping, cleanup) limits itself to: appending record rows
  (NC log, scans, tsv/json rebuilds), preserving volatile data (scratchpad → test_data), fixing
  VERIFIED factual errors in existing text, and updating surfaces it owns.
- Before editing a shared file (guide, px4/MEMORY.md), check mtime vs session start — if another
  session is actively writing (fresh mtimes), re-read immediately before editing and keep the edit
  window small; on "modified since read", re-read and re-apply against the live state.
- Duplicate data-preservation check: before copying scratchpad data to test_data/, check whether the
  owner session already preserved its own copy (it did for the speed sweep at
  `test_data/Rover_SpeedSweep/` one minute after writing its memory).

Existing related convention: per-subfolder memory indexes exist precisely to avoid cross-chat push
collisions (px4/MEMORY.md header note).

**Update 2026-07-02 — the root-index SHRINK changes the pointer map (and shrinks the collision
surface):** the legacy root `Memory/MEMORY.md` (238 lines/58 KB, truncated at auto-load) is now a
**45-line auto-loaded CORE** holding only cross-cutting rules + pointers. All phase hooks live in
the per-phase indexes — `px4/MEMORY.md` and `matlab/MEMORY.md` each gained a "Legacy … index"
section with the migrated pre-06-19 flat-file hooks; 19 superseded lines were dropped (listed by
name in the root; files keep their ⛔ stamps); the pre-shrink index is recoverable via
`git show d01c5c0:Memory/MEMORY.md`. **Lane implication:** sessions append/edit ONLY their own
phase index; the root is COLD (touch it only to add/adjust a cross-cutting one-liner rule), so
root-index collisions should no longer occur — the remaining HOT shared surfaces are
`px4/MEMORY.md` (top banners + entries) and the tuning-guide STATUS block.
