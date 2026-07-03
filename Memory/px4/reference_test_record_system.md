---
name: reference_test_record_system
description: "PX4 test-record tooling + the all-runs SP scan: build_test_record.py scans test_data/ (41 top-cfg/3107 reps/789 SP @2026-07-02 — SPs dominated by the GT-FB era), then refresh_scan_sheets.py rebuilds the 2 auto ods sheets; emits json+tsv+md; parameter_record.ods has 7 sheets (manual NC log — read its LAST ROW for currency; it lags unless each session appends); only ~few genuine perception-ON closed-loop SP exist"
metadata: 
  node_type: memory
  type: reference
  originSessionId: 5919ddc2-a345-4241-aa4a-569e274e340e
---

**Test-record system (built 2026-06-23, commits 43f3b40/e8df658).** Single source of
truth = a scan of `PX4_Gazebo/test_data/` per-rep `Ground_Truth.npy['SoftPrecise']`
(xy_err, rel_vel, precise, soft, target_lost), verdict taxonomy from
`tools/scan_all_landings.py::classify_rep`.

TOOLS (all re-runnable; venv `~/ws/scripts/env2025/bin/python3`):
- `tools/build_test_record.py` — scans all leaf runs, aggregates per top-level config,
  emits `test_data/test_record_runs.json` (machine + `sp_reps[]` per-rep SP list),
  `test_data/test_record.tsv` (grep/awk one-liner reference), `docs/TEST_RECORD.md`
  (reading digest + Genuine-SP section). ⭐ grep the TSV for fast lookup, not the ods.
- `tools/refresh_scan_sheets.py` — ⭐ IDEMPOTENT: rebuilds BOTH auto-scan sheets
  (`All_Test_Runs` + `Genuine_SP_Reps`) from the json. **Run this after every
  build_test_record.py scan** to update the ods. Supersedes add_sp_drilldown.py (deleted).
- `tools/restructure_parameter_record.py` — ONE-SHOT, ALREADY DONE (do NOT re-run; it
  removes row 0 + re-adds sheets): historical record of the original NewCal_Notes rescue.

REFRESH WORKFLOW (every "update PX4 test records"): `git pull` → `build_test_record.py`
→ `refresh_scan_sheets.py` → commit ods+tsv+json+md. The manual sheets (gain matrix,
NC log) are user-maintained — NOT auto-updated; PX4 sessions append NC rows by hand.

ODS now has 7 sheets: PX4_Gain_Record (trials 1-60, wide 62-col gain matrix),
PX4_NewCal_Record (36-col body — header at row 0; ⚠ MANUAL — read the LAST ROW for currency; NC174-182
were back-filled 2026-07-02 for the 06-30..07-02 sessions and later sessions keep appending), Removed_Parameters,
MATLAB_Test_Record, NewCal_Notes, All_Test_Runs, Genuine_SP_Reps. Backup before edits
(convention [[feedback_parameter_record_logging]]); git-tracked via carve-out.

KEY FINDING (the scan, 3137 reps @2026-06-26): 276 configs, **18 "full SP"** — but 2 are frozen-GT
false-SP (xy≈1e-21, [[feedback_false_sp_frozen_gt]]) and MANY of the 16 genuine are
SPCampaign **b9*/b10B open-loop-HANDOFF INVALID** reps (last ~25cm not closed-loop, per
PX4_Gain_Record remarks + [[feedback_convergence_ordering]]). The only trustworthy
closed-loop SP candidates: 2 **CoordDescent** reps (0.052m/0.061, 0.066m/0.165) + a few
non-handoff SPCampaign (b13/b14/b2A). Consistent with "no genuine sub-10cm SP in saved
R3 data" [[feedback_false_sp_frozen_gt]] [[feedback_coord_descent_sp_lucky_ic]].

**Update 2026-07-02:** scan = **41 configs / 3107 reps / 789 "full SP"** — the jump from 18 SP is the
GT-FB era (idealized sensing, Z_REG-fixed harness; [[project_why_sp_achieved]]) + the rover campaign;
the "only ~few genuine closed-loop SP" finding still holds for PERCEPTION-ON runs. ⚠ Reps that abort
before SoftPrecise evaluation are INVISIBLE to the scan by design (e.g. `Rover_AB_rover` shows n=1 of 3
— the 2 fly-aways never wrote SoftPrecise): read A/B fly-away RATES from the memory files / harness
logs (`test_data/Rover_AB_harness/`), never from the tsv alone.
