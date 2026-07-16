---
name: test-data-cleanup-20260602
description: Test data now lives at PX4_Gazebo/test_data/ (moved from ~/ws/Test_Data 2026-06-02 after 18GB->0.9GB cleanup); SP reps git-tracked; closed-sweep raw data is gone
metadata: 
  node_type: memory
  type: project
  originSessionId: d6332b67-4f1a-4377-8694-65dc8d3d45b4
---

Test data location (since 2026-06-02): **`PX4_Gazebo/test_data/`** inside the git project. The old `~/ws/Test_Data` is now a symlink to it (kept so the untouchable `~/ws/scripts/` baseline pipeline still works). All repo code/scripts/docs reference the new path. Older memories mentioning `~/ws/Test_Data/...` paths resolve via the symlink.

Before the move, 17.8 GB of closed-campaign data was deleted (May 18–28 sweeps, CoordDescent raw passes, BigSensitivity, legacy Calibration/, pre-May-29 Landing_Test runs). That raw data **no longer exists** — don't go looking for it; conclusions live in the feedback memories and git history.

**Git-tracked (commits 233e431 + follow-up, ~30 MB, irreplaceable):**
- All **5** SP-class reps ever: `Interventions/20260523-162914/rep2` (SP #1), `NewDefaults/20260525-015341/rep5` (SP #2), `CoordDescent/20260525-201532/pass1/RHOFOV0_241.8_rep2` (SP #3), `CoordDescent/20260526-014855/pass1/E_Z_0.1067_rep1` (SP #4), `HybridFlow_AB/20260529-133034/hybrid_rep5` (SP #5, hybrid-flow arm, most recent). `analyze_why_no_precise.py`'s candidates dict is the canonical SP registry. NOTE: summary.tsv layouts differ across sweeps (HybridFlow has an extra `arm` column) — always parse headers, never fixed column indices.
- Reference reps: `RhoFov05_KP125/20260524-183744/rep6` (PRECISE-only), `Krp4/20260525-011046/rep9` + `rep2`
- Sweep summaries, coord-descent state+logs, ImpulseResponse bundles
- `tools/analyze_why_no_precise.py` points at these (paths updated + stale timestamp fixed)

**Gitignored (in project dir but not in git):** Landing_Test/ (active landing output, May 29+ runs), ICValidation/, HybridFlow_AB/, KLT_AB/, NewCal/, Test_Videos/ (must exist — img_data.py writes into it).

**Lost beyond plan (cleanup script bug):** per-rep summary.tsv for ~45 closed sweeps and the strict 3-pass CoordDescent log.jsonl. Key stats survive in memory/git but raw per-rep numbers are gone. Related: [[feedback_coord_descent_sp_lucky_ic]], [[precision-softness-frontier]].


---

**INVALID wx/wy-live bundles quarantined (2026-06-04).** All LateralRestore bundles run with CTRL_ZERO_WXY=0
(live, badly-calibrated w_x/w_y angular-flow feedforward = the regression) are marked invalid — do NOT use as
performance/baseline data. 8 bundles: 3 already-named (_ABORTED_dhdmax50_evidence, _ABORTED_no_theta_floor_evidence,
_INVALID_kmfix_baseline_broken) + 5 renamed with _INVALID_wxy_live (the staircase 192937, c1_baseline_n10, c1_fw19,
c1_noKM, c4_n10). VALID (keep): c1_zerowxy (restored baseline, wx/wy zeroed, 2 SP/10) and c1_imuderot (deliberate
IMU-de-rotation experiment). None were git-tracked (all gitignored), so the repo + parameter_record.ods are clean
(I never logged the invalid runs as gain trials). NOT DELETED — kept as the audit trail for the regression-hunt
story (same annotate-don't-delete convention as [[historical-cal-confound]]). IC2-5 campaign bundles (ICValidation/)
are VALID (ran on the restored baseline w/ CTRL_ZERO_WXY=1 default).