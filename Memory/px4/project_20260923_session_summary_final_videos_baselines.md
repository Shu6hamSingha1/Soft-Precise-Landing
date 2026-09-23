---
name: project_20260923_session_summary_final_videos_baselines
description: "2026-09-23 session map: VISTA-GT videos aligned to manuscript symbols, 4-baseline x 10-case comparison campaign (test_data/Final/<TAG>-GT), montage touchdown bug, cho2022 root cause, zf/B_T fixes, re-records, and process lessons. Start here, then follow the links."
metadata:
  type: project
  modified: 2026-09-23T18:00:00.000Z
---

One-page map of the 2026-09-23 session (details live in the linked memories; this file
only records the arc, the final state, and what is still open).

## What was delivered (all pushed; origin/main at a0de0a36 or later)
- **VISTA-GT** (`test_data/Final/VISTA-GT/`, the 10 GT-FB cases): montage + overlay_h videos
  regenerated -- nu replaces h, `w=` on every case, ᴵ-frame symbols in Computer-Modern
  (`mathtext.fontset="cm"`), PiP captions removed. -> [[project_20260923_manuscript_video_symbol_convention]],
  [[reference_finalized_montage_video_layout]]
- **Comparison baselines** (`LIN2022-GT`, `ZHANG2026-GT`, `LIN2023-GT`, `CHO2022-GT`, each with a
  MANIFEST.md): 4 baselines x 10 cases, ONE attempt each, no SP gate, via
  `scripts/record_baseline_cases.sh` + `scripts/run_baseline_campaign.sh` (`BASELINES=...`
  override to re-record a subset). Videos = onboard + chase + montage only (no s/alpha/h/w
  overlay PiPs). -> [[project_20260923_comparative_baseline_campaign]]
- **Final landed counts** (0/40 precise+soft): lin2022 6/10, zhang2026 10/10 (hard, xy 2.6-12m),
  lin2023 4/10 (re-recorded, zf=0.2), cho2022 1/10 (re-recorded, zf=0.2 + B_T sat).

## Bugs found and fixed
1. **Montage touchdown bug** (`tools/make_landing_montage.py`): argmin(uz) treated a hover-noise dip
   as touchdown on never-landed runs, truncating plots AND video. Now gated on
   `SoftPrecise.xy_err`. 17 montages regenerated. -> [[feedback_montage_touchdown_argmin_bug]]
2. **cho2022 never lands** (root-caused): FF-IBVS regulator converges to exact hover at a fixed
   depth setpoint; baseline runs bypass `self.PLASMC()` so the loom touchdown latch never runs;
   only PX4-native contact detection remains and is never triggered. The `B_T` blow-up (~3M) came
   from `accel_to_rate_thrust` dividing by cos(roll)cos(pitch) with a 1e-6 guard -> now saturated to
   [-11.08, 31.22]. `use_sq_comp` is dead code (N is always 5) -- still unfixed, matches MATLAB.
   -> [[feedback_cho2022_never_lands_rootcause]]
3. **Stale `BASELINE_ZF`** default 0.3 -> 0.2 (MATLAB Constants.m). The 0.01 z-depth clamp matches
   MATLAB and is NOT stale. Fix affects both `needs_features` baselines (lin2023, cho2022), both
   re-recorded. Re-record showed no consistent improvement (n=1) -- it is parity, not performance.

## Still open / do not overstate
- cho2022's single "landing" (Linear, min_alt 0.023m) is *inferred* incidental deck-heave contact,
  not verified.
- Baseline results are n=1 per case; per-case swings between recordings are noise.
- `use_sq_comp` dead code and the non-sign-preserving z clamp are unfixed (both mirror MATLAB).
- No baseline overlay PiPs were produced (scope decision, predates finding
  `tools/overlay_image_features.py` is reusable).

## Process lessons (this session's own mistakes)
- **Run SITL recordings in the background**, never under a short foreground `timeout` (an aborting
  run takes ~4 min).
- **Never broad `pkill -f`** for cleanup: it killed my own shell and likely a peer's run. Kill by exact
  PID (`pgrep -x`), and check `pgrep -fa "run_rover_landing|record_"` for a peer's batch BEFORE
  launching (a peer's stack also caused a port-8888 bind failure). Peer batch involved:
  `test_data/RoverIC_raw/record_rover_ic_raw.sh`.
- **Grep batch logs for the tool's own anomaly line** (`drone 0f`) before copying outputs; an exit
  code of 0 does not mean valid output (LIN2022-GT/IC3 had a missing onboard video).
- **Verify before claiming**: I wrote "valid montage" in memory before checking it, and stated the
  Linear-landing cause too firmly; both were corrected. Check first, then write it down.
- Concurrent sessions share this checkout: another session's commits/untracked dirs appear in
  `git status`; stage explicit paths only, and re-verify source freshness before copy-over steps.
