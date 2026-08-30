---
name: project_20260830_perception_touchdown_detect_broken
description: "2026-08-30: the perception-mode touchdown detector (both paths in controller.py::_touchdownDetect) is broken and has effectively never been validated — GT-feedback bypasses it via the _td_gt_* path. Offline replay on 25 clean GT-FB landings: the loom-independent 2nd path (_td_ext_only) false-fires at ALTITUDE in 23/25; the loom-spike 1st path never fires in 22/25. Perception-mode landing is BLOCKED on fixing this."
metadata: 
  node_type: memory
  type: project
  originSessionId: 0c94ab6a-894a-4d39-9867-91dec8322965
  modified: 2026-08-30T06:33:41.327Z
---

# Perception-mode touchdown-detect is broken (both paths)

Found while starting the roadmap's GT-ablation (perception-mode cross-marker,
`PLASMC_GT_FEEDBACK` off) after the IC5 non-SP investigation was closed.

## Symptom

IC1 n=5 perception-mode (bridge default-on, `MARKER_TYPE=cross WORLD=cross_marker`,
`PLASMC_DTHETA_HREF=1 CBF_HZ_AWARE_DRIFT=1`, GT-FB OFF): **0/5 SP, and none actually
landed** — every rep's recording ends at **2.4–3.4 m altitude**, scored as a touchdown by
the harness. Run logs:
```
[controller] TOUCHDOWN-DETECT: extent stalled near-max (loom-independent,
  112px vs running_max=114px, x3 within 3-frame window) |s_e_n|=0.04 -> LANDED
```
Detection health was 100% — this is purely the touchdown heuristic.

## Root cause (both paths in `controller.py::_touchdownDetect`)

**GT-feedback never exercises either path.** Under GT-FB, `_touchdownDetect` takes the
`_td_gt_*` branch (real GT camera-marker depth, hard `_td_gt_max_depth=0.25 m` ceiling +
windowed-rate + progress-since-arm guards) and `return`s before reaching the perception
paths. So the perception logic below has effectively **never been validated**.

**2nd path — loom-independent extent-only (`_td_ext_only_hist`, added 2026-08-26):**
latches on `_ext_ok = _extentGrowthFlattened() and _extentTouchdownProximate() and
_td_ext_armed`, sustained `_td_frames` in a rolling window, gated by `|s_e_n| < _td_sen(0.6)`.
- `_extentTouchdownProximate()` = `MARKER_EXTENT_PX >= 0.6 * _td_ext_max` where `_td_ext_max`
  is the **running max seen so far this flight**. During a monotone descent
  `current ≈ running_max` almost every frame → this is ~always True → carries NO proximity
  information. The path collapses to `flattened + centered`.
- `_extentGrowthFlattened()` = `|d(ln extent)/dt| < 0.3/s` over only **6 samples**. A normal
  descent grows extent at ~0.3/s (78 px @ 3.7 m → 162 px @ 1.7 m, GT-FB IC1) — right at the
  threshold — so the noisy 6-sample local rate dips below it constantly → false "flattened"
  at altitude. Perception-mode's jerkier descent (7–9 s vs GT-FB's 23 s) makes it worse.

**1st path — loom-spike (`h_z > _td_spike(0.0)` ×3 in window) + extent corroboration:**
too conservative — perception `h_z` (`Img_Data['h_V'][:,2]`) essentially never produces a
3-in-window positive spike, even at true touchdown. (Matches the code's own 2026-08-26 note:
"14/28 flights never latched" on the loom path.)

## Offline replay proof (25 clean GT-FB landings, `ICValidation/20260830-074756`)

Replayed the perception state machine against real `Img_Data` `MARKER_EXTENT_PX` +
`h_V[:,2]` + `Detection Status` (s_e_n from Control_Data — minor caveat; `_td_sen=0.6` gate
is loose). All 25 truly landed (min_alt ≈ −0.01 m).
- **2nd path false-fires at altitude in 23/25.** Fire heights: mostly 0.8–1.0 m, IC4 at
  **2.9–3.3 m**, several IC1/2/3 at 2.1–2.3 m. Only the 2 IC5 fly-aways don't fire
  (`|s_e_n| > 0.6` blocks the gate).
- **1st path never fires in 22/25** (fires only in the 2 long-flight reps, at 0.57 m).

Script: `scratchpad/td_replay.py`.

## Confirming re-run

IC1 n=5 perception with `PLASMC_TD_EXT_ONLY=0` (new env flag, added this session, default
1 = unchanged — gates the 2nd path off): flights now descend properly to **~0.9 m** then
the descent loop exits with `FC_node.LANDED=False` — **nothing latches touchdown**, drone
hangs / drifts (xy ~0.3 m, rel_vel ~0.5 m/s), some reps `TARGET_LOST` at 0.05–0.24 m. Also
hit a persistent `gz_bridge/lockstep race` on every retry (infra, possibly aggravated by
the longer flights). Confirms: with the broken 2nd path disabled and the 1st path never
firing, **there is no perception-only touchdown detector at all**.

## What needs fixing before perception-mode landing is viable

1. **2nd path `_extentTouchdownProximate`**: re-base on **frame size**, not running max —
   e.g. `extent >= frac * min(H, W)` (working detection frame is 480w×640t;
   `LANDING_STALE_COMMIT_EXTENT` already uses an absolute-px convention). At true touchdown
   the cross marker fills a large fraction of the frame (~300+ px); at 3 m it's ~110 px.
2. **2nd path `_extentGrowthFlattened`**: longer window and/or an absolute-extent floor so a
   mid-descent rate dip at 110 px can't read as "flattened near touchdown".
3. **1st path**: revisit the perception `h_z` spike criterion (magnitude/persistence) — it
   currently never fires.
4. Investigate the `gz_bridge/lockstep race` retry storm on longer perception-mode flights.

## Env flag added this session (uncommitted)

`PLASMC_TD_EXT_ONLY` in `controller.py` (default `"1"` = current behavior). `=0` disables
the loom-independent 2nd path. Diagnostic lever; NOT a fix (nothing lands with it off).
`_prev_flow_uid` cell-ID fix + `CROSS_CENTER_BRIDGE_FRAMES=10` (commit e86c798a) are
unaffected and still in.
