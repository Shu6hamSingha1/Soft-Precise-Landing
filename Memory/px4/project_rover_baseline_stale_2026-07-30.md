---
name: project_rover_baseline_stale_2026-07-30
description: "The 2026-07-02 rover moving-platform landing baseline (3/3 on-platform) no longer reproduces against current baked gains — n=3 re-test on 2026-07-29/30 was 0/3, all missing the platform by 0.7-4.3 m with no target-lost/perception flags."
metadata: 
  node_type: memory
  type: project
  originSessionId: 691885cb-f375-4354-a734-54384181d77a
  modified: 2026-07-29T22:22:07.074Z
---

Re-ran the documented moving-target baseline command
(`HEADLESS=1 PLASMC_GT_FEEDBACK=1 ROVER_MOTION=1 ROVER_TRAJ=Linear ROVER_SPEED_MULT=0.3
PLASMC_YAW_ALPHA_FILT=0 MAX_ATTEMPTS=5 LANDING_AUTOSAVE=1 bash scripts/run_rover_landing_retry.sh`,
from [[MOVING_TARGET_PREP.md]]) on 2026-07-29/30. First single run + a follow-up n=3 sweep:
0/4 total landed near the platform. `SoftPrecise.min_alt_xy` = 1.48, 0.74, 4.31 m (n=3 sweep)
plus 1.47 m (first run) — all far outside the 0.3 m platform half-width the 2026-07-02 doc
reports 3/3 inside (0.044-0.28 m). No `target_lost`, no `terminal_perception_loss`, no
`descent_anomaly` flags fired in any rep — the controller tracked fine end-to-end, it just
consistently lands short/behind the moving platform, with inconsistent miss magnitude
rep-to-rep (not a fixed bias) — points at a tracking-lag/lead-compensation issue, not a
hard failure mode.

**Why:** the 2026-07-02 rover validation was against the stationary-config gains of that date.
Since then, multiple bake sessions changed the defaults (dense-homography-recovery default-on,
`FLOW_FUSE_RING` 1→0, `MARKER_KLT_RELAX_GATE=1`, s-extrapolation ordering fix — all 2026-07-09,
plus whatever baked 07-15→07-29). The run logs' non-default-param printout confirms current
defaults differ substantially from what the rover doc assumed (XI2_*, P20_*, GAMMA_*, KAPPA0_*
all print as non-default vs. the doc's implied baseline). Moving-target tracking was never
re-validated against the post-07-09 stack.

**How to apply:** don't trust `MOVING_TARGET_PREP.md`'s "3/3 on-platform" claim as current
truth — it's a stale snapshot. Before further rover-speed work, either (a) re-run stationary
`run_ic_validation.sh` to confirm the current baked config is sound in isolation, then
re-derive/re-tune rover tracking-lag compensation against it, or (b) bisect which specific
07-09+ bake broke rover parity (GT-FB mode means it's not the perception-layer bakes
themselves, so suspect the funnel/kappa gain changes carried in the non-default-param list).
