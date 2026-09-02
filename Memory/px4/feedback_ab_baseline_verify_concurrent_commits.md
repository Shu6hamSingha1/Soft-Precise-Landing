---
name: feedback_ab_baseline_verify_concurrent_commits
description: "Before calling an A/B result a no-op, verify what the BASELINE actually contains. Multiple claude sessions commit to this repo concurrently, so the working tree can gain the very fix you are testing mid-session -- which makes a real fix measure as 'changes nothing' (fix-vs-fix). Re-check git log + the live env defaults at A/B time, not just at session start."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**2026-09-02, learned the hard way (twice in one session).** Sibling of
[[feedback_check_concurrent_sitl_before_launch]] -- same root fact (multiple `claude`
sessions run against this machine and this repo at once), different consequence: not a
process collision, a **silently invalid experimental baseline**.

## What happened

Diagnosed the `_isolate_marker_by_shape` component-selection bug (largest-area pick grabs
the platform shadow bar over the thin cross), proposed a fill-ratio fix, measured
18 %->79 % detOK on one recording. Then re-ran the A/B on the curated
`test_data/DetectorFrameset` eval set and got **~0 % improvement everywhere** -- and
wrongly retracted the whole root cause.

The retraction was wrong. Another session had committed **exactly that fix** (`ee858086`,
fill band `[0.02, 0.25]`, DEFAULT ON) at 08:56:41; the A/B ran at 08:59:48 against a
working tree that already contained it. The "baseline" arm was the fixed code. Measuring
fix against fix returns no delta, which reads exactly like "the idea doesn't work."

Re-run with the real legacy behaviour (`CROSS_ISOLATE_FILL_HI=1.0`) it is unambiguous:
rover_IC2 44.5->59.0 %, rover_IC4 77.8->87.9 % (`lt2_angle_clusters` 102->33), flat
100 % unchanged. The fix works.

## The rules

1. **`git log` at session start is not enough.** Re-check `git log --oneline -5` (and
   `git status`) immediately BEFORE an A/B, and again before writing up a null result.
   A no-op result is the specific signal that should trigger this check.
2. **Pin the baseline explicitly by env, never by "the default".** Every knob in this
   codebase has a documented revert value (`CROSS_ISOLATE_FILL_HI=1.0`,
   `CROSS_ADAPT_GATE=0`, ...). Set it on the control arm rather than assuming today's
   default is yesterday's.
3. **Print what you actually loaded.** The A/B harness should echo the live constant
   (`cmd.ISOLATE_FILL_HI`) per arm. A one-line print would have caught this instantly.
4. **A null result deserves the same scrutiny as a positive one.** The reflex on "my fix
   does nothing" should be "is my baseline what I think it is?" before "my mechanism was
   wrong."

## Related trap in the same session (recording provenance)

The recording the mechanism was first found on (`Test_Videos/Tue Sep  1 17-17-20 2026_raw`,
17:17) PREDATES the rover_cross platform `<diffuse>` 0.25->0.6 scene fix applied the same
evening (22:34, per the `*.bak_before_platformcolor_20260901` mtimes). That made it a
*starker* instance of the mechanism, not an invalid one -- but the general rule holds:
**check a recording's mtime against the scene/code fix timestamps before treating it as
representative of current behaviour.** Scene fixes live OUTSIDE the repo
(`~/.gazebo/models/`, `~/PX4-Autopilot/Tools/simulation/gz/`), so `git log` will not show
them -- use the dated `.bak_*` backups alongside the edited file.
