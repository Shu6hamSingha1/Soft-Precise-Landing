---
name: project_ic1_ds_guard_gap_reset
description: "2026-07-23: IC1's 9.28m fly-away (IC1_rep1, ICValidation/20260723-185307) traced to the centroid ds outlier-hold's _s_prev never being reset on a marker-loss gap (only its flow sibling _flow_prev was, despite the comment naming both) -- froze s for several cycles post-reacquisition then dumped a compounded jump, detonating kappa/a_u. Fixed in commit 9075a97, validated n=5 IC1 clean."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-23T15:17:53.716Z
---

**Bug:** `img_data.py`'s marker-loss-gap reset block (fires when `FEATURE_IS_VISIBLE`
transitions to False after `CHECK_NUM` misses) reset `self._flow_prev = None` with the
comment "ds/dh gate: re-init after a marker-loss gap (don't hold stale)" -- naming BOTH the
flow (dh) and centroid (ds) outlier-hold guards -- but only actually reset `_flow_prev`.
`_s_prev` (the centroid guard's detection reference, `FLOW_DS_MAX=0.15`) was left stale.

**Mechanism:** after the marker reacquired post-loss, the centroid guard compared the fresh
raw position against the STALE pre-gap `_s_prev`, rejected it (real motion during the gap
exceeded 0.15), and kept rejecting for multiple consecutive control cycles if real motion
continued (since each rejected frame's raw value becomes the next comparison's `_s_prev`,
but if motion is smooth/sustained the delta stays above threshold every frame) -- `s` froze
at a stale value for ~3 control cycles (confirmed: `[0.033,-0.732]` identical 3x despite
healthy 184-corner decode) while kappa never saw the growing real error, then the missed
motion landed as one compounded jump the instant a delta finally cleared
(`[-0.878,-2.334]` in one step) -- detonating kappa/a_u.

**Fix (commit 9075a97):** reset `_s_prev = None` alongside `_flow_prev` in the same gap-reset
block (img_data.py, the `elif self.FEATURE_IS_VISIBLE:` branch, `CHECK_NUM`-miss path).

**Validated:** n=5 IC1 (ICValidation/20260723-191407), 5/5 landed, zero TARGET_LOST (vs the
pre-fix 9.28m fly-away), max xy 1.645m, mean 0.658m, 1 soft+precise. n=3 IC3/IC4 clean too.

See also [[project_ic2_ic5_20260723_investigation]] -- same session, follow-up fix for a
DIFFERENT (complementary) gap in the same guard family, found via the IC2-5 gate this fix's
validation triggered.
