---
name: project_dense_recover_ab_20260728
description: "2026-07-28: isolated A/B (n=25/24, IC1-5) for dense-homography recovery -- closes out the 2026-07-07 'mixed, don't bake' status. Net improvement (TARGET_LOST 60%->46%, mean xy 1.11->0.92m, DESCENT_ANOMALY 1->0). BAKED default-on, commit 381f669."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-28T00:08:02.552Z
---

**Closes out [[project_dense_recovery_and_failure_tagging]]'s "MIXED, do NOT bake" status.**
That memory's block was from a confounded 2026-07-07 retest (dense-recovery's unified staleness
gate landed in the same batch as `h`-extrapolation and a separately-traced `_savgol_predict`
fly-away) -- never a clean isolated A/B.

**This A/B (2026-07-28):** same-day, same config otherwise, `run_ic_validation.sh` IC1-5 n=5
each. OFF = bundle `20260728-025649` (the just-run go-around-removal/DESCENT_ANOMALY gate,
reused as the control since `PLASMC_DENSE_RECOVER` already defaulted off). ON = bundle
`20260728-042130` (`PLASMC_DENSE_RECOVER=1`), n=24 valid (IC2_rep1 dropped -- `rc=139` SIGSEGV,
the known pre-existing Gazebo/PX4 infra flake, unrelated).

| metric | OFF (n=25) | ON (n=24) |
|---|---|---|
| TARGET_LOST rate | 15/25 (60%) | 11/24 (46%) |
| DESCENT_ANOMALY | 1/25 | 0/24 |
| mean xy_err | 1.11m | 0.92m |
| UNKNOWN failure cause | 11 | 6 |

Per-IC: consistent improvement on IC1-4 (both TARGET_LOST rate and mean xy). **IC5 alone got a
slightly worse TARGET_LOST rate (4/5->5/5)** but mean xy still improved (2.94->2.13) --
consistent with IC5's already-catalogued structural issue (large offset pushes the marker
fully OUT of the FoV, not partial occlusion --
[[project_goaround_removed_descent_anomaly_20260727]]) being outside what a partial-view
homography recovery (needs SOME trackable points) can fix.

**BAKED default-on** (`img_data.py`, `self._dense_recover` default `"0"`->`"1"`, commit
381f669). `PLASMC_DENSE_RECOVER=0` still available to revert/A-B against.

**Caveat:** single-session n=25/24 comparison, not a multi-day replication. The elevated
baseline `TARGET_LOST` rate (60%, mostly `UNKNOWN` cause) this session is itself higher than
historical norms and not yet explained -- if it turns out to be a transient infra/config issue
rather than a stable baseline, this A/B's *absolute* numbers may not replicate, though the
*relative* OFF-vs-ON comparison (same day, same infra state for both arms) should still hold
directionally.
