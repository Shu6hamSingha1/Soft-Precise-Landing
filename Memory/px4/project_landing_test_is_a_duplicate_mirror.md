---
name: project-landing-test-is-a-duplicate-mirror
description: test_data/Landing_Test is largely a byte-identical autosave mirror of the curated campaign/ICValidation dirs — dedupe before any size-based cleanup
metadata: 
  node_type: memory
  type: project
  originSessionId: d5ea67ff-56f9-4c30-84f2-28d7d6cfab7d
  modified: 2026-09-02T04:02:06.282Z
---

`apps/landing_test.py` autosaves EVERY run into `test_data/Landing_Test/<ctime-style timestamp>/`,
**and** the harness scripts (`run_ic_validation.sh`, `run_multi_ic_landing.sh`, the `*_ab.sh`
family) independently copy that same run into their campaign dir. So the two are duplicates,
not complements.

Measured 2026-09-02, before cleanup: Landing_Test = 3751 reps / 27.84 GiB, of which
**2629 reps / 19.91 GiB were byte-identical** (verified per-file with `filecmp.cmpfiles`,
2629/2629 matched, 0 rejected) to a copy in another dir — 17.26 GiB of that in
`ICValidation/` alone, the rest in `Multi_IC/`, `AzLiftGain_*`, `KappaRatchet_*`,
`RadiusCap_Sweep`, `CBF_Phase2_AB`, `GTFB_*`. Only 7.93 GiB was unique (genuine ad-hoc runs).
The reverse also holds: `ICValidation` was 17.26/17.54 GiB mirrored into Landing_Test.

**Why:** the whole `test_data/` tree was 60 GB and looked like it needed era-based judgment
calls about which experiments to sacrifice. It didn't — a third of it was pure duplication
that could be dropped with zero information loss and zero tracked-file changes.

**How to apply:** before proposing ANY size-based test_data cleanup, dedupe first — hash
`Ground_Truth.npy` per rep to group candidates, then confirm with a full per-file compare
before deleting. Keep the CAMPAIGN copy, delete the Landing_Test one: it is better labelled
(`IC3_rep2` vs `Sat Aug 29 22-48-41 2026`) and carries a per-rep `.log`. Also worth screening:
reps with `Ground_Truth.npy` < 20 KB are aborted runs that logged no data, and
`Test_Videos/*_raw/` PNG dumps are spent intermediates wherever the `.mp4` is already rendered.
Ordering matters — deleting Landing_Test mirrors is safe ONLY while the campaign copy survives,
so it must precede any pruning of old ICValidation gates.

Executed in `1f0b6cab` (21.8 GiB freed, 60 GB -> 39 GB). Cleanup convention and prior phases:
[[project_obsolete_cleanup]] / `docs/OBSOLETE_CLEANUP_HANDOFF.md`.
