---
name: project-landing-test-is-a-duplicate-mirror
description: test_data/Landing_Test is largely a byte-identical autosave mirror of the curated campaign/ICValidation dirs — dedupe before any size-based cleanup
metadata: 
  node_type: memory
  type: project
  originSessionId: d5ea67ff-56f9-4c30-84f2-28d7d6cfab7d
  modified: 2026-09-02T09:46:11.563Z
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

## Tier 2 — investigated 2026-09-02, DEFERRED by the user ("we will do it later")

Don't re-derive this. Manifest is built and committed: `test_data/TIER2_MANIFEST.tsv`
(regenerate with `tools/build_tier2_manifest.py`), commit `6c9bb2bf`. **7.57 GiB** reclaimable:
139 ICValidation runs + 582 Landing_Test reps + 6 calibration dirs.

A naive "drop June+July" cut (~17 GB) was investigated and **REJECTED** — it would have:
- orphaned **114 cited ICValidation runs**, 6+ of them cited from LIVE SOURCE as the provenance
  for guards still running (`img_data.py:3436` "fly-away (ICValidation/20260716-211434). Fix:
  REJECT (not clip)", `planar_map.py:203`, and 7 memory topic files);
- destroyed the raw traces behind **581 of 938 genuine SP reps (62%)**;
- deleted `calibration_data/output_cross_stale_pre20260805`, which `cross_marker_perception.py`
  explicitly records as "moved ... **not deleted**".

Keep rule that replaced it: keep if 2026-08+ **or** cited anywhere **or** contains a genuine SP rep.

⚠ **SEQUENCING TRAP when this is finally executed:** `test_record_runs.json` / `test_record.tsv` /
`TEST_RECORD.md` are the ONLY surviving record of the dropped reps (they carry per-rep
`xy_err`/`rel_vel`). Re-running `build_test_record.py` afterwards — the documented post-cleanup
step, so easy to do by reflex — would drop those rows and destroy the aggregate history on top of
the traces. **Snapshot the record first; do not blindly re-scan.**

Optional extra ~0.56 GiB: 29 uncited `diag_*` calibration dirs are kept only by the August date
rule (one-off cross-marker diagnostics, not cal provenance) — user's call whether they count as spent.

## "Should we delete the artifact-chasing phase's data?" — ALREADY DONE (checked 2026-09-03)

Asked and answered; don't re-investigate. The cal-contaminated era is precisely **2026-05-12 →
06-01** (the May-12 cal, `h_z` under-read **13.2×**, lateral 3.8-4.7×, controller at 0.08-0.21× design
— [[feedback_historical_cal_confound]]), and the June 2026 cleanup already archived 101 config dirs
+ 843 Landing_Test reps from it. **Only 2 May reps survive on disk.**

What remains from before the 2026-06-21 gain-parity fix is **209 reps / 1.16 GiB**, and it is the
deliberately PROTECTED set, not the artifact bulk: `SPCampaign` 131 reps/876 MiB (the 06-03 campaign
that PROVED the fix — 28% SP @ 10 cm, 1.8 cm best precision, an order of magnitude past the supposed
floor), `ICValidation` 71 reps (06-12/14 gate baselines), `DhdClampSweep` 5, `CoordDescent` 2
(genuine SP). **User decided 2026-09-03: keep it.**

⚠ Keep the two staleness kinds distinct when reasoning about deletion — conflating them is what makes
Tier 2 look bigger than it is:
- **artifact-contaminated** (pre-06-01) — the numbers are WRONG (gain-starved controller). Deleted.
- **regime-superseded** (06-21 → 08-26) — the numbers are CORRECT FOR THEIR REGIME but not comparable
  to current runs (camera fx 270→135, ArUco→cross). This is the Tier 2 set, and its remaining value
  is not the metrics at all: 114 of those runs are cited as PROVENANCE for guards still running
  (`ICValidation/20260716-211434` is *why* a fly-away guard rejects rather than clips).
