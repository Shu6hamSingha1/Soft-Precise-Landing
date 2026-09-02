---
name: project-repo-git-size-and-hardware-data
description: .git is 4.2 GB because tracked test data is 92% of history — do NOT propose a history rewrite, 192 cited commit SHAs would break
metadata:
  type: project
---

Investigated 2026-09-02. `.git` pack = 4.11 GiB, attributed by path:

```
PX4_Gazebo/test_data   2.05 GiB   (force-added campaign dirs)
Hardware/Test_Data     1.74 GiB   (100% tracked, NO gitignore carve-out)
everything else        0.32 GiB
```

Data is **92% of repo history**. Note the Tier-1 cleanup freed 21.8 GB of *working tree* but shrank
`.git` by nothing, because everything it removed was untracked — the tracked 3.79 GiB is permanent.

**⛔ Do NOT propose `git filter-repo` / BFG to shrink this.** 192 hex strings in `Memory/`,
`PX4_Gazebo/docs/`, `src/`, `tools/` and `CLAUDE.md` resolve to REAL commits in this repo
(`b963e207`, `4d7bc210`, `ee858086`, …), including citations inside live source comments that
document why a guard exists. A rewrite changes every SHA and dangles all 192. Trading 3.7 GB of
disk for the project's provenance-citation web is a bad trade at any repo size — the same class of
loss as the rejected naive Tier-2 cut, but worse. **User decided 2026-09-02: leave it alone.**

`Hardware/Test_Data` specifics (so this need not be re-surveyed): 4.15 GiB, 6605 files, all
2026-07/08 — `.npy` 4008/2.33 GiB, `.ulg` 918/1.17 GiB, `.mp4` 367/0.50 GiB; subtrees `Landing/`
2.4 G, `FlightLogs/` 1.2 G, `Calibration/` 593 M. **Only 0.22 GiB (5%) is exact-duplicate** — this
is NOT the PX4 `Landing_Test` mirror pattern (71%), see [[project_landing_test_is_a_duplicate_mirror]].
It is real-flight data: SITL reps are re-runnable, a July 2026 flight is not, so the "delete the
redundant copy" reasoning does not transfer. The dupes that exist look partly intentional
(`Calibration/Input` vs `Input_Clean`).

**Known open (user declined to change it 2026-09-02):** `Hardware/` has NO `.gitignore` carve-out,
unlike `PX4_Gazebo/`, so every new hardware recording is committed permanently and silently. If
repo growth ever becomes a problem, the fix is a go-forward carve-out (ignore bulk `.npy`/`.ulg`,
force-track summaries + curated sets) — never a history rewrite.
