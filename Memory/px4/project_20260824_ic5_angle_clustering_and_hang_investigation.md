---
name: project_20260824_ic5_angle_clustering_and_hang_investigation
description: "⭐⭐⭐⭐ 2026-08-26 ROOT CAUSE FOUND (deeper than the angle-clustering framing below, which was itself a downstream symptom): both catastrophic IC5 reps (pre-fix 2.76m AND post-fix-with-a87ac00 4.78m) are actually the ALREADY-DIAGNOSED `dtheta` az-visibility-filter defect from project_20260824_dtheta_ic5_flyaway_rootcause, NOT primarily a Hough/angle-clustering geometry bug. Confirmed via Control_Data.npy: both bad reps show 55-74 dtheta_correction cycles PINNED AT THE 2.0 m/s^2 CAP within the first 2s of flight (vs 0-16 in clean reps) -- a sustained near-continuous burst of max extra lift right at launch, BEFORE any perception failure (verified: at post-fix rep5's worst pinned burst, Detection Status was 100% 'ok'). This destabilizes the trajectory (GT descent_anomaly_cause=ASCENDING) FIRST; lt2_angle_clusters dominance follows ~4s later as a DOWNSTREAM CONSEQUENCE of the resulting bad geometry, not the initiating cause. Explains why a87ac00 only partially helps: it hardens perception (the symptom), not the dtheta control-law defect (the cause). The actual fix (PLASMC_DTHETA_HREF=1, breaks the th_curr self-defeating feedback loop) already exists in controller.py but defaults OFF, pending its own n>=5 validation (see project_20260824_dtheta_href_continuous_compensation)."
metadata:
  node_type: memory
  type: project
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-26T05:08:33.045Z
---

## ⭐⭐⭐⭐ ROOT CAUSE (2026-08-26): the catastrophic mechanism is `dtheta`'s uncapped-duration cap-pinning, NOT primarily angle-clustering

Investigated why the two catastrophic reps (pre-fix rep2 xy=2.76m, post-fix-with-a87ac00 n=5
rep5 xy=4.78m) fail so severely, since the earlier framing (angle-clustering dominates the
fail-reason tally in both) doesn't explain WHY the geometry degrades that badly in the first
place. `Ground_Truth.npy`'s own `descent_anomaly_cause` field for BOTH bad reps reads
`'ASCENDING'` -- the drone climbed away rather than descending -- with `target_lost=False` in
both cases (this is NOT the `CBF_CORNERS_STALE_ABORT`/`TARGET_LOST` open-loop-fallback path
originally assumed; it's a genuine closed-loop instability under otherwise-normal GT-feedback
position control).

**`Control_Data.npy` has the smoking gun** (`dtheta_correction(t)`, `dtheta_az(t)`,
`theta_cone(t)` are all logged per-cycle from the 2026-08-24/25 `dtheta` investigation's own
instrumentation -- this data was already being recorded, just not looked at for this specific
question):

| rep | outcome | pinned-at-2.0-cap cycles in first 2s | longest pinned run (whole flight) |
|---|---|---|---|
| pre-fix rep2 | BAD (2.76m) | 55 | -- |
| post-fix n5 rep5 | BAD (4.78m) | 74 | 26 (t=0.75-1.0s) |
| post-fix n5 rep4 | good (0.015m) | 5 | 3 |
| post-fix n3 rep1 | good (0.062m) | 0 | 0 |
| post-fix n5 rep2 | good (0.021m) | 1 | -- |
| post-fix n5 rep3 | good (0.047m) | 16 | -- |

`dtheta_correction` is `clip(gain=10.0 * ||th_desired - th_safe||, 0, PLASMC_DTHETA_AZ_CAP=2.0)`
(`controller.py` ~line 3345-3352) -- the per-cycle CAP is unconditionally applied by default
(no env var needed to activate it, only to change its value), but nothing stops it from
PINNING at that ceiling for many consecutive cycles when the CBF keeps suppressing lateral
demand hard (which IC5's steep initial viewing angle, near the visibility cone's ~0.766 rad/44
deg ceiling from frame 1, reliably does). A sustained 26-cycle pinned run is ~0.3-0.4s of
continuous MAXIMUM extra-lift injection (2.0 m/s^2, over a fifth of g) right at launch.

**Causality direction confirmed, not just correlated**: mapped post-fix rep5's Control_Data
clock to its Img_Data clock (cross-correlating `MARKER_EXTENT_PX` against GT `1/z`, best-fit
offset 27.0s, r=0.71) and checked Detection Status at the exact moment of the worst pinned
burst (t=0.75-1.0s, control clock) -- **100% `'ok'`, zero misses**. The `lt2_angle_clusters`
domination (239/580 misses, 41%) only appears later, mapping to roughly t=4.4-7.7s in the
control clock -- well after the early dtheta burst. **So the sequence is: dtheta burst
destabilizes the trajectory FIRST -> the resulting bad geometry (position/attitude excursion)
degrades real perception SECOND -> `lt2_angle_clusters` is a downstream symptom of the crash
already being underway, not its cause.**

**This reframes the whole a87ac00 finding**: `a87ac00` hardens the Hough/angle-clustering
PERCEPTION path -- exactly the downstream symptom -- so it plausibly reduces how often a
`lt2_angle_clusters` cascade compounds an already-destabilized trajectory into a full crash
(consistent with the observed ~halving of the failure rate), but it cannot touch the actual
`dtheta` control-law defect that destabilizes the trajectory in the first place. That defect
was already root-caused and a real fix designed on 2026-08-24/25
([[project_20260824_dtheta_az_filter_self_defeating_feedback]],
[[project_20260824_dtheta_href_continuous_compensation]]) -- `PLASMC_DTHETA_HREF=1` (breaks
the `th_curr` self-defeating attitude-history feedback loop that lets the trigger persist
instead of settling) -- but it defaults OFF pending its own n>=5 validation, so every sweep in
this whole investigation (pre-fix AND post-fix) ran with the actual root-cause fix disabled.

**Open item / real next step**: validate `PLASMC_DTHETA_HREF=1` at IC5 with n>=5 (isolated,
no concurrent SITL, current HEAD code) -- if it suppresses the sustained-cap-pinning pattern
(fewer/no early 50+ cycle pinned runs) and the catastrophic-failure rate drops further, that
would be the actual fix for this IC, not further perception hardening.

## ⭐⭐⭐ FINAL (2026-08-26): a87ac00 helps, does NOT fix -- n=5 confirm sweep

The n=3 "apparently already resolved by a87ac00" reading directly below was itself too
optimistic -- a small-sample false negative, same class of error as the original "falsified"
mistake earlier this session (see the RETRACTION section further below), just less severe.
A proper n=5 confirm sweep on current HEAD (unchanged code, same isolated-SITL discipline)
reproduced the SAME catastrophic failure at nearly the same severity.

**Combined post-fix data (2 sweeps, n=8 total, all current HEAD / a87ac00 applied):**

| sweep | rep | detect-ok | `lt2_angle_clusters` share of misses | outcome | xy_err |
|---|---|---|---|---|---|
| n=3 run | 1 | 82% | 0% | SP | 0.062m |
| n=3 run | 2 | 88% | 0% | landed, not SP | 0.061m |
| n=3 run | 3 | 94% | 0% | SP | 0.032m |
| n=5 run | 1 | 62% | 39% (65/168) | landed, not SP | 0.039m |
| n=5 run | 2 | 82% | 0% | SP | 0.021m |
| n=5 run | 3 | 90% | 0% | SP | 0.047m |
| n=5 run | 4 | 93% | 0% | SP | 0.015m |
| n=5 run | 5 | 30% | **41% (239/580)** | **PX4 Impact-detected, hard failure** | **4.78m** |

Post-fix combined: **SP 5/8 (62.5%)**, `lt2_angle_clusters` present at a meaningful level in
2/8 reps, 1/8 a genuine catastrophic hard-impact failure. Compare pre-fix (below): SP 1/3
(33%), `lt2_angle_clusters` meaningful in 2/3 reps, 1/3 catastrophic. **Both catastrophic
reps (pre-fix rep2 xy=2.76m, post-fix n=5 rep5 xy=4.78m) ended via PX4's raw impact detector
(`|a|>50 m/s^2`), NOT the clean loom-inversion touchdown disarm that every good rep hit** --
same failure signature, both before and after the fix.

**Conclusion: `a87ac00` roughly halves the failure rate (and raises SP rate) but does not
eliminate the underlying mechanism.** IC5's steep ~42deg viewing angle can still starve the
corner-join filter of enough real segment pairs to prevent a bad angle-cluster often enough
to cause a hard landing, at a rate on the order of 1-in-5 to 1-in-8 reps. Do not describe this
IC as fixed. If pursuing further: either loosen `merge_tol_deg`/tune the corner-join gap
tolerance specifically for IC5-range viewing angles, or add a grace/retry path in
`landing_test.py` for a sustained `lt2_angle_clusters` streak specifically (distinct from the
existing `CBF_CORNERS_STALE_ABORT`, which is a `_cbf_corners_none_streak` catch-all not
keyed to fail-reason).

## ⭐⭐ RETRACTION + CORRECTED FINDING (2026-08-26): the "falsified" entry below was premature; the bug was real and is apparently already fixed by a87ac00

The FALSIFIED entry immediately below this one (also written 2026-08-26, same session) drew its
conclusion from only ONE data point: a clean re-run on CURRENT HEAD code landing 3/3 tight. That
re-run had an uncontrolled confound -- `cross_marker_detector.py` changed (`a87ac00`, the
corner-join Hough-segment filter) the morning AFTER the original 2026-08-24 failing investigation,
so "clean re-run" was actually "clean environment + different code", not a true isolation of the
contamination variable alone. Called out by the user ("I feel you are jumping the gun here").

**Proper controlled experiment**: temporarily swapped `cross_marker_detector.py` to the EXACT
pre-fix code (`git show 686a66e:...`, the version live during the original 08-24 failing
investigation), re-verified no concurrent SITL, ran the same isolated IC5 n=3 sweep
(`WORLD=cross_marker MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1 HEADLESS=1 IC_LIST="IC5" N_REPS=3`),
then restored current HEAD exactly (verified byte-identical via diff against the pre-swap backup).

| rep | detect-ok | dominant fail reason | landing |
|---|---|---|---|
| 1 | 258/279 (93%) | insufficient_fit_points 13, centroid_mismatch 39 | 0.025m, clean |
| 2 | 156/777 (20%) | **lt2_angle_clusters 370 (60% of 621 misses)** | **2.76m -- degraded** |
| 3 | 230/400 (57%) | lt2_angle_clusters 60 (35% of 170 misses), centroid_mismatch 43 | 0.063m, mildly degraded |

This directly reproduces the ORIGINAL 2026-08-24 signature -- wide detect-ok variance across
identical-IC reps (20-93%, matching the original's 17-94% range) with `lt2_angle_clusters` as the
dominant fail reason specifically in the bad rep -- **with zero concurrent SITL sessions running**.
The failure-reason-to-outcome link (dominant `lt2_angle_clusters` share tracks directly with landing
degradation across the 3 reps) is much stronger evidence than the original's outcome-only landing
stats. **Conclusion: the angle-clustering fragility at IC5's steep oblique viewing angle is a real,
probabilistic, code-level failure mode -- not a contamination artifact.**

**Then, separately: current HEAD code (with `a87ac00` applied) was re-tested clean (n=3, see the
now-superseded FALSIFIED section below for that data) and showed ZERO `lt2_angle_clusters`
occurrences across all 3 reps, landing tight (0.032-0.062m).** `a87ac00`'s docstring frames itself
purely as a "2nd shadow-contamination layer" (filtering Hough segments from the drone's own cast
shadow before they can seed a bad angle cluster) -- it was never explicitly validated against or
credited for fixing the IC5 angle-clustering failure mode specifically. But mechanistically it runs
`_filter_segments_by_corner_join` BEFORE `_cluster_line_angles`, stripping exactly the kind of
spurious/unpaired Hough segments that would otherwise seed a bad cluster -- plausible that it fixes
BOTH the shadow-contamination case it was built for AND this steep-angle case, which may share the
same underlying "spurious segment pollutes the cluster" mechanism even though the original framing
(arms projecting too close to parallel) is geometrically distinct from shadow contamination.

**Net assessment**: the bug was real (confirmed above), and current code most likely already fixes
it as a side effect of an unrelated-sounding fix -- but this is n=3 vs n=3, not a large-sample
confirm. **Recommend a real n>=5 IC5 sweep on current HEAD before fully closing this** -- the
post-fix 3/3 clean result could still partly be luck, same as the original single "passed rep" was.
If `merge_tol_deg=12` or the pre-cluster segment count is still occasionally marginal at IC5's ~42
deg viewing angle, a larger sample is the only way to see it.

---

## ⛔⛔ SUPERSEDED (2026-08-26, was briefly the leading entry, itself retracted above): "FALSIFIED -- no angle-clustering bug at IC5"

This section's own conclusion is WRONG (see the retraction above) -- kept for the process lesson
(a same-code-family "clean" comparison isn't controlled if the code isn't actually the same) and
because its underlying data point (current-HEAD clean sweep = 0/3 lt2_angle_clusters, tight
landings) is still valid and now correctly re-interpreted above as "current HEAD already fixes it",
not "there was never a bug".

Isolated re-run (confirmed no concurrent SITL/claude session touching SITL beforehand),
`WORLD=cross_marker MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1 HEADLESS=1 IC_LIST="IC5" N_REPS=3`
via `run_ic_validation.sh`, on CURRENT HEAD code (post a87ac00, NOT the code live during the
original investigation -- this is the confound that made the "falsified" conclusion premature):

| rep | xy_err | rel_vel | soft | precise |
|---|---|---|---|---|
| 1 | 0.0615m | 0.0330 m/s | yes | yes |
| 2 | 0.0607m | 0.0786 m/s | no | no |
| 3 | 0.0318m | 0.0289 m/s | yes | yes |

All 3 landed cleanly, tight band, no TARGET_LOST, no hard impact.

## ⛔ CORRECTION (2026-08-25, same day, discovered via a DIFFERENT session's memory write)

**This entire IC5 angle-clustering investigation below was very likely run WHILE a
concurrent session was ALSO running its own IC5 sweep** — discovered only after the fact
via [[feedback_check_concurrent_sitl_before_launch]]'s "Independently confirmed" addendum
(that other session found a live SITL process tree spanning 11:04→11:33+, squarely
overlapping this session's `ic5_hang_chase_*` reps). Concrete consequences for THIS
memory's findings:
- `MicroXRCEAgent` binds a single global port (8888) — the repeated launch flakes here
  (attributed below to leaked `/dev/shm` fastrtps_* state) may actually/also be
  `port 8888 already bound` collisions with the other session, not a resource leak alone.
- Two concurrent `gz sim` stacks compete for CPU. Sim TIME stays deterministic (lockstep),
  but real-wall-clock-dependent subsystems — image capture rate, OpenCV decode timing,
  thread scheduling — do NOT, and that's exactly the machinery behind cross-marker
  detection reliability that this memory's "IC5 fails via angle-clustering" conclusion
  rests on.
- The extreme alpha-std variance found across reps (5.4° → 133.3° at the IDENTICAL IC)
  is consistent with genuine steep-viewing-angle fragility, but is EQUALLY consistent
  with resource contention corrupting perception timing unpredictably rep-to-rep. Cannot
  currently distinguish these two explanations with the data gathered.

**What still stands**: `CBF_CORNERS_STALE`'s GT-feedback bypass (`controller.py:1004-1005`)
is a static code fact, read directly, not dependent on sim performance — that ruling-out
is NOT weakened by this correction. What's downgraded is the "IC5 has a genuine
angle-clustering detection bug" conclusion itself, and the "the hang was an infra
artifact, not a control bug" conclusion (equally now explained by the concurrent-session
collision as by the `/dev/shm` leak).

**Before trusting anything below**: re-run the IC5 sweep with verified NO concurrent SITL
session (per the new standing rule), and check whether the failure rate / alpha-std
variance holds up in isolation. If it does, the angle-clustering conclusion is confirmed.
If IC5 lands cleanly and consistently in isolation, the entire investigation below was
chasing a contamination artifact.

Continues [[project_20260824_touchdown_groundcontact_and_perception_hardening]] (same
session). That memory covers the ground-collision-height finding + 3 perception fixes;
this one covers the follow-up IC1-5 validation sweep and the IC5-specific failure dig.

## IC1-5 GT-feedback sweep at true ground contact (`test_data/GTFB_GroundContact_IC1to5/`)

Flat `cross_marker` world (not rover), true ground collision height
(`model.sdf.bak_before_legext_20260809` reverted, per the prior memory), all 3 same-session
perception fixes live (Hough corner-join filter, hw coast+freeze KF, touchdown rolling
window).

| IC | Result | xy_err | rel_vel | min_alt |
|---|---|---|---|---|
| IC1 (0,0,5) | SOFT+PRECISE | 0.049m | 0.076 m/s | -0.01m |
| IC2 (2,2,5) | SOFT+PRECISE | 0.072m | 0.043 m/s | -0.01m |
| IC3 (-2,2,5) | SOFT+PRECISE | 0.042m | 0.013 m/s | -0.01m |
| IC4 (2,2,7) | SOFT+PRECISE | 0.030m | 0.044 m/s | -0.01m |
| IC5 (2,2,3) | **TARGET_LOST / hard impact** | 2.429m | 2.149 m/s | -0.02m |

IC1-4 confirm the ground-contact fix generalizes (not an IC1-only artifact); all latch
`TOUCHDOWN-DETECT` cleanly at true ground. IC5 is the outlier.

## IC5 root cause: angle-clustering failure at steep oblique viewing angle

IC5 = `ENU (2,2,3)`, i.e. low altitude (3m) + large lateral offset (2,2) -> a much
steeper viewing angle (~42 deg) than IC1-4 (same lateral offset but 5-7m altitude, or
IC1's centered geometry). This is a DIFFERENT failure mode from the near-touchdown
shadow/Hough-line-COUNT collapse this session's other fixes target (Fix 1's corner-join
filter, `project_20260824_touchdown_groundcontact_and_perception_hardening`) -- here
Hough finds segments fine, but they fail to separate into >=2 distinct ANGLE CLUSTERS
(`_cluster_line_angles`'s `merge_tol_deg=12` tolerance), because the cross's two arms
project too close to parallel/merged at this oblique angle.

**Confirmed via 3 independent reps at the identical IC**, revealing the failure is
probabilistic (threshold effect), not deterministic:

1. **Failed rep** (`Mon Aug 24 21-51-45 2026`): 17% detect-ok (116/686), TWO long
   consecutive miss streaks (287 and 263 frames), `lt2_angle_clusters` dominant fail
   reason (388/570 misses, 68%) + `hough_lt2_lines` (63) + `color_gate_empty` (62) --
   90% of failures never resolved marker geometry at all. Hover-settle alpha
   std=14.6 deg.
2. **Passed rep** (`Mon Aug 24 22-43-33 2026`, same IC, re-run): 94% detect-ok
   (303/324), longest miss streak only 14 frames, dominant fail reason
   `insufficient_fit_points` (16/21, 76%) -- geometry resolved fine, just a few frames
   short on surviving pixels post-pruning (a MUCH milder failure class). Hover-settle
   alpha std=11.8 deg. Landed SOFT+PRECISE (xy=0.003m).
3. **Extreme rep** (`ic5_dbg_1.out`, `PLANAR_MAP_DBG=1` trace): hover-settle alpha
   std=133.3 deg (near uniform-random -- essentially garbage even before descent
   starts). `_cbf_corners_none_streak` grew monotonically with ZERO resets from frame
   ~7 through the observed trace (up to 172) -- direct printed proof of a sustained,
   total detection loss, not intermittent flicker. This rep hung past 180s rather than
   reaching TARGET_LOST or landing (see hang investigation below).

**Mechanism, not just correlation**: `_cbf_corners_none_streak` (which the printed
`[cbf_corners]` trace exposes directly) is exactly what feeds `CBF_CORNERS_STALE_ABORT`
(`controller.py:1017-1034`, threshold 350 frames) -- which is the ONLY thing gating
`landing_test.py`'s `feature_fresh` under GT-feedback (`GT_FEEDBACK==1` always shorts the
OR true, so `feature_fresh = not CBF_CORNERS_STALE_ABORT` exactly, by deliberate design --
see that property's own docstring, "ANDed in... so it can force feature_fresh=False even
when every other signal says fine"). A long enough `lt2_angle_clusters` streak at IC5
crosses that fuse and triggers the open-loop `TARGET_LOST` fallback -> hard impact.

## CBF_CORNERS_STALE (kappa-freeze) ruled out as a contributing mechanism

Checked directly in code (`controller.py:1004-1005`): `CBF_CORNERS_STALE` (the FAST,
30-frame version that freezes kappa's ODE integration, separate from the 350-frame
ABORT version) has an explicit `if self._gt_feedback is not None: return False` --
hard-bypassed under GT-feedback since 2026-08-19 (ported from the Hardware fork,
specifically because it was found pinning kappa at its KAPPA0 bootstrap value on real
GT/HW-position-feedback flights). All IC5 reps used `PLASMC_GT_FEEDBACK=1`, so kappa was
integrating normally throughout, never frozen by this path. Ruled out with certainty by
reading the property, not inferred.

## The "hung past 180s" rep: NOT reproduced, most likely an infra artifact

The extreme rep (#3 above) uniquely didn't reach TARGET_LOST or landing -- the outer
harness killed it as hung past 180s wall-clock, with `_cbf_corners_none_streak` still
only at 172 (well under both the 30-frame CBF_CORNERS_STALE fast-path -- irrelevant here,
bypassed -- and the 350-frame ABORT threshold). Attempted to reproduce with fresh
instrumentation (`PLANAR_MAP_DBG=1 TD_DEBUG=1`) 4 times in a row -- **all 4 retries
failed at the LAUNCHER level** (PX4 stuck waiting for Gazebo's `/world/cross_marker/clock`
topic, "Unable to get simulation time", 60s timeouts), never even reaching the control
loop. Root-caused those launcher failures to **~489 leaked `fastrtps_*` files in
`/dev/shm`** (24 SysV shm segments too), accumulated from repeated `kill -9` teardowns of
PX4/Gazebo across today's ~25+ SITL launches -- DDS/FastRTPS shared-memory transport
artifacts that don't get cleaned up on a forced kill. Removed the `fastrtps_*`-prefixed
entries specifically (user-approved, left the other ~366 unrelated files alone -- /dev/shm
is a shared system resource). The very next launch after cleanup worked cleanly and
landed SOFT+PRECISE with `_cbf_corners_none_streak` never exceeding 0 the whole flight --
no new evidence on the hang mechanism, since detection was perfect that rep.

**Conclusion (circumstantial, not proven)**: the original hang is most likely the SAME
class of PX4/Gazebo clock-sync degradation as the 4 failed retries, not a genuine
control-code stall -- especially since the one code-level stall hypothesis
(CBF_CORNERS_STALE/kappa-freeze) is definitively ruled out under GT-feedback. Treated as
closed/environment-attributed. If it recurs under verified-clean `/dev/shm` conditions,
re-open as a real bug.

## Process lesson: concurrent-session check gap, caught live by the user

Mid-investigation, cleanup kill loops targeting `px4_sitl`/`landing_test.py`/
`mavsdk_server`/`MicroXRCEAgent` were run WITHOUT the `grep -qa claude
/proc/$p/cmdline`-style ownership guard that the project's own launcher scripts already
use for `gz sim` kills specifically to avoid killing another Claude-owned process. User
asked directly whether concurrent SITL use had been checked -- it had not. `ps` confirmed
2 other `claude` sessions were running on this machine (different ptys). User then
confirmed no concurrent SITL use was actually happening, so no real harm done, but the
gap was real. See [[feedback_check_concurrent_sitl_before_launch]] (new standing rule,
also linked from the px4/MEMORY.md banner) for the full checklist -- apply this BEFORE
any future SITL launch or kill-loop cleanup in this project, not just when troubleshooting
flakiness (that's exactly when the temptation to skip the check is highest).

## Open item / natural next step

The angle-clustering fragility itself is NOT fixed -- `_cluster_line_angles`'s
`merge_tol_deg=12` (or another parameter in that path) likely needs to be less brittle
under high viewing-angle tilt, OR `landing_test.py`'s grace/abort logic needs to tolerate
IC5-class steep-angle noise better. Neither attempted this session; this memory documents
the confirmed root cause only, not a fix.
