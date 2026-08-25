---
name: project_20260824_ic5_angle_clustering_and_hang_investigation
description: "⛔ CONFIDENCE DOWNGRADED 2026-08-25 (see correction below): IC1-5 GT-feedback sweep at true ground contact found IC5 (2,2,3 ENU) failing intermittently via TARGET_LOST/hard impact, provisionally attributed to cross_marker_detector.py's angle-clustering step failing at IC5's steep oblique viewing angle (lt2_angle_clusters dominant fail reason; alpha std during hover-settle ranged 5.4-133.3 deg across reps). BUT a concurrent session's own IC5 sweep was found running AT THE SAME TIME as several of these reps (shared MicroXRCEAgent port 8888 + competing gz sim CPU load) -- the wild variance may be contamination, not genuine viewing-angle fragility. NEEDS A CLEAN, ISOLATED RE-RUN before trusting this root cause. CBF_CORNERS_STALE (kappa-freeze) ruling-out stays valid (a code-level fact, not sim-timing-dependent)."
metadata:
  node_type: memory
  type: project
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-25T06:08:07.974Z
---

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
