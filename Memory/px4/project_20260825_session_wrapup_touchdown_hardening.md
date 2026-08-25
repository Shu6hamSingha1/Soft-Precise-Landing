---
name: project_20260825_session_wrapup_touchdown_hardening
description: "Session wrap-up (2026-08-24 -> 2026-08-25): 3 cross-marker touchdown/perception fixes COMMITTED (d8b06cf, a87ac00) and validated under GT-feedback; ground-collision-height (model.sdf) root-caused as the real reason GT-feedback landings appeared to stall, and is now DELIBERATELY at true ground contact (user directive, ongoing work, not a leftover); IC5 angle-clustering finding DOWNGRADED to unconfirmed (concurrent-session SITL contamination found); first perception-mode (non-GT) test of the 3 fixes found a NEW bug -- false TOUCHDOWN-DETECT at 3.75m altitude, plus a hang recurrence under CONFIRMED-clean SITL conditions. Next-session priorities listed at the bottom."
metadata:
  node_type: memory
  type: project
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-25T07:25:16.395Z
---

Ties together [[project_20260824_touchdown_groundcontact_and_perception_hardening]] and
[[project_20260824_ic5_angle_clustering_and_hang_investigation]] (both same session) into
one wrap-up with corrected/updated conclusions and a clear next-steps list, written because
the user is starting a fresh chat.

## What's committed (safe, done, don't re-do)

1. **`d8b06cf`** — `cross_marker_perception.py`: hw coast+freeze KF ported verbatim from
   `img_data.py`'s already-validated ArUco pattern. Fixes a reintroduction of
   [[feedback_kf_frozen_during_marker_loss]] (`self._hw = np.zeros(6)` on every miss,
   present since the file's first commit, never revisited). `controller.py`: touchdown
   rolling-window streak (`_td_hist = deque(maxlen=_td_window)`), no-op by default
   (`PLASMC_TD_WINDOW` defaults to `_td_frames`).
2. **`a87ac00`** — `cross_marker_detector.py`: corner-join Hough-segment filter (2nd
   shadow-contamination layer, complements the same-session `_robust_fit_line` fix from
   commit `5c6400a`). Filters raw Hough segments before angle-clustering, keeping only
   those with a geometrically-joined partner at 90 or 45 degrees (a real marker-corner
   signature the drone's own cast shadow doesn't share). This was originally committed
   (`fa187a2`) then reverted (`686a66e`) by the USER DELIBERATELY, specifically so this
   session could commit it cleanly itself (not a sign it was rejected — confirmed by user
   directly after initial confusion).
3. Both commits were isolated via hunk-level `git apply --cached` splitting from a SEPARATE
   concurrent session's unrelated work (AZ-visibility-filter-v3/dtheta feature) that was
   interleaved in the same uncommitted `controller.py`/`cbf_visibility*.py` — that other
   session's work is its own, already committed separately as `727e73a`, not touched by
   this session.

All 3 fixes were validated under **GT-feedback only** (multiple clean SITL reps, no
regression, IC1-4 all SOFT+PRECISE at true ground contact). **NOT yet validated under real
perception** — see the new finding below, which is exactly why that gap matters.

## `model.sdf` ground-collision height — DELIBERATE, ongoing, do not "restore"

`~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf` (OUTSIDE this repo) is
currently at **TRUE ground contact** (`base_link_collision_3`/`_4` Z pose `-0.2195`,
`model.sdf.bak_before_legext_20260809`). This was root-caused mid-session as the reason
GT-feedback cross-marker landings appeared to "stall"/"loom flip at 0.5m" — a real
collision floor deliberately raised +0.5m on 2026-08-09 for camera standoff (texture-blur
mitigation), NOT a control bug. Reverting to true ground gave immediate clean SOFT+PRECISE
landings.

**User confirmed (2026-08-25) this true-ground state is INTENTIONAL and ongoing** — the
goal is to understand and improve the landing height itself, not to re-hide the gap by
raising the floor again. **Do not revert to the 0.487m-raised version
(`model.sdf.bak_before_gtfb_groundcontact_test_20260824`) without an explicit new user
directive.** Perception-mode cross-marker work under true ground contact will hit the
texture-resolution blur ceiling (~0.79m crossover vs ~0.34m camera-to-marker distance at
touchdown) — that's the real open problem this phase is working toward, not a bug to
route around.

## IC5 angle-clustering finding — CONFIDENCE DOWNGRADED, not confirmed

Original finding: `cross_marker_detector.py`'s angle-clustering step (`lt2_angle_clusters`
fail reason) appeared to fail intermittently at IC5's steep oblique viewing angle (low
altitude + large lateral offset). **This was very likely investigated while a concurrent
session was ALSO running its own IC5 sweep** (discovered after the fact via a different
session's own memory write) — shared `MicroXRCEAgent` port 8888 + `gz sim` CPU contention
can corrupt exactly the real-wall-clock-dependent perception timing this finding rests on.
**Needs a clean, isolated re-run (verified no concurrent SITL) before trusting this root
cause.** See [[project_20260824_ic5_angle_clustering_and_hang_investigation]]'s correction
section for full detail. What DOES still stand: `CBF_CORNERS_STALE` (kappa-freeze) is
definitively ruled out as a factor under GT-feedback — that's a static code fact
(`controller.py:1004-1005`), not sim-timing-dependent.

An attempt to use a DIFFERENT concurrent session's own live IC5 run (running in parallel,
per user's suggestion) to evaluate this turned out to be a mismatch — that session was
testing ArUco (not cross-marker) with its own CBF-margin-reserve fix, a completely
different code path from `cross_marker_detector.py`'s angle-clustering. Not usable evidence
either way for this specific finding.

## NEW FINDING (2026-08-25, this session's last action): false TOUCHDOWN-DETECT under real perception

First-ever perception-mode (non-GT-feedback) test of the 3 committed fixes, IC1 (centered,
simplest case), true ground contact, confirmed-clean SITL (checked before launch AND no
other session's processes present):

```
[controller] TOUCHDOWN-DETECT: loom spiked (h_z>0.0 x3 within 3-frame window) |s_e_n|=0.06 -> LANDED
[landing_test] Landing classification: FAIL (xy_err=0.235m, rel_vel=1.274 m/s)
[landing_test] Honest precision @ min-alt: xy=0.228m, rel_vel=1.047 m/s, min_alt=3.75m
```

**`TOUCHDOWN-DETECT` fired at 3.75m altitude — nowhere near the ground.** This is a
GENUINE NEW GAP: all prior validation of the touchdown-detect logic (this session's rolling
window included) was done exclusively under GT-feedback, where `h_z` is exact/noise-free
by construction. Real perception noise can apparently produce a spurious 3-frame positive
loom spike at ANY altitude, not just near touchdown — the rolling-window widening (which
only helps tolerate MISSED spikes near real touchdown) does nothing to prevent a FALSE
spike triggering prematurely. Not root-caused yet — added to `diagnose-flight-data` skill
as a new "watch for" pattern (check `h_z` + `Detection Status` at the disarm timestamp for
the false-positive signature: a spike coinciding with a miss/recovery transition, not a
sustained near-ground trend).

**Same rep also hung again** ("hung past 180s wall-clock") during the post-touchdown
5s video-tail capture — this time with CONFIRMED clean SITL (checked before launch,
monitored throughout, nothing else running). This WEAKENS the earlier
"hang was probably concurrent-session contamination" conclusion for this specific
hang class — it reproduced under genuinely isolated conditions. May be related to the
false touchdown (a premature disarm followed by an unexpected state the IMG_RECORD
tail-capture logic doesn't handle cleanly), or a separate bug. Not investigated further
this session — output at `test_data/Rover_AB_harness/perception_ic1_1.out` and
`test_data/Landing_Test_Cross_Perception/` (only run so far under this
`LANDING_OUT_BASE`).

## Next-session priorities, in order

1. **Root-cause the false TOUCHDOWN-DETECT at altitude** (perception-mode, IC1, this
   session's last finding) — highest priority, this is a regression risk for ANY
   perception-mode landing now that the 3 fixes are committed and presumably will see
   real use. Check `h_z` and `Detection Status` around the 3.75m disarm event first.
2. **Re-investigate the "hung past 180s" pattern** now that it's reproduced under
   confirmed-clean SITL — check whether it's tied to the false-touchdown/premature-disarm
   sequence (a plausible common cause) rather than being purely environmental as
   previously suspected.
3. **Re-run the IC5 angle-clustering investigation from scratch**, verified-isolated
   (no concurrent SITL), before trusting or discarding the original finding.
4. **Continue perception-mode validation of the 3 committed fixes across IC2-5**, not just
   IC1 — only one perception-mode rep has been run so far, and it surfaced a new bug before
   even reaching a clean pass/fail signal on the original fixes' own effectiveness.
5. Always run `ps -eo pid,ppid,tty,user,lstart,cmd | grep -E "px4_sitl|gz sim|landing_test.py|MicroXRCEAgent"`
   before ANY SITL launch or kill-loop this session's mistakes made clear how easy this is
   to skip under pressure — see [[feedback_check_concurrent_sitl_before_launch]] and
   `PX4_Gazebo/docs/SH_REFERENCE.md` §8 pitfall 10 (added this session).

## Untracked backlog (pre-existing, not this session's responsibility, FYI only)

~240+ untracked files (memory `.md` files from 2026-08-13 onward, numerous `.bak_*` files
across `src/`, `apps/`, `scripts/`, `Hardware/`) predate this session and were not created
or touched by it. Not committed as part of this session's wrap-up — would need separate
review/triage, likely spanning multiple other sessions' work.
