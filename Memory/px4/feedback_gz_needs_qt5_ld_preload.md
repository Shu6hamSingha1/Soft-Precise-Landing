---
name: feedback-gz-needs-qt5-ld-preload
description: gz sim cannot start on this machine without LD_PRELOAD of the PyQt5 libQt5Gui.so.5 — run_aruco_landing.sh lacks it, so the canonical launcher fails while the project's own harnesses work
metadata:
  type: feedback
---

`gz sim` fails to start on this machine with:

```
Library error for [/usr/lib/x86_64-linux-gnu/libgz-sim8-gz.so.8.15.0]:
  /lib/x86_64-linux-gnu/libQt5Quick.so.5: undefined symbol:
  _ZNK16QDoubleValidator8validateER7QStringRi, version Qt_5
```

PX4 then sits in `Waiting for Gazebo world...` until `ERROR [init] Timed out waiting for Gazebo
world`, and every rep reports `landed=NO` with no data. **The failure looks like the change under
test broke the controller — it is an environment fault and identical in both arms of an A/B.**

**Fix (already used by the project's own harnesses):**
```bash
env LD_PRELOAD=/home/shubham/cvenv/lib/python3.8/site-packages/PyQt5/Qt5/lib/libQt5Gui.so.5 \
    HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross bash scripts/run_aruco_landing.sh
```
✅ **FIXED 2026-09-03: the canonical launcher now sets this itself.** `scripts/run_landing.sh`
(renamed from `run_aruco_landing.sh`) defines `QT_PRELOAD` with this path as its default and folds
it into the PX4 launch env in BOTH headless and GUI modes, warns and continues if the file is
missing, and `QT_PRELOAD=` (empty) opts out. So a bare `bash scripts/run_landing.sh` now works.
Historical note: `record_robustness_set.sh:12,29` and `spanrescue_ab.sh:61` carried it first —
the canonical launcher did not, which is what made the failure look like a code regression. Verified 2026-09-03: without the preload `gz sim -r -s worlds/cross_marker.sdf`
dies instantly; with it, it runs clean and a full IC2 landing flew (1298 samples, xy 0.117 m).

**How to apply:** nothing to do for a normal launch — `run_landing.sh` handles it. Only export
`LD_PRELOAD` yourself when invoking `gz sim` DIRECTLY (bypassing the launcher). If a sweep returns
all-`NO` with no rep dirs, check for the Qt symbol error in `run_logs/px4_sitl.log` BEFORE
suspecting the code under test.

**Launcher renamed in the same commit:** `run_aruco_landing.sh` → **`run_landing.sh`**, and
`run_aruco_landing_retry.sh` → **`run_landing_retry.sh`**. The "aruco" was misleading (WORLD /
MARKER_TYPE are env-driven, and the standing rule makes every run cross-marker), and it caused a
real "are you running ArUco?" doubt during a cross-marker-only session. 184 references across code,
docs, memory and ~50 committed A/B harnesses point at the old names, so **both old names survive as
exec shims** — nothing breaks, but USE THE NEW NAMES in new work. The launcher now also prints
`[run] FLYING: WORLD=... MARKER_TYPE=...` so any log answers the question directly.

⚠ Do not confuse this with the *other* all-`NO` cause found the same day: a stale/orphaned SITL
holding `udp:8888` (`bind error errno 98`) — see
[[feedback_check_concurrent_sitl_before_launch]]'s 2026-09-03 section. Both produce identical
`landed=NO` summaries; the distinguishing evidence is in the per-rep `.log`
("Timed out waiting for Gazebo world" vs "bind error ... port: 8888").
