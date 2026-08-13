# PENDING: port the NaN self-latch fix to Hardware/scripts/controller.py

**For whichever Claude session (Windows or otherwise) picks this up.**

## What's wrong

`Hardware/scripts/controller.py` (~line 1195-1197) computes wall-clock loop
timing with no minimum-dt floor and no finite check:

```python
_now_loop = self._time.perf_counter()
if self._last_loop_t is not None:
    self._last_loop_dt = _now_loop - self._last_loop_t
self._last_loop_t = _now_loop
```

A near-duplicate `perf_counter()` read (near-zero or zero `dt`) divides
straight into the SMC's `dh_d` finite-difference downstream, producing a
literal NaN that self-latches through `kappa`'s RK5 state for the rest of
the flight (commanded body-rate/thrust all going NaN — a real, flight-
ending failure mode, not a cosmetic bug).

## Where the fix already exists

`PX4_Gazebo/src/controller.py` has the fix, found/shipped in a PX4/Gazebo
SITL session on 2026-08-13 (commit `2a8ab76`, "Fix NaN self-latch bug:
unguarded near-zero wall-clock dt in dh_d finite-diff"). It's the SAME
pattern, same relative code location — `PX4_Gazebo`'s `controller.py`
inherited it from `Hardware/scripts/controller.py` via a 2026-08-04 port
originally, so this file is the ancestor of the bug, not a separate
occurrence. Port two things from that commit:
1. A dt floor on `self._last_loop_dt` (matching `gz_subscriber.py`'s
   existing dt-floor convention on the PX4/Gazebo side — Hardware has no
   direct equivalent file, so just clamp to a small positive epsilon,
   e.g. `max(dt, 1e-3)`, consistent with the loop's actual expected rate).
2. An independent finite-check guard on the `dh_d` division itself (belt
   and suspenders — don't rely solely on the dt floor).

`git show 2a8ab76 -- PX4_Gazebo/src/controller.py` in this repo has the
exact diff to mirror.

## Does NOT affect / is unrelated to

The CBF extent-blindness fix and radius-cap/split work (`PX4_Gazebo`
commits `c24d486`..`27410ed`) are cross-marker-specific and don't apply to
Hardware at all — `Hardware/scripts/controller.py` has no `MARKER_TYPE`
branching and no cross-marker code path (confirmed via full git history
search 2026-08-14: zero commits touching `Hardware/` mention cross-marker).
Don't port anything from those commits here; this file is ONLY about the
NaN fix.

## When you're done

Verify (unit-check the dt-floor/finite-check guards, plus a couple of
clean live flights if hardware time is available), then **delete this
file** as the completion signal — the PX4/Gazebo-side session will look
for its absence (or its removal in a subsequent `git log`) to confirm the
port landed, rather than assuming from a status claim alone.
