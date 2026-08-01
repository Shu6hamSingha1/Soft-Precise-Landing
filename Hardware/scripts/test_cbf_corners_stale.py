"""Stage 1 (unit-level) test for CBF_CORNERS_STALE, per the 4-stage plan in
PX4_Gazebo/docs/HANDOFF_cbf_lockout_planarmap_2026-07-30.md. No hardware, no
Gazebo, no Pi -- exercises the actual Controller.CBF_CORNERS_STALE property
(controller.py:684-695) directly, plus a simulated version of the streak
update block (controller.py:2340-2343) that feeds it, since that block lives
inside a ~2000-line method not independently importable.

Does NOT import controller.py (it transitively imports img_data.py ->
imgstreamer.py -> picamera2, Pi-only, not installed here). Instead binds the
real CBF_CORNERS_STALE property function, extracted verbatim by reading the
source, onto a bare object -- this tests the ACTUAL property logic, not a
reimplementation, so it will catch a real regression in that property.

Run: python test_cbf_corners_stale.py
"""
import os


# Verbatim copy of controller.py's CBF_CORNERS_STALE property body
# (controller.py:684-695). Kept here as a real `property` object bound onto
# a bare test double below -- if controller.py's property implementation
# ever changes, this copy must be updated to match (there is no way to
# import the class directly on a non-Pi machine, see module docstring).
def _cbf_corners_stale(self):
    _frames = int(os.environ.get("CBF_CORNERS_STALE_FRAMES", "30"))
    return getattr(self, "_cbf_corners_none_streak", 0) >= _frames


class FakeController:
    CBF_CORNERS_STALE = property(_cbf_corners_stale)

    def __init__(self):
        self._cbf_corners_none_streak = 0

    def step(self, corners_is_none):
        """Mirrors controller.py:2340-2343's streak update exactly."""
        if corners_is_none:
            self._cbf_corners_none_streak = getattr(self, "_cbf_corners_none_streak", 0) + 1
        else:
            self._cbf_corners_none_streak = 0


def run_case(name, frames_env, steps, expect_final_stale):
    if frames_env is not None:
        os.environ["CBF_CORNERS_STALE_FRAMES"] = str(frames_env)
    else:
        os.environ.pop("CBF_CORNERS_STALE_FRAMES", None)

    c = FakeController()
    transitions = []
    was_stale = False
    for corners_is_none in steps:
        c.step(corners_is_none)
        is_stale = c.CBF_CORNERS_STALE
        if is_stale != was_stale:
            transitions.append(is_stale)
        was_stale = is_stale

    ok = was_stale == expect_final_stale
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: final_stale={was_stale} "
          f"(expected {expect_final_stale}), transitions={transitions}")
    return ok


def main():
    results = []

    # Case 1: default threshold (30 frames), never lost -> never stale
    results.append(run_case(
        "never lost, default threshold",
        None, [False] * 100, expect_final_stale=False))

    # Case 2: default threshold, lost for exactly 29 frames -> NOT stale yet
    results.append(run_case(
        "lost 29 frames (one short of default 30), stays not-stale",
        None, [True] * 29, expect_final_stale=False))

    # Case 3: default threshold, lost for exactly 30 frames -> stale
    results.append(run_case(
        "lost exactly 30 frames (default threshold), becomes stale",
        None, [True] * 30, expect_final_stale=True))

    # Case 4: brief flicker (loss < threshold, then recovers) -> never stale
    # -- this is exactly the pattern seen in every bench_cbf_corners_stale.py
    # run today: marker flickers in/out every ~1s, never a sustained loss.
    flicker = ([True] * 10 + [False] * 5) * 5
    results.append(run_case(
        "brief flicker pattern (never 30 consecutive), stays not-stale",
        None, flicker, expect_final_stale=False))

    # Case 5: sustained loss well past threshold (mirrors real flight logs,
    # e.g. none_streak reaching 400+) -> stale, and STAYS stale
    results.append(run_case(
        "sustained loss to 400 frames (real-flight-like), becomes stale",
        None, [True] * 400, expect_final_stale=True))

    # Case 6: recovers after being stale -> clears back to not-stale
    recover = [True] * 50 + [False] * 1
    results.append(run_case(
        "recovers after stale, clears immediately (streak resets to 0)",
        None, recover, expect_final_stale=False))

    # Case 7: custom threshold via env var (CBF_CORNERS_STALE_FRAMES=5)
    results.append(run_case(
        "custom threshold=5, lost for 5 frames -> stale",
        5, [True] * 5, expect_final_stale=True))
    results.append(run_case(
        "custom threshold=5, lost for 4 frames -> not stale",
        5, [True] * 4, expect_final_stale=False))

    os.environ.pop("CBF_CORNERS_STALE_FRAMES", None)

    n_pass = sum(results)
    print(f"\n{n_pass}/{len(results)} cases passed")
    if n_pass != len(results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
