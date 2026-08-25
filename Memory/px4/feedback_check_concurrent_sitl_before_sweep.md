---
name: feedback_check_concurrent_sitl_before_sweep
description: "SUPERSEDED/MERGED -- see feedback_check_concurrent_sitl_before_launch, which folds this finding in as an appended section. Kept as a redirect stub to avoid two near-duplicate topic files."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 0f9c8dc0-a837-4722-95e7-0ea102167469
  modified: 2026-08-25T06:06:12.098Z
---

Discovered 2026-08-25 mid-way through validating [[project_20260825_cbf_margin_reserve_fix]]: a completely separate, independent SITL session (`test_data/Rover_AB_harness/ic5_hang_chase_*.out`, different process tree, own `LANDING_OUT_BASE`) had been running its own `run_aruco_landing.sh` continuously (11:04→11:33+) squarely inside the window of an n=3 IC5 sweep being run in THIS session -- discovered only via `ps aux` after the sweep already showed unexpectedly noisy/inconclusive results.

**Why: don't guess, check.** `MicroXRCEAgent` binds a single global port (8888) -- a second concurrent session's SITL stack collides directly, causing exactly the kind of "bind error: port 8888, errno 98" / "not the race condition" launch flakes this project has fought all session and attributed to generic SITL flakiness. Beyond launch failures (harmless -- fail-fast, no corrupted data), two simultaneous `gz sim` physics+rendering stacks compete for CPU; Gazebo/PX4 LOCKSTEP timing stays deterministic, but anything real-wall-clock-dependent (image processing rate, OpenCV decode timing, Python thread scheduling, ROS bridge latency) is NOT -- exactly the machinery behind perception decode reliability, which most SITL sweeps in this project are measuring. Contention there is a plausible, NOT retroactively provable, noise source.

**How to apply:** before starting any n>=3 sweep meant to produce a trustable directional or quantitative result (not a quick smoke test), run `ps aux | grep -iE "gz sim|px4|xrce|landing_test"` and confirm nothing unexpected is already running. If a sweep's results come back noisier/more inconclusive than the mechanism would predict, check this BEFORE concluding the fix itself is weak or the phenomenon is genuinely stochastic -- it may just be contaminated data, not a real negative result. A confound found only after the fact isn't fixable retroactively; the sweep needs a clean re-run, not a reinterpretation of the tainted numbers.
