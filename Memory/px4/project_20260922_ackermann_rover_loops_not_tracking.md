---
name: project_20260922_ackermann_rover_loops_not_tracking
description: "2026-09-22: at slow position setpoints (~0.15 m/s Lissajous) the Ackermann rover does NOT track -- it parks ~0.5 m off then drives re-approach loops at 1.5-2.2 m/s. All 11 Lissajous_final reps have GT target peaks 1.6-2.8 m/s. The k=0.1 'SOFT+PRECISE' (23-16-02) landed on a rover parked for its final 6.5 s. Invalidates the Lissajous speed-retune conclusions."
metadata:
  node_type: memory
  type: project
  originSessionId: eb3af863-38fc-4d7b-86dd-b7c9bc16b44d
  modified: 2026-09-22T17:56:52.860Z
---

**Finding (2026-09-22, from GT `Target Pose` + the rover's PX4 ulog `rootfs/1/log/2026-09-22/17_36_42.ulg`).**
The Ackermann rover under `rover_drive.py` position setpoints (`set_position_ned`, 5 Hz in the
ulog) cannot follow a setpoint moving at ~0.15 m/s. It stays in offboard (nav_state 14, no
failsafe) but alternates: **parked ~0.46 m from the setpoint → once error grows, a full loop at
1.5-2.2 m/s → parked again**. It already does this every ~6 s during the pre-gate "holding start
pos" phase (the hold point is a fixed setpoint it still cannot settle on).

**Evidence, rep `test_data/RecordGTFB_dev/Lissajous_final/Tue Sep 22 23-07-55 2026`**
(= `Test_Videos/chase_2026-09-22_23-06-44.mp4`, identical md5): commanded 0.150 m/s median;
GT target 0.00 m/s for t=0-3.8 s, then a U-turn (heading 165°→0°, radius ~1 m) peaking **2.17 m/s
at t=5.3 s**; rover EKF speed matches GT (max 2.26). Drone failed (xy 2.65 m, rel_vel 4.3 m/s).
All 11 Lissajous_final reps (k=0.4/0.2/0.1): GT peak 1.6-2.8 m/s.

**Consequences:**
- The k=0.4→0.2→0.1 "speed retunes" in [[project_20260922_lissajous_cbf_and_divergence_mechanism]]
  changed the COMMAND only; the drone never saw a slow Lissajous target.
- The k=0.1 SOFT+PRECISE rep (`23-16-02`, xy=0.013 m) — the rover looped at up to 1.6 m/s for
  t=0-3.5 s, then was **parked for the final 6.5 s** until touchdown. That is effectively a static
  landing, not "Lissajous solved".
- Likely also the "unresolved ~3x Lissajous speed-tracking gap" noted in
  [[project_20260922_rover_drive_wallclock_pacing_bug]] — a different mechanism from the clock bug.
  Other slow profiles (CircularYaw ~0.26 m/s, etc.) may be affected too — not yet checked.

**How to apply:** never quote a rover-profile speed from `rover_trajectory.py`/`ROVER_SPEED_MULT`;
compute it from each rep's `Ground_Truth.npy` `Target Pose` (dedupe stale samples, 0.2-0.5 s
window). Candidate fix (untested): velocity/feedforward setpoints in `rover_drive.py`, or keep
commanded speed above the rover's minimum controllable speed. pyulog isn't in env2025 — use a
scratch venv to read the rover ulog.
