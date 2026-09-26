# FIX LOG — bugs, fixes, and how to confirm each fix

Purpose: work across several Claude Code sessions (Windows / Ubuntu / Pi) without losing what a fix was
supposed to do or how to prove it. **Every bug gets an entry while it is being worked on** (rule from the user,
2026-09-26). Newest first. Do not delete entries; change Status and fill Result.

Status values: `open` (found, not fixed) -> `in-repo` (committed, not deployed/flown) -> `deployed` (on Pi/PX4, unflown)
-> `verified` (confirmed with the data below) or `failed` (checks did not pass; say why).

Entry template:

```
### FIX-NNN <short title>  [status] (opened YYYY-MM-DD)
- Symptom / evidence:  what was observed, with data paths / run ids / numbers
- Root cause:          file:function, why it happens (mark "hypothesis" if unverified)
- Fix:                 what changes, files, commit, env knob + default
- Confirm with:        data to collect (sim IC set / flights / recording), command or script, expected numbers (PASS = ...)
- Must not regress:    other checks that must stay unchanged
- Result:              filled when checked (date, numbers, verdict)
```

---

### FIX-001 Cross detector rejects an off-frame junction (partial visibility)  [open] (opened 2026-09-26)
- Symptom / evidence: Gazebo GT scoring (`PX4_Gazebo/tools/score_partial_visibility.py test_data 0.5`, 74 consistent reps,
  alt > 0.3 m). Centre in frame: 95% detected, 2.9 px median error. Centre outside frame by 0-20 / 20-40 / 40-80 / >80 px:
  42 / 49 / 50 / 31% "ok" but centre error 170 / 176 / 203 / 347 px (far more than the distance outside = wrong lock).
  Requirement: any two partially visible lines must be enough.
- Root cause: `cross_stroke_detector.py` `side_stats()` returns `bal=None` when J is at/outside the frame edge and the pair loop
  does `if ba is None and bb is None: continue`; `cross_marker_perception.py` never reads `det.in_fov`.
  (Details + line refs: `PX4_Gazebo/docs/HANDOFF_partial_visibility_offframe_junction.md`.)
- Fix: (planned) accept off-frame J with >=2 angled lines each verified on the visible side, capped extrapolation, `in_fov=False`;
  perception uses `in_fov` with inflated KF noise; env knob, default ON only after the gate passes.
- Confirm with: (1) `tools/score_partial_visibility.py test_data 0.5` before/after. PASS = off-frame rows' centre error within
  ~2x the in-frame error + small extrapolation error (not 170-350 px), off-frame detected rate up, in-frame row unchanged
  (>=95% detected, <=3 px median). (2) `tools/validate_detector_gt.py --set test_data/PerceptionEvalSet --variant stroke`:
  accuracy/POISON no worse. (3) `scripts/run_ic_validation.sh` IC2-5 (only when the user asks, HEADLESS=1) still passes.
- Must not regress: in-frame precision; no two border lines meeting at a frame corner accepted as an X.
- Result: pending.

### FIX-002 HW_POS_FEEDBACK reference marker point does not match the real marker  [open] (opened 2026-09-26, hypothesis)
- Symptom / evidence: 2026-09-25 run `Fri Sep 25 10-32-10` (video `10-31-48`): the pixel projection of the analytic reference s
  sits 60-80 px from the real cross and moves the opposite way to it later in the descent (ref y 208 -> 500 while the real cross
  moves up in the image). Controller steers to the snapshotted point, not the physical marker.
- Root cause: unknown (candidates: marker snapshot taken from an off-centre pose, EKF drift, geometry/sign error in
  `Hardware/scripts/hw_pos_feedback.py` V-frame/lever arm, Control_Data index misalignment). NOT investigated.
- Fix: none yet.
- Confirm with: mocap or a hand-measured marker position vs the snapshot NED point; replay: project the reference into the
  video frames for several runs (`Hardware/scripts/perception_hw_common.py` `ref_pixel`, `depth_yaw`) and check it stays on the
  cross once the offset is fixed. PASS = projected point within ~10 px of the cross over the descent in >= 80% of runs.
- Must not regress: touchdown logic and the other 2026-09-25 fixes (`FLIGHT_ANALYSIS_2026-09-25.md`).
- Result: pending.

### FIX-003 Cross perception (s, alpha, h, w) not running on the Pi  [open] (opened 2026-09-26)
- Symptom / evidence: `Img_Data.npy` for 09-24 (39 runs) and 09-25 (47 runs) is 100% `coast`; no online cross h/w/alpha.
  Offline reprocessing of the videos: stroke detector 60% miss on visible frames (weak > 2.5 m and < 1.2 m), legacy 55% wrong-place.
- Root cause: `cross_marker_perception` not ported (`Hardware/docs/CROSS_MARKER_PORT_PLAN.md` S4); detectors tuned at f=135,
  Pi is fx~513 / 35 deg hfov, thin strokes at 320x240.
- Fix: port perception + log raw detector output in `Img_Data`; tune the detector on real footage after the user's mocap recording.
- Confirm with: mocap-GT recording (heights, offsets, yaw, lighting) then `Hardware/scripts/hw_perception_quality.py`,
  `oracle_scan.py`, `score_cross_perception.py`; corr/nRMSE of s, alpha, h, w vs mocap GT. PASS thresholds to be set with the user
  (Gazebo reference: s corr 0.97-0.99 > 1.5 m; alpha 10-15 deg circ-RMS; h_z corr 0.72-0.88).
- Result: pending.
