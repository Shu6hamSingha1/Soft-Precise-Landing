---
name: project_goaround_removed_descent_anomaly_20260727
description: "2026-07-27: go-around (07-26/27) REVERTED per user directive -- TARGET_LOST goes straight to open-loop leveling fallback again, no retry. Added a new DESCENT_ANOMALY failure classification: oscillating or net-ascending during the closed-loop approach is now a failure on its own, independent of touchdown xy/vel. Commit fca2c86."
metadata:
  node_type: memory
  type: project
  originSessionId: 68163648-1a9b-4336-962f-9c4c77471aea
  modified: 2026-07-27T15:07:32.170Z
---

**Go-around removed.** User: "Remove go-around fix. Tag target lost as a failed landing. Any
data after target lost is useless. We record it to identify the cause of failure." Both the
07-26 unconditional go-around and the 07-27 progress-gate fix on top of it
([[project_go_around_progress_gate_20260727]]) are reverted -- `TARGET_LOST` beyond grace goes
straight to `in_final_descent` (the open-loop leveling fallback), same structure as before
07-26. `target_lost` stays sticky (set on first genuine loss). The take-away that survives: any
data recorded after `target_lost` fires is for POST-HOC failure-cause diagnosis only, not a
signal to act on live -- recovery attempts (go-around) don't help because a retry just re-enters
the same geometry that caused the loss (see the 07-27 progress-gate memory for why).

**New: DESCENT_ANOMALY classification.** User: "it is a failed landing if the drone started
oscillating or ascending (instead of descending). We want a smooth descent ending with soft
touchdown." Added a live detector (`landing_test.py`, right after the existing hover/stall
watchdog) computed only during the closed-loop phase (`not in_final_descent`):
- **Ascending:** current ENU altitude exceeds the running best (lowest) altitude reached so
  far by `LANDING_DESCENT_ASCENT_TOL_M` (default 0.5m).
- **Oscillating:** a smoothed vertical-rate sign (over a `LANDING_DESCENT_RATE_SPAN_S`=0.5s
  window, deadbanded at `LANDING_DESCENT_RATE_DEADBAND_MPS`=0.05 m/s) reverses
  `LANDING_DESCENT_OSC_MAX_REV`=4+ times within a `LANDING_DESCENT_OSC_WINDOW_S`=3s sliding
  window.
Both flags are sticky once set, feed a new `descent_anomaly`/`descent_anomaly_cause` field in
`SOFT_PRECISE`, and take priority in the final tag (`DESCENT_ANOMALY [ASCENDING|OSCILLATION]`)
the same way `target_lost` does -- forces `precise=False, soft=False` regardless of the
touchdown endpoint. Pure classification: does not itself alter control or force
`in_final_descent` (unlike `target_lost`), since the user's ask was about labeling the outcome
correctly, not intervening further.

**Why this matters / validated finding.** The OLD classification only looked at the touchdown
endpoint (`xy_err`/`rel_vel` at `FC_node.LANDED`), which is exactly the metric the whole
session's "post-kick ballooning" work ([[feedback_terminal_root_lateral_zeta_r]] and others)
already showed is dishonest for a fly-away that happens to re-enter near the target. Live-
validated on the known `IC1_rep2` marker-switch-triggered fly-away (25.5m/8.4m/s at endpoint):
previously graded `FAIL` (a bare label with no cause attached in the classification itself);
now correctly caught mid-flight as `DESCENT_ANOMALY [ASCENDING]` (alt climbed 0.62m above its
own running best + 0.5m tolerance), which is both more informative and structurally correct
-- this WAS a real ascent event, not merely "didn't end up precise."

**Not yet re-run:** a full IC1-5 gate under the new code (only an n=3 IC1 smoke test done,
which is what surfaced the IC1_rep2 example above). Existing IC5 go-around-era data
([[project_flowlatreduced_targetlost_gate_20260726]], [[project_go_around_progress_gate_20260727]])
is now stale for IC5 characterization purposes since the recovery mechanism it was measuring no
longer exists -- IC5's structural early-marker-loss finding itself (large offset/short runway
pushes the marker out of frame before descent can start) still stands as the mechanism, just
not the go-around-specific "stuck near start altitude" framing.
