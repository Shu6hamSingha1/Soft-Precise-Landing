---
name: feedback_dt_staleness_after_detection_dropout
description: "cross_marker_perception.py's dt for the flow Jacobian came from the outer polling clock (advances every call) while the LK 'previous frame' state only advances on successful detections -- after any dropout, the next good frame divided a multi-frame displacement by a one-frame dt"
metadata:
  type: feedback
---

A per-frame optical-flow velocity computation (`vel = (curr_pt - prev_pt) / dt`)
needs `dt` to be the elapsed time between the ACTUAL two frames being
differenced, not between the two most recent *calls* to the function --those are
only the same thing when every call succeeds.

`cross_marker_perception.py`'s `process_frame` computed `dt` from
`t - self._last_t`, and updated `self._last_t = t` unconditionally on every
call, whether detection (`det.ok`) succeeded or not. But the LK tracker's
"previous frame" state (`self._prev_gray`, `self._prev_flow_pts`) only advances
on a *successful* detection -- it stays frozen across any dropout, since there's
no fresh mask to track from. So after an N-frame detection gap, the next
successful frame's flow solve used a `dt` reflecting only the most recent
polling interval (~5-17ms), while LK was tracking points across the FULL N-frame
gap (accumulated real motion). Dividing a multi-frame displacement by a
one-frame `dt` inflates the computed velocity by roughly the gap length -- a
large spike hitting all six solved Jacobian parameters (h and w alike), on every
single recovery-from-dropout frame.

**Why this stayed hidden:** the bug requires BOTH a detection gap AND a
tracking-continuity assumption to interact -- neither condition alone is
suspicious, and every individual quantity (`dt`, `self._prev_gray`) looks
correctly-maintained in isolation. It also compounds with, and was masking the
benefit of, a separate real bug (see
[[feedback_missing_vframe_leveling_port]]) -- fixing the V-frame leveling first
without this fix showed no improvement and even a Hz regression, because this
noise source dominated. Only fixing both together revealed the true underlying
signal quality (raw Hx/Hy-vs-GT correlation: ~0.01-0.10 -> consistent
0.74-0.87).

**Fix:** track the timestamp actually paired with `self._prev_gray`
(`self._prev_frame_t`), separate from the outer "last call" timestamp, and
compute the Jacobian's dt from that.

**How to apply:** whenever a stateful per-frame pipeline has an "only advance
state on success" branch (common for any detector-gated tracker), audit every
OTHER piece of state that's supposed to stay paired with it (timestamps,
attitude, anything else read alongside the tracked points) for the same
gate -- a partial-update bug where SOME state advances unconditionally while
PAIRED state doesn't is a systematic source of exactly this class of silent
corruption. Suspect it whenever a fix that should clearly help (like V-frame
leveling here) shows no effect or an inconsistent one across repeated tests.
