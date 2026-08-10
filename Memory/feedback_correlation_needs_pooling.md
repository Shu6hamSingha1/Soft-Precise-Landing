---
name: feedback_correlation_needs_pooling
description: Pearson correlation on a near-constant true signal is noise-dominated; pool across independent flights before trusting its sign
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3600b91d-f44b-4754-86bc-066d9ec45b18
  modified: 2026-08-09T22:37:08.882Z
---

Don't trust a Pearson correlation computed within a narrow window (e.g. a
1m altitude band) on a maneuver where the TRUE signal barely varies there
(e.g. a linear-ramp constant-velocity descent) — the correlation is
dominated by measurement noise, not signal, and its SIGN can flip from one
flight to the next by chance (empirically: -0.70 to +0.71 across 12
supposedly-identical flights on the cross-marker's Hz channel).

**Why:** discovered 2026-08-09/10 investigating a claimed "Tz sign-flip
below 2m altitude" in the cross-marker pipeline. One batch of 5 flights
launched back-to-back happened to land 5/5 on the negative side of this
noise distribution (~3% chance under independent flips) and was reported
as a confirmed root cause. A follow-up batch showed the opposite sign;
pooling all 12 available flights (per-flight demeaned) gave the TRUE
correlation as positive — no real inversion ever existed. See
`PX4_Gazebo/docs/HANDOVER_cross_marker_hz_signflip_20260809.md` for the
full trace.

**How to apply:** before treating a narrow-band correlation-sign finding as
a real mechanism, (a) check whether the true signal has real variance in
that window (compare its std against the raw measurement's own noise
floor), and (b) pool ≥3 independent flights/sessions (not just repeat the
same tight batch) before trusting the sign. A single batch launched in
immediate succession is not 5 independent trials for this purpose if
whatever produces the noise is itself session-correlated — but even then,
don't conclude a mechanism without a cross-session check.
