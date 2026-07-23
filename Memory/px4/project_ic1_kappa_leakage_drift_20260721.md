---
name: project_ic1_kappa_leakage_drift_20260721
description: "Traced (not yet fixed) root cause of a residual 55.78m IC1 fly-away surviving a full night of perception-pipeline fixes (map confidence/rate gates, decode-KLT confidence scaling): kappa's leakage term drains the middle-loop adaptive gain even while the outer-loop position error grows, because kappa only reacts to the funnel-NORMALIZED flow error sigma, not raw physical error. Same family as feedback_dont_conclude_lag_floor / feedback_kappa_clamp_bandaid, now with a concrete 2026-07-21 instance."
metadata: 
  node_type: memory
  type: project
  originSessionId: d1a12071-0b1a-4822-8666-315502069261
  modified: 2026-07-22T04:54:22.885Z
---

**Context.** 2026-07-21/22 session baked a long chain of perception-pipeline fixes in
`img_data.py` (alpha KF sin/cos-pair sub-filter for the wrap bug, the
CENTROID_FROM_MAP/ALPHA_FROM_MAP override-vs-KF ordering bug, source-aware `cal_s`,
"map is authoritative once accepted" (decode is cross-check only, not a blended
fallback), confidence-scaled KF measurement noise `r` for map-sourced samples, the
ds-outlier-hold/KF ordering bug, a windowed-rate plausibility gate for map-sourced
centroid/alpha, and finally the SAME confidence-scaling reused for decode's own
KLT-fallback path). All committed/pushed; see git log fda359f..ce881f4 for the full
diff trail — those are pure code changes, fully covered by their own commit messages,
not restated here.

**What's left after all of that: this memory.** A full IC1-5 n=5 validation sweep after
the LAST fix (decode-KLT confidence scaling) came back clean everywhere except one rep:
`IC1_rep5` (test_data/ICValidation/20260721-141516) flew away to **55.78m, rel_vel
8.34 m/s**. Traced end-to-end:

1. **Not a perception-measurement-trust failure.** `Centroid Map Trust` correctly
   tapered and stopped firing before the worst of it; `a_u` never spiked violently
   (peak ~72, vs hundreds/thousands in every OTHER fly-away this session) — every fix
   above was doing its job.
2. **A slow positive-feedback spiral, not a single event.** t=49.6-50.7s: `s` near
   zero, `MARKER_EXTENT_PX` normal (~52-57px) — healthy. t=50.7-54.0s: `s` drifts
   steadily away (0.03→-0.66) while `ext` grows NORMALLY (52→127px, ordinary descent) —
   drift and marker-size growth are independent at this point, tracking is fine. t=54.4s:
   `ext` drops 127→25px (a normal big→small marker handover). But `s` never stops
   drifting through the handover (→-1.04→-2.9→-5.9→-7.2), and as it grows, the SMALL
   marker itself appears smaller/more foreshortened (`ext` 25→13→7.7px, frozen) —
   confirmed by the shadow log: `decode_calls` frozen despite `fresh_decode=True`
   intermittently, meaning ArUco WAS detecting the marker but its quad was flagged
   ill-conditioned (near-edge/degenerate) and rejected. Each turn of the spiral (drift →
   smaller/more-degenerate marker → worse tracking → less correction → more drift) feeds
   the next. It terminates in a genuine, ~2+ second, fully-uncorrected KF coast once the
   marker is gone entirely — that coast is the SYMPTOM, not the root cause.
3. **The actual trigger, traced back to t=50-51s: `kappa` (middle-loop adaptive gain,
   κ-ODE) decays steadily (0.5→0.335 over 3s) while the real position error is
   simultaneously, visibly growing** — with `a_u_norm` staying tiny (0.4-0.9) the whole
   time, i.e. no kick, no glitch, just steady under-response. Root cause in the κ-ODE
   itself (`controller.py::_kappaSolver`):
   ```
   dkappa/dt = theta * N @ G @ |sigma| - N @ P @ kappa
   ```
   `sigma` is built from `zeta = log((1+ratio)/(1-ratio))`, `ratio = h_e/p` — the
   FUNNEL-NORMALIZED middle-loop (flow) tracking error, not the raw physical error and
   not the outer-loop position error directly. The leakage term `-N@P@kappa` is
   unconditional. So if the funnel `p(t)` (which itself only shrinks slowly,
   exponentially, via `_updatePerfFunc`) is still comfortably wide at this point in the
   flight, a real, growing error can stay well inside it (`ratio` small → `sigma` small)
   even as the physical drift compounds — kappa drains via leakage instead of ratcheting
   up to fight it.

**This is NOT a novel mechanism** — it's a live, concrete instance of the exact tension
already flagged in [[feedback_dont_conclude_lag_floor]] ("one knob one job: P=κ-bound,
E=stiffness... a masked-then-exposed failure is a TUNING TARGET, not proof of an
unfixable floor") and [[feedback_kappa_clamp_bandaid]] ("the real levers... are the
ones that keep kappa small and sigma inside E NATURALLY... κ-ODE adapt/leakage (N/P)").
Those memories name the tension abstractly from earlier GT-FB-era trials; this is a
fresh, dated instance under the current perception stack, with a specific failed rep to
reproduce against.

**Status: traced, NOT fixed.** Did not check the current live `P` (leakage) or funnel
decay-rate (`_gamma`) values against what's tuned/expected — that's the natural next
step. Success metric per [[feedback_kappa_clamp_bandaid]]: kappa should stay near its
natural equilibrium and ratchet UP under a real, growing error, not drain under one —
verify against `IC1_rep5`'s specific t=50-54s window (test_data/ICValidation/20260721-141516)
once a fix is tried.

See also [[feedback_fix_causes_not_limits]] (sibling rule, funnel width=gain, never
widen to make room) and [[feedback_per_axis_tuning]] (this drift showed on BOTH x/y
axes near-identically, worth checking whether that's expected or itself informative).
