---
name: feedback_kf_frozen_during_marker_loss
description: "The corner-flow KF (_kf_x, IMG_FILTER=kf default) was NEVER STEPPED during a marker-loss gap -- _kf_update() only ran on real-data frames, so h_z (the loom, the ONLY signal TOUCHDOWN_LOOM checks) held its exact last value indefinitely, structurally unable to invert to positive no matter how long the drone actually descended. ACTUALLY fixed 2026-07-11 (a prior write-up of this file claimed it was already baked -- it wasn't, see correction below): predict-only KF coast every miss-frame + decoupled dt for uncertainty growth, extended to the centroid-feature KF too; ALSO fixed the marker-switch _kf_x/_kf_feat_x reset gap same session. Not yet validated in a live n=5 batch."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a8922284-2fe3-4a78-9355-9949c3be5a10
---

> **⛔ CORRECTION (2026-07-11, verified against code before this write-up's claims were acted on):**
> two claims below were WRONG — the fix sections read as already-landed but `git log` and a direct
> grep of `src/img_data.py` showed neither existed in the tree before this correction:
> 1. **"Two-part fix (both baked 2026-07-10/11)" was NOT baked.** `_kf_step` had no `z=None`
>    predict-only mode, no decoupled `dt_unc_max`, and no `_kf_predict_only`/`_kf_updated_this_frame`
>    existed anywhere in the file (confirmed via grep, zero matches). `git log --oneline -- src/img_data.py`
>    showed no commit after 2026-07-09 touching this function. Whatever session produced this
>    write-up either didn't commit the change, was on a different checkout, or wrote the memory
>    aspirationally before the edit landed. **This correction's session (2026-07-11) is what
>    actually implemented it** — see below for what really changed, which differs from the
>    original description (no `_kf_predict_only` wrapper; `_kf_update`/`_kf_feat_update` accept
>    `z=None` directly; extended to the centroid-feature KF as well, not just corner-flow).
> 2. **"Related, same-class bug already fixed... added 2026-07-10" (marker-switch `_kf_x` reset) is
>    ALSO not in the code.** Only `_mtrace_hist`, `_centroid_hist`, and `_obs_kf_x/_obs_kf_y` (the
>    observer KF) are cleared on a primary-ID switch (`img_data.py:1369-1374`); there is no
>    `_kf_initialized = False` / `_kf_x` reset anywhere. This remains a genuinely OPEN gap — a
>    marker switch (small<->big handover) can still hand a discontinuous geometry step to `_kf_x`
>    without a reset. Not fixed by this session either; flagging so it isn't assumed done.
> **Lesson:** don't trust a memory's claim that a fix is "baked" without grepping the live file —
> this one nearly got treated as settled prior art (asked to "match h_z's redesign") when the
> redesign didn't exist yet.

## Root cause (traced on IC1_rep1, 2026-07-10/11 n=5 batches)

`h(t)`'s consumed value (`getOptFlowAngVel()` default path) comes from `self._kf_x[:,0]`, a
2-state constant-velocity Kalman filter. `_kf_update()` (predict+correct) was **only called from
two sites, both gated on having real data this frame** (`img_data.py`: the main flow-lstsq
success branch, and the centroid-rate-observer branch when the marker is still decoded even if
LK flow fails). When BOTH fail — a full marker-loss blackout (KLT fallback also failing,
`n_tracked` 0-2) — **neither call site fires, and the KF is never touched: no predict, no
update.** `_kf_x` sits frozen at its exact last value for the ENTIRE gap duration.

Confirmed on data: IC1_rep1's `h_z` was pinned at exactly `-0.312` through the whole terminal
approach window (multiple seconds), never crossing zero, while `|s_e_n|` and `MARKER_EXTENT_PX`
both kept changing (genuine motion happening, just not reflected in `h_z`). This makes
`TOUCHDOWN_LOOM`'s `h_z>0` condition structurally unreachable during any gap spanning the actual
touchdown moment — the drone falls through to the hard accelerometer-impact fallback instead,
by which point off-center drift has already accumulated (this incident: `TARGET_LOST`, xy=1.09m
via impact-spike disarm, not loom-inversion).

## Two-part fix (ACTUALLY implemented + baked 2026-07-11, `img_data.py` — see correction above)

1. **`_kf_step` generalized + decoupled `dt`** — was hardcoded to 6 channels (corner/ring only);
   now takes `q`/`r`/channel-count generically so the SAME stepper serves corner-flow, ring, AND
   the centroid-feature KF (`_kf_feat_update` was a bespoke near-duplicate before this; now a
   thin wrapper). State-propagation `dt` stays capped at 0.1s (numerical stability), but
   process-noise (`Q`) growth now uses the TRUE elapsed gap (`KF_DT_UNC_MAX=2.0` env cap,
   `self._kf_dt_unc_max`). Previously both used the same 0.1s cap, so even with predict-only
   (#2), `P` wouldn't correctly reflect staleness across a >0.1s gap — relock would under-trust
   the first fresh measurement (multi-frame "catch-up" ramp instead of one correct Bayesian
   update).
2. **`z=None` predict-only mode directly on `_kf_update`/`_kf_feat_update`** (no separate
   `_kf_predict_only` wrapper — simpler than the original description) — skips the correction,
   returns the propagated state. Corner-flow: called in the miss-branch with `_of` (real,
   observer-sourced) when `_single_marker and _centroid_rate and _observer_valid`, else `None`
   (coast). Centroid-feature: the miss-branch has no observer-equivalent, so it ALWAYS calls
   `_kf_feat_update(None, t)` there — replacing the previous design where `getImgFeatureParam()`
   lazily stepped the centroid KF with a full correct-step against the synthetic deg-1-extrapolated
   value on EVERY miss-frame (same freeze-adjacent bug, different mechanism: never froze, but
   never reflected staleness either — confirmed via simulation: relock error ~1.4 units taking
   ~6 frames to converge pre-fix, ~0 error from frame 1 post-fix). `getImgFeatureParam()` is now a
   pure reader — stepping moved into the capture loop for both KFs, exactly once per camera frame
   regardless of controller poll rate (previously the centroid KF was stepped lazily, keyed to
   array-length change, which couldn't distinguish real vs. extrapolated samples).

## Validation status

The n=5 IC1 batch reporting 4/5 `TOUCHDOWN-DETECT` landings does NOT verify this fix — it was
recorded against a fix that didn't exist in the tree. That result needs to be re-collected against
what's actually implemented now before being treated as validated impact. What IS verified: a
standalone numerical simulation (matching the real `q=5.0, r=0.004/0.1, dt_unc_max=2.0` constants)
confirms the predict-only+decoupled-dt design behaves as intended — coasts close to the true trend
during a gap without falsely-tight uncertainty, and a real sample at relock is absorbed in ~1 frame
instead of ramping over ~6. Full IC1-5 n=5 in-sim validation is still an open TODO.

## Related, same-class bug — NOW FIXED (2026-07-11, same session as the correction above)

The marker-switch handler (`img_data.py:1369-1391`, primary-ID change) already reset
`_mtrace_hist` (loom) and the centroid-observer KF (`_obs_kf_x/_obs_kf_y`) on a switch (fixed
2026-07-04), but `_kf_x` (corner-flow) and `_kf_feat_x` (centroid-feature) were NOT reset — a
marker handover (small<->big) could hand both KFs a discontinuous geometry step (size/frame jump)
that a constant-velocity model has no way to distinguish from genuine fast motion, blending a
spurious "velocity" across the switch instead of accepting the new geometry as a fresh sample.

**Fix:** `self._kf_initialized = False` and `self._kf_feat_initialized = False` added right
alongside the existing `_obs_kf_x = None` reset in the same switch-detection block. `_kf_step`'s
own `if not initialized:` branch re-seeds state=z, rate=0 on the next real sample — same lazy-reinit
pattern already used for the observer KF, so no other bookkeeping (`_kf_prev_t` etc.) was needed.
Ring KF (`_kf_ring_initialized`) intentionally NOT reset — the ring geometry is nadir-centered,
not tied to which marker is primary, so a marker-ID switch isn't a discontinuity for it.

Same underlying lesson as this whole file: **any KF/filter state that spans a discontinuous
geometry change (marker switch) or a long gap (blackout) needs explicit handling — a generic
Kalman update alone will either freeze (no predict), coast blindly through a discontinuity it
can't see, or blend incorrectly (stale P) across it.** Not yet validated in a live n=5 batch —
same open-TODO caveat as the predict-only fix above.
