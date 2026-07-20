---
name: feedback_loom_sign_guard_blocks_touchdown_detect
description: "FLOW_LOOM_SIGN_GUARD (default-on since 2026-06-22, clamps consumed loom h_z<=0) structurally PREVENTS TOUCHDOWN_LOOM (loom-inversion touchdown detector, baked 2026-06-29, requires h_z>0) from ever firing -- the two features were never validated together. Sign-guard REMOVED (default 1->0) DECIDED 2026-07-10, but NOT actually applied to img_data.py until 2026-07-11 (same phantom-fix pattern as feedback_kf_frozen_during_marker_loss -- verify code before trusting this file's own 'fix' claims)."
metadata:
  node_type: memory
  type: feedback
  originSessionId: a8922284-2fe3-4a78-9355-9949c3be5a10
---

> **⛔ CORRECTION (2026-07-11, separate session):** this memory's own "Fix: `FLOW_LOOM_SIGN_GUARD`
> default flipped 1→0" claim was FALSE until this correction — `img_data.py` still had
> `os.environ.get("FLOW_LOOM_SIGN_GUARD", "1")` (default ON) right up until this session's
> edit. This is the SAME phantom-fix pattern already caught twice for
> [[feedback_kf_frozen_during_marker_loss]] in this project's memory — a memory claims a fix
> landed, but the code was never actually touched. Root-caused via live IC1 n=5 data (this
> session, real run against real code): traced `h_z` at genuine touchdown+bounce in a SOFT+PRECISE
> rep (real ArUco corner data, `ncorn=184`, not ring fallback) — it correctly inverted to +0.81 for
> 6+ sustained frames, well past the detector's 3-frame/`|s_e_n|<0.6` gates (reconstructed
> `s_e_n`≈0.01-0.015, ~40-60x under threshold) — yet `TOUCHDOWN-DETECT` never printed, because the
> guard was silently clamping the value to 0.0 before the controller ever saw it. Default is now
> ACTUALLY `0` as of 2026-07-11. **Lesson restated a third time in this project's memory: grep the
> live file before trusting any "fixed"/"baked"/"removed" claim, including this one's own history.**
>
> **FULLY REMOVED (2026-07-11, user directive, same session):** rather than leave it default-off
> (still present, re-enablable via `FLOW_LOOM_SIGN_GUARD=1`), the user asked for the guard and all
> references to be deleted from the code entirely. Removed from all 4 sites in `img_data.py`: the
> `__init__` attribute/env-var, the `_ekf_fuse_step` fusion-EKF clamp, and both clamps in
> `getOptFlowAngVel` (kf path + savgol path). `FLOW_LOOM_SIGN_GUARD` is no longer a recognized env
> var anywhere in the codebase (verified via grep, zero hits outside memory/comments). **The
> ORIGINAL ring-loom wrong-sign problem this guarded against (traced on IC3_rep1, wrong-sign loom
> 86/93 frames at `aruco=0`/ring-takeover) is now fully re-exposed with no band-aid at all** — see
> "The ring-loom sign-flip's ROOT CAUSE is still unfinished" section below for the two unconfirmed
> hypotheses if that failure mode resurfaces and needs a real (not blunt-clamp) fix.

## The incompatibility (found 2026-07-10, investigating why IC1's TOUCHDOWN_LOOM never fires)

Two independently-baked features directly contradict each other, and nobody validated them together:

1. **`FLOW_LOOM_SIGN_GUARD`** (img_data.py, default-on since 2026-06-22, commit 758dcb2) — clamps
   the consumed loom `h_z <= 0`. Added as a band-aid for a ring-loom wrong-sign issue (marker
   overflow near deck → EKF fusion falls back to noisy ring loom → swings positive → z-SMC
   commands UP thrust → fly-away).
2. **`TOUCHDOWN_LOOM`** (controller.py, loom-inversion touchdown detector, baked default-on
   2026-06-29) — latches LANDED when `h_z > 0` holds for 3 consecutive frames + centered
   (`|s_e_n| < 0.6`). The premise: a real ground-contact rebound inverts the loom sign.

With the sign-guard active, `h_z > 0` is **structurally unreachable** — the detector can never fire,
regardless of what the drone physically does. This was invisible for weeks because IC2-5 mostly
land via other paths (accelerometer impact spike, PX4 `_getLandedState`), and IC1 test coverage
in the 2026-07-09 sweep attributed the resulting failures to a different mechanism
([[project_baked_sweep_and_posttouchdown_divergence]]'s "post-touchdown attitude divergence" —
now understood to be a *consequence* of the touchdown detector never firing, not an independent
root cause: the drone stays armed and fighting because nothing ever tells it to stop).

## Fix

`FLOW_LOOM_SIGN_GUARD` default flipped `1`→`0` (img_data.py). Env var still works for debugging.

## Why removing it didn't immediately fix IC1

Removing the guard alone was NOT sufficient — `h_z` still froze during marker-loss gaps
(see [[feedback_kf_frozen_during_marker_loss]]) and a corrupted flow measurement at marker
handover could still explode the control law before touchdown
(see [[project_ic1_terminal_kick_root_cause_chain]]). All three fixes were needed together;
n=5 IC1 validation only cleared after all three landed. See
[[project_touchdown_detect_velocity_gate_gap]] for the remaining open gap (detector has no
velocity check, so successful detections are precise but rarely soft).

## The ring-loom sign-flip's ROOT CAUSE is still unfinished

The sign-guard's original justification (ring loom going wrong-signed near the deck) was
**not** actually resolved by this fix — it was just unblocked from interfering with the touchdown
detector. Two competing partial diagnoses exist from this session, neither fully confirmed:
1. Ring-station LK survival is texture-limited (steady-state ~15.7% even at altitude, unrelated
   to tilt) — background/ground texture-poverty, not geometry.
2. A primary-marker-ID handover (small→big nested marker) can inject a real, large, previously
   correctly-canceled attitude disturbance into a differently-conditioned corner set —
   see [[project_ic1_terminal_kick_root_cause_chain]].

Flagged explicitly by the user as "unfinished" — don't assume either diagnosis is complete;
re-investigate with fresh eyes before trusting either.
