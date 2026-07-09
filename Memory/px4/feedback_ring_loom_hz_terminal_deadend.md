---
name: feedback_ring_loom_hz_terminal_deadend
description: "DEAD-END (2026-07-06) — driving h_z from the ring loom at the terminal (ring-commit's h_z->ring OR PLASMC_LOOM_RING_ON_LOSS) CAUSES fly-aways. Ring loom too noisy/over-reporting to be an h_z source. Keep both OFF."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 3c2f4c67-05c1-4e6f-966b-0e62018fc8a7
---

> **✅ 2026-07-09 addendum — the FUSION path is now ALSO closed.** This file covered ring-COMMIT /
> loom-RING-on-loss switching, but the always-on `FLOW_FUSE_RING=1` EKF fusion (default ON since
> 06-07) kept injecting ring loom into the controller's h(t) anyway — the contradiction went
> unnoticed until a terminal-kick trace. Default flipped to 0 (BAKED, commit bb0a675) + the WHY
> behind the ring's terminal wrongness root-caused (fixed radii overlap the grown marker):
> [[feedback_ring_fusion_marker_overlap]].

**DEAD-END, do not retry: driving `h_z` from the ring loom at the terminal is HARMFUL.** Two
env-gated implementations, both default-OFF, both confirmed to CAUSE fly-aways when they engage:
- `PLASMC_TERMINAL_RING_COMMIT=1` (h_xy->0 + h_z->ring loom, gated handover+over_target+settled)
- `PLASMC_LOOM_RING_ON_LOSS=1` (the simpler user rule: h_z->ring on decode-loss + OVER_TARGET + not-drifted-off)

**Evidence (IC2 perception x5, LOOM_RING_ON_LOSS + OVER_TARGET gate + DESCENT_GATE):** PERFECT
correlation — when the loom-ring FIRED (reps 3,4,5: 36-41 firings, all at the terminal over-target),
the drone reached the deck PRECISE (GT 0.038-0.119m) then **TARGET_LOST fly-away (peak 2.82 / 44.3 /
4.63 m)**; when it stayed INERT (reps 1,2: 0 firings), it **LANDED bounded** (SOFT / hard, peak 2.7-3.4).
Fires->fly 3/3, no-fire->land 2/2. Debug (`LOOM_RING_DBG=1`) showed the ring `h_z` swinging **-0.83 to
+0.92** (noisy, SOMETIMES POSITIVE = "ascend") -> launches the z-loop.

**Why:** the ring loom (pure_div) at the terminal OVER-reports 1.34-2.08x AND is noisy/positive-swinging
(few texture-poor MAD-survivor stations at the deck; see [[feedback_vds_kf_q_severity_bandaid]] scale
data). Too poor to be an h_z feedback source. AND it's the wrong target anyway — the terminal fly-away
is **LATERAL** (observer h_xy goes open-loop at the marker overflow, drifts to ~1 m/s), NOT vertical,
so fixing h_z can't help.

**What DID work (keep):** the `OVER_TARGET` gate (nadir inside the small marker's V-frame quad) correctly
rejected the altitude decode-flicker firings (0 fires at altitude vs 64 ungated) -> no more premature
hover. And the merged handover overflow-signature (small-M) rejects the altitude big->small flickers.
Those gates are sound; the ring-loom ACTION they gate is the dead-end.

**Terminal decode-loss ROOT correction (video 2026-07-06):** the deck `Ncorn->0` is NOT (primarily)
marker OVERFLOW — the DRONE'S OWN ARM/PROP occludes the marker (~0.2m, marker ~200px, still in-frame).
User decision: DON'T reposition the camera. So the terminal perception loss is a fixed given.

**Live baseline that WINS:** observer (4 fixes [[feedback_centroid_rate_observer_fixes]]) + DESCENT_GATE,
NO ring-commit/loom-ring -> 5/5 reach the deck, GT lateral 0.055-0.095m, ZERO fly-aways (rc2 batch).
Occasionally HARD vertically (~0.4 m/s) but bounded+precise. The residual soft-touchdown gap is NOT
solvable by the ring loom; the hardness is partly lateral (h_xy open-loop at the deck). See
[[project_terminal_velocity_handover_design]] (superseded action; gates survive).

**UPDATE 2026-07-07 — 3 real ring-signal bugs found+fixed; VERIFIED the fly-away persists ANYWAY via an
INDEPENDENT mechanism (confirms the dead-end verdict from a different, stronger angle).**
1. **Arm/prop occlusion** (video inspection): the down-camera sees the drone's OWN static arms/props as
   fixed raw-image y-bands (top ~<155px, bottom ~>445px, 480x640). Those ring stations have ~0 flow ->
   corrupt pure_div/moment. Fix: `PLASMC_RING_ARM_MASK=1` drops them. Cut terminal-band noise ~3x
   (pure_div std 2.27->0.67; moment std 1.03->0.60) and sign-flip rate (51%->21% / 45%->11% positive).
2. **Unpaired-station translation leak** (user insight): pure_div's radial-dot cancels lateral
   translation ONLY across geometrically-opposite (180 deg) station pairs. Dropping a station (LK-fail
   OR arm-mask) without also dropping its partner leaks translation into BOTH pure_div AND ring_moment
   (moment is NOT translation-immune despite being mean-centered — verified empirically: |moment| grows
   0.18->0.61 monotonically with GT lateral speed 0-5 m/s, unpaired). Fix: `PLASMC_RING_PAIRED=1`
   (`_ring_opp_idx`, involution i<->i+npts/2 per radius) drops a station's partner too so both looms
   share one paired V0/V1 set. A/B(paired vs unpaired, arm-masked+moment, IC2 x5): land-rate 3/5->4/5,
   but the ONE remaining failure got MUCH worse (peak 15.57m unpaired -> 114.70m paired) — paired sets
   are smaller, so a marginal frame can drop below a robust N and produce a rare extreme outlier instead
   of many small ones. Net: real improvement, not a full fix.
3. **FAKE-ZERO SENTINEL BUG (the big one).** `_compute_ring_flow`'s degenerate-return path
   (`valid.sum()<6`, e.g. right as the paired-set collapses) returned `pure_div=0.0` (a LITERAL zero,
   not NaN). The controller's `elif isfinite(pdiv): RING_LOOM=pdiv*scale` then treated that fabricated
   0.0 as REAL data (isfinite(0.0)=True) -> injected a fake "stationary" h_z reading for several frames
   while the drone was genuinely descending (marker_hz=-0.38, matching GT) -> a discontinuous WRONG
   step-input into the sign-sensitive high-gain z-loop right at the ring-loom firing onset. Caught live
   via `LOOM_RING_DBG=1` printing `Nstations=0-2` alongside `ring_hz=+0.00` at a firing's first ~150ms.
   FIXED: sentinel + degenerate-return both -> NaN (`np.nan`, not `0.0`) so the controller's isfinite
   check correctly rejects it (holds last-known RING_LOOM instead of a fake zero).

**VERIFICATION (LOOM_RING_DBG=1, arm-mask+paired+moment, all 3 fixes, IC2 x5): the ring loom is now
CLEAN throughout — GT-cross-checked `ring_hz` tracks `marker_hz` within ~0.05 at up to 56 paired
stations, zero fake-zero firings, zero wild outliers. YET a fly-away still occurred (rep2, peak
100.77m).** Full GT trajectory trace: descent reaches the deck PRECISE (lat 0.02m, alt 0.05-0.16m,
t44-51) -> loom-ring goes SILENT (no firings) at t49.86 -> **the launch begins at t50.94, >1s AFTER the
last ring-loom firing, with the mechanism completely inactive.** So this fly-away is driven by an
INDEPENDENT process, not the ring loom (which was clean and off by then) — consistent with the
standing conclusion that the terminal failure is a LATERAL/bounce event
([[feedback_terminal_overflow_deck_flyaway]]), not a vertical/loom one. **Net verdict: the 3 ring bugs
were real and are now fixed (keep the fixes — cleaner ring signal is good engineering), but they were
NEVER the dominant cause of the terminal fly-away. The dead-end verdict (keep ring-commit/loom-ring
OFF; don't chase h_z fixes for this failure) is now confirmed via a cleaner signal, not just noisy data.**

**Separately (user flag, verified):** `gt_feedback.py`'s `_v_frame`/convention is algebraically identical
to `img_data._getVirtualPts` (already offline-validated, corr 0.83-0.95) — NOT a bug. But MY ad-hoc
analysis scripts this session had a real time-sync bug: `GTFeedback` is STATEFUL (LS velocity window);
gap-feeding it (calling `.update()` only for samples inside a band, skipping the rest) leaves the window
empty at band-entry -> 2-4 spurious zero-h_z readings -> inflated std up to ~2x in short windows. Fix:
always feed EVERY sample in chronological order, only FILTER which results you collect. Any future
ad-hoc GT-vs-perception script MUST feed `GTFeedback`/similar stateful estimators continuously.
