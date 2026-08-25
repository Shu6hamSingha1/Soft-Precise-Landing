---
name: project_20260824_dtheta_cap_flyaway_fix
description: "Implemented PLASMC_DTHETA_AZ_CAP (default 2.0 m/s^2, clamps the final per-cycle I_a[2] correction) to fix the IC5 fly-away root-caused in project_20260824_dtheta_ic5_flyaway_rootcause. Cap ALONE was insufficient (still ASCENDING, peak alt ~5.9m vs uncapped 7-10m -- reduced severity, not fixed) -- a bounded-but-nonzero correction sustained ~89-92% of frames for 5+s still integrates into real climb. Cap COMBINED with the v3 crossfade (PLASMC_DTHETA_HREF=1, which decays the direct term over sustained-active time) eliminated the vertical fly-away entirely (descent_anomaly=False, monotonic 3.0m->0.49m descent). Landing STILL fails at IC5 (target_lost=True, xy_err~9.4, lateral drift to -6.3/+6.7m) -- a different, smaller, NOT YET DIAGNOSED failure mode (no more altitude runaway, but lateral tracking still can't hold this IC)."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-24T16:13:56.005Z
  originSessionId: 0f9c8dc0-a837-4722-95e7-0ea102167469
---

Same-day follow-up to [[project_20260824_dtheta_ic5_flyaway_rootcause]]. User agreed the fly-away needed a cap but flagged (correctly, confirmed by this data) that capping wouldn't resolve "the main issue" -- distinguished the two independent defects: (1) the uncapped-correction fly-away (stability/safety), (2) the `th_curr` self-defeating attitude-history loop (functional, [[project_20260824_dtheta_az_filter_self_defeating_feedback]]). This entry covers implementing and testing fix (1).

## Implementation (`controller.py`, at the `cbf2_filter` call site, ~line 3168-3204)

`PLASMC_DTHETA_AZ_CAP` (default `2.0`, m/s²) clamps the FINAL correction (post-gain, post-`dtheta_norm`, post-crossfade-weight) to `[0, cap]` before subtracting from `I_a[2]`. Default picked from the observed median-to-p90 of the uncapped gain=10 distribution (median 1.49, p90 2.33) in the original `DthetaGain_IC5` sweep -- clips the runaway tail without flattening typical operation. New log `dtheta_correction(t)` (final applied value) added to Control_Data.

## Test 1: cap alone (IC5, gain=10, cap=2.0, `PLASMC_DTHETA_HREF` unset)

`test_data/DthetaCap_IC5/20260824-193500/gain10_cap2`. **Reduced severity but did NOT fix it**: peak altitude ~5.9m (vs uncapped 7-10m), lateral drift to ~-10.4/+8.8m (vs uncapped -22 to -28m), still `target_lost=True`, `descent_anomaly_cause=ASCENDING`. Correction log confirms the cap engaged (max exactly 2.000, only 1.9% of frames actually hit the ceiling) but `dtheta` was still active 89.3% of frames -- a bounded-but-nonzero bias sustained that consistently for 5+ seconds still integrates into meaningful climb. **A per-cycle magnitude cap alone cannot fix a chronically-active, non-decaying bias** -- it needed a companion mechanism that also bounds the DURATION/cumulative persistence, not just the instantaneous size.

## Test 2: cap + v3 crossfade together (IC5, gain=10, cap=2.0, `PLASMC_DTHETA_HREF=1`)

`test_data/DthetaCap_IC5/20260824-193500/gain10_cap2_href1`. **Fixed the vertical fly-away**: `descent_anomaly=False`, `descent_anomaly_cause=N/A` -- altitude descends monotonically 3.0m -> 0.49m, no climb at any point. Correction median dropped 1.21 -> 0.11 (crossfade's decay-over-sustained-active-time property, already implemented in [[project_20260824_dtheta_href_continuous_compensation]] for a different original purpose, turns out to be exactly the missing "doesn't accumulate indefinitely" piece the cap alone lacked). Mechanistically: cap bounds the spike, crossfade bounds how long a near-cap correction can be sustained -- together they prevent the runaway that either alone could not.

**But the landing still fails**: `target_lost=True`, `xy_err≈9.4`, lateral drift to `x=-6.3, y=+6.7` by touchdown. This is a DIFFERENT, smaller failure mode than the fly-away -- no altitude runaway, but lateral tracking still can't hold this IC. NOT YET DIAGNOSED whether this is the previously-documented off-center kappa-leakage wall ([[project_20260824_crossmarker_offcenter_convergence_wall]]) reasserting now that the vertical explosion is out of the way, or a distinct mechanism specific to this extreme IC. Next step if pursued: trace this remaining lateral divergence the same way the fly-away was traced (trajectory + Control_Data inspection), don't assume it's the known wall without checking.

## Status

⛔ **CORRECTION (user pushback, same day, correct): "fixed" above overstates it.** Cap+crossfade suppresses the VISIBLE SYMPTOM (climbing altitude) by clamping and fading the correction's output -- it does not address why `dtheta` runs chronically high in the first place (the `th_curr` self-defeating loop / whatever is keeping the CBF constrained near-continuously at this IC). That is a band-aid on the output, not a fix to the cause -- consistent with this project's own standing rule [[feedback_clamps_during_tuning]] ("clamps are band-aids"). The suppressed vertical pressure very plausibly just re-surfaced as the residual lateral failure traced in [[project_20260824_crossmarker_offcenter_convergence_wall]] (see that memory's update) rather than actually being resolved.

`PLASMC_DTHETA_AZ_CAP` implemented, default-on at 2.0 (active whenever `dtheta` correction is used, independent of `PLASMC_DTHETA_HREF`). Suppresses the ASCENDING symptom when paired with `PLASMC_DTHETA_HREF=1` -- NOT when used alone. Does NOT fix the FUNCTIONAL issue (`th_curr` self-defeating loop, still open) and should NOT be read as having solved the IC5 fly-away at the root -- it is masking, not removing, the underlying chronic-`dtheta` pressure. n=1 per config -- directionally solid (mechanism trace: descent_anomaly flag flipped cleanly, correction magnitudes measured directly) but not a validated sweep.
