---
name: project_20260824_dtheta_href_continuous_compensation
description: "Implemented AZ VISIBILITY FILTER v3 (PLASMC_DTHETA_HREF): continuous exponential h_ref compensation keyed on dtheta (upstream of cbf2_filter) + a crossfaded direct I_a[2] term, replacing the stepped/thresholded _descent_gate shape for this trigger -- built to mitigate the self-defeating attitude-history loop found in project_20260824_dtheta_az_filter_self_defeating_feedback. Mechanism confirmed firing correctly and the theta_safe/theta_desired ratio moved toward/above 1 (less over-suppression) at both gain points tested. NOT YET VALIDATED at n>=5 -- outcome (SP/xy_err) unusable at IC5 due to the known off-center convergence-wall confound; all 4 smoke-test runs (2 baseline, 2 v3) hit that unrelated wall."
metadata: 
  node_type: memory
  type: project
  modified: 2026-08-24T12:15:22.086Z
  originSessionId: 0f9c8dc0-a837-4722-95e7-0ea102167469
---

> ⛔⛔ **OBSOLETE MECHANISM (stamped 2026-09-03).** `_dtheta_correction` was REMOVED from
> `controller.py` in `e110b8a7` (2026-08-31); the descent-rate/lateral-margin trade now lives
> INSIDE the joint QP as `cbf_visibility.py::CBF_AZ_COST_GAIN` (`controller.py:254`).
> `grep -c _dtheta_correction src/controller.py` == 0. Anything in this file that diagnoses
> `_dtheta_correction` or proposes validating `PLASMC_DTHETA_HREF` is testing a mechanism that
> no longer exists — that env survives (`:247`, default 0) but now gates only the SEPARATE
> upstream `h_ref_eff` shaping. Findings about CAUSALITY (control destabilises first,
> perception second) may still hold; the named fixes and next-steps do not. Current mechanism:
> `CBF_JOINT_QP` (baked default-on) + `CBF_AZ_COST_GAIN`.

Same-day follow-up to [[project_20260824_dtheta_az_filter_self_defeating_feedback]], per user design discussion: (1) the direct `I_a[2]` `dtheta` correction is instantaneous but is exactly the mechanism carrying the self-defeating attitude-history loop (applied AFTER `cbf2_filter`, invisible to that cycle's QP solve); (2) gating `h_ref` (the descent reference `_h_ref = -0.30`) instead acts UPSTREAM of `cbf2_filter` -- same-cycle coherent, avoids the one-cycle-blind-spot -- but is SLOW to bite (propagates through the kappa-SMC + the `tau_ia=0.08s` LPF, ~0.08-0.3s / several cycles before it shows up in real `I_a`); (3) user asked to combine both (direct term bridges the lag, `h_ref` gate takes over as it ramps in) with the direct term crossfaded down over time so the two channels don't both run at full strength forever; (4) user asked for the `h_ref` compensation itself to be a SMOOTH CONTINUOUS function of `dtheta`, not the existing `_descent_gate`'s stepped/thresholded (`slo`/`shi` plateau) shape.

## Implementation (`controller.py`)

New knobs, all default-OFF / backward-compatible when `PLASMC_DTHETA_HREF=0`:
- `PLASMC_DTHETA_HREF` (0/1) -- master switch, gates both the continuous `h_ref` compensation AND the direct-term crossfade.
- `PLASMC_DTHETA_HREF_GMIN` (default 0.15) -- descent floor, mirrors `_dgate_gmin`.
- `PLASMC_DTHETA_SCALE` (default 0.18) -- the `dtheta` value at which `g` decays to `1/e` of its range; picked from the mean active-frame `dtheta` measured at gain 5-10 in the earlier sweep (0.16-0.23) -- a first guess, not derived/validated.
- `PLASMC_DTHETA_HREF_TAU` (default 0.5) -- LPF tau on the gate state, same role/value as `_dgate_tau`.
- `PLASMC_DTHETA_XFADE_TAU` (default 0.3) -- crossfade decay time constant for the direct `I_a[2]` term.

Mechanics:
- `g(dtheta) = g_min + (1-g_min)*exp(-dtheta_prev/scale)` -- smooth, no dead zone/plateau, computed from the PREVIOUS cycle's `dtheta` (this cycle's isn't known until `cbf2_filter` runs later in the same iteration -- same lag structure `cbf2_filter` itself uses for `th_curr`). LPF'd (`_dtheta_href_tau`) then multiplies `h_ref_eff` -- composable with the existing `s_e_n`-gated `_descent_gate` (independent, both can run).
- Direct-term crossfade: an active-time accumulator (`_dtheta_active_t`, resets when `dtheta<=1e-3`, else grows by `dt`) drives `weight = exp(-active_t/xfade_tau)`; `I_a[2] -= gain*dtheta*weight`. When `PLASMC_DTHETA_HREF=0` this weight is forced to 1 (exact old behavior).
- New Control_Data logs: `theta_desired(t)`, `dtheta_href_g(t)`, `dtheta_xfade_w(t)` (the last two NaN/1.0 when the mechanism is off).

## Smoke test (IC5 cross-marker, n=1 each -- NOT a validated sweep)

`test_data/DthetaHref_IC5/20260824-173718/{gain10_href1,gain40_href1}` vs the earlier baseline `test_data/DthetaDesired_IC5/20260824-163554/{gain5,gain40}`:

| config | ratio theta_safe/theta_desired | mean dtheta | href_g median | xfade_w median |
|---|---|---|---|---|
| baseline gain5 (no href) | 1.2543 | 0.226 | n/a | n/a |
| **v3 gain10, href=1** | **1.7012** | 0.220 | 0.589 | 0.134 |
| baseline gain40 (no href) | 0.9529 | 0.034 | n/a | n/a |
| **v3 gain40, href=1** | **1.0261** | 0.036 | 0.881 | 1.000 |

**Mechanism fires correctly**: gain10's `dtheta` is active 92.4% of frames (median crossfade weight 0.134 -- correctly faded down, median `h_ref` gate 0.589 -- meaningful continuous descent-rate reduction); gain40's `dtheta` active only 39.4% (gain40 already self-suppresses `dtheta` per the original mechanism, so the new gate engages less -- consistent).

**The ratio moved toward/above 1 at both gain points** (less QP over-suppression relative to the desired ask) -- the intended mechanistic effect is present.

**Outcome (SP/xy_err) is NOT usable evidence**: all 4 runs (2 baseline, 2 v3) show `target_lost: True` / `descent_anomaly_cause: ASCENDING` -- the known **off-center kappa-leakage convergence wall** documented in [[project_20260824_crossmarker_offcenter_convergence_wall]], a pre-existing confound at IC5 unrelated to this change (same reason the original `dtheta` self-defeat investigation deliberately used mechanistic metrics, not SP, at this IC).

## Status / next steps

Mechanism implemented and confirmed working as designed. NOT validated at n>=5, NOT baked, NOT shown to fix the IC5 outcome (can't be shown until the off-center wall is separately addressed or a cleaner IC is used for outcome evaluation). Suggested next steps if pursued: (1) re-run the ratio/mechanistic comparison at IC2 (much lower `dtheta` firing rate, ~20%, cleaner of the off-center-wall confound) to see if the improved ratio translates to an actual SP/rel_vel improvement there; (2) n>=5 before drawing any outcome conclusion, per [[feedback_sensitivity_sweep_methodology]].
