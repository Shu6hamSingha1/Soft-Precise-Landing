---
name: project_20260824_ic5_angle_clustering_and_hang_investigation
description: "⭐⭐⭐⭐⭐⭐ 2026-08-29 ROOT-CAUSED (CORRECTED 2026-08-29, same day): the IC3/IC5 'lateral-drift' failure from the 2026-08-28 gate is NOT a new mechanism -- it's the kappa-ratchet (failure mode 11, previously confirmed RESOLVED by project_20260828_kappa_ratchet_campaign, but that campaign never tested with PLASMC_DTHETA_HREF=1 on). Confirmed via Control_Data.npy: kappa_y 0.17->4.3 (IC5_rep2) / 0.08->10.8 (IC3_rep3), sigma_y diverging past -9, a_u_y to -152/+378 -- the classic ratchet signature, not a new drift. TRIGGER MECHANISM (CORRECTED): h_ref_eff has ALWAYS multiplied the full s[:3] vector in h_d_ff -- this is the original, correct MATLAB-ported PLASMC design (visualControl_IBVS_adaptive.m:665-666, `V_h_d = ... + (h_rd - dot(...))*V_s(1:3)`, present in the Python port since 2026-06-01, ~3 months before PLASMC_DTHETA_HREF existed) -- NOT an oversight or bug, and NOT something DTHETA_HREF introduced. What IS new (2026-08-24) is DTHETA_HREF modulating this pre-existing, correctly-coupled term with a NEW time-varying gain (dtheta_href_g, 0.15-1.0) driven by dtheta_az -- a signal that is itself UNCAPPED (only the gain-scaled dtheta_correction is capped at 2.0) and can spike to outlier magnitudes (measured 8.175 rad) right at the geometrically hardest moment (near the FoV edge, where s_x/s_y are already large -- IC3/IC5-specific geometry). That spike collapses dtheta_href_g transiently, injecting a lateral h_d perturbation through the PRE-EXISTING (correct) s[:3] coupling at exactly the moment sigma is most vulnerable -- coincides with sigma_y's acceleration into the diverging regime that then ratchets kappa. NOT YET FIXED. Candidate fix: cap/smooth dtheta_az itself (or gate dtheta_href_g's rate of change) before it drives the exponential -- do NOT restrict h_ref_eff to s[2] only, that would break the intentional MATLAB-matched coupling for every OTHER caller of h_ref_eff, not just DTHETA_HREF. DTHETA_HREF remains validated + safe for IC1/IC2/IC4; IC3/IC5 need a fix before it can be considered unconditionally safe to bake."
metadata:
  node_type: memory
  type: project
  originSessionId: 4d44a921-8d4d-4924-a38e-243fbd1cb835
  modified: 2026-08-29T04:00:00.000Z
---

## ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (later still): `CBF_JOINT_QP` implemented INSIDE cbf2_filter --
## solves directly for I_a (not theta), a second blowup bug caught+fixed offline, then
## validated clean in SITL

Per user follow-up question ("can we replace theta with I_a[:2]/a_z since now we are
optimizing I_a not theta") -- extended the `PLASMC_AZ_JOINT` idea (a post-hoc downstream
patch) into a genuinely joint solve INSIDE `cbf2_filter` itself (`cbf_visibility.py`,
`CBF_JOINT_QP` env flag, default off, requires `A_CAP`/`g` now threaded in from
`controller.py`).

**Derivation**: `theta = P@I_a[:2]/a_z` where `P = Rz_p90b@Rzm` (the same forward rotation
already used to build `th`). So `Lw2@theta = (Lw2@P/a_z)@I_a[:2]` -- the box constraint's
Jacobian w.r.t. `I_a[:2]` directly is `M = Lw2@P/a_z`. The alternating-projection algorithm
runs UNCHANGED, just on `I_a[:2]` with `M` instead of `theta` with `Lw2`. Interleaved (6
outer x 5 inner iterations) with a projection onto the deliverability sphere
`|I_a+g*e3|<=A_CAP` (full 3-vector, `a_z` re-estimated each outer pass from the CURRENT
iterate) -- a genuine coupled fixed-point solve, not a sequential lateral-then-vertical patch.

**Offline validation (before any SITL time), all via a standalone script, not
`tools/validate_cbf.py` (not yet extended to cover this path)**:
- 200 random trials: sphere constraint held in EVERY case, 0 violations.
- 200 non-saturating trials: `CBF_JOINT_QP=1` output matches `CBF_JOINT_QP=0` (theta-based)
  output to MACHINE PRECISION (max diff 9.16e-16) -- confirms the derivation is exactly
  equivalent when not saturating, not just approximately similar.
- **Extreme-input stress test (500 trials) caught a SECOND instance of the exact same bug
  class found in `PLASMC_AZ_JOINT`'s first draft**: `theta_cone` hit 3.52 rad (>200deg,
  nonsensical) -- the sphere projection bounds thrust MAGNITUDE, not the derived angle
  RATIO; if `a_z` ends up small while lateral stays large after the sphere projection,
  `th_safe = P@(Ia_lat/az_final)` can still blow up. **Fixed the same way as before**: added
  the SAME `a_z`-aware sanity clip (`arccos(a_z_final/A_CAP)`, always finite 0..pi/2),
  keeping `I_a[:2]` consistent with the (possibly-clipped) `th_safe` via the inverse
  rotation. Re-tested: 1000 extreme trials, max `theta_cone`=1.563 (bounded, vs pi/2=1.571),
  sphere constraint still 0 violations, non-saturating equivalence still exact
  (9.16e-16) -- the clip is a true no-op away from the pathological regime.
- `tools/validate_cbf.py` 12/12 throughout (tests the default/off path only, unaffected).

**SITL smoke test (n=3, IC5, `CBF_HZ_AWARE_DRIFT=1 CBF_JOINT_QP=1`): 2/3 SP, worst miss
1.22m -- the mildest result of ANY variant tested this session.** `theta_cone` confirmed
bounded [0, 0.766] throughout all 3 reps (no blowup). The downstream sphere cap never
engaged (0 frames in all 3 reps -- `a_z` still doesn't approach `A_CAP` at this IC, expected
per the earlier physics correction). **No `dtheta_az` outliers this run** (max 1.31, vs the
2.0+ threshold that previously flagged catastrophic events) -- the one miss (rep2, 1.22m,
`TARGET_LOST`) shows only a MILD `kappa` rise to 1.42, not a full ratchet (prior ratchet
events hit 3.3-10.8). This is the cleanest single-rep-miss severity seen across every
mechanism tested this session (drift fix alone, `PLASMC_AZ_JOINT`, and now `CBF_JOINT_QP`).

**n=3 only -- not yet a proper n>=5 confirm.** Given this is the best single-test result so
far AND a structurally more principled implementation (joint solve inside the barrier itself,
not a downstream patch), this is the strongest current candidate for further validation.

**Two hard-won process lessons, now doubly confirmed**: (1) ANY a_z/thrust-deliverability
mechanism in this codebase needs its own explicit angle-sanity bound -- bounding thrust
MAGNITUDE alone is not sufficient, the derived lean RATIO can still blow up independently.
Check for this pattern before trusting any future variant. (2) Offline stress-testing with
EXTREME/edge-case inputs (not just realistic-flight-range values) caught both blowup bugs
before they reached SITL -- the realistic-range tests alone (first equivalence check) would
have missed both.

---
## ⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (later still): `PLASMC_AZ_JOINT` implemented, a real bug caught
## and fixed pre-merge, validated SAFE but LOW PRACTICAL IMPACT at IC3/IC5

Implemented the joint `a_z`-in-QP design scoped earlier this session, per explicit user
direction ("complete the joint CBF approach... irrespective of where we are"). `controller.py`
only (`cbf_visibility.py` untouched -- `tools/validate_cbf.py` stayed 12/12 throughout).

**First draft had a real bug, caught in offline review before SITL**: raised `a_z` alone
without rescaling `I_a[:2]`, which SHRINKS the realized lean instead of preserving it (exactly
backwards). Rewritten to defer to the codebase's own ALREADY-EXISTING, unconditional
thrust-magnitude sphere cap (`|I_a+g*e3|<=A_CAP`, `controller.py` "DELIVERABLE-THRUST-
MAGNITUDE CAP", 2026-08-23) rather than duplicating it -- numerically verified this existing
mechanism correctly preserves direction under saturation.

**First SITL smoke test (n=3, IC5) FAILED: 0/3 SP.** Root cause: the simplified version fully
SKIPPED the fixed-angle `theta_cap` clip, not realizing that clip does DOUBLE DUTY -- it's
both a deliverability bound AND the ONLY thing preventing a pathological/degenerate QP output
(`theta_desired = I_a[:2]/a_z` can blow up) from reaching Fix B's `rd3` (attitude direction)
undamped. Observed live: `theta_cone` hit **21.68 rad (~1240deg)**, nonsensical, driving the
attitude command directly. The thrust-magnitude sphere cap only bounds MAGNITUDE -- a garbage
DIRECTION with correct magnitude is still garbage.

**Fixed**: never fully skip the angle clip. Instead make its BOUND `a_z`-aware --
`arccos(a_z_current/A_CAP)` (the true max deliverable angle AT THE CURRENT `a_z`, using the
same boundary/max-achievable-lean derivation as the existing hover-based constant) instead of
the fixed `arccos(g/A_CAP)`. Always finite (0 to pi/2 rad), reduces to the EXACT original
constant at `a_z=g` (hover), correctly tightens as `a_z` moves away from hover -- addresses
the real "assumes hovering" gap without ever removing the safety bound. `PLASMC_AZ_JOINT=0`
preserves the exact original behavior byte-for-byte.

**Re-test (n=3, IC5, corrected version): 2/3 SP, worst miss 1.73m -- no catastrophic
failures, `theta_cone` confirmed bounded (max 0.766, well under pi/2) throughout all 3
reps.** Comparable to the drift-fix-only baseline's performance at this IC.

**Honest assessment: correct and safe, but LOW PRACTICAL IMPACT here.** The joint deliverability
check (`az_joint_delta(t)`, newly logged) engaged in only 0-1 frames per rep (out of 600-1400) --
`a_z` never comes anywhere near `A_CAP` in these reps, so the constraint essentially never
binds. This is CONSISTENT with the mid-session physics correction: the empirically-observed
ratchet mechanism doesn't involve thrust saturation (dtheta_correction tops out ~2 m/s^2 on
top of hover ~9.81, vs A_CAP~13.6 -- nowhere near the ceiling), so a saturation-only joint
constraint has little to contribute to THIS specific failure. The one remaining IC5 miss
(rep2, 1.73m) still shows the ratchet (`kappa` to 3.91, `dtheta_az` outlier to 29.48,
`TARGET_LOST`-contained) -- neither `CBF_HZ_AWARE_DRIFT` nor `PLASMC_AZ_JOINT` fixes it.

**Conclusion**: `PLASMC_AZ_JOINT` is implemented, validated safe (no regression, always
bounded), and correctly does what it was designed to do (an exact, non-heuristic
deliverability bound) -- but it's not, on its own, the fix for the failures this investigation
has been chasing, since those don't appear to involve thrust saturation. It remains a
legitimate, principled replacement for the old hover-assumed cap (worth keeping/baking on
its own correctness merits, e.g. it would matter more at genuinely aggressive/high-`a_z`
maneuvers), but the REMAINING IC3/IC5 gap is still open, and per the earlier finding in this
file (IC5_rep4-vs-rep5), likely needs EITHER the `th_curr` self-defeating-loop fix (unrelated
to `a_z`/thrust budget) OR the Phase-2 detection-outage mitigation (a minimum-authority floor
on `theta_cone` during a decode-fail streak) -- not further `a_z`-budget work.

**n=3 only** -- not yet a proper n>=5 confirm; given the low practical impact found, a larger
sample is lower priority than pursuing the two mechanisms actually implicated in the
remaining failures.

---
## ⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (later same day): TWO DISTINCT TRIGGERS for the IC3/IC5
## kappa-ratchet, not one -- the IC5_rep4-vs-rep5 ignition discriminator is a genuine,
## sustained CBF Phase-2 detection outage, INDEPENDENT of DTHETA_HREF/h_ref entirely

Traced why `IC5_rep4` (n=5 `CBF_HZ_AWARE_DRIFT=1` confirm sweep, `DTHETA_HREF` NOT set) hit a
large outlier `dtheta_az` spike (11.15 rad, 6 events) yet landed clean, while `IC5_rep5`
(same config) ignited the ratchet. **This sweep ran entirely without `PLASMC_DTHETA_HREF`
set** -- so the `dtheta_href_g`/`h_ref` leak mechanism described above CANNOT be what
happened in rep5; it's a structurally different trigger for the same downstream symptom
(kappa ratcheting, `a_u` exploding).

**The two reps track nearly identically through t~2.0s** (`s_e_n_y`/`kappa_y` both converging
normally, near-identical trajectories) **then bifurcate**: rep4's `s_e_n_y` continues
converging (0.60->0.38 by t=4.8); rep5's reverses and diverges (0.60->1.14 by t=4.8) -- well
BEFORE any dtheta outlier appears in rep5 (its spike doesn't occur until t~5.75-5.81, long
after the divergence is already well underway) and while `kappa_y` is still behaving almost
identically in both reps at the bifurcation point.

**Root cause of the bifurcation, confirmed via `theta_cone`/`theta_desired` (`Control_Data.npy`):**
`cbf2_filter` fell into its Phase-2 fallback (`corners=None`, decode failed) in BOTH reps, but
with very different character:

| | rep4 (no ignition) | rep5 (ignites) |
|---|---|---|
| Total time in Phase-2 | 28.6% | **88.0%** |
| Longest continuous Phase-2 run | 234 frames (~2s) | **554 frames (~5s, t=0.72-5.74)** |
| `theta_cone` during that run | 0.766 (near-ceiling, healthy) | **0.0 for 99.3% of it** |

Phase-2's `theta_tight` computation (`cbf_visibility.py:264-278`,
`effective_margin = max(m2_p2 - |cr_ref+dft_ref|, 0)`) freezes `theta_cone` at EXACTLY 0 --
i.e. ZERO lateral authority -- whenever the marker's LAST KNOWN position before a decode-fail
streak was already near the FoV edge. This is the CBF working exactly as designed
(conservative: "don't know where the marker is, last saw it somewhere risky, so no lateral
authority until I know more") -- **not a CBF defect**. The actual problem is upstream:
IC5_rep5 hit a genuine, ~5-second, near-total cross-marker DETECTION OUTAGE (88% Phase-2)
starting almost immediately (t=0.72) from an already-marginal last-known position, freezing
the vehicle level with zero lateral correction authority while badly off-center -- `s_e_n_y`
has nowhere to go but grow for that whole window, and by the time detection recovers the
error has grown large enough that kappa's growth term dominates and the ratchet ignites once
authority returns. The later `dtheta_az` outlier spike (t~5.75) is a LATE SYMPTOM of the
divergence already being severe, not its cause -- confirms/extends the earlier finding (same
day, IC5_rep2 post-fix analysis) that dtheta outliers are downstream symptoms, generalizing
it to a SECOND, DTHETA_HREF-independent root cause.

**This connects back to the detection-reliability thread** (`lt2_angle_clusters`,
oblique-viewing-angle fragility at IC5's steep initial geometry -- see the FALSIFIED/CONFIRMED
sections in this same file's history) rather than being a CBF-design or `dtheta`/`h_ref`
issue. The joint `a_z`-in-QP redesign scoped above would NOT have prevented this specific
rep's failure -- it only addresses the `dtheta`-driven trigger, not this
detection-outage-driven one.

**Candidate mitigation, NOT YET IMPLEMENTED**: Phase-2's `theta_tight` currently allows a hard
freeze to EXACTLY 0 with no floor. A modest minimum-authority floor (e.g. never let
`theta_cone` drop below some small epsilon even on a fully-exhausted margin) would trade a
little visibility conservatism for avoiding the total lateral deadlock that lets `s_e_n` grow
unbounded during a long outage -- worth prototyping and A/B-ing similarly to the drift-model
fix, but is a genuinely different lever from anything scoped so far (Phase-2 fallback policy,
not the Phase-1 QP's drift model or a joint `a_z` constraint).

**Updated overall picture**: the IC3/IC5 kappa-ratchet has (at least) TWO independent
triggers -- (1) `DTHETA_HREF`'s `dtheta_href_g`/`h_ref` leak (only when that flag is set,
original finding above) and (2) a sustained CBF Phase-2 detection-outage freeze (occurs
regardless of `DTHETA_HREF`, this finding). The `h_z`-aware drift fix helps with (2)
indirectly (better drift-awareness reduces how often/how badly the marker is lost near the
edge in the first place, explaining IC3's full recovery and IC5's rate improvement) but
doesn't structurally prevent a genuine multi-second detection outage from still occasionally
occurring and freezing authority. Both triggers need their own fix; neither is fully closed.

---

## ⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (later same day) — `CBF_HZ_AWARE_DRIFT` PROTOTYPE: substantially reduces (not fully eliminates) the IC3/IC5 ratchet, WITHOUT `DTHETA_HREF`

User's counter-proposal (in-session): rather than freeze `h_ref` and patch around the ratchet
downstream, fix the CBF's own drift model at the source. `cbf2_filter`'s Phase-1 drift term
`dft = tau*state["d"]` (`cbf_visibility.py`) assumes the currently-measured feature drift
rate holds CONSTANT over the lookahead horizon `tau` -- but under continued descent with any
lateral offset, translational drift scales with `1/Z`, so it actually ACCELERATES as Z
shrinks. The constant-velocity model systematically undershoots near-touchdown drift, making
the QP reactive (only tightens after drift has ramped up) instead of proactive -- independently
corroborates `reference_cbf_visibility_architecture`'s already-flagged Gap #1 (missing `δ_m`
linearization-residual margin).

**Prototype implemented** (`cbf_visibility.py`, gated `CBF_HZ_AWARE_DRIFT`, default off,
scale-free): passes `h_z` (`self._h[-1][2]`, the already-computed loom/closing-rate proxy,
`~= -Zdot/Z`) into `cbf2_filter`. When enabled, `dft` becomes an exponential extrapolation
`state["d"] * (expm1(closing_rate*tau)/closing_rate)` (`closing_rate = max(-h_z, 0)`) instead
of the linear `tau*d` -- reduces EXACTLY to the old formula at `h_z=0` (verified numerically,
also `tools/validate_cbf.py` still 12/12). `cbf_visibility_aruco.py` accepts-but-ignores `h_z`
for call-site compatibility only (ArUco untouched, out of scope). No Z/altitude used anywhere
-- fully scale-free per the project's hard constraint.

**n=5 IC3+IC5 confirm (GT-FB, `CBF_HZ_AWARE_DRIFT=1`, `DTHETA_HREF` NOT set -- tests the fix
alone):**

| IC | no-fix baseline | with h_z-aware drift |
|---|---|---|
| IC3 | 4/5 SP (1 miss, 1.99m) | **5/5 SP** |
| IC5 | 3/5 SP (1 miss, **5.46m catastrophic**) | 4/5 SP (1 miss, **1.71m**) |
| Combined | 7/10 SP | **9/10 SP** |

Both failure RATE and SEVERITY improved substantially with zero regression on the other 9/10
reps, and this required NO `DTHETA_HREF` at all -- supports the hypothesis that fixing the
QP's own drift model at the source reduces the need for the downstream reactive
suppression/slowdown layer, rather than just patching around it.

**NOT a complete fix.** The one remaining IC5 miss (rep5) still shows the ratchet signature
(`kappa_y` 0.5->3.3, `dtheta_az` spiked to 15.1 rad with 5 outlier events >2rad) -- milder
than before (`a_u_y` peaked at 72, vs -152/+378 pre-fix) and caught by `TARGET_LOST`'s
open-loop fallback before full divergence (1.71m vs 5.46m), but the underlying mechanism
(uncapped `dtheta_az` -> ratchet) is only reduced in frequency, not eliminated.

**User's parallel proposal, not yet implemented**: rather than only clip `a_x`/`a_y` in the
QP (current design), also bring `a_z` into the SAME constrained optimization -- since
`theta_max_deliverable = arccos(a_z/A_CAP)` (`THETA_CAP_DEG_DERIVED`, `controller.py:197`,
already correctly derived from measured thrust margin), growing `a_z` (the current dtheta/
`h_ref` "extra lift" approach) PHYSICALLY SHRINKS the max deliverable lean at the SAME fixed
thrust ceiling -- it was never going to reliably help, independent of the `th_curr`
feedback-loop bug. A properly joint QP could correctly weigh the `a_z`/lean tradeoff instead
of blindly growing `a_z`. Scoped as the next step if the drift-model fix alone (once
`DTHETA_HREF`'s original `ASCENDING`-fix role is separately re-checked) doesn't fully close
the remaining gap.

**Per-rep confirmation (2026-08-29, later same day): outlier `dtheta_az` spikes are necessary
but NOT sufficient for the ratchet to ignite.** Full per-rep `dtheta_az`/`kappa`/`a_u` check
across all 10 IC3+IC5 reps:

| rep | outcome | dtheta max | outlier spikes (>2rad) | kappa max | a_u max |
|---|---|---|---|---|---|
| IC3 rep1-5 | all SP (0.019-0.034m) | 0.6-1.4 | **0/5 reps** | 0.50 (pinned) | <=1.43 |
| IC5 rep1-3 | SP (0.013-0.018m) | 0.07-0.74 | 0 | 0.50 (pinned) | <=1.89 |
| IC5 rep4 | SP (0.017m), clean | **11.15** | **6** | 0.50 (pinned, NO ratchet) | 1.62 |
| IC5 rep5 | miss (1.71m) | 15.11 | 5 | 3.30 (ratchet) | 72.24 |

IC3 is now completely clean (zero outlier spikes across all 5 reps). IC5_rep4 is the key new
data point: it hit a comparably large/numerous outlier spike run (11.15 rad, 6 events) to the
failing rep5 (15.11 rad, 5 events), yet `kappa` never left its 0.50 init and the rep landed
clean -- confirming the fix reduces BOTH how often outlier spikes occur (still 2/5 at IC5,
down from the implied near-universal rate pre-fix) AND, independently, the vehicle's
vulnerability to a given spike actually cascading into the ratchet (now only 1/5 spike-bearing
reps ignites, vs the pre-fix baseline where every observed bad rep ignited). Two separate,
still-open sub-problems, not one: (a) why outlier `dtheta_az` spikes still occur at all under
the improved drift model, and (b) what precisely differs between a spike that ignites (rep5)
vs one that doesn't (rep4) -- worth a direct rep4-vs-rep5 trace comparison before the joint
`a_z`-QP work, since it may reveal a cheaper interim mitigation (e.g. a rate-limit on
`dtheta_href_g`, if `DTHETA_HREF` is ever re-enabled) independent of the larger redesign.

**Next steps, in order**:
1. Check whether `CBF_HZ_AWARE_DRIFT=1` alone (still without `DTHETA_HREF`) also prevents the
   original `ASCENDING` fly-away pattern under real (non-GT-feedback) perception -- this
   smoke test was GT-feedback only, which may not exercise that failure mode the same way.
2. Trace IC5_rep4 vs IC5_rep5 directly (both hit large outlier dtheta_az spikes; only rep5
   ignited) to understand the ignition-vs-non-ignition discriminator before committing to the
   full joint-QP redesign -- may reveal a cheaper interim fix.
3. If a residual gap remains after (1)/(2), scope+implement the joint `a_z`-in-QP design
   (scoped 2026-08-29, see the design-scope conversation this session: replace the current
   hover-assumed `theta_cap = arccos(g/A_CAP)` with the true spherical deliverability
   constraint `|I_a+g*e3|<=A_CAP`, jointly optimized with the visibility box via a 3rd
   sphere-projection step added to the existing alternating-projection QP; requires
   re-deriving the forward-invariance theorem under the joint constraint before it can be
   trusted the way the current barrier is -- bigger, multi-session task, not a quick add).
4. Full IC1-5 regression gate with `CBF_HZ_AWARE_DRIFT=1` (currently only IC3/IC5 tested;
   IC1/IC2/IC4 not yet re-checked with this change, though it's a no-op there whenever
   `h_z>=0`/not closing).

---

## ⭐⭐⭐⭐⭐⭐ ROOT CAUSE (2026-08-29, CORRECTED same day): the IC3/IC5 gate failures are the kappa-ratchet, reignited by DTHETA_HREF modulating a pre-existing (correct) h_ref->lateral coupling

The "lateral-drift" failure flagged in the 2026-08-28 IC1-5 gate entry below was a
MISDIAGNOSIS -- closer inspection of `Control_Data.npy` shows it's the exact
kappa-ratchet signature (failure mode 11) that `project_20260828_kappa_ratchet_campaign`
confirmed resolved in the current baked config -- **but that campaign never tested with
`PLASMC_DTHETA_HREF=1` set**, and this gate did.

**Confirmed ratchet data:**
- IC5_rep2 (xy=5.46m): `kappa_y` 0.17 (t=6s) -> 4.3 (t=11.5s), `sigma_y` diverges past
  -9 (E=1 bound), `a_u_y` explodes to -152.
- IC3_rep3 (xy=1.99m): `kappa_y` (later, x too) 0.08 -> 10.8, `a_u_y` to +378.
- Both: `s_e_n` grows monotonically instead of converging, matching every prior
  documented ratchet trace in this codebase.

**⛔ CORRECTION (same day, caught by user): the original write-up below mischaracterized
`h_d_ff = (h_ref_eff - dot(cross_ws,e3)) * self._s[-1][:3]` (`controller.py:2390`) as an
"oversight" leaking `h_ref_eff` into the lateral channel. It is NOT a bug and NOT new.**
Verified via `git log` + the MATLAB reference:
- This exact formula has existed in the Python port since the earliest recoverable
  commit (`4e3b6ed2`, 2026-06-01) -- ~3 months before `PLASMC_DTHETA_HREF` was added
  (2026-08-24). `git log -S` on the term confirms no intervening change to this
  multiplication.
- It is a direct, intentional port of `visualControl_IBVS_adaptive.m:665-666`:
  `V_h_d(:,idx) = V_ds_d + cross(V_w, V_s(1:3)) + (h_rd - dot(cross(V_w, V_s(1:3)), e3))*V_s(1:3)`
  -- MATLAB's own `h_rd` (same role as `h_ref_eff`) multiplies the FULL `V_s(1:3)`, byte-
  for-byte matching the Python. The code's own comment even cites this ("MATLAB
  visualControl_IBVS_adaptive.m:369-370 EXACTLY").
- This is correct IBVS kinematics: `s` is the full 3D normalized bearing/position vector;
  a scalar desired closing-rate naturally projects through it into all 3 axes of the
  feedforward `h_d`. Not a wiring mistake.

**Trigger mechanism, corrected: the risk is entirely in what DTHETA_HREF adds (2026-08-24), not in the pre-existing s[:3] coupling itself.**
1. `PLASMC_DTHETA_HREF`'s `dtheta_href_g` is a NEW, time-varying gain (0.15-1.0) that
   multiplies `h_ref_eff` before it flows through the (correct, pre-existing) `* s[:3]`
   coupling.
2. `dtheta_href_g`'s driver, `dtheta_az` (the raw, UNCAPPED suppressed-demand norm --
   only the gain-scaled `dtheta_correction` is capped at 2.0), can spike to outlier
   magnitudes (measured: 8.175 rad, ~4x typical) right at the geometrically hardest
   moment -- when `s_x`/`s_y` are already large (near the FoV edge), which is exactly
   when IC3/IC5's specific geometry (IC5: steep low-altitude viewing angle; IC3:
   opposite-quadrant `-2,2,5` spawn) makes the CBF suppress hardest.
3. That spike collapses `dtheta_href_g` transiently (measured: 0.95 -> 0.77), and because
   `h_ref_eff` legitimately couples into the lateral channel (see above), this transient
   propagates into `h_d`'s x/y feedforward at precisely the moment `sigma` is most
   vulnerable -- coincides with `sigma_y` accelerating from a normal ~-0.3/cycle step
   into a sustained divergent trajectory, after which `kappa`'s growth term dominates
   its leakage term and ratchets.

**Why IC1/IC2/IC4 don't show it**: they share IC3/IC5's lateral-offset magnitude but at
gentler viewing geometry (IC2/IC4 are higher-altitude, same offset; IC1 is centered), so
`dtheta_az` rarely produces an outlier-magnitude spike there.

**NOT YET FIXED. Candidate fix, corrected**: the earlier proposal ("restrict
`dtheta_href_g`'s scaling to `s[2]` only") is WRONG -- `h_ref_eff` is shared by every
other caller too (the `_descent_gate`/`_dgate_g` path, the non-`DTHETA_HREF` baseline),
and none of them are broken; special-casing `s[:3]` only for the `DTHETA_HREF` path would
diverge from the validated MATLAB coupling for no justified reason. The actual fix target
is `dtheta_az` (or `dtheta_href_g`'s rate of change) being unbounded per-cycle: either cap
`dtheta_az` itself before it drives the exponential (mirroring `dtheta_correction`'s
existing 2.0 cap), or rate-limit `dtheta_href_g` so a single-cycle outlier can't collapse
it that fast. Neither implemented or tested yet.

**Recommendation**: do NOT bake `PLASMC_DTHETA_HREF=1` as an unconditional default yet.
It is validated + safe for IC1/IC2/IC4, but IC3/IC5 need a fix (cap/rate-limit the
`dtheta_az`->`dtheta_href_g` path, NOT touch the `s[:3]` coupling) + re-validation before
it's safe everywhere.

---

## ⭐⭐⭐⭐⭐ IC1-5 GATE (2026-08-28): DTHETA_HREF's target mechanism confirmed fixed + no IC1/2/4 regression, but IC5 still fails via a DIFFERENT residual lateral-drift issue

Full IC1-5 gate, n=5/IC (25 reps), `WORLD=cross_marker MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1
PLASMC_DTHETA_HREF=1 HEADLESS=1`, isolated (verified no concurrent SITL before launch):

| IC | SP | mean xy | max xy | notes |
|---|---|---|---|---|
| IC1 | 5/5 | 0.003m | 0.005m | clean, no regression |
| IC2 | 5/5 | 0.019m | 0.022m | clean, no regression |
| IC3 | 4/5 | 0.413m | 1.990m | 1 miss, `anomaly='N/A'` -- not checked in depth |
| IC4 | 5/5 | 0.023m | 0.025m | clean, no regression |
| IC5 | 3/5 | 1.458m | **5.463m** | 2 misses, see below |

**The ASCENDING/dtheta-cap-pinning mechanism did NOT recur anywhere in this 25-rep gate** --
checked all 3 misses (IC5_rep2 xy=5.46m, IC5_rep4 xy=1.77m/target_lost, IC3_rep3 xy=1.99m):
all show `descent_anomaly_cause='N/A'` and mild `dtheta_correction` pinning (longest
consecutive pinned run 7-12 cycles, nowhere near the 55-74-cycle catastrophic signature).
**So `PLASMC_DTHETA_HREF=1` continues to do exactly what it was designed to do, and shows
zero regression at IC1/IC2/IC4** (all 5/5 SP, tight).

**But IC5 has a DIFFERENT, previously-undercharacterized failure mode.** IC5_rep2 (5.46m, the
worst xy_err of the whole investigation) traced via GT trajectory: altitude descends cleanly
and monotonically the entire flight (3.0m -> ~0m, NO climb/ASCENDING event) -- the failure is
a rapid LATERAL drift-away starting around t=7s (already at ~0.6-0.8m altitude): GT y goes
from +0.07m at t=7s to -4.55m by t=11s, a ~1.1 m/s sustained lateral divergence in the final
descent phase. Nothing to do with `dtheta_az` (pinning stayed mild throughout, 46/1247 total
cycles, longest run 10).

**Net assessment**: `PLASMC_DTHETA_HREF=1` is validated for what it targets (ASCENDING
cap-pinning fly-away) and safe re: IC1/IC2/IC4 regression. IC5 is NOT solved -- it now fails
via a different, unfixed lateral-drift mechanism near touchdown that this investigation has
not yet root-caused. **Do not describe IC5 as fixed even with this bake.** Combining this
gate's IC5 data with the earlier isolated n=7 (SP 5/7): IC5 across all DTHETA_HREF=1 testing
is now 8/12 SP (67%), 0/12 ASCENDING events, but the tail risk (up to 5.46m) via the
lateral-drift mechanism is real and unaddressed.

**Open item / real next step**: root-cause the terminal lateral-drift mechanism seen at
IC5_rep2 (and possibly IC3_rep3) -- check whether it's the same class of issue as the
already-known `feedback_terminal_root_lateral_zeta_r` / `feedback_terminal_smc_actuator_wall`
lateral-wall findings, or something new specific to the late-descent phase at these ICs.

---

## ⭐⭐⭐⭐⭐ FIX VALIDATED (2026-08-28): `PLASMC_DTHETA_HREF=1` eliminates the catastrophic mechanism

Isolated n=5 IC5 sweep (verified no concurrent SITL after two prior attempts got contaminated by
a different session's `kr_rp.sh` IC-sweep script -- one attempt lost 3/5 reps to launch-level
port/gRPC collisions with that concurrent sweep, a second attempt failed to even launch due to a
`cd` mistake in the background shell; third attempt clean), `WORLD=cross_marker MARKER_TYPE=cross
PLASMC_GT_FEEDBACK=1 PLASMC_DTHETA_HREF=1 HEADLESS=1 IC_LIST="IC5" N_REPS=5`:

| rep | pinned cycles (first 2s) | pinned total | longest pinned run | outcome |
|---|---|---|---|---|
| 1 | 0 | 3/919 | 2 | SP, xy=0.018m |
| 2 | 6 | 14/638 | 7 | miss, xy=1.80m, `target_lost=True`, `descent_anomaly='N/A'` |
| 3 | 0 | 2/916 | 2 | SP, xy=0.018m |
| 4 | 0 | 7/902 | 5 | SP, xy=0.025m |
| 5 | 0 | 0/875 | 0 | SP, xy=0.018m |

Combined with an earlier partial 2-rep run (same env, contaminated to n=2 by the concurrent
sweep -- rep A: 0.019m SP, 0 early-pinned; rep B: 0.57m miss/`target_lost`, 8 early-pinned):
**n=7 total, SP 5/7 (71%), ZERO `ASCENDING` descent anomalies, max pinned-cycle count ever
observed = 8** (vs the no-fix baseline's 55-74 in both catastrophic reps). The mechanism
root-caused below (sustained near-continuous dtheta-correction saturation at the 2.0 m/s^2 cap,
destabilizing the trajectory before any perception failure) did not recur even once with the fix
on.

**The one remaining miss (rep2, 1.80m) is NOT the same failure** -- `descent_anomaly_cause`
reads `'N/A'` (no ASCENDING event), it's a genuine `target_lost=True` with only 6/14 pinned
cycles, well below the 55+ threshold that characterized the catastrophic mechanism. A milder,
different failure mode, out of scope for this fix.

**Recommendation: bake `PLASMC_DTHETA_HREF=1` as a new default** -- it is validated at IC5
(where the mechanism was found and is most severe) and directly targets the root cause (breaks
the `th_curr` self-defeating attitude-history feedback loop that let `dtheta_correction` sustain
at its cap instead of settling). NOT yet checked: IC1-4 regression (does turning this on change
anything at the ICs where dtheta rarely engages?) -- per [[feedback_ic_validation]], any
gain/control-law default change needs the full IC1-5 gate before baking, not just the IC where
the bug was found. This validation covers IC5 only.

---

## ⭐⭐⭐⭐ ROOT CAUSE (2026-08-26): the catastrophic mechanism is `dtheta`'s uncapped-duration cap-pinning, NOT primarily angle-clustering

Investigated why the two catastrophic reps (pre-fix rep2 xy=2.76m, post-fix-with-a87ac00 n=5
rep5 xy=4.78m) fail so severely, since the earlier framing (angle-clustering dominates the
fail-reason tally in both) doesn't explain WHY the geometry degrades that badly in the first
place. `Ground_Truth.npy`'s own `descent_anomaly_cause` field for BOTH bad reps reads
`'ASCENDING'` -- the drone climbed away rather than descending -- with `target_lost=False` in
both cases (this is NOT the `CBF_CORNERS_STALE_ABORT`/`TARGET_LOST` open-loop-fallback path
originally assumed; it's a genuine closed-loop instability under otherwise-normal GT-feedback
position control).

**`Control_Data.npy` has the smoking gun** (`dtheta_correction(t)`, `dtheta_az(t)`,
`theta_cone(t)` are all logged per-cycle from the 2026-08-24/25 `dtheta` investigation's own
instrumentation -- this data was already being recorded, just not looked at for this specific
question):

| rep | outcome | pinned-at-2.0-cap cycles in first 2s | longest pinned run (whole flight) |
|---|---|---|---|
| pre-fix rep2 | BAD (2.76m) | 55 | -- |
| post-fix n5 rep5 | BAD (4.78m) | 74 | 26 (t=0.75-1.0s) |
| post-fix n5 rep4 | good (0.015m) | 5 | 3 |
| post-fix n3 rep1 | good (0.062m) | 0 | 0 |
| post-fix n5 rep2 | good (0.021m) | 1 | -- |
| post-fix n5 rep3 | good (0.047m) | 16 | -- |

`dtheta_correction` is `clip(gain=10.0 * ||th_desired - th_safe||, 0, PLASMC_DTHETA_AZ_CAP=2.0)`
(`controller.py` ~line 3345-3352) -- the per-cycle CAP is unconditionally applied by default
(no env var needed to activate it, only to change its value), but nothing stops it from
PINNING at that ceiling for many consecutive cycles when the CBF keeps suppressing lateral
demand hard (which IC5's steep initial viewing angle, near the visibility cone's ~0.766 rad/44
deg ceiling from frame 1, reliably does). A sustained 26-cycle pinned run is ~0.3-0.4s of
continuous MAXIMUM extra-lift injection (2.0 m/s^2, over a fifth of g) right at launch.

**Causality direction confirmed, not just correlated**: mapped post-fix rep5's Control_Data
clock to its Img_Data clock (cross-correlating `MARKER_EXTENT_PX` against GT `1/z`, best-fit
offset 27.0s, r=0.71) and checked Detection Status at the exact moment of the worst pinned
burst (t=0.75-1.0s, control clock) -- **100% `'ok'`, zero misses**. The `lt2_angle_clusters`
domination (239/580 misses, 41%) only appears later, mapping to roughly t=4.4-7.7s in the
control clock -- well after the early dtheta burst. **So the sequence is: dtheta burst
destabilizes the trajectory FIRST -> the resulting bad geometry (position/attitude excursion)
degrades real perception SECOND -> `lt2_angle_clusters` is a downstream symptom of the crash
already being underway, not its cause.**

**This reframes the whole a87ac00 finding**: `a87ac00` hardens the Hough/angle-clustering
PERCEPTION path -- exactly the downstream symptom -- so it plausibly reduces how often a
`lt2_angle_clusters` cascade compounds an already-destabilized trajectory into a full crash
(consistent with the observed ~halving of the failure rate), but it cannot touch the actual
`dtheta` control-law defect that destabilizes the trajectory in the first place. That defect
was already root-caused and a real fix designed on 2026-08-24/25
([[project_20260824_dtheta_az_filter_self_defeating_feedback]],
[[project_20260824_dtheta_href_continuous_compensation]]) -- `PLASMC_DTHETA_HREF=1` (breaks
the `th_curr` self-defeating attitude-history feedback loop that lets the trigger persist
instead of settling) -- but it defaults OFF pending its own n>=5 validation, so every sweep in
this whole investigation (pre-fix AND post-fix) ran with the actual root-cause fix disabled.

**Open item / real next step**: validate `PLASMC_DTHETA_HREF=1` at IC5 with n>=5 (isolated,
no concurrent SITL, current HEAD code) -- if it suppresses the sustained-cap-pinning pattern
(fewer/no early 50+ cycle pinned runs) and the catastrophic-failure rate drops further, that
would be the actual fix for this IC, not further perception hardening.

## ⭐⭐⭐ FINAL (2026-08-26): a87ac00 helps, does NOT fix -- n=5 confirm sweep

The n=3 "apparently already resolved by a87ac00" reading directly below was itself too
optimistic -- a small-sample false negative, same class of error as the original "falsified"
mistake earlier this session (see the RETRACTION section further below), just less severe.
A proper n=5 confirm sweep on current HEAD (unchanged code, same isolated-SITL discipline)
reproduced the SAME catastrophic failure at nearly the same severity.

**Combined post-fix data (2 sweeps, n=8 total, all current HEAD / a87ac00 applied):**

| sweep | rep | detect-ok | `lt2_angle_clusters` share of misses | outcome | xy_err |
|---|---|---|---|---|---|
| n=3 run | 1 | 82% | 0% | SP | 0.062m |
| n=3 run | 2 | 88% | 0% | landed, not SP | 0.061m |
| n=3 run | 3 | 94% | 0% | SP | 0.032m |
| n=5 run | 1 | 62% | 39% (65/168) | landed, not SP | 0.039m |
| n=5 run | 2 | 82% | 0% | SP | 0.021m |
| n=5 run | 3 | 90% | 0% | SP | 0.047m |
| n=5 run | 4 | 93% | 0% | SP | 0.015m |
| n=5 run | 5 | 30% | **41% (239/580)** | **PX4 Impact-detected, hard failure** | **4.78m** |

Post-fix combined: **SP 5/8 (62.5%)**, `lt2_angle_clusters` present at a meaningful level in
2/8 reps, 1/8 a genuine catastrophic hard-impact failure. Compare pre-fix (below): SP 1/3
(33%), `lt2_angle_clusters` meaningful in 2/3 reps, 1/3 catastrophic. **Both catastrophic
reps (pre-fix rep2 xy=2.76m, post-fix n=5 rep5 xy=4.78m) ended via PX4's raw impact detector
(`|a|>50 m/s^2`), NOT the clean loom-inversion touchdown disarm that every good rep hit** --
same failure signature, both before and after the fix.

**Conclusion: `a87ac00` roughly halves the failure rate (and raises SP rate) but does not
eliminate the underlying mechanism.** IC5's steep ~42deg viewing angle can still starve the
corner-join filter of enough real segment pairs to prevent a bad angle-cluster often enough
to cause a hard landing, at a rate on the order of 1-in-5 to 1-in-8 reps. Do not describe this
IC as fixed. If pursuing further: either loosen `merge_tol_deg`/tune the corner-join gap
tolerance specifically for IC5-range viewing angles, or add a grace/retry path in
`landing_test.py` for a sustained `lt2_angle_clusters` streak specifically (distinct from the
existing `CBF_CORNERS_STALE_ABORT`, which is a `_cbf_corners_none_streak` catch-all not
keyed to fail-reason).

## ⭐⭐ RETRACTION + CORRECTED FINDING (2026-08-26): the "falsified" entry below was premature; the bug was real and is apparently already fixed by a87ac00

The FALSIFIED entry immediately below this one (also written 2026-08-26, same session) drew its
conclusion from only ONE data point: a clean re-run on CURRENT HEAD code landing 3/3 tight. That
re-run had an uncontrolled confound -- `cross_marker_detector.py` changed (`a87ac00`, the
corner-join Hough-segment filter) the morning AFTER the original 2026-08-24 failing investigation,
so "clean re-run" was actually "clean environment + different code", not a true isolation of the
contamination variable alone. Called out by the user ("I feel you are jumping the gun here").

**Proper controlled experiment**: temporarily swapped `cross_marker_detector.py` to the EXACT
pre-fix code (`git show 686a66e:...`, the version live during the original 08-24 failing
investigation), re-verified no concurrent SITL, ran the same isolated IC5 n=3 sweep
(`WORLD=cross_marker MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1 HEADLESS=1 IC_LIST="IC5" N_REPS=3`),
then restored current HEAD exactly (verified byte-identical via diff against the pre-swap backup).

| rep | detect-ok | dominant fail reason | landing |
|---|---|---|---|
| 1 | 258/279 (93%) | insufficient_fit_points 13, centroid_mismatch 39 | 0.025m, clean |
| 2 | 156/777 (20%) | **lt2_angle_clusters 370 (60% of 621 misses)** | **2.76m -- degraded** |
| 3 | 230/400 (57%) | lt2_angle_clusters 60 (35% of 170 misses), centroid_mismatch 43 | 0.063m, mildly degraded |

This directly reproduces the ORIGINAL 2026-08-24 signature -- wide detect-ok variance across
identical-IC reps (20-93%, matching the original's 17-94% range) with `lt2_angle_clusters` as the
dominant fail reason specifically in the bad rep -- **with zero concurrent SITL sessions running**.
The failure-reason-to-outcome link (dominant `lt2_angle_clusters` share tracks directly with landing
degradation across the 3 reps) is much stronger evidence than the original's outcome-only landing
stats. **Conclusion: the angle-clustering fragility at IC5's steep oblique viewing angle is a real,
probabilistic, code-level failure mode -- not a contamination artifact.**

**Then, separately: current HEAD code (with `a87ac00` applied) was re-tested clean (n=3, see the
now-superseded FALSIFIED section below for that data) and showed ZERO `lt2_angle_clusters`
occurrences across all 3 reps, landing tight (0.032-0.062m).** `a87ac00`'s docstring frames itself
purely as a "2nd shadow-contamination layer" (filtering Hough segments from the drone's own cast
shadow before they can seed a bad angle cluster) -- it was never explicitly validated against or
credited for fixing the IC5 angle-clustering failure mode specifically. But mechanistically it runs
`_filter_segments_by_corner_join` BEFORE `_cluster_line_angles`, stripping exactly the kind of
spurious/unpaired Hough segments that would otherwise seed a bad cluster -- plausible that it fixes
BOTH the shadow-contamination case it was built for AND this steep-angle case, which may share the
same underlying "spurious segment pollutes the cluster" mechanism even though the original framing
(arms projecting too close to parallel) is geometrically distinct from shadow contamination.

**Net assessment**: the bug was real (confirmed above), and current code most likely already fixes
it as a side effect of an unrelated-sounding fix -- but this is n=3 vs n=3, not a large-sample
confirm. **Recommend a real n>=5 IC5 sweep on current HEAD before fully closing this** -- the
post-fix 3/3 clean result could still partly be luck, same as the original single "passed rep" was.
If `merge_tol_deg=12` or the pre-cluster segment count is still occasionally marginal at IC5's ~42
deg viewing angle, a larger sample is the only way to see it.

---

## ⛔⛔ SUPERSEDED (2026-08-26, was briefly the leading entry, itself retracted above): "FALSIFIED -- no angle-clustering bug at IC5"

This section's own conclusion is WRONG (see the retraction above) -- kept for the process lesson
(a same-code-family "clean" comparison isn't controlled if the code isn't actually the same) and
because its underlying data point (current-HEAD clean sweep = 0/3 lt2_angle_clusters, tight
landings) is still valid and now correctly re-interpreted above as "current HEAD already fixes it",
not "there was never a bug".

Isolated re-run (confirmed no concurrent SITL/claude session touching SITL beforehand),
`WORLD=cross_marker MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1 HEADLESS=1 IC_LIST="IC5" N_REPS=3`
via `run_ic_validation.sh`, on CURRENT HEAD code (post a87ac00, NOT the code live during the
original investigation -- this is the confound that made the "falsified" conclusion premature):

| rep | xy_err | rel_vel | soft | precise |
|---|---|---|---|---|
| 1 | 0.0615m | 0.0330 m/s | yes | yes |
| 2 | 0.0607m | 0.0786 m/s | no | no |
| 3 | 0.0318m | 0.0289 m/s | yes | yes |

All 3 landed cleanly, tight band, no TARGET_LOST, no hard impact.

## ⛔ CORRECTION (2026-08-25, same day, discovered via a DIFFERENT session's memory write)

**This entire IC5 angle-clustering investigation below was very likely run WHILE a
concurrent session was ALSO running its own IC5 sweep** — discovered only after the fact
via [[feedback_check_concurrent_sitl_before_launch]]'s "Independently confirmed" addendum
(that other session found a live SITL process tree spanning 11:04→11:33+, squarely
overlapping this session's `ic5_hang_chase_*` reps). Concrete consequences for THIS
memory's findings:
- `MicroXRCEAgent` binds a single global port (8888) — the repeated launch flakes here
  (attributed below to leaked `/dev/shm` fastrtps_* state) may actually/also be
  `port 8888 already bound` collisions with the other session, not a resource leak alone.
- Two concurrent `gz sim` stacks compete for CPU. Sim TIME stays deterministic (lockstep),
  but real-wall-clock-dependent subsystems — image capture rate, OpenCV decode timing,
  thread scheduling — do NOT, and that's exactly the machinery behind cross-marker
  detection reliability that this memory's "IC5 fails via angle-clustering" conclusion
  rests on.
- The extreme alpha-std variance found across reps (5.4° → 133.3° at the IDENTICAL IC)
  is consistent with genuine steep-viewing-angle fragility, but is EQUALLY consistent
  with resource contention corrupting perception timing unpredictably rep-to-rep. Cannot
  currently distinguish these two explanations with the data gathered.

**What still stands**: `CBF_CORNERS_STALE`'s GT-feedback bypass (`controller.py:1004-1005`)
is a static code fact, read directly, not dependent on sim performance — that ruling-out
is NOT weakened by this correction. What's downgraded is the "IC5 has a genuine
angle-clustering detection bug" conclusion itself, and the "the hang was an infra
artifact, not a control bug" conclusion (equally now explained by the concurrent-session
collision as by the `/dev/shm` leak).

**Before trusting anything below**: re-run the IC5 sweep with verified NO concurrent SITL
session (per the new standing rule), and check whether the failure rate / alpha-std
variance holds up in isolation. If it does, the angle-clustering conclusion is confirmed.
If IC5 lands cleanly and consistently in isolation, the entire investigation below was
chasing a contamination artifact.

Continues [[project_20260824_touchdown_groundcontact_and_perception_hardening]] (same
session). That memory covers the ground-collision-height finding + 3 perception fixes;
this one covers the follow-up IC1-5 validation sweep and the IC5-specific failure dig.

## IC1-5 GT-feedback sweep at true ground contact (`test_data/GTFB_GroundContact_IC1to5/`)

Flat `cross_marker` world (not rover), true ground collision height
(`model.sdf.bak_before_legext_20260809` reverted, per the prior memory), all 3 same-session
perception fixes live (Hough corner-join filter, hw coast+freeze KF, touchdown rolling
window).

| IC | Result | xy_err | rel_vel | min_alt |
|---|---|---|---|---|
| IC1 (0,0,5) | SOFT+PRECISE | 0.049m | 0.076 m/s | -0.01m |
| IC2 (2,2,5) | SOFT+PRECISE | 0.072m | 0.043 m/s | -0.01m |
| IC3 (-2,2,5) | SOFT+PRECISE | 0.042m | 0.013 m/s | -0.01m |
| IC4 (2,2,7) | SOFT+PRECISE | 0.030m | 0.044 m/s | -0.01m |
| IC5 (2,2,3) | **TARGET_LOST / hard impact** | 2.429m | 2.149 m/s | -0.02m |

IC1-4 confirm the ground-contact fix generalizes (not an IC1-only artifact); all latch
`TOUCHDOWN-DETECT` cleanly at true ground. IC5 is the outlier.

## IC5 root cause: angle-clustering failure at steep oblique viewing angle

IC5 = `ENU (2,2,3)`, i.e. low altitude (3m) + large lateral offset (2,2) -> a much
steeper viewing angle (~42 deg) than IC1-4 (same lateral offset but 5-7m altitude, or
IC1's centered geometry). This is a DIFFERENT failure mode from the near-touchdown
shadow/Hough-line-COUNT collapse this session's other fixes target (Fix 1's corner-join
filter, `project_20260824_touchdown_groundcontact_and_perception_hardening`) -- here
Hough finds segments fine, but they fail to separate into >=2 distinct ANGLE CLUSTERS
(`_cluster_line_angles`'s `merge_tol_deg=12` tolerance), because the cross's two arms
project too close to parallel/merged at this oblique angle.

**Confirmed via 3 independent reps at the identical IC**, revealing the failure is
probabilistic (threshold effect), not deterministic:

1. **Failed rep** (`Mon Aug 24 21-51-45 2026`): 17% detect-ok (116/686), TWO long
   consecutive miss streaks (287 and 263 frames), `lt2_angle_clusters` dominant fail
   reason (388/570 misses, 68%) + `hough_lt2_lines` (63) + `color_gate_empty` (62) --
   90% of failures never resolved marker geometry at all. Hover-settle alpha
   std=14.6 deg.
2. **Passed rep** (`Mon Aug 24 22-43-33 2026`, same IC, re-run): 94% detect-ok
   (303/324), longest miss streak only 14 frames, dominant fail reason
   `insufficient_fit_points` (16/21, 76%) -- geometry resolved fine, just a few frames
   short on surviving pixels post-pruning (a MUCH milder failure class). Hover-settle
   alpha std=11.8 deg. Landed SOFT+PRECISE (xy=0.003m).
3. **Extreme rep** (`ic5_dbg_1.out`, `PLANAR_MAP_DBG=1` trace): hover-settle alpha
   std=133.3 deg (near uniform-random -- essentially garbage even before descent
   starts). `_cbf_corners_none_streak` grew monotonically with ZERO resets from frame
   ~7 through the observed trace (up to 172) -- direct printed proof of a sustained,
   total detection loss, not intermittent flicker. This rep hung past 180s rather than
   reaching TARGET_LOST or landing (see hang investigation below).

**Mechanism, not just correlation**: `_cbf_corners_none_streak` (which the printed
`[cbf_corners]` trace exposes directly) is exactly what feeds `CBF_CORNERS_STALE_ABORT`
(`controller.py:1017-1034`, threshold 350 frames) -- which is the ONLY thing gating
`landing_test.py`'s `feature_fresh` under GT-feedback (`GT_FEEDBACK==1` always shorts the
OR true, so `feature_fresh = not CBF_CORNERS_STALE_ABORT` exactly, by deliberate design --
see that property's own docstring, "ANDed in... so it can force feature_fresh=False even
when every other signal says fine"). A long enough `lt2_angle_clusters` streak at IC5
crosses that fuse and triggers the open-loop `TARGET_LOST` fallback -> hard impact.

## CBF_CORNERS_STALE (kappa-freeze) ruled out as a contributing mechanism

Checked directly in code (`controller.py:1004-1005`): `CBF_CORNERS_STALE` (the FAST,
30-frame version that freezes kappa's ODE integration, separate from the 350-frame
ABORT version) has an explicit `if self._gt_feedback is not None: return False` --
hard-bypassed under GT-feedback since 2026-08-19 (ported from the Hardware fork,
specifically because it was found pinning kappa at its KAPPA0 bootstrap value on real
GT/HW-position-feedback flights). All IC5 reps used `PLASMC_GT_FEEDBACK=1`, so kappa was
integrating normally throughout, never frozen by this path. Ruled out with certainty by
reading the property, not inferred.

## The "hung past 180s" rep: NOT reproduced, most likely an infra artifact

The extreme rep (#3 above) uniquely didn't reach TARGET_LOST or landing -- the outer
harness killed it as hung past 180s wall-clock, with `_cbf_corners_none_streak` still
only at 172 (well under both the 30-frame CBF_CORNERS_STALE fast-path -- irrelevant here,
bypassed -- and the 350-frame ABORT threshold). Attempted to reproduce with fresh
instrumentation (`PLANAR_MAP_DBG=1 TD_DEBUG=1`) 4 times in a row -- **all 4 retries
failed at the LAUNCHER level** (PX4 stuck waiting for Gazebo's `/world/cross_marker/clock`
topic, "Unable to get simulation time", 60s timeouts), never even reaching the control
loop. Root-caused those launcher failures to **~489 leaked `fastrtps_*` files in
`/dev/shm`** (24 SysV shm segments too), accumulated from repeated `kill -9` teardowns of
PX4/Gazebo across today's ~25+ SITL launches -- DDS/FastRTPS shared-memory transport
artifacts that don't get cleaned up on a forced kill. Removed the `fastrtps_*`-prefixed
entries specifically (user-approved, left the other ~366 unrelated files alone -- /dev/shm
is a shared system resource). The very next launch after cleanup worked cleanly and
landed SOFT+PRECISE with `_cbf_corners_none_streak` never exceeding 0 the whole flight --
no new evidence on the hang mechanism, since detection was perfect that rep.

**Conclusion (circumstantial, not proven)**: the original hang is most likely the SAME
class of PX4/Gazebo clock-sync degradation as the 4 failed retries, not a genuine
control-code stall -- especially since the one code-level stall hypothesis
(CBF_CORNERS_STALE/kappa-freeze) is definitively ruled out under GT-feedback. Treated as
closed/environment-attributed. If it recurs under verified-clean `/dev/shm` conditions,
re-open as a real bug.

## Process lesson: concurrent-session check gap, caught live by the user

Mid-investigation, cleanup kill loops targeting `px4_sitl`/`landing_test.py`/
`mavsdk_server`/`MicroXRCEAgent` were run WITHOUT the `grep -qa claude
/proc/$p/cmdline`-style ownership guard that the project's own launcher scripts already
use for `gz sim` kills specifically to avoid killing another Claude-owned process. User
asked directly whether concurrent SITL use had been checked -- it had not. `ps` confirmed
2 other `claude` sessions were running on this machine (different ptys). User then
confirmed no concurrent SITL use was actually happening, so no real harm done, but the
gap was real. See [[feedback_check_concurrent_sitl_before_launch]] (new standing rule,
also linked from the px4/MEMORY.md banner) for the full checklist -- apply this BEFORE
any future SITL launch or kill-loop cleanup in this project, not just when troubleshooting
flakiness (that's exactly when the temptation to skip the check is highest).

## ⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (BAKED): `CBF_JOINT_QP` flipped to the default path

Per explicit user direction ("implement CBF_JOINT_QP into controller.py as the default
path"). Two edits:
- `cbf_visibility.py`: `_joint_qp = env.get("CBF_JOINT_QP", "0"...)` -> default `"1"`. Still
  gated on `A_CAP is not None and A_CAP > 0`; `controller.py`'s `cbf2_filter` call always
  passes `A_CAP=A_CAP, g=g` unconditionally, so this is what actually runs there now.
  `CBF_JOINT_QP=0` still available for A/B.
- `controller.py`: the downstream `PLASMC_AZ_JOINT` block's fixed-hover `theta_cap` re-clip
  (`_cap_eff = arccos(a_z_current/A_CAP)` when `PLASMC_AZ_JOINT=1`, else the fixed
  `self._theta_cap`) is now SKIPPED entirely whenever the joint QP ran (new
  `_cbf_joint_active` gate, same `CBF_JOINT_QP` env default). Reason: the joint QP already
  applies its OWN `a_z`-aware angle clip internally (see the CBF_JOINT_QP section above) --
  a second clip with the fixed hover-assumed `theta_cap` afterward would fight it (redundant
  at best, wrongly TIGHTER than the true deliverable angle when `a_z` sits below hover at
  worst). `PLASMC_AZ_JOINT`/fixed-cap path is preserved for `CBF_JOINT_QP=0` comparisons.

**Re-validation after the flip:**
- `tools/validate_cbf.py`: 12/12 (that harness never passes `A_CAP`, so it always exercises
  the pre-joint theta-based path — unaffected by the default flip, as expected).
- **Process lesson (2 false-alarm "regressions" caught and resolved this session):**
  1. First smoke test used `run_aruco_landing_retry.sh` with no `MARKER_TYPE`/`WORLD` env
     override -> defaults to `MARKER_TYPE=aruco`, which imports `cbf_visibility_aruco.py`
     (a SEPARATE, untouched copy per the project's "ArUco stays comparison-only" rule) --
     `CBF_JOINT_QP` only exists in `cbf_visibility.py` (cross-marker). Result (0/3 SP,
     `theta_cone` up to 2.47 rad) reflected the UNCHANGED ArUco path, not the joint QP at
     all. Fix: `WORLD=cross_marker MARKER_TYPE=cross`.
  2. Second smoke test (correct pipeline) still didn't match the previously-documented 2/3
     SP number (still 0/3, xy_err ~2.7-2.9m) because it omitted `PLASMC_GT_FEEDBACK=1` --
     the standard config every prior IC5 validation in this file used
     (`run_ic_validation.sh IC_LIST="IC5" N_REPS=3 PLASMC_GT_FEEDBACK=1
     WORLD=cross_marker MARKER_TYPE=cross`, sometimes + `CBF_HZ_AWARE_DRIFT=1`) --  a hand-
     rolled `INITIAL_DRONE_ENU=...` loop around `run_aruco_landing_retry.sh` is NOT
     equivalent to that harness.
  3. **Third smoke test, matching the exact prior harness**
     (`HEADLESS=1 WORLD=cross_marker MARKER_TYPE=cross PLASMC_GT_FEEDBACK=1
     CBF_HZ_AWARE_DRIFT=1 IC_LIST="IC5" N_REPS=3 bash scripts/run_ic_validation.sh`,
     `CBF_JOINT_QP` now unset/default-on): **2/3 SP, xy_err 0.018 / 0.018 / 1.574m** --
     matches the pre-flip validated result (rep1/rep3 land essentially on-target, rep2 the
     same ~1.2-1.6m single miss class seen throughout this thread). Confirms the default
     flip is behavior-preserving relative to the explicitly-enabled config.
- **Takeaway for future comparisons in this codebase**: always reproduce the EXACT
  launcher + env combination a prior number was measured with
  ([[feedback_check_concurrent_sitl_before_launch]]-adjacent lesson: mismatched harnesses
  looks exactly like a regression). `run_ic_validation.sh` (not a hand-rolled loop) +
  `PLASMC_GT_FEEDBACK=1` + `WORLD=cross_marker MARKER_TYPE=cross` is this project's
  canonical IC-validation invocation.

## Open item / natural next step

The angle-clustering fragility itself is NOT fixed -- `_cluster_line_angles`'s
`merge_tol_deg=12` (or another parameter in that path) likely needs to be less brittle
under high viewing-angle tilt, OR `landing_test.py`'s grace/abort logic needs to tolerate
IC5-class steep-angle noise better. Neither attempted this session; this memory documents
the confirmed root cause only, not a fix.
