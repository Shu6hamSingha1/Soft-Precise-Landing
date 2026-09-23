---
name: feedback_cho2022_never_lands_rootcause
description: "cho2022 (FF-IBVS baseline) 0/10 landed root-caused: it converges to a stable hover ~0.5m above the surface (its own fixed depth setpoint) and never continues into physical ground contact; baseline runs also bypass PLASMC's loom-inversion touchdown latch entirely, leaving only PX4-native contact detection as the (unreached) landing signal."
metadata:
  type: project
  modified: 2026-09-23T04:10:00.000Z
---

2026-09-23, user asked to investigate why [[project_20260923_comparative_baseline_campaign]]'s
cho2022 baseline hits an identical descent-stall abort on all 10/10 recorded cases.
Root-caused by tracing code, not by re-running experiments first (confirmed after with
one).

**Chain of evidence:**

1. **The control law is a pure feature-error regulator with a fixed, shallow depth
   setpoint.** `src/baselines.py::Cho2022.step()`: `vd = -diag(lambda_ibvs) @
   pinv(Ls) @ (Pd - Pi)`. `Pd` (`controller.py::_baselineStep`, `s["px_d"] = (135.0 /
   (2*0.2)) * key[:2]`) is a FIXED target corresponding to a ~0.4m camera-to-marker
   depth (the code's own comment: "0.4 m desired depth == 0.1 m camera-marker
   touchdown"). Once `Pi -> Pd`, feature error -> 0, `vd -> 0`, `I_a` settles to
   EXACT hover. Verified directly in `Control_Data.npy` (CHO2022-GT/IC1): `I_a_z`
   converges to -9.81 m/s^2 (B_T mean 0.055, essentially hover) by t~9s and holds
   there for the remaining ~23s. GT altitude confirms: descends 5m -> 0.497m by
   t=8.86s, then flat +-2mm for the rest of the flight (matches the montage-bug
   investigation's [[feedback_montage_touchdown_argmin_bug]] finding of a long flat
   hover plateau).
2. **Baseline runs skip `self.PLASMC()` entirely — including ITS touchdown
   detector.** `controller.py`'s main loop: `if self._baseline is not None:
   self._baselineStep() else: self.PLASMC()`. The loom-inversion touchdown latch
   (`_touchdownDetect`/`_touchdownDetectV2`, sets `self._touchdown` ->
   `TOUCHDOWN_DETECTED`) lives INSIDE `PLASMC()` and reads PLASMC-internal state
   (`_s_e_n`, `_h`, `_s_dot_meas`) a baseline run never populates (confirmed: those
   Control_Data fields are empty arrays for baseline recordings). This applies to
   ALL 4 baselines, not just cho2022 -- it's just that the other 3 sometimes trigger
   the fallback below anyway.
3. **The only remaining landing signal is PX4-native, and both paths require GENUINE
   PHYSICAL CONTACT:** `src/flight_controller.py` sets `FC_node.LANDED=True` via
   either (a) PX4's own `LandedState.ON_GROUND` telemetry (needs sustained
   near-ground low thrust) or (b) an accelerometer impact spike >50 m/s^2 (a real
   collision, 15.8-900 m/s^2 range in this sim). Since cho2022 parks in stable hover
   at essentially exact hover-thrust rather than continuing to push into the ground,
   NEITHER fires -- correctly, since the vehicle genuinely is still airborne, just
   close to the surface.
4. **Why lin2022/zhang2026/lin2023 sometimes land and cho2022 never does:** those
   baselines' control laws keep commanding descent authority PAST hover and punch
   through hard enough to trigger the accelerometer spike (zhang2026 lands with
   rel_vel up to 18.5 m/s -- consistent with "hits the ground hard", not "settles
   gently"). cho2022's smooth regulation-to-zero-error law never produces that
   forcing.

**Experiment (2026-09-23, same session): tried lowering the depth-target constant.**
Made it env-tunable (`controller.py::_baselineStep`, `BASELINE_CHO_DEPTH_TARGET`,
default `0.2` == the original hard-coded `0.4m` target depth) and re-ran
`cho2022/IC1` with `BASELINE_CHO_DEPTH_TARGET=0.05` (target depth 0.1m instead of
0.4m). **Result: it landed** (`SoftPrecise.xy_err=0.333m, rel_vel=0.670m/s,
min_alt=0.523m`) -- confirming the mechanism diagnosis (a deeper equilibrium forces
continued descent past the old parking point).

**But NOT a clean fix -- read the actual B_T trace before citing this as "solved":**
`Control_Data.npy` shows `B_T` (thrust deficit) blowing up to numerically absurd
values right before touchdown (`-2,971,096` at t=8.54s, `+2,949,884` at t=10.34s) --
a near-singular `pinv(Ls)` interaction-matrix inversion, not a controlled push-through.
The "landing" was registered because that blow-up produced a violent enough motion to
trip the accelerometer impact detector, essentially by accident, not because the
shallower target genuinely fixed the control law's convergence behavior. `min_alt`
(0.523m) is barely different from the original ~0.497-0.499m plateau -- it's touching
down at almost the SAME altitude as before, just via a numerical spike instead of a
smooth approach. This matches the "~0.497m is where controllers already plateau, the
missing piece is momentum/overshoot not depth" caveat predicted before running this --
**confirmed, not refuted**. A genuine fix would need to address why `Ls` goes
near-singular / why the law has no overshoot mechanism, not just retarget the setpoint.
Left `BASELINE_CHO_DEPTH_TARGET` as an env-tunable default-off knob for further
investigation; the campaign's recorded cho2022 data (0/10 landed) is UNCHANGED --
this was a one-off diagnostic rep, not a re-recording of the dataset.

**Follow-up investigation (2026-09-23, same day): traced the B_T blow-up mechanism
precisely (not just "near-singular", the actual chain).** Quantified with real numbers,
not hand-waving:

1. `Ls`'s condition number DOES grow substantially as z shrinks (rebuilt Ls with the
   code's actual 5-point marker geometry: cond=10 at z=0.5m -> cond=434 at the code's
   own z=0.01m floor, a ~40x growth) but feeding this through pinv with plausible
   pixel errors alone only produced sub-1-m/s velocities -- NOT the observed millions.
   `Ls` growing ill-conditioned is real but not sufficient on its own.
2. **Found a genuine latent bug along the way:** `Cho2022.step()`'s `use_sq_comp`
   safety path only fires `if N==4`, but `marker_key_points()` always returns 5 points
   (4 arm tips + 1 asymmetric stub at x=2.29, >2x outside the square's +-1.10
   half-width) -- so `use_sq_comp=True` in `K_CHO2022` is DEAD CODE, never actually
   applied. Doesn't explain the blow-up by itself but is worth fixing regardless.
3. **The actual smoking gun is in `accel_to_rate_thrust` (src/baselines.py):**
   `B_T = mass*(I_a_cd[2]+G) / max(cos(roll),1e-6) / max(cos(pitch),1e-6)` where
   roll/pitch are the VEHICLE'S ACTUAL REALIZED ATTITUDE (not the desired one) and
   the guard floor is only `1e-6` -- far too permissive. Back-of-envelope check
   against the real flight data: at t=8.54s, I_a_z=-11.2 -> (I_a_z+g)~-1.4, mass~1.5kg
   -> numerator ~-2.1; divided by the 1e-6 floor gives ~-2,100,000, closely matching
   the observed -2,971,096.

**Mechanism, end to end:** as z shrinks near touchdown, Ls's growing ill-conditioning
amplifies noise/error into an increasingly aggressive commanded acceleration
DIRECTION (Rd=-I_a/|I_a| can swing sharply even for a modest-magnitude I_a change).
The attitude-rate loop chases that abrupt direction change aggressively; if the REAL
vehicle actually tips toward gimbal-lock during that chase, B_T's division by
cos(roll)*cos(pitch) of the ACTUAL attitude explodes -- a genuine near-flip event
triggered by the depth-scaled IBVS gain growing right at the most sensitive moment,
not a benign numerical artifact. This is architecturally UNIQUE to cho2022 among the
4 baselines: lin2022/zhang2026 are PBVS (no image-Jacobian depth division at all),
lin2023's IBVS uses area-RATIO moment features (self-normalizing, depth-invariant by
construction, no direct 1/z division) -- cho2022 is the only one using a raw
classical image Jacobian with explicit, barely-guarded division by shrinking z.

Not fixed in this session (diagnostic only, per user's "investigate" request, not
"fix"): candidates for a real fix would be (a) an actual B_T saturation/cap instead of
the 1e-6 floor, (b) a sign-preserving z clamp, (c) actually wiring use_sq_comp to
apply regardless of N (or dropping the stub point for Cho2022 specifically), (d) a
depth-independent (moment/ratio-based) reformulation matching lin2023's approach.
