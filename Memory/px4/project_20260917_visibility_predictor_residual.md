---
name: project_20260917_visibility_predictor_residual
description: "Multi-session thread (2026-09-17 to 09-22), five major results, in order: (1) visibility-CBF predictor residual measured; (2) a TRANSPOSED phi axis bug found+FIXED+BAKED in the visibility CBF (96271ba6), SITL-validated 14/14; (3) the touchdown-detect flow-freeze false-positive root-caused+FIXED+BAKED (2177670b), SITL-validated 22%->0%; (4) six candidate mechanisms tested for the perceived-h_z terminal 'divergence' -- five ruled out/insufficient, and the sixth (dt/fps) turned out to be the answer; (5) CLOSED 2026-09-22: the ~5.6x reconstruction gap that drove all this mechanism-hunting was a bug in the INVESTIGATION'S OWN offline replay tooling, not the live controller -- every replay tool computed the raw flow solve's dt as Time[i]-Time[i-1], but process_frame() actually uses dt=1/fps, which differs by 3-8x in the terminal window (polling-loop-vs-native-camera-rate decoupling). Fixed a dead FPS/AngVel/Stamp logging path (a1ffbf02, was never wired since 08-12), got fresh recordings including a genuine large spike (IC1_rep3, KF ramps -0.21->-0.68), and the correct-dt reconstruction now matches logged h_V_z to <2% throughout, including at the spike. N_z adaptive-law tuning remains correctly ABANDONED. (6) ANSWERED 2026-09-22: the terminal h_z ramp is a REAL perception error (confirmed vs independently-computed GT loom via gt_optical_flow.py -- GT stays bounded/decelerates near touchdown, measured h_z overshoots by up to 2.4x), correlating tightly with marker overfill (MARKER_EXTENT_PX frozen at 318px > the 240px frame_min threshold) and a ~2x rise in flow-solve rel_resid (poor rigid-body model fit), NOT with near-grazing rays or ill-conditioning (both stay healthy in this window) -- and the error direction is NOT consistent (overshoot in one rep, undershoot in another with the same frozen extent), ruling out a simple sign-bias fix. THREE candidate fixes identified but NOT YET tested: terminal hold/clamp on h_z at overfill, enabling/tuning the already-existing CROSS_SCALE_RATE_FUSE (default off) which may already be positioned to help, or improving the rigid-body fit itself at overfill. See the SESSION CLOSE section for the full index."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6f7de16e-4b89-4098-aff3-6ef2d19e558b
  modified: 2026-09-22T04:46:55.883Z
---

## ===== 2026-09-22 (cont'd) -- dead FPS/AngVel/Stamp logging found+fixed, 6th mechanism untestable not ruled out =====

**Resumed the soft-touchdown investigation** ("Investigate to find the Soft-touchdown
root cause"), picking up the two untried candidates flagged at session close: the sensor
calibration matrix's end-to-end effect, and a `dt`/jitter discrepancy between the offline
replay and the live controller's actual per-frame timing.

**Sensor-cal candidate: RULED OUT cleanly.** Traced `getOptFlowAngVel()` = `_sensor_cal_hw
@ getRawOptFlowAngVel()` (line ~3180) -- the cal gain is applied in the GETTER, i.e.
downstream of `self._hw` (the coast+freeze KF's own state). But `Img_Data.npy`'s logged
`"h_V"` field is `self._perception._hw_log`, populated from `self._hw` directly (line
~3020), with its own inline comment confirming "optical flow (raw, before cal)". So the
logged `h_V_z` this whole thread has been comparing against is ALREADY raw/uncalibrated --
the same units every replay tool (`replay_hw_kf.py`, `replay_hw_kf_gyro.py`,
`replay_flow_solve_conditioning.py`) has been producing. No unit mismatch, no cal-gain
explanation possible for the ~5.6x gap. This closes the sensor-cal candidate definitively,
not just "dismissed as small."

**dt/jitter candidate: found a real bug, but it makes the hypothesis UNTESTABLE
retroactively, not ruled out.** `process_frame(img_prev, img_curr, t, fps, ...)`
(cross_marker_perception.py ~2714) computes `dt = 1.0/fps` and uses THAT to divide pixel
displacement into velocity for the raw flow solve -- NOT `t - prev_t` from consecutive
calls. Every replay tool in this thread instead used `Img_Data["Time"][i] -
Img_Data["Time"][i-1]` (the log's own consecutive timestamps), because that's the only
dt available -- `Img_Data["FPS"]` reads `getattr(self, '_pending_fps', np.nan)`
(~line 3118), on a comment claiming `CrossMarkerNode.run()` sets `self._pending_fps` "just
before calling process_frame()". **Grepped for the assignment: it does not exist anywhere
in the file.** Same for `_pending_angvel` (feeds the already-known-dead "IMU AngVel" log
field from earlier in this thread) and `_pending_stamp`. All three have been silently dead
since the 2026-08-12 dt/frame-pairing rewrite -- `process_frame` receives `fps`/`angvel_*`/
`t` as its own direct call arguments and never stored them back onto `self._pending_*`.

Checked one specific rep (`ICValidation/20260921-144320/IC1_rep1`, the exact rep this
thread's `-0.771` KF number came from): `Img_Data["FPS"]` is NaN for all 1347 frames,
confirming the dead path there directly. (A DIFFERENT, older rep,
`ICValidation/20260917-224720/IC1_rep1`, showed a constant `62.5` instead of NaN -- not
live per-call data either given the confirmed-absent assignment; some other stale/constant
source, not verification of the hypothesis. Do not treat that number as real.)

**This means the dt/jitter hypothesis was never actually tested in this thread** --
every reconstruction to date implicitly assumed `dt_replay == dt_live`, and that assumption
itself was unverifiable with the logging as it stood. It remains a live, untested candidate
for the ~5.6x gap, not a ruled-out one.

**Fix applied** (`src/cross_marker_perception.py`, top of `process_frame`): sets
`self._pending_fps = fps`, `self._pending_stamp = t`, `self._pending_angvel = angvel_curr`
from the call's own real arguments, so `Img_Data["FPS"]`/`["Stamp"]`/`["IMU AngVel"]` will
finally hold real values on the NEXT recording. Pure logging fix -- does not touch any
control/perception math, `process_frame`'s dt computation is unchanged, just now observable.
Compiled clean (`py_compile`). **NOT yet SITL-validated** -- a peer session (`soft-precise-
landing-53`) was running a headless rover SITL gate and asked to hold SITL at the time this
fix was made, so no new recording was taken this session. `_kf_step`'s own internal dt
(`t - prev_t`, general-purpose, line ~422) is unaffected either way -- only the RAW
per-frame flow solve's dt was ever in question.

### Next step for whoever picks this up
Get ONE new perception-mode IC1-5 recording (any WORLD=cross_marker MARKER_TYPE=cross run
is enough, doesn't need to be a full gate) with this fix in place, then compare
`Img_Data["FPS"][i]` against `1.0/(Img_Data["Time"][i]-Img_Data["Time"][i-1])` directly in
the terminal touchdown window. If they diverge meaningfully (as the OLD, unverifiable
62.5-constant rep hinted they might, at a ~2x ratio in one spot-check before this fix), redo
`replay_flow_solve_conditioning.py`'s raw solve using the REAL logged `dt=1/fps` instead of
`Time` deltas and see if that closes some/all of the ~5.6x gap. If they match closely, this
6th candidate is also ruled out and the mechanism remains genuinely open.

## ===== SESSION CLOSE 2026-09-22 -- READ THIS FIRST =====

Four threads, in chronological order. Each has its own detailed trail below (headed by
`##` date-stamped sections) -- this block is the map, not a replacement for it.

### 1. Visibility-CBF predictor residual (2026-09-17) -- DONE, informational
Measured the visibility CBF's one-step predictor (`c_next = r~ + L_e*dy`) against
realized centre motion, IC1-5 gate, 25 reps. Meaningful horizon is ~125-144ms (the
attitude-realization time), not one control step -- at 1 step the predictor is WORSE
than assuming no motion. Buffer `b=0.15` covers the bulk (p95) but not the tail (p99/
p99.9 exceed it on 1.2% of frames). `tau*d` (the moving-target lead) is a ~10% MEDIAN
correction only, does not improve the safety-relevant tail. `tau` is correctly understood
as a PLANT property (the realization horizon), not a scenario one -- applies to
stationary targets too.

### 2. Visibility-CBF axis-transposition bug -- FOUND, FIXED, BAKED (`96271ba6`)
`marker_tangent()` applies a `_SWAP` to the measured centre `c`; `fov_limit()` did NOT
apply the same swap to the barrier `phi` -- one image axis's barrier sat OUTSIDE the
physical sensor edge (permanently inert), the other was 36% over-tight. Found while
implementing a per-axis buffer (§1's own follow-up). Fixed at 3 sites (the CBF itself,
`controller.py`'s drift-off trigger, and the validator's own oracle -- the oracle carried
the SAME transposition, which is why "15/15" never caught it). SITL-validated:
WORLD=cross_marker MARKER_TYPE=cross explicit (a §19-class trap independently found the
same session), PD-FB, IC1-5 gate -- 14/14 genuine touchdowns (`terminal_state_ok`), no
regression vs the pre-fix baseline (11/14 vs ~17/21 precise, both ~79-81%).
**Retracted along the way:** a proposed "degenerate deliverability ball" fix (`y_max=0`
when `a_z>=a_cap`) -- implementing it broke the validator's deliverability-by-construction
checks; `y_max=0` there is the mathematically CORRECT answer (the feasible set is
genuinely empty), not a bug.

### 3. Touchdown-detect flow-freeze false-positive -- FOUND, FIXED, BAKED (`2177670b`)
Found while validating fix #2: the IC1-5 gate showed 4/18 reps (22%) never reaching the
surface at all (a live-reporting gotcha caught along the way -- `run_ic_validation.sh`'s
`landed` column means "a recording was saved," not "touchdown occurred"; the authoritative
field is `Ground_Truth.npy`'s `SoftPrecise.terminal_state_ok`). All 4 false touchdowns
fired via the SAME path (`_touchdownDetectV2`'s flow-freeze), root-caused to THREE
independent defects: (a) `ff_hi`/`ff_lo` were hardcoded px thresholds sitting INSIDE the
normal background-flow noise floor at the current focal length (51-81% of ordinary
frames already below `ff_lo`); (b) no confidence gate on the flow solve (unlike its
sibling `condition_drift`, which already gates the same `rel_resid` signal); (c) no LIVE
check the marker is actually absent -- `_td_ext_armed` is a stale one-time flag, so the
path could (and did) fire while the marker was continuously tracked, directly violating
its own documented purpose ("catches a soft OFF-marker settle"). Fixed all three
(resolution-invariant tangent units, `rel_resid` confidence gate, `FEATURE_IS_VISIBLE`
live-visibility gate -- NOT a minimum-extent gate, which was considered and rejected: a
genuine off-marker settle has extent -> 0, so requiring HIGH extent would exclude the
real target case). SITL-validated: 25/25 genuine touchdowns, ZERO flow-freeze firings,
22%->0% false-touchdown rate. Self-audited against the `diagnose-flight-data` skill right
after pushing (timestamp-verified the trigger-frame matches, checked for frozen-field
artifacts) -- audit confirmed rather than overturned the fix.
**Open, not done:** flow-freeze's OWN theoretical niche (genuine soft off-marker settle)
still has ZERO positive evidence across 43 combined reps now -- the fix removes a bug, it
does not prove the path earns its ongoing complexity. Worth revisiting once the rover
thread can produce a genuine off-marker settle to test against.

### 4. Soft-touchdown investigation -- OPEN, root cause only partially identified
User's premise ("GT-FB achieves soft touchdown, so tune the vertical adaptive law") was
RIGHT to push on -- an earlier framing in this thread ("soft touchdown looks structural")
was WRONG and is the exact mistake `feedback_dont_conclude_lag_floor` exists to prevent
(a masked failure is a tuning target, not proof of an architectural ceiling).

- **Confirmed the premise**: GT-FB, `h_rd=-0.38` (MATLAB's then-current value), IC1 n=3:
  3/3 soft+precise, rel_vel 0.014-0.023 m/s (10x under threshold).
- **N_z (kappa-ODE adaptation rate) explored, then correctly ABANDONED before its planned
  gate**: offline kappa-ODE replay confirmed a real, too-slow-to-respond mechanism
  (17.4x disturbance in 10% of kappa_z's own tau); small-n live trial (N_z=0.3) showed no
  ratchet but a weak, statistically-lost-in-noise effect. Abandoned once the REAL driver
  was found (below) -- a faster-responding kappa reacting to an already-wrong signal
  would apply an even LARGER erroneous correction, not a softer landing.
- **Parallel peer session (`7476400a`, MATLAB) independently converged on the SAME
  kappa-sensitivity mechanism**, went much further with a coordinated N/Pleak/E/chi_z/
  p_hinf retune (25-IC gate, 25/25 SP), and REVERTED h_rd back to -0.30 (PX4's original
  value) with that retune in place -- superseding the h_rd=-0.38 recommendation. Neither
  retune is yet ported+validated on PX4 (gain VALUES don't port directly,
  `feedback_matlab_gains_not_portable`) and neither addresses the perception-side finding
  below, which is PX4-real-camera-specific and has no MATLAB analog.
- **Root-caused (partially) the actual mechanism**: perceived `h_z` genuinely diverges
  from ground truth in the terminal ~150-450ms before touchdown (confirmed via
  `tools/gt_optical_flow.py`, Z_REG-regularized, sync-verified -- GT loom stays smooth
  and even flares naturally, matching clean GT-FB; perceived `h_z` diverges to ~1.6x the
  true value over the same window). Cross-validated by a second, independent signal
  within the SAME pipeline (`Width Loom Rate`/`Scale Loom Rate`, a different sensing
  principle, ALSO stay flat through the identical window).
- **Five candidate mechanisms tested for WHY it diverges, in order, each via direct
  offline reconstruction against real recorded data (not inferred from correlation
  alone)**:
  1. Matrix ill-conditioning (`cond(A)`) -- FALSIFIED. Stays modest (7-16) throughout,
     both a gradual-divergence rep and a sharp-spike rep.
  2. Near-grazing-ray perspective-divide amplification (small `z_v`) -- REAL, confirmed
     directly (a mechanism the code's own 2026-08-02 comment predicted but never
     confirmed), but does NOT survive point-exclusion testing: filtering out the
     grazing points (`CROSS_Z_V_MIN_FLOW`, isolated from the unrelated
     `CROSS_FLOW_ANG_MAX` knob that backfired before) barely changes the sharp-spike
     case and is a complete no-op for the gradual case (z_v never gets low enough
     there). Retracted as a proposed fix.
  3. KF constant-velocity rate-buildup -- REAL, directionally confirmed by replaying
     `_kf_step`'s exact math, but only accounts for ~1/5.6 of the observed magnitude,
     even after correcting to the RIGHT solve path (see next item).
  4. Loom R-schedule / scale-rate fusion / hard loom backstop / loom innovation gate --
     all RULED OUT directly against already-logged fields (each inactive by default in
     this data, or the innovation gate's slew-based trigger is the wrong shape for a
     gradual ~450ms ramp vs the single-frame spike it's built to catch).
  5. Gyro-availability for the reduced 4-unknown solve -- RESOLVED (gyro was live the
     whole flight, confirmed via `Telemetry_Data.npy`'s `Angular Velocity FRD`, not
     inferred from a dead log field) -- but redoing the KF replay with the CORRECT
     solve path barely changed the reconstruction, so this was not the missing piece
     either.
- **Net: none of the five fully explains the divergence.** ~5.6x of the observed
  magnitude remains unaccounted for after the most careful reconstruction attempted.
  Flagged to the user as diminishing returns on mechanism-hunting at this layer.

**Tools committed this thread** (all read-only, reproducible, documented with their own
replication scope/limits): `tools/scan_vis_safeset.py`, `measure_vis_predictor_residual.py`
(thread 1); `tools/replay_touchdown_flowfreeze_gate.py` (thread 3);
`tools/replay_flow_solve_conditioning.py`, `replay_zvmin_filter.py`, `replay_hw_kf.py`,
`replay_hw_kf_gyro.py` (thread 4's mechanism-hunting).

**What's genuinely open for a future session:**
- Thread 4's root cause remains ~5.6x unexplained. Untried candidates: the sensor
  calibration matrix's end-to-end effect with the KF in the loop (dismissed early as
  "too small" via its diagonal value alone, never rigorously verified with the KF
  active); a `dt`/jitter discrepancy between this offline replay and the live
  controller's actual per-frame timing.
- A practical mitigation not requiring full mechanism attribution may be more tractable
  than continuing to chase it: e.g. damping the KF's rate-state growth specifically in
  the terminal/high-extent window, or a simple proximity-triggered hold/clamp on `h_z`.
- The coordinated MATLAB adaptive-law retune (`7476400a`) is not yet ported or
  SITL-validated on PX4.
- Thread 3's flow-freeze path still has zero positive evidence for its own stated
  purpose, across 43 combined reps.

---

**Stating positively what the 2026-09-17 audit block left implied** (peer
`soft-precise-landing-42` asked for this, correctly: if τ equals the attitude-realization
horizon then it is a **plant property, not a scenario property**, so framing
`CBF_DRIFT_TAU=0.15` as "for moving targets" is mis-framed — the stationary case has
self-motion flow too). Continues [[project_20260909_visibility_projection_wire_in]].

## Method (the alignment is proven, not assumed)

`Img_Data` runs longer than `Control_Data` (image node has its own cadence: 1306 vs 1168
samples on IC1_rep1) — truncating to `min` silently misaligns, the
[[feedback_recurring_analysis_mistakes]] §1 trap. Instead: interpolate `Img_Data.Quat`
onto the control clock `Control_Data["t"]` via `Img_Data["Time"]`, renormalise, and
**self-check** by reconstructing `arccos(R33)` and comparing to the independently-logged
`theta_current(t)` → p50 err **0.013°**, p99 **0.33°**. Frames with a >40 ms nearest-sample
gap (1.9%) are dropped. Any future offline use of `Quat` against control-rate logs should
carry this same self-check.

## 1. The predictor's horizon is the realization time, not one control step

Residual of `ĉ = c + L_e·Δy_now` vs the realized centre, by horizon (IC1-5, ~24k pairs):

| horizon | rot-only p95 | null (no-motion) p95 |
|---|---|---|
| 1 step (~10 ms) | 0.0183 | **0.0152** |
| ~125 ms | 0.0776 | 0.0830 |
| ~144 ms | 0.0841 | 0.0915 |

Over one control step the attitude barely moves, so `Δy ≈ 0` and `L_e` mostly amplifies
attitude noise — **the model is worse than doing nothing there.** It only earns its keep at
≳125 ms. Measured attitude-realization horizon (high-passed lean-command → realized-tilt
cross-correlation) is ~144 ms, IQR 112-288.

**This is NOT a safety hole for the rotational term** (an earlier framing of mine, corrected):
the QP bounds the centre at the *fully-realized* lean, and the realized path travels the
segment `c → c_next`, so by convexity of the box every intermediate state is inside too. What
is *not* on that segment is translation during the realization window — which is where the
residual actually lives.

## 2. `b = 0.15` covers the bulk, not the tail

At the realization horizon, per-axis buffer `b·R/(2f) = [0.133, 0.178]`:

| predictor | p50 | p95 | p99 | p99.9 | % frames over buffer |
|---|---|---|---|---|---|
| null | 0.0219 | 0.0915 | 0.2133 | 0.3525 | 1.653% |
| rot-only | 0.0194 | 0.0841 | 0.1941 | 0.3074 | **1.235%** |
| rot+drift | 0.0169 | 0.0807 | 0.1968 | 0.2974 | 1.319% |

p95 sits comfortably inside the buffer; **p99 and p99.9 both exceed it.** Consistent with the
independently measured 0.27% buffered-set exits and 0% sensor exits — the buffer absorbs the
bulk, the box is breached occasionally, the sensor never is. A residual-aware (per-axis,
possibly altitude-dependent) buffer now has a number behind it rather than a guess.

## 3. `τ·d` is a modest MEDIAN correction — not a tail fix ⚠

Reduces residual on **54.1%** of frames, **mean 9.9%**, p50 0.0194 → 0.0169 — but the
over-buffer rate goes **1.235% → 1.319%**, i.e. **no improvement where it matters.**

⚠ **Scope correction to how this got recorded elsewhere.** `5a5fc2a6` (peer) records the
`gT²/(6Z)` framing as "a better justification" for τ=0.15. Correct about τ's *scale*;
overstated about its *benefit*, on two counts:
- The `gT²/(6Z)` shortfall is an exact geometric quantity (0.6% of predicted displacement at
  Z=5 m, 6% at Z=0.5 m) but it is **small compared to the measured total residual** — so the
  translational term is **not the dominant residual source**. Measurement noise and unmodelled
  dynamics dominate. That is exactly why correcting it buys only ~10%.
- Benefit is median-only and does not touch the tail.

Honest scope of the check: used raw `h[:2]` and actual elapsed `dt`, **not** the
`condition_drift`-conditioned `d` at fixed τ — a proxy for the implemented term, not the term
itself.

**Net:** keep `CBF_DRIFT_TAU=0.15`; justify it as *the plant's attitude-realization horizon*
(applies to stationary and moving alike, no scenario framing); claim a ~10% median
prediction improvement and **no** tail/safety improvement. Do not cite it as closing a
moving-target gap ([[feedback_dont_judge_cbf_by_sp]]).

---

## CORRECTION 2026-09-17 (same day): `phi` is TRANSPOSED against `c` in the live code

**Found while trying to implement a per-axis buffer.** `marker_tangent()` applies
`_SWAP = [[0,1],[-1,0]]`; `fov_limit()` does **not** apply it to the intrinsics. So:

    c[0] = +(y_px - cy)/f  -> spans +-cy/f = +-1.185   (the 320-tall axis)
    c[1] = -(x_px - cx)/f  -> spans +-cx/f = +-0.889   (the 240-wide axis)
    phi  = CENTER/focal*(1-b) = [0.889, 1.185]*(1-b)   <-- NOT reversed

The physical half-extent in `c`'s own axis order is `CENTER` **reversed**, `[1.185, 0.889]`.
Consequences, both live:
- **axis 1 barrier is INERT**: `phi_1 = 1.007` at `b=0.15` is OUTSIDE the physical edge
  `0.889` -- on that image axis the constraint cannot bind before the marker has left.
- **axis 0 over-tight by 36%**: `phi_0 = 0.756` against a true edge of `1.185`.

Airtight from `center = _resolution/2 = (120,160)` on the 240-wide x 320-tall rotated frame.
Data agree asymmetrically: `|c[0]|` reaches 1.504 and exceeds 0.889 on 0.92% of frames;
`|c[1]|` exceeds it on 0.19% and never passes 1.124.

**Both my tools inherited it** (they mirrored the code). Fixed in `tools/scan_vis_safeset.py`
and `tools/measure_vis_predictor_residual.py`; v1 copies in `Obsolete/tools/*_v1_transposed_phi.py`.

### What changed in the recorded numbers

| quantity | as first recorded | corrected |
|---|---|---|
| IC1-5 sensor exits | 0.00% | **0.00%** (unchanged -- headline survives) |
| IC1-5 buffered-box exits | 0.27% | **1.11%** |
| rover raw sweep, off / lead | 2.24% / 2.10% | **0.45% / 0.28%** (lead better) |
| rover conditioned, off / lead | 0.47% / 1.36% | **0.29% / 0.63%** (lead worse) |
| residual %>buffer, rot / drift | 1.235% / 1.319% | **1.711% / 1.619%** |
| per-axis b to cover p99 | [0.218, 0.164] | **[0.164, 0.218]** (I had it backwards) |

Residual *quantiles* are unaffected (norms of residual vectors, independent of `phi`):
p50 0.0194 / p95 0.0841 / p99 0.1941 / p99.9 0.3074 all stand, as does the 1-step-vs-null
result and the ~144 ms horizon.

**Two conclusions flip:**
1. `tau*d` now **slightly improves** the over-buffer tail (1.619% vs 1.711%), where the
   transposed numbers made it look slightly worse. §3 above overstated the case against it --
   still only ~10% median, but it is no longer "no tail improvement".
2. The rover sweeps now **disagree in direction** (raw: lead better; conditioned: lead worse).
   That is what n=2/cell noise looks like, and it **reinforces** rather than weakens
   [[project_20260909_visibility_projection_wire_in]]'s one-armed-statistic finding: the
   same-metric comparison does not support "tau*d closes the moving-target gap" in either
   direction. The 278-frame figure being a tau=0-arm-only count is unaffected -- that was
   about which arms were compared, not the extents.

**Fix priority: this outranks both the `y_max=0` degenerate-ball defect and any buffer
re-sizing** -- a per-axis `b` is meaningless until the box is on the right axes, and one axis
of the guarantee is currently not running. Preferred fix is to reverse the intrinsics in
`fov_limit()` (keeps `c` in the frame the `h_xy` identity-map was validated against) rather
than touching `marker_tangent()`. It is a genuine behaviour change -- it activates a
previously-inert constraint -- so it needs the IC2-5 gate, with `vis_slack` watched.

### FIXED 2026-09-17 — `96271ba6`, three sites, and why "15/15" never caught it

1. `src/visibility_projection.py` `fov_limit()` — reverse the `center/focal` quotient.
2. `tools/validate_visibility_projection.py` — **the reason it survived validation.** The
   validator passed `CENTER=[160,120]` (reversed vs what `controller.py` passes) and set
   `SENSOR = CENTER/FOCAL`, transposed for its own geometry too. **Module and oracle carried
   the SAME error**, so they agreed with each other while both disagreed with the real
   camera. Fixed module + fixed oracle = 15/15 on seeds 0-4; fixed module + old oracle =
   10/15 (checks 1 and 12 fail). → **An independent validator that shares the code's
   convention is not independent.** When an oracle hard-codes intrinsics, check them against
   what the live caller actually passes.
3. `src/controller.py` `_p_10_tan` — was deliberately un-reversed with the comment
   "marker_tangent applies its own [y,-x] swap". Backwards: `c` being swapped is *why* the
   half-extent must be swapped to match. The drift-off pull-back therefore fired early on one
   image axis and could never fire on the other.

**`y_max=0` "degenerate ball" fix RETRACTED** (was listed as the top defect before this).
Implementing it failed validator checks 10/11 and the checks are right: at `a_z >= a_cap`,
`||a*|| = a_z*sqrt(1+||y||^2)` exceeds `a_cap` already at `y=0`, so the feasible set is
genuinely EMPTY and `y_max=0` is the correct answer. Widening the ball manufactures a lean
the vehicle cannot deliver and breaks deliverability-by-construction. The infeasibility is in
the CALLER's `a_z` — a caller-side clamp before the solve is the principled fix, but it
modifies the vertical channel and breaks the two-tier separation, so it needs its own
decision. Reasoning left in-code so it is not retried.

**Still open:** IC2-5 gate (the fix activates a previously-inert constraint — expect
`vis_active`/`vis_slack` to rise; slack going routinely non-zero is the tripwire). Buffer
re-sizing deliberately deferred until AFTER the gate: per-axis effective margins change with
this fix, so `b` must be re-derived on top of it, not alongside.

---

## IC2-5 gate run 2026-09-17 — 0/25 precise: NOT the axis fix, a perception-layer environment collapse

Ran the mandated IC2-5 gate for `96271ba6` (N_REPS=5, HEADLESS=1, cross-marker):
`test_data/ICValidation/20260917-224720`. Result: **0/25 precise, 0/25 soft**, IC2-5 mean xy
1.3-1.8 m (vs the Sep-12 bundle used for the residual analysis, `20260912-040029`: 16/25
precise, xy mostly 0.01-0.15 m). Looked catastrophic at first read.

**Before attributing this to the fix, validated the bisect endpoint** — this project's own
hard-learned rule ([[feedback_recurring_analysis_mistakes]] §10-15,
[[project_20260916_curve_qgate_revalidation]]/`e7882829`: "a worktree rebuilds CODE not the
EXPERIMENT... out-of-repo camera SDF"). Same-day, same-environment, interleaved A/B on IC2
(5 reps/arm, `git worktree` at `4ba07bb8` = `96271ba6^` for OLD, `LANDING_OUT_BASE` set per
arm per the `run_visproj_gate.sh` autosave-collision lesson):

| arm | xy_err (5 reps) | precise |
|---|---|---|
| OLD (pre-fix code, today) | 2.36, 0.99, 0.87, 1.09, 3.67 | 0/5 |
| NEW (fixed code, today) | 2.69, 2.71, 2.39, 2.61, 2.31 | 0/5 |

**OLD code fails exactly as badly as NEW, today.** The `96271ba6` fix is NOT implicated.

Root cause traced one level further — **marker-alive rate** (`N Flow Corners > 0` fraction)
per rep:

| | Sep 12 (good) | Sep 17 OLD | Sep 17 NEW |
|---|---|---|---|
| marker-alive % | **100.0%** every rep | 25.9 / 78.6 / 91.9 / 40.8 / 25.5 | 23.6 / 23.9 / 56.1 / 28.6 / 23.5 |

**This is a perception-layer collapse present identically in both code versions.** Something
in the environment (Gazebo world state, marker rendering, camera plugin, lighting — not yet
isolated) degraded between 2026-09-12 and 2026-09-17, independent of any `src/` change.
Camera SDF checked and unchanged (320x240, hfov 1.74) at the time of this test; no other
SITL/PX4/bridge process was running before either test. Not yet root-caused further — this
smells related to the still-unexplained out-of-repo state that caused the curve-cycle
mystery ([[project_20260916_curve_qgate_revalidation]]), possibly the same drift, but that
is a hypothesis, not established.

**Consequence for the axis fix (`96271ba6`):** cannot be validated as beneficial OR harmful
under the current environment — no landing-quality signal is trustworthy right now for
ANY change. The fix's correctness stands on its own terms (mathematical derivation from the
SDF + validator's independent oracle, 15/15 across 5 seeds) and is NOT reverted. Do not
re-attempt an IC2-5 landing-quality gate until the marker-alive collapse is diagnosed —
otherwise every gate from here forward returns the same false "everything is broken" signal
regardless of what changed.

**Action item, higher priority than any further CBF tuning:** diagnose the marker-alive
collapse. Suspect areas to check first: Gazebo world/marker model state (a stale spawn,
lighting, or renderer setting), ros_gz_bridge health, whether any residual state was left by
recent SDF experimentation (peer 29's temporary 640x480 restore — SDF file itself reads
correct 320x240, but check for cached/stale Gazebo model resources), PX4 firmware/parameter
drift. This blocks all landing-outcome gating, not just this thread.

---

## CORRECTION 2026-09-18: the "perception-collapse environment drift" diagnosis above was WRONG — it was the wrong marker type, not drift

Root-caused fully. **Not an environment mystery.** `scripts/run_ic_validation.sh` never sets
`WORLD`/`MARKER_TYPE`, and both default to `"aruco"` (`controller.py:73`,
`run_landing.sh:23`) — a claim to the contrary in the 2026-09-03 rename commit's own message
does not match the code. So the 2026-09-17 IC1-5 gate and my same-day OLD-vs-NEW A/B both
silently ran **ArUco**, not cross-marker. Confirmed definitively from `Img_Data.npy`'s own
keys: `Centroid Map Raw`/`Ring Opt Flow Ang Vel`/`Alpha Map Raw` (ArUco, `img_data.py`) vs.
the Sep-12 baseline's `FEATURE_IS_VISIBLE`/`Detection Status`/`Fail Reason`/`MARKER_EXTENT_PX`
(cross-marker, `cross_marker_perception.py`). ArUco's sensor cal is documented (CLAUDE.md) as
stale for 320x240 since 2026-07-17 — exactly sufficient to explain the marker-alive collapse,
identically in both controller code versions, with zero need for any environment drift.

This is the SAME defect as [[feedback_recurring_analysis_mistakes]] §18 (rover launcher,
found by another session ~1 hour before this one), on the stationary path — now §19.

**Retracted:** the suggestion that this "smells related to the still-unexplained... curve-
cycle mystery" ([[project_20260916_curve_qgate_revalidation]]). No evidence connects them;
that was a guess based on both being "unexplained today," not a shared mechanism. Do not
carry that link forward.

**Still true and NOT retracted:** the OLD-vs-NEW same-day A/B was the right move and did its
job — it correctly stopped a false regression from being pinned on `96271ba6`, even though it
took one more step (comparing `Img_Data.npy` keys) to find why both arms failed. The `96271ba6`
axis fix remains uncontaminated by any of this: still not landing-quality gated, re-running
now with `WORLD=cross_marker MARKER_TYPE=cross` set explicitly.

---

## BAKED 2026-09-18: `96271ba6` landing-quality validated, no regression

IC1-5 gate, WORLD=cross_marker MARKER_TYPE=cross explicitly set (the §19 trap), PD-FB (real
perception — `PLASMC_GT_FEEDBACK` defaults `"0"`, never set here). 18 reps total (IC1×5,
IC2×5, IC3×1, IC4×6 across two sub-runs, IC5×1); 14 genuine touchdowns (`terminal_state_ok`),
4 false-positive-touchdown-detect failures (see below — pre-existing, not this fix).

| | pre-fix (Sep-12, real touchdowns) | post-fix (this session, real touchdowns) |
|---|---|---|
| precise | ~17/21 (81%) | 11/14 (79%) |
| soft | 1/21 (5%) | 0/14 (0%) |

No regression on any measured axis. IC4's one investigated outlier (`IC4_rep4`, this gate:
0.788 m / 1.655 m/s) has `vis_active=0%` for its entire flight — the CBF never engaged — and
the pre-fix worst IC4 rep (`IC4_rep3`, Sep-12: 0.486 m / 1.409 m/s) shows the identical
signature (also `vis_active=0%`, also a short ~5s flight). Same failure mode before and
after; the fix has zero involvement in either.

**`96271ba6` is BAKED.** Already on `main` unconditionally (no flag), so nothing to flip —
this entry is the landing-quality validation record the fix was missing.

**Methodology note for next time:** `run_ic_validation.sh`'s own `landed` column (used
throughout this gate's live reporting) means "a recording was saved," NOT "the vehicle
touched down." The authoritative field is `Ground_Truth.npy`'s `SoftPrecise.terminal_state_ok`
— check it before trusting any precise/soft rate computed from the summary.tsv's `landed=YES`
rows. Caught late in this thread; corrected before baking, but the wrong count ("18/18 landed")
was stated to the user first. Worth its own checklist entry.

## NEW FINDING (out of scope for `96271ba6`, do not conflate): false-positive touchdown
## detection is the dominant landing-quality blocker, not the CBF

4/18 reps this gate (22%) never reached the surface — timed out mid-descent at altitudes from
0.24 m up to **3.77 m**. All four show the identical signature:

    [controller] TOUCHDOWN-DETECT v2 [flow-freeze]: extent=78/240px n_corn=148
      flow_disp=0.19px |s_e_n|=0.27 -> LANDED (disarm before bounce)
    [landing_test] Landing classification: NOT_LANDED [never reached surface: min 3.77 m above it]

The `PLASMC_TOUCHDOWN_LOOM`/`PLASMC_TD_V2` "flow-freeze" touchdown detector fires on a
transient optic-flow/extent pattern unrelated to actual ground proximity, tells the controller
the vehicle has landed, and the controller stops commanding descent and attempts to disarm.
PX4 correctly refuses ("Disarming denied: not landed") but the flight is over as far as the
controller thread is concerned — this is a hard flight failure, not an imprecision.

Pre-fix baseline (Sep-12) shows the SAME mechanism at a similar rate (4/25) but the misses
there were near-threshold (min alt 0.21-0.30 m, essentially "landed but the 0.20 m check
missed narrowly"). This gate's three worst cases (2.66, 3.50, 3.77 m) are genuine early
triggers, not threshold noise — not enough n to say whether that's a real difference or just
which draws landed in each n=small sample; needs its own n>=5 gate with the touchdown-detect
diagnostic isolated (log `extent`/`flow_disp`/`s_e_n` at the trigger frame across many reps,
check whether false triggers cluster by IC/altitude/marker-fill state) before concluding
anything about severity trend.

**Separately, soft touchdown is essentially never achieved (0/14 this gate, 1/25 pre-fix,
~4%) and looks like a design characteristic rather than a bug.** Velocity breakdown at
touchdown (`Telemetry_Data.npy["Velocity Body"]`, MAVSDK `VelocityBody`, body-frame — NOT a
plain array, `.x_m_s`/`.y_m_s`/`.z_m_s`): **14/16 checked reps are dominated by VERTICAL
velocity**, consistently ~0.35-0.6 m/s vs the 0.20 m/s soft threshold, lateral velocity
usually small by comparison. Points to the descent reference `h_rd` being a constant (by
deliberate prior design decision — memory already carries a caution against reintroducing a
time-varying one): no terminal flare, so the vehicle touches down near its steady-state
descent rate. This is not something the visibility CBF touches or could fix.

**Priority for future work, by leverage:** (1) false-positive touchdown detection — fixing
it converts hard failures into landings and directly raises BOTH precise and soft rates,
since a rep that never lands can be neither; (2) a terminal descent flare/deceleration
mechanism, if soft touchdown becomes a target — bigger scope, was previously avoided for
good reasons (memory: three prior "slow the descent" attempts failed) so any new approach
needs to reckon with why those failed, not repeat them.

---

## `2177670b` (2026-09-21): flow-freeze false-touchdown fix, self-audited against `diagnose-flight-data`

Implemented three fixes to `_touchdownDetectV2`'s flow-freeze path (all 4 of this session's
false touchdowns fired via this path): resolution-invariant tangent-unit thresholds
(renamed `PLASMC_TDV2_FF_HI/_LO` → `_HI_TAN/_LO_TAN`, unit change); a confidence gate
reusing `_bgflow_health`'s `rel_resid` at the same `0.45` the CBF's `condition_drift`
already uses; and the dominant fix, a live-visibility precondition (`FEATURE_IS_VISIBLE`)
restoring flow-freeze to only fire on a genuine off-marker settle, which is what its own
docstring claims but the code never actually checked (`_td_ext_armed` is a stale one-time
flag, not a live check).

Evidence at commit time: `tools/replay_touchdown_flowfreeze_gate.py` confirmed all 4 false
positives would be suppressed, 0 collateral effect on the 14 genuine touchdowns (flow-freeze
never fired in any of them).

**Self-audit, right after pushing, against the newly-surfaced `PX4_Gazebo/.claude/skills/
diagnose-flight-data` skill** (its two direct warnings both apply to this kind of work):

1. *"Verify timestamp sync directly, don't assume it."* The original check matched each
   false-positive's trigger frame by extent VALUE alone within the last 60 frames — not a
   timestamp-verified match, a real gap versus the skill's standard. Redone: JOINT match on
   extent AND corner count (matching the exact two numbers the log line itself prints)
   lands within 1-6 frames of the true end of each recording — consistent with a terminal,
   flight-ending event, not a spurious coincidental match. `FEATURE_IS_VISIBLE` reads `True`
   for every frame from the matched index through the end in all 4 cases, so the conclusion
   is robust even to residual indexing slop.
2. *"Watch for a stale/frozen field masquerading as live data."* Checked directly: long
   frozen-extent runs (86-102 frames, exactly 318px) DO exist, but sit at the very START of
   every recording (frames 1-~100), never near a trigger. For the 3 cases the visibility
   gate addresses, extent is smoothly, monotonically growing right up to the false trigger
   (e.g. 104->116px) -- genuinely live. For the 4th (the confidence-gate case), 318px
   recurs AT its trigger too, but corroborates rather than contradicts: that's the marker
   genuinely overfilling the frame, consistent with the already-identified terminal-overfill
   `rel_resid` degradation there.

**Both checks confirm the fix rather than overturn it.** No code change needed from this
audit; recorded because the verification gap was real even though the conclusion held --
next time, do the joint/timestamp-verified match FIRST, not as a post-hoc check.

**Still not done:** live SITL re-validation (this and the earlier gate work are all
offline replay against recorded logs -- "would this gate have fired differently on data we
already have," not a live re-run). The 3 genuine-touchdown paths (overfill/backstop/
IMU-spike) are structurally untouched (flow-freeze never fired in any of those 14 reps), so
regression risk there is low, but only a live gate confirms it.

---

## `2177670b` SITL-validated 2026-09-21: 22% -> 0% false-touchdown rate

IC1-5 gate, `WORLD=cross_marker MARKER_TYPE=cross` explicit, `test_data/ICValidation/
20260921-144320`, 25 reps (N_REPS=5).

| | pre-fix (axis-fix gate, `20260918-*`) | post-fix (this gate) |
|---|---|---|
| genuine touchdowns (`terminal_state_ok`) | 14/18 (78%) | **25/25 (100%)** |
| flow-freeze firings | 4 (all false) | **0** |
| precise (of genuine) | 11/14 (79%) | 21/25 (84%) |
| soft (of genuine) | 0/14 | 0/25 (unchanged, expected -- this fix never touched the
  constant-`h_rd`/no-flare mechanism §2 identified earlier) |

Every one of the 25 landings caught by `overfill` (23) or the independent IMU accel-spike
backstop (2) -- the two paths that were already reliable. `backstop` still never fires (not
this fix's concern). IC4 (source of 3/4 original false positives, still the hardest IC by
design -- 7 m start) goes 5/5 genuine touchdowns, 3/5 precise -- no longer losing flights
outright.

**Verdict: the touchdown-detect fix (resolution-invariant units + confidence gate +
live-visibility gate) is SITL-validated.** 22%->0% false-touchdown rate at n=25 is not a
marginal/fragile result -- BAKED alongside `2177670b`.

Two things this does NOT resolve, both already scoped as separate: (1) soft touchdown
(0/25, structural -- constant `h_rd`, no terminal flare, tracked separately); (2) whether
flow-freeze's OWN theoretical niche (genuine soft off-marker settle) is ever worth its
complexity -- it fired zero times in 43 combined reps across both gates now (18+25), so
there is still no positive evidence for it, only the negative evidence removed. Worth
revisiting once the rover thread reaches a stable-enough approach to observe a genuine
off-marker settle, per [[feedback_dont_judge_cbf_by_sp]]'s general principle (judge a
component by what it's supposed to do, not by outcome noise) -- applies here too: don't
call this "proof flow-freeze earns its complexity," it's proof the bug is fixed.

---

## 2026-09-21: soft-touchdown gap root-caused — perceived h_z corrupts in the terminal
## ~0.3m, NOT a gain-tuning gap. N_z investigation correctly abandoned before its gate ran.

Continues the touchdown-quality thread. User's premise ("GT-FB achieves soft touchdown, so
tune the vertical adaptive law") was RIGHT to push on -- my earlier "soft touchdown looks
structural" framing was wrong (see [[feedback_dont_conclude_lag_floor]], directly
applicable: don't conclude an architectural ceiling while a masked failure is a tuning
target). But the actual diagnosis lands somewhere neither the user's nor my first framing
expected: it's a PERCEPTION defect, not a control-gain one.

### Step 1 -- confirmed GT-FB reference, and found the MATLAB/PX4 h_rd divergence
GT-FB IC1 n=3, `LANDING_REF_RAD_OPT_FLOW=-0.38` (MATLAB's current re-tuned value,
`P.h_rd=-0.38`, vs PX4's baked default -0.30): **3/3 soft+precise**, rel_vel 0.014-0.023
m/s (10x under the 0.20 threshold), xy 0.0002-0.0034m. `test_data/ICValidation/
20260921-160526`. Confirms the premise unambiguously.

MATLAB's own comment on -0.38 is worth recording: PX4's -0.30 (ported into MATLAB
2026-09-03) was found there to SLOW the descent ~40% (t_f 10.3->16.7s) with NO accuracy
gain -- "the entire moving-traj regression vs the manuscript numbers." -0.42 (older locked
value) is "too aggressive noiseless." -0.38 is MATLAB's sweet spot on the CURRENT stack
(two-tier CBF + drift lead + yaw rate law + per-axis theta) -- gain VALUES don't port
between the two ([[feedback_matlab_gains_not_portable]], the 38ms-lag mechanism), but this
result is still evidence PX4's -0.30 may be under-motivated on the current architecture and
worth its own PX4-side validation (not done yet -- see Open below).

### Step 2 -- N_z (kappa adaptation rate) offline replay: real disturbance, too-slow response
Measured PD-FB `kappa_z`'s ODE against its own recorded `sigma_z`: a genuine 17.4x
disturbance spike lasting only 10% of `kappa_z`'s own tau (`1/(N_z*P_z)`=2.0s at the
baked N_z=0.1/P_z=5.0) -- kappa moved only 1.2x. Small-n live trial (N_z=0.3, IC1, n=3):
NO kappa-ratchet signature (unlike the XY-axis N=0.1 history, kappa_z stayed bounded,
a_u modest) but the terminal kappa_z response barely moved the needle (0.042-0.047 vs
baseline 0.027-0.039) and rel_vel showed no real change (0.40-0.63, indistinguishable from
baseline). `test_data/ICValidation/20260921-1[6-7]*` (small-n runs).

### Step 3 -- the decisive check: GT loom vs perceived h_z, same PD-FB flight, terminal window
Per [[feedback_recurring_analysis_mistakes]] / `PX4_Gazebo/.claude/skills/
diagnose-flight-data`: computed genuine ground truth via `tools/gt_optical_flow.py`
(Z_REG=0.2 regularized, valid to the deck) for `ICValidation/20260921-144320/IC1_rep1`, and
compared directly against the PERCEIVED `h(t)[:,2]` actually fed to the control law.
Sync verified first (Control_Data `t[0]` == GT `Start Time` exactly).

| t | alt (m) | GT loom (truth) | perceived h_z |
|---|---|---|---|
| 11.44 | 0.19 | -0.472 | -0.347 |
| 11.50 | 0.18 | -0.462 | -0.537 |
| 11.58 | 0.17 | -0.440 | **-0.686** |

**GT loom stays smooth in this window and even begins its OWN gentle taper (-0.49->-0.44)
-- the same natural flare character the clean GT-FB run showed independently.** The
PERCEIVED signal diverges the opposite way, to ~1.6x the true value, over the same ~150ms.
The vehicle's real motion is fine; the MEASUREMENT is not.

**Root cause matches an already-flagged, still-open defect from earlier this session.**
Checked `extent`/`rel_resid` in the exact same window (aligned by absolute time, not
`Img_Data`'s own longer-running clock which extends past touchdown for the video tail):
`extent=318px` (saturated -- the marker massively overfills the 240px detection frame) and
`rel_resid=0.44-0.85` (frequently above the 0.45 confidence-gate threshold used to fix the
flow-freeze false-positive the same session). **This is the SAME terminal-overfill
degradation already found corrupting the CBF's `condition_drift` drift lead (flagged,
unfixed) and one of the four original flow-freeze false-touchdowns (fixed via the
confidence gate, `2177670b`).** Three previously-separate-seeming symptoms, one root cause.

### Verdict
**N_z tuning is the WRONG lever and was correctly abandoned before its planned n>=5 gate
ran.** A faster-responding kappa_z reacting to a signal that spikes to 1.6x the true value
applies an even LARGER erroneous correction, not a softer landing. Gain-tuning against a
corrupted signal was about to repeat exactly the mistake
[[feedback_dont_conclude_lag_floor]]'s RULE 2 warns about in spirit (masking a real defect
with a compensating gain, at the wrong layer) -- caught here by checking against GT before
committing to the tuning direction, not after.

### What's actually needed (not yet implemented)
Fix belongs in the PERCEPTION pipeline, in the terminal ~0.2-0.3m where extent saturates:
gate/hold `h_z` on the same `rel_resid` confidence signal that already exists
(`_bgflow_health`), analogous to two patterns already live in this codebase --
`condition_drift`'s own resid gate (visibility_projection.py) and the KF-freeze-during-
marker-loss pattern ([[feedback_kf_frozen_during_marker_loss]]). NOT yet designed or
implemented. Natural next scope: fix ALL THREE overfill-exposed consumers
(condition_drift's drift lead, h_z, and re-verify flow-freeze's own resid gate covers this
case fully) under one terminal-overfill perception fix rather than three separate patches,
since they share the exact same root signal (`extent`/`rel_resid` saturating near the
deck).

### Open, not done
- PX4-side validation of `h_rd=-0.38` on IC2-5 (only GT-FB IC1 n=3 checked so far;
  `feedback_matlab_gains_not_portable` means this needs its own PX4 gate, not a port-on-
  faith, though the result direction is encouraging).
- The terminal-overfill `h_z`/perception fix itself (design + implement + validate).
- Whether `h_rd=-0.38` ALSO needs to be tested under PD-FB once the perception fix lands
  (the two may interact: a faster commanded descent reaching the corrupted-signal altitude
  band sooner/differently).

---

## 2026-09-21 (same day, parallel session): MATLAB independently converged on the SAME
## kappa-adaptation-too-slow mechanism, went much further, and REVERTED h_rd to -0.30

Found via `7476400a` (peer session, `MATLAB/VDF_ASMC/vdf_params.m`) immediately after
pushing the above. Directly overlapping work, reconciling now.

**Independent confirmation of the mechanism.** The peer's own comment: "adaptation speed;
stability is set by kappa's SENSITIVITY dkappa/dWx (theta*G/P and N)" -- the exact
diagnosis this thread reached via the offline kappa-ODE replay (§ above), reached
independently via MATLAB-side work.

**A coordinated retune, far more aggressive and complete than the N_z=0.3 trial here:**

| param | old | new | ratio |
|---|---|---|---|
| N (xy,z) | 0.1,0.1,0.1 | 2.0,2.0,**5.0** | 20x/50x |
| Pleak (xy,z) | 2.5,2.5,5.0 | 0.5,0.5,**0.1** | 5x/50x down |
| E (xy,z) | 1.0,1.0,0.5 | 0.1,0.1,**0.02** | 10x/25x down (stiffer) |
| chi_z | 0.1 | 0.5 | 5x |
| p_hinf z | 1.5 | 0.3 | 5x tighter |
| kappa0 xy | 0.5,0.5 | 0.1,0.1 | (z unchanged 0.25) |
| kappa_max xy | 30,30 | 1,1 | (z unchanged 3.0) |

Note N_z*Pleak_z tau is UNCHANGED (0.1*5.0 = 5.0*0.1 = 0.5 -> tau=2.0s either way) -- the
fix is not "faster tau," it's the GROWTH TERM theta*N*G*|sigma| scaling directly with N
(50x for z) while Pleak drops the SAME 50x, raising kappa's reachable equilibrium by that
same factor. A materially different (and apparently much more effective) lever than the
"raise N, hold P" shape this thread's own N_z=0.3 trial used.

**Validated: 25-IC gate, 25/25 SP, t_f 9.65s** (MATLAB, presumably noiseless/synthetic --
not yet PX4-validated, and per [[feedback_matlab_gains_not_portable]] the VALUES will not
port as-is; needs its own PX4-side re-derivation/gate, not a straight copy).

**h_rd REVERTED 2026-09-09's -0.38 back to -0.30** (PX4's original value), with the
retuned adaptive law in place: "5-seed, 35 cases: 33/35 soft vs 24/35 at -0.38." This
DIRECTLY SUPERSEDES the "Open" item this thread flagged above (test h_rd=-0.38 on PX4) --
once the adaptive law itself is properly retuned, -0.38's faster commanded descent is
apparently no longer needed or even net-negative; -0.30 (already PX4's live default) wins.
**Do not port h_rd=-0.38 to PX4** -- the peer's own newer result reverses that
recommendation on MATLAB's own turf. GT-FB's PX4-side h_rd=-0.38 n=3 result recorded above
(3/3 soft+precise) still stands as a fact about PX4's -0.38, but is no longer the
comparison to chase -- the coordinated N/P/E/chi_z/p_hinf retune is the more promising
direction, tested against PX4's OWN existing -0.30.

**What this does NOT address: the terminal-overfill h_z perception corruption** found
independently in this thread (§ immediately above). MATLAB's synthetic pixel-noise model
is not the same failure mode as a real camera's extent saturating / flow-solve confidence
collapsing at extreme close range -- there is no reason to expect the peer's retune (tuned
against MATLAB's noise model) to fix a defect that exists only in PX4's real perception
pipeline. Both fixes are likely needed: the coordinated adaptive-law retune (ported +
re-validated on PX4) for the CONTROL side, and the h_z confidence-gate (this thread's
finding) for the PERCEPTION side. They may also interact -- a stiffer, faster-adapting
kappa_z reacting to the SAME corrupted terminal h_z could make the corruption's effect
WORSE, not better, until the perception fix lands. Recommend sequencing: perception fix
first (removes the confound), THEN port+validate the coordinated retune on PX4 PD-FB,
rather than porting the retune first into a still-corrupted signal.

---

## CORRECTION 2026-09-21 (same day): the extent/rel_resid mechanism was UNDER-INVESTIGATED --
## real diagnosis is more precise and points at a different, already-recommended fix

User: "First investigate the issue properly. Don't assume things here." Right call -- the
prior entry's causal story (extent saturates -> rel_resid collapses -> h_z corrupts) was
built from ONE window read at coarse granularity and doesn't survive a finer check.

**Traced the full data path first** (code, not inference): `Control_Data["h(t)"]` <-
`self._img_node.getOptFlowAngVel()` -> `CrossMarkerPerception.getOptFlowAngVel()` =
`_sensor_cal_hw @ getRawOptFlowAngVel()` = `self._hw` = a coast+freeze KF's state
(`_hw_kf_x[:,0]`), updated via `_kf_update_hw(z, t)`. Ruled out two alternate explanations
by checking the actual code and logs, not assuming:
- `_ring_committed` (deliberate h_z<-RING_LOOM substitution): grepped the log, never fired
  this flight (`RING-COMMIT` absent). `_loom_ring_on_loss`: default OFF, unset.
- Loom values ARE excluded from the savgol lateral-spike-reconstruction (comment: "not
  depth-free predictable" -- the loom passes through as the raw/KF measurement, not
  smoothed-over).
- **A loom-channel innovation gate ALREADY EXISTS** (`CROSS_LOOM_INNOV_GATE`, default ON,
  NIS>25 AND slew>12/s -> inflate r[2] 1000x). Checked the logged `Loom Gate` field: **fired
  0/1347 frames the whole flight.** Computed why: the actual slew rate through the
  divergence (e.g. -0.364->-0.565 in 76ms = 2.6/s) is far under the 12/s trigger -- the gate
  is built to catch discrete single-frame SPIKES; this is a gradual ~450ms RAMP, a
  structurally different signature the existing gate cannot see regardless of threshold.

**Finer-grained look at extent/rel_resid falsifies the simple story.** `extent=318px`
(saturated) and `rel_resid` (0.5-0.95, mostly above the 0.45 confidence-gate value used
elsewhere) are BOTH elevated for the ENTIRE window t=10.5-11.6s -- including t=10.5-11.0,
where `h_V_z` was tracking WELL (improving -0.19->-0.05 toward zero). The sharp reversal
specifically starts ~t=11.08 with no corresponding step-change in either signal at that
moment. So "high rel_resid + saturated extent" is present but NOT discriminating -- it was
true during the good part of the trace too. A rel_resid gate would have thrown away good
frames along with bad ones.

**Stronger, better evidence: independent cross-check via the SAME pipeline's OWN
scale-based loom estimates.** `Width Loom Rate` and `Scale Loom Rate` (derived from marker
SIZE/WIDTH change -- a different physical principle than optical flow, already computed,
already logged) stay SMALL and roughly FLAT through the identical window (-0.03..-0.26 and
-0.002..-0.10) while `h_V_z` (flow-based) diverges to -0.77. `Detection Status`/`Fail
Reason` both read "ok" throughout -- the pipeline itself never flags anything wrong. Two
independent references now agree (GT position-derived loom from § above, AND this
same-pipeline scale-derived loom) that the true signal is much smaller than what the
flow-based estimate reports in this window.

**This connects to, and is a specific instance of, prior project work rather than a new
finding needing a new mechanism.** `project_20260908_line_width_loom_investigation.md`:
continuous width/scale-loom fusion (`CROSS_SCALE_RATE_FUSE`) was tried and REJECTED (biased
normal-tracking loom-setpoint, caused fast/erratic arrival) -- but that investigation's own
recorded next step, never executed: "(c) consider limiting it to only VETO a pinv spike
(|pinv h_z - scale_rate| large) rather than continuously correcting." **This data is exactly
the case that recommendation was written for.**

### Revised diagnosis
Not "terminal-overfill corrupts confidence, gate on rel_resid." Rather: **the flow-based
loom estimate specifically diverges from the pipeline's own independently-computed
scale-based loom estimate in a ~150-450ms terminal window, for a reason not yet identified
at the mechanism level** (the KF's coast/freeze internals, a specific geometric effect of
the flow solve at extreme close range, or something else -- NOT YET FOUND, do not assume
further without checking). What IS well-established: (1) it is a flow-solve-specific
artifact, not a real vehicle motion (2 independent cross-checks agree); (2) the existing
loom innovation gate cannot catch it (wrong failure shape -- ramp not spike); (3) the
already-computed, already-logged scale-loom signal stays reliable exactly where flow
diverges and is the natural veto/substitution signal, per the peer's own prior
recommendation.

### Still open, correctly scoped now
- WHY the flow-based loom specifically diverges in this window (KF dynamics? geometric
  effect of the flow solve near saturation? something else?) -- not yet found.
- Design the VETO (not continuous fusion) using `|pinv h_z - scale_rate|`, matching the
  prior investigation's own untried recommendation, rather than a fresh rel_resid gate.
- Whether this same mechanism explains the CBF drift-lead's "terminal-overfill" exposure
  flagged earlier this session -- plausible given the shared window, NOT yet verified with
  the same rigor applied here. Don't assume it's the same without checking.

---

## 2026-09-22: cond(A_reduced) reconstructed directly -- ill-conditioning FALSIFIED,
## near-grazing-ray perspective-divide amplification CONFIRMED (a known, predicted,
## never-confirmed mechanism)

User: "go the extra step and reconstruct cond(A_reduced) precisely." Could not confirm
whether the LIVE solve used the gyro-reduced 4-unknown path -- `Img_Data["IMU AngVel"]`
is NaN across all 1347 frames of `IC1_rep1`, traced to a DIFFERENT, apparently-unwired
log source (`self._pending_angvel`, distinct from `getAngVels()`'s `_angvel_deque` that
`_solve_jacobian` actually receives) -- so this doesn't establish gyro availability either
way. Computed the FULL 6-unknown `cond(A)` instead, which is well-defined regardless.

### Reconstruction method
Replicated `_getVirtualPts` + `_fill_A` + `np.linalg.lstsq`/`np.linalg.cond` exactly
(script: scratchpad, not yet committed -- see Still Open) against the raw
`Flow Points Prev/Curr Px` + `Quat` already saved in `Img_Data.npy`. Applied to
`IC1_rep1` (gradual divergence) and `IC4_rep2` (sharp single-frame spike).

### Result 1 -- FALSIFIED: `cond(A_full)` stays modest throughout
IC1_rep1: 7.8-10.4 across the ENTIRE terminal window, no jump at the divergence onset.
IC4_rep2: 7-16 through its spike window too. Neither shows the blow-up a genuine
ill-conditioning/rank-deficiency event would produce. **The point-spread correlation found
in the prior entry was real, but the mechanism it correlates with is NOT matrix
ill-conditioning of the lstsq system.**

### Result 2 -- CONFIRMED: near-grazing-ray perspective-divide amplification
Discovered while checking `IC4_rep2`'s exact spike frame: the RAW single-frame solve's
`Tz` jumps `-0.451 -> -1.860` in one ~16ms step (t=13.320->13.336) -- the logged (KF-state)
`h_V_z` only shows `-0.126 -> -0.678` at the same transition, i.e. **the KF is DAMPING the
raw spike, not amplifying it** (reverses an earlier worry about the KF being the
corrupting stage). Simultaneously, `zv_min` (minimum per-point ray height above the
gravity-leveled V-frame, `_getVirtualPts`'s own diagnostic) drops steadily through this
exact window: 0.95->0.83->0.67->**0.60 (at the spike)**->0.47. `IC1_rep1` shows the
identical, more gradual trend (0.97->0.87-0.89).

**This is a mechanism the code's own 2026-08-02 comment already predicted and flagged as
unconfirmed:** "A near-zero z_v blows up the perspective divide into a huge or sign-flipped
point... without tripping any of the existing n_kept/cond/rel_resid diagnostics (those
check the LSTSQ FIT, not the per-point PROJECTION that feeds it)." That is now confirmed
directly, not inferred.

### A partially-tested lever for exactly this already exists
`CROSS_Z_V_MIN_FLOW` (default 0.0, drops only z_v<=0 behind-camera rays -- "unambiguously
correct, can't starve the solve"). Comment: raising toward ~0.4 "ALSO drops amplified-but-
not-flipped near-grazing rays -- TESTED at 0.4 bundled with the (reverted) angular window
[`CROSS_FLOW_ANG_MAX`] and the pair regressed terminal h_x/h_y badly; **0.4 in ISOLATION is
untested.**" The angular-window backfire is a SEPARATE, unrelated failure mode (once the
marker fills the frame, a centered angular restriction leaves only 4-5 near-collinear
survivors -> unbounded few-point Tx/Ty noise, worse than the edge-point bias it targeted)
-- do not conflate the two knobs; my finding is specifically about `Z_V_MIN_FLOW`, not
`FLOW_ANG_MAX`. My measured `zv_min` at the spike (0.47-0.6) is mostly ABOVE the
previously-tried 0.4, so 0.4 alone likely would not have caught this case either --
suggests something closer to 0.5-0.6, `Z_V_MIN_FLOW` ONLY, `FLOW_ANG_MAX` left at its
inert 99 default.

### Verdict / next step
Root mechanism for the terminal loom (h_z) divergence: CONFIRMED as near-grazing-ray
perspective-divide amplification, not matrix conditioning, not a KF artifact (KF damps
it), not the extent/rel_resid correlation originally proposed (real but non-causal --
elevated for the whole window, not discriminating). Next, well-scoped experiment:
`CROSS_Z_V_MIN_FLOW` raised in isolation (~0.5-0.6) to drop near-grazing points from the
loom solve specifically, leaving `FLOW_ANG_MAX` untouched. Should be checked first via the
SAME offline replay method (re-run the geometry-rejection filter against the already-
recorded point clouds, count how many spike frames it would have suppressed) before any
live SITL test, matching this thread's now-established practice.

### Still open
- The offline reconstruction script lives only in scratchpad -- commit it (matching this
  session's practice of shipping the tool alongside the claim) before relying on this
  finding further.
- Whether raising `Z_V_MIN_FLOW` alone actually helps needs to be checked by REPLAYING the
  filter against recorded point clouds first (offline), then a live SITL gate -- not done.
- The gyro-availability ambiguity (does the live solve actually run the reduced 4-unknown
  path?) was not resolved and doesn't block this finding, but is worth resolving before
  touching the gyro-derotation code path specifically.

---

## CORRECTION 2026-09-22 (same session): CROSS_Z_V_MIN_FLOW does NOT fix the divergence,
## on either test case -- the near-grazing-ray finding is real but insufficient/irrelevant

User: "go ahead" (replay the filter offline before any SITL test). Built
`tools/replay_zvmin_filter.py`, replicating the live `_geokeep` block +
`MIN_FLOW_POINTS_SOLVE=4` count-floor fallback EXACTLY, and tested candidate
`CROSS_Z_V_MIN_FLOW` thresholds {0.3,0.4,0.5,0.6,0.7} against both reps. Result is a clean
negative on the proposed fix, for two DIFFERENT reasons per rep:

**IC4_rep2 (the sharp spike):** filtering to Zv>=0.7 drops 43-121 of 143-179 points, but
the spike barely moves -- at the exact spike frame (t=13.336): filtered sol_Tz=-1.864 vs
unfiltered -1.860. Nearby frames show the same (t=13.352: -2.035 filtered vs -2.097
unfiltered). **The near-grazing points are not the ones driving the blow-up -- the
REMAINING, non-grazing points still jointly solve to a large Tz.** So while zv_min
genuinely correlates with (and precedes) this spike, as the prior entry found, it is not
a small-number-of-bad-points problem that point-exclusion can fix; something about the
AGGREGATE fit changes, not a few outlier rays.

**IC1_rep1 (the gradual divergence):** `zv_min` never drops below 0.87 anywhere in the
whole terminal window -- the filter is a complete no-op at every threshold tested, up to
0.7. Yet the divergence still happens (raw solve drifts -0.02->-0.20 over the same
window). **The near-grazing-ray mechanism doesn't even apply here** -- z_v isn't remotely
close to a relevant threshold for this rep's failure mode.

**A further wrinkle, found while comparing raw-vs-KF across the two reps:** in IC4_rep2
the KF DAMPS a large raw spike (raw -1.860, KF -0.678 at the same frame -- the previous
entry's finding). In IC1_rep1 it's the OPPOSITE: the raw per-frame solve stays modest
(max ~-0.2) throughout this window, while the KF-reported h_V_z grows much LARGER (-0.77)
-- the KF's own STATE is more extreme than any single measurement feeding it. This points
at the KF's own temporal dynamics (predict step, R-scheduling, or something accumulating
across cycles) as the driver for the gradual case, not the per-frame geometry at all.

### Corrected verdict
`CROSS_Z_V_MIN_FLOW` is NOT the fix -- retracting that recommendation. The near-grazing-
ray correlation from the prior entry is real (genuinely co-occurs with, and precedes, the
sharp-spike case) but is not SUFFICIENT to explain it (excluding those points doesn't
suppress the spike) and is IRRELEVANT to the more common gradual-divergence case entirely.
Two apparently different failure SHAPES (sharp single-frame spike vs gradual multi-frame
drift) may have two different root mechanisms, or a shared one that manifests differently
-- not yet established.

### Next direction (not yet investigated)
The KF's own dynamics -- specifically the LOOM R SCHEDULE (`_loomRMult()`, adjusts r[2]
by some already-computed multiplier every frame, mechanism not yet read), the predict-vs-
update balance, and the coast/freeze logic -- are now the more promising lead, especially
for the gradual-divergence case where the RAW per-frame solve doesn't show the problem at
all but the KF STATE does. Have not yet read `_loomRMult()`'s implementation or traced the
KF's predict step.

### Tool
`tools/replay_zvmin_filter.py` committed alongside this finding, replicating the exact
live filter + fallback logic so the negative result is reproducible, not asserted.

---

## 2026-09-22 (cont.): KF predict/update mechanism -- PARTIALLY confirmed, magnitude
## unexplained. Loom R-schedule, scale-fuse, and hard backstop all ruled out directly.

Continues the "why does the KF state (-0.77) exceed any single raw measurement (~-0.2)
for IC1_rep1's gradual case" question. Read the full hw-KF pipeline in
`cross_marker_perception.py` and checked each candidate mechanism against the ALREADY-
LOGGED fields for this exact flight (not assumed from code alone):

- **`_loomRMult()` (CROSS_LOOM_R_SCHEDULE)**: U-shaped r[2] inflation at extent extremes,
  would suppress measurement trust near touchdown IF active. Checked `Img_Data["Loom R
  Mult"]`: exactly 1.00 for all 1347 frames. Default OFF (`CROSS_LOOM_R_SCHEDULE=0`).
  RULED OUT.
- **Scale-rate fusion (`CROSS_SCALE_RATE_FUSE`)**: default OFF, confirmed via source.
  RULED OUT.
- **Hard loom backstop (`CROSS_LOOM_ABS_MAX=20.0`)**: clamps |loom|>20 and kills the
  rate. Far above anything observed here (max magnitude ~2.1 in IC4_rep2's raw solve).
  RULED OUT.
- **Loom innovation gate**: already established in the 09-21 correction entry --
  `"Loom Gate"` logged 0 the whole flight, wrong failure shape (spike detector, this is
  a ramp).

### Direct KF replay: rate-buildup is REAL but does not explain the full magnitude
Built `tools/replay_hw_kf.py`: replays `_kf_step`'s exact predict+update math
(`FLOW_KF_Q=5.0`, `FLOW_KF_R=0.1`) as a standalone scalar (value,rate) KF, fed the RAW
per-frame solve (from the same geometry as `replay_flow_solve_conditioning.py`) as `z`
at EVERY frame from the start of the recording (29s of warm-up before the window of
interest, well-settled).

Result for IC1_rep1: the reconstructed KF value DOES diverge beyond the raw solve, in
the SAME direction, with a growing negative rate (0.02 -> -0.16 over the window) --
**confirming the constant-velocity rate-buildup mechanism is real and contributes.** But
the MAGNITUDE falls far short: at t=11.560, reconstructed value=-0.137 vs the actually
logged h_V_z=-0.771 -- roughly 5.6x smaller. **Not a full explanation.**

### What's still unaccounted for
This reconstruction has NOT replicated: the gyro de-rotation to the reduced 4-unknown
solve (availability still unresolved -- see the 09-22 entry above), or the sensor
calibration matrix. Either could shift the raw per-frame solve's own values enough to
change what the KF is tracking, independent of the KF math itself. 29s of warm-up before
the window rules out "insufficient settling time" as the gap's explanation.

### Verdict
Two mechanisms now stand as PARTIAL, not full, explanations: near-grazing rays (real,
confirmed, but doesn't survive point-exclusion -- see the CROSS_Z_V_MIN_FLOW correction)
and KF rate-buildup (real, confirmed directionally, but ~5.6x short on magnitude). Niether
alone accounts for the full observed divergence. The gyro-derotation path is now the most
likely remaining unresolved piece -- if the live solve genuinely uses the reduced
4-unknown [Tx,Ty,Tz,Wz] form (still not established either way), replicating THAT exactly
(not the full 6-unknown fallback this thread has used throughout) is the natural next
step, since it would change the raw solve values feeding everything downstream.

### Tool
`tools/replay_hw_kf.py` committed alongside this finding.

---

## 2026-09-22 (cont.): gyro-availability RESOLVED (was live) -- but doesn't close the gap

**Resolved directly, not inferred.** `getAngVels()` is fed from `self._FC.getAngVelIMU()`
on EVERY image callback (`src/gz_subscriber.py:image_callback`), and `getAngVelIMU()`
(`src/flight_controller.py:455`) is `None` only before the IMU task's FIRST sample at
boot -- trivially past that by 10+ seconds into flight. Confirmed empirically:
`Telemetry_Data.npy["Angular Velocity FRD"]` holds real, finite MAVSDK
`AngularVelocityFrd` objects throughout `IC1_rep1` (same absolute clock as `Img_Data`'s
`Time`, `IMU Timestamp` range overlaps it directly). **Gyro WAS live-available the whole
flight** -- the earlier ambiguity (`Img_Data["IMU AngVel"]`=NaN) was entirely the dead
`_pending_angvel` logging path, unrelated to what `_solve_jacobian` actually received.

**Redid the KF replay with the CORRECT reduced [Tx,Ty,Tz,Wz] gyro-derotated solve**
(`tools/replay_hw_kf_gyro.py`, angvel aligned from `Telemetry_Data` by nearest
timestamp). Result: **almost no change** from the earlier (wrong-path, full 6-unknown)
reconstruction. At t=11.560: raw solve -0.129 (was -0.103), KF-reconstructed value
-0.138 (was -0.137) -- still ~5.6x short of the actually logged h_V_z=-0.771.
**Gyro-derotation was NOT the missing piece.** Plausible reason: Wx/Wy are apparently
small enough in this near-hover descent that substituting their true gyro values for
the jointly-solved ones doesn't materially shift Tx/Ty/Tz/Wz.

### State of the investigation, honestly
Four candidate mechanisms tested, all either falsified or insufficient:
1. Matrix ill-conditioning -- FALSIFIED (cond(A) modest throughout).
2. Near-grazing rays -- real correlation, confirmed via direct reconstruction, but does
   NOT survive point-exclusion testing (CROSS_Z_V_MIN_FLOW correction).
3. KF constant-velocity rate-buildup -- real, directionally confirmed, ~5.6x short on
   magnitude even with the corrected solve path.
4. Loom R-schedule / scale-fuse / hard backstop / innovation gate -- all ruled out
   directly against logged fields (inactive or wrong failure shape).

**None of the four fully explains the observed divergence.** The ~5.6x gap between the
KF math replayed exactly (now confirmed with the correct solve path) and the actual
logged h_V_z remains unresolved. Candidates not yet checked: the sensor calibration
matrix's actual effect (dismissed early as "too small," 0.9513 z-diagonal, but not
rigorously verified end-to-end with the KF in the loop); a possible discrepancy in HOW
`dt` is computed live (frame-to-frame Img_Data Time deltas used here vs whatever the
live controller actually uses, e.g. `getFPS()`'s own jitter-rejection logic could differ
subtly); or a genuinely different z-value ordering/sign convention error in this
reconstruction that happens to preserve rough SHAPE but not magnitude.

### Recommendation
This is the 4th consecutive partial/negative result on the SAME mechanism-hunting
question. Diminishing returns at this layer -- flagging to the user rather than
continuing to guess. The near-grazing-ray correlation and KF rate-buildup are BOTH real
contributing factors even though neither is sufficient alone; a practical mitigation
(e.g. damping the KF's rate-state growth specifically in the terminal window, or a
much simpler terminal-proximity hold/clamp on h_z) may be more tractable than fully
attributing the residual gap before acting.

### Tools
`tools/replay_hw_kf_gyro.py` committed alongside this finding.

## ===== 2026-09-22 (cont'd 2) -- SITL freed, fresh recording confirms dt-fix hypothesis =====

SITL lane freed by `soft-precise-landing-53` (rover gate finished 03:05). Got ONE fresh
headless `WORLD=cross_marker MARKER_TYPE=cross` recording with the FPS-logging fix in
place (`test_data/Landing_Test/Tue Sep 22 03-22-59 2026`; PRECISE-only landing,
xy=0.038m, rel_vel=0.383 m/s -- not a soft touchdown, but that's irrelevant to this check).

**Confirmed `Img_Data["FPS"]` is now live** (0/1240 NaN, values 50-83.3 Hz, varying frame
to frame -- the fix works).

**Confirmed the dt mismatch is real and large in the terminal window**: printed
`Time[i]-Time[i-1]` (`dt_log`, what every prior replay tool used) against `1/FPS[i]`
(`dt_fps`, what `process_frame`'s raw solve actually divides by) for the last 0.6s before
touchdown -- `dt_log` is **3-8x LARGER** than `dt_fps` throughout (e.g. t=-0.14s:
dt_log=0.128, dt_fps=0.016, ratio=8.0). Mechanism: the `run()` polling loop's own call
cadence (governed by new-stamp arrival + `time.sleep(0.002)`) can be much slower than the
camera's native frame rate, but `imgs[0]/imgs[1]` (from `Image_Node`'s adjacent-pair deque)
stay ONE native frame apart regardless -- so the raw solve's own dt is correctly small,
but a lot of real elapsed time (and un-observed marker motion) can pass between
consecutive PROCESSED calls without process_frame's raw solve ever seeing it.

**Redid the raw-solve reconstruction using the CORRECT dt** (`1/FPS[i]`, not `Time` deltas)
against this fresh recording, feeding it through the same `_kf_step` math as always
(KF's own internal dt is `t-prev_t`, unaffected -- only the RAW measurement's own dt
changes). Result: reconstructed KF state now matches the logged `h_V_z` almost exactly
throughout the terminal window (e.g. -0.262 vs -0.238, -0.247 vs -0.243, -0.322 vs -0.238
at the very last frame) -- **no ~5.6x gap, no unexplained divergence in this rep.**

**Interpretation, held to the honest standard this thread has used throughout**: this is
strong evidence that a meaningful fraction (possibly most) of the previously "unexplained"
divergence in earlier reconstructions was an ARTIFACT of the investigation's OWN replay
tooling using the wrong dt -- not necessarily evidence of a live controller defect. BUT this
confirmation rep is a well-behaved landing (no large spike observed even in the ORIGINAL
h_V_z here, max magnitude ~-0.32) -- it validates the METHODOLOGY (correct-dt reconstruction
now tracks the KF's actual output essentially perfectly, which is itself the strongest
sanity check this thread has produced on the reconstruction tools generally) but does NOT
yet directly confirm that correct-dt reconstruction also explains a genuine LARGE spike case
(the original `-0.771` `IC1_rep1` case this whole 5.6x-gap chase was about). That rep's own
`FPS` field is dead/NaN (recorded before this fix), so it can't be redone with correct dt
retroactively -- need a NEW recording that reproduces a comparable large spike.

### Honest net state
Root cause is LIKELY (not yet certain) the investigation's own dt bug, now fixed both in
the live logging (so future recordings are diagnosable) and understood mechanistically
(polling-loop-vs-native-rate decoupling). **Not closed**: get one more recording where a
large terminal h_z excursion actually occurs (the original investigation's reps clustered
around a marker-overfill/near-grazing-ray condition -- IC1 close-in approaches were the
recurring spike case), and confirm the correct-dt reconstruction tracks THAT too, not just
a well-behaved landing. If it does, this closes the whole soft-touchdown perception-side
investigation: the apparent h_z corruption was a REPLAY-TOOL artifact, and the live
controller's actual behavior (subject to separate scrutiny of whether the polling-loop dt
decoupling itself degrades control, a genuinely different question from "was my offline
reconstruction right") needs to be evaluated on its own terms, not through this thread's
prior (dt-wrong) reconstructions.

## ===== 2026-09-22 (cont'd 3) -- CLOSED: dt bug confirmed against a real spike, thread resolved =====

Ran 4 fresh IC1 reps (`test_data/ICValidation/20260922-032613`, `HEADLESS=1
WORLD=cross_marker MARKER_TYPE=cross IC_LIST=IC1 N_REPS=4`) specifically to catch a
genuine large terminal excursion, since the first confirmation rep was too well-behaved
to be a real test. 4/4 precise, 0/4 soft (as usual under default config). `IC1_rep3`
delivered exactly the needed case: the KF-reported `h_V_z` ramps from -0.21 to -0.68 over
the terminal 0.6s, with the raw per-frame solve spiking to -1.28 at t=-0.212s -- the same
SHAPE of excursion the original `-0.771` case showed.

**Redid the correct-dt (`1/FPS[i]`) reconstruction against `IC1_rep3`'s actual spike.**
Result: reconstructed KF state matches logged `h_V_z` to within 0.000-0.013 across the
entire terminal window, including exactly at the steepest part of the ramp (-0.655 vs
-0.651 at t=-0.212s). Checked `IC1_rep4` too (smaller excursion, -0.18 to -0.46): same
result, diffs 0.004-0.046, still well under 10% of the swing.

**This closes the mechanism.** The ~5.6x gap that drove five rounds of mechanism-hunting
(ill-conditioning, near-grazing-rays, KF rate-buildup, R-schedule/scale-fuse/backstop/gate,
sensor-cal) was a bug in THIS investigation's own offline replay tooling -- every one of
those tools computed the raw flow solve's dt as `Img_Data["Time"][i] - Time[i-1]`, but the
live `process_frame()` actually uses `dt = 1/fps`, and these differ by 3-8x in the terminal
window because the `run()` polling loop's call cadence decouples from the camera's native
frame-pair rate (a frame's own image_callback pairing stays one native interval apart even
when many real-time (and real motion) has passed since the previous PROCESSED call). Using
the wrong (too-large) dt divides displacement into an ARTIFICIALLY SMALL velocity --
exactly the direction and magnitude of "why does my reconstruction fall short of the
logged spike" this thread kept hitting.

**What this means for the live controller, separated cleanly from what this closes:**
- CLOSED: "why couldn't earlier reconstruction attempts reproduce the logged h_z spike" --
  answered. It's a tooling bug (a1ffbf02 fixes the underlying dead logging that made it
  undiagnosable), not a mystery mechanism in the perception/KF/controller stack.
- STILL OPEN, genuinely different question, NOT addressed by this fix: does the spike
  itself (real, in `h_V_z`, confirmed via the two independent-GT cross-checks earlier in
  this thread -- true descent rate stays smooth while `h_V_z` ramps) represent a REAL
  perception error, or is `h_V_z`'s terminal ramp itself legitimate given how a marker
  genuinely fills the frame at close range (near-grazing rays ARE real per the earlier
  finding, just not sufficient alone to explain the OLD reconstruction gap -- that
  insufficiency is now explained by the dt bug, not by the near-grazing-ray mechanism
  being wrong). Whether THIS ramp is itself the soft-touchdown-preventing cause, and
  whether it's controllable/attenuable, was never actually re-examined once the dt-bug
  explanation emerged -- it remains the next real open question for whoever picks this up,
  now on solid tooling.
- The polling-loop-vs-native-rate decoupling itself (large real time gaps between
  PROCESSED frames near touchdown, even though each processed pair's own dt is small) may
  independently be worth investigating as a controller-facing issue (large real gaps mean
  the KF's own `t-prev_t` update interval is large too, so its OWN uncertainty growth
  between updates is large near touchdown) -- untested, flagged not investigated.

### Net state: this specific 5-mechanism-then-dt-bug chase is DONE.
The soft-touchdown investigation's remaining open question is now narrower and cleaner:
is the terminal h_z ramp (now confirmed accurately measured, not a reconstruction
artifact) itself real/legitimate perception behavior, and if so is it the actual soft-
touchdown blocker, and is it fixable. That reframing is the correct next entry point, not
further reconstruction-accuracy work.

## ===== 2026-09-22 (cont'd 4) -- terminal h_z ramp is REAL (confirmed vs GT), correlates with marker overfill =====

Investigated the reframed question head-on: is the terminal h_z ramp (now confirmed
accurately MEASURED per the dt-bug closure above) itself legitimate/GT-accurate, or a
genuine perception error, and is it fixable. Followed the diagnose-flight-data skill's
hard rule: compared against `tools/gt_optical_flow.py`'s independently-computed GT loom
(Z_REG-regularized, valid to touchdown), NOT the controller's own reference.

**Verdict: the ramp does NOT track ground truth -- it's a genuine perception error, not a
reconstruction artifact and not legitimate signal.** `IC1_rep3`: GT loom stays bounded
(peaks ~-0.32 around alt=0.25m, then DECREASES toward 0 as the vehicle physically
decelerates approaching alt=0.20m -- correct real-world behavior), while measured h_z (KF)
ramps the opposite way, from -0.29 to -0.67 over the same window, overshooting GT by up to
2.4x at the worst point (t=11.312s: GT=-0.258, meas=-0.670).

**Mechanism: correlates with marker overfill, not near-grazing rays or ill-conditioning.**
Re-checked the two previously-tested geometric diagnostics (now with the CORRECT dt) across
the exact spike window: `zv_min` stays healthy (0.94-0.97, nowhere near zero -- near-grazing
rays is NOT what's happening in this rep) and `cond(A)` stays modest (9-10, no blow-up).
But `rel_resid` (how well the rigid-body 6-DOF image-Jacobian model actually fits the
tracked point correspondences) roughly DOUBLES in the terminal window vs mid-descent
baseline (median 0.24 mid-descent -> median 0.61 terminal, n=6 sampled frames each). This
exactly coincides with `MARKER_EXTENT_PX` being FROZEN AT 318px for the entire terminal
window in BOTH IC1_rep3 and IC1_rep4 -- i.e. the marker has overfilled the 240px frame
(318 > the detector's own `frame_min=240px` overfill threshold, the same constant the
touchdown-detect v2 "[overfill]" branch fires on) and stays saturated there.

**Direction is NOT consistent -- rules out a simple sign-bias explanation.** Checked
`IC1_rep4`'s terminal window the same way: extent is ALSO frozen at 318 throughout, but
measured h_z UNDER-reads GT there (e.g. -0.127 vs GT's -0.212), the OPPOSITE direction from
rep3's overshoot. So marker overfill degrades the flow solve's ACCURACY generally (matches
the doubled rel_resid -- a poor model fit, which can push the solved Tz either direction
depending on which specific correspondences are noisy that frame), not a deterministic
overshoot bug to patch with a one-line sign/scale fix.

### Is it fixable -- options, NOT yet implemented (needs a design decision, flagging for
### the user rather than picking one unilaterally)
1. **Terminal-proximity hold/clamp on h_z once overfill triggers** (extent>=frame_min) --
   a previous entry in this same file (2026-09-21 era, before this dt-bug detour) already
   flagged this as "a practical mitigation not requiring full mechanism attribution."
   Cheap, doesn't require fixing the underlying flow-solve degradation, but a naive
   freeze/clamp risks masking genuine motion in the exact window touchdown-critical control
   is running.
2. **Fall back to (or fuse in) the scale-loom signal during overfill** -- this file's own
   earlier entries (2026-08 era) note scale-loom "stays reliable exactly where flow
   diverges" for a different investigated case; `_scale_rate_fuse` already exists in
   `cross_marker_perception.py` (default OFF, `CROSS_SCALE_RATE_FUSE`) and fuses into
   channel 2 specifically -- this mechanism may already be positioned to help here if
   enabled/re-tuned, rather than needing new code. Worth testing with it ON before building
   anything new.
3. **Improve the rigid-body fit itself at overfill** (e.g. relax/adapt which points are
   trusted once extent saturates, or switch flow-point sampling strategy near max-extent) --
   more invasive, addresses the root geometric cause (doubled rel_resid) rather than
   papering over its output.

None of these three have been tested yet -- this entry stops at diagnosis (confirmed real,
confirmed correlated with overfill, GT-independent, cross-rep-consistent), not a fix.
