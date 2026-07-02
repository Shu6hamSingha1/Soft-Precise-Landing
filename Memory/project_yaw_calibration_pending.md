---
name: yaw-calibration-pending
description: "RESOLVED 2026-07-02 (was: TODO calibrate the yaw/alpha channel): cal_s[3]=1.0 is CORRECT — alpha tracks GT yaw r=1.00, no scale/offset cal needed; the moving-rover 'yaw cal' task dissolved (see px4/feedback_rover_yaw_cal_resolved — the only turning-target gap is the controller alpha-rate cap, not calibration). This file = the full alpha-investigation history."
metadata:
  type: project
  originSessionId: 7faf44bf-c5f1-4b57-a701-f6d868abfdc1
---

> ✅ **RESOLVED 2026-07-02** ([[feedback_rover_yaw_cal_resolved]], px4/): `cal_s[3]=1.0` is CORRECT
> (alpha tracks GT yaw r=1.00, confirmed again on the rover); stationary-rover yaw verified clean.
> No scale/offset calibration is needed; the only TURNING-target gap is the controller alpha-rate
> cap `PLASMC_YAW_ALPHA_MAX_RATE` (control-side, not cal). This file remains as the investigation history.

**User request (2026-06-03): "we need to calibrate yaw under output calibration later."**

**Current state:** the orientation feature alpha (`Feature Params[:,3]` = `s[3]`, board principal-axis angle) is
UNCALIBRATED — `_sensor_cal_s = diag([1.0986, 1.0562, 1.0, 1.0])` (4th channel = identity) and
`_board_alpha_0 = 0.0` (env BOARD_ALPHA0). It's used raw from the board→V homography.

**Why it's currently uncalibrated (and why that's been OK):**
- Empirically alpha is INERT for the stationary aligned-board landing ([[convergence-ordering]] analysis
  2026-06-03): SP vs non-SP reps statistically indistinguishable in alpha/yaw; yaw error stays ~2-3°; |r| vs
  xy_err ≤0.27. So calibrating it would not move stationary-board landing performance.
- The output-cal aggregator couldn't reliably derive the alpha/yaw scale anyway: the V-frame g-sign bug
  ([[feedback_getvirtualpts_g_sign]]) made the yaw-channel cal correlation +0.02 vs +0.92 for x/y/z, so it was
  left at 1.0 rather than fit a bogus value.

**What a proper yaw calibration requires (the work to do later):**
1. **Fix the V-frame g-sign bug first** (`_getVirtualPts`: `g = R.T @ [0,0,1]`, not `R @ ...`) — otherwise the
   yaw channel's L-matrix is evaluated at a tilt-corrupted V_aruco_norm and the derived cal is garbage.
2. **Excite yaw properly in output_calibration** — bump CALIB_AMP_YAW (default 10°→30°) for SNR; the ω_z cal
   CI is signal-floor-limited at the default amp ([[reference_aggregate_calibration]]).
3. **Calibrate two distinct things:** (a) `board_alpha_0` offset — the steady alpha when hovering aligned over
   the board (so alpha=0 at equilibrium heading); (b) the `sensor_cal_s[3]` scale for the alpha channel.
4. **If switching to V_YAW_SOURCE=alpha** (compass-independent, marker-aligned V): a FULL sensor_cal redo is
   needed afterward ([[v-yaw-source-alpha]]) since the whole L-matrix is then evaluated in a marker-aligned frame.

**Priority:** low for the current stationary-ArUco scenario (alpha inert); becomes REQUIRED for moving/rotating
targets (must track target heading) and is a prerequisite for compass-independent IBVS via V_YAW_SOURCE=alpha.
Also note yaw is a physical rate-loop lag floor (~275ms, [[input-cal-yaw-lag-anomaly]]) — calibrating alpha
improves the yaw SENSING, not the yaw actuation bandwidth.


---

**ALPHA CALIBRATION INVESTIGATED (2026-06-04) — alpha (s[3]) is NOT calibratable as-is: it does not track the drone-vs-board heading.**
Prereq update: the V-frame g-sign bug is ALREADY FIXED in img_data.py (line ~1028, `g = R.T @ [0,0,1]`, 2026-06-01) — not a blocker.
Empirical (output_postreboot recal recordings, YAW phase, drone yaw swept ~70°): image alpha vs GT drone-yaw:
slope = -0.09 (should be ±1), r = -0.04 to -0.29 (raw), r_smoothed = -0.04 to -0.17, best lag-aligned r = +0.15 to +0.22.
alpha noise is TINY (std 0.016-0.021 rad) -> it is a CLEAN signal that simply does NOT correlate with drone yaw.
So alpha cannot be calibrated via offset/scale (board_alpha_0 / cal_s[3]) — there is no correlation to lock onto.
The aggregator already SKIPS s[3] (keeps 1.0, line 271 "axes 2,3 keep at 1.0") — and correctly so, given no signal.
ROOT ISSUE is NOT the alpha math (read _board_feature: it fits a homography from the known board layout to the
V-frame, then measures board +x angle in V — correct). It is the V-FRAME YAW REFERENCE: _getVirtualPts builds V
from the EKF attitude quaternion, and EKF yaw DRIFTS 30-46deg during yaw maneuvers ([[compass-yaw-drift]]). So
alpha is measured in a mis-yawed V -> decouples from true heading. The calibration is confounded by the SAME
compass drift that corrupts the controller's V-frame.
**This explains the IC2-5 yaw runaway** ([[ic-validation]]): the yaw SMC drives psi_d from e_a derived off alpha,
but alpha doesn't represent the true heading error -> the yaw chases a non-signal -> psi_d runs to 60-120deg.
TWO PATHS: (a) fix the alpha computation (why board principal-axis doesn't track V yaw) — needed for moving/
rotating targets; (b) PRAGMATIC for the STATIONARY board: the heading is fixed+known, so HOLD the initial aligned
heading instead of chasing alpha (analogous to CTRL_ZERO_WXY for the angular flow). Note the IC2-5 dominant
failure is OVERSHOOT (braking/K_rd), being tested separately — yaw/alpha is secondary there.

**REFINED (2026-06-04): alpha is confounded by EKF-yaw drift in the V-frame, not a math bug.** To calibrate
alpha OR make yaw control reliable, V needs a drift-free yaw reference -> V_YAW_SOURCE=alpha (marker-aligned,
compass-independent) is the real fix, but it requires the full sensor_cal redo ([[v-yaw-source-alpha]]). For the
STATIONARY board the pragmatic answer remains: hold the initial aligned heading (board orientation is fixed+known)
rather than chase the drift-confounded alpha. Recommend NOT spending more on alpha offset/scale cal until V is
drift-free; calibrating against a drifting reference would bake in the drift.

---

**CORRECTION (2026-06-04, later same day): THE "ALPHA DOESN'T TRACK HEADING" CONCLUSION ABOVE IS WRONG —
IT WAS A CLOCK-MISALIGNMENT ARTIFACT.** See [[imgdata-gt-clock-skew]]. The slope≈-0.09 / r≈-0.1 numbers came
from correlating `Img_Data` (perf_counter clock) against `Ground_Truth` (mission clock) BY INDEX — comparing
different moments in time. Re-derived with both on the GT clock (`Ground_Truth['Img Feature Params']` alpha vs
`Ground_Truth['UAV Pose']` yaw, yaw phase, n=5 runs): **alpha tracks GT yaw at slope = +1.00, r = 1.00**
(alpha ptp 75-80° == yaw ptp 74-79°). **Alpha is a PERFECT heading sensor.** The "EKF-yaw-drift confound in V"
explanation is also wrong (the raw camera-pixel edge angle tracks identically — nothing is lost in V).

Consequences:
- **NO alpha calibration is needed beyond BOARD_ALPHA0=0.0** — already correct. With the geometric-alpha fix
  (below), the steady alpha at aligned hover is 0.000° (std 0.017° over 17 reps).
- **V_YAW_SOURCE=alpha is well-founded** (alpha is reliable); the "needs full sensor_cal redo because alpha is
  unreliable" worry is moot. (A cal redo may still be wanted for the L-matrix frame change, but not because
  alpha itself is bad.)
- **The IC2-5 yaw-runaway attribution to "yaw SMC chasing a non-signal alpha" is WRONG** — alpha is clean, so
  that failure needs re-diagnosis from scratch.

**ALPHA-FLIP FIX (2026-06-04, the REAL bug behind "alpha gives garbage for ArUco board"):** alpha was computed
TWO incompatible ways and the pipeline switched between them per-frame:
- board path (`_board_feature`): geometric board +x dir in V, full 2π, offset 0.0 — this is the GOOD one (r=1.00).
- single-marker fallback (`_getImgFeatures`, used when LK drops the full-corner set, e.g. touchdown): 2nd-moment
  principal axis, period π, offset -0.9379 — INCOMPATIBLE convention.
Switching board->fallback jumped alpha by ~54° (offset diff) + a possible 180° (π-symmetry) = the garbage flips
(all at ~97% flight as n_corners->0). FIX (img_data.py, applied): unified the fallback onto the geometric board+x
convention (TL->TR & BL->BR edges of the axis-aligned marker), full 360°, offset `_board_alpha_0` (0 in alpha-mode);
made `_board_feature` use the same offset logic. Offline-validated: 0 flips (was 2-3), smooth. NEEDS a 2-rep landing
validation before defaulting (changes yaw input) — but strictly removes garbage. wz (yaw rate) is also well-observed
(r=0.75); only wx/wy are genuinely unobservable (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]]).

**!! THE GEOMETRIC-ALPHA FIX WAS REVERTED (commit 0008ba1) — IT CATASTROPHICALLY REGRESSED THE YAW LOOP !!**
(2026-06-04, later). The unified geometric alpha (f193d6d) removed the flips OFFLINE (0 flips, tracks GT r=1.00)
but the 2-rep IC1+IC4 landing validation was a disaster: desired-yaw EA_d[2] maxed to ~180 deg on EVERY rep
(pre-fix was 112-126), xy 2.2-10.4 m (pre-fix 0.74-2.45), vel 4-5 m/s; even centered IC1 went 0.4 -> 2.8 m.
ROOT CAUSE: **the yaw SMC is TUNED to the OLD single-marker-fallback alpha convention** (2nd-moment principal
axis, period pi, offset -0.9379, [4,3,2,1] corner weights) — those are LOAD-BEARING. Switching the fallback to
the geometric board-+x convention (period 2pi, offset 0) changed the sign/scale/period the SMC expects ->
positive-feedback runaway to the +-180 deg equilibrium -> cascades into lateral via PX4 mixer yaw->lateral coupling.
So the "garbage flips" were NOT actually the controller's problem — the controller works fine WITH the flippy
moment alpha (that IS the baseline). Reverted img_data.py to c3f868d; KEPT the harmless yawagg output-cal phase +
3-way notebook cell. **LESSON: offline flip/tracking validation does NOT capture the controller's dependence on the
exact alpha convention — always IC-validate a perception change touching the control path BEFORE trusting it**
(cf. [[feedback_image_center_bug]], same class). A proper flip fix must re-derive the yaw SMC for a geometric alpha,
OR sign/scale/period-match the geometric alpha to the moment convention, then validate IC1 + IC2-5 before merge.
The findings BELOW (alpha tracks GT r=1.00, cal_s[3]=0.994, wx/wy unobservable (OVERTURNED 2026-06-07 — wx/wy ARE observable w/ the spread board; zeroing is a level-target choice; [[wxy-unobservable-imu-fusion-deferred]]), alpha drift-free) STILL HOLD —
they were the analysis; only the controller-side fix was wrong.

**IC2-5 YAW RUNAWAY RE-DIAGNOSED (2026-06-04) — it was the OLD alpha FLIP bug, not "alpha is a non-signal".**
Worst-drift IC4 rep (IC4kp15 rep2, pre-fix), correctly clock-aligned (Control 't' is ABSOLUTE perf_counter, GT
'Time' is relative -> align via gt['Start Time']; landing GT does NOT co-sample 'Img Feature Params', so use
Control_Data 's(t)'[:,3] for alpha — and watch the clock, it bit me twice this session, see [[imgdata-gt-clock-skew]]):
- the drone yawed 112 deg because the CONTROLLER commanded it — desired-yaw EA_d[2] ran to -111 deg (the runaway).
- OLD recorded alpha vs GT yaw: r=0.73 with a 170 deg max deviation = a ~180 deg FLIP (the dual-definition bug).
  The yaw SMC was fed flipped-garbage alpha -> drove psi_d to -111 deg.
- EKF tracks GT (r=0.999) but drifts up to 30.8 deg -> corrupts compass-V (secondary amplifier).
Chain: OLD-flip-alpha -> yaw SMC garbage -> -111 deg desired yaw -> runaway, amplified by EKF V-frame drift. The
geometric-alpha fix (commit f193d6d) removes the flip; V_YAW_SOURCE=alpha removes the EKF-drift part. DECISIVE
test = a landing rep with FIXED code: does psi_d stay near 0? (= pending task 1).

**Off-center landings are NOT gentle in yaw** (corrects an earlier wrong claim of mine): IC4 reps yaw 90-135 deg
during convergence with EKF-GT drift 1.5-30.8 deg. So EKF/compass drift IS operationally relevant off-center
(negligible only at centered IC1). alpha is 100% available throughout landing.

**alpha is DRIFT-FREE under aggressive yaw** (yawagg phase, fresh run with FIXED code, 2026-06-04): alpha tracks
GT yaw at r=0.994 EVEN while the EKF drifts ~30 deg -> alpha follows TRUE yaw, not the drifting compass (the
marker's physical rotation in the camera dominates alpha, so it is inherently compass-drift-immune). This is the
empirical basis for V_YAW_SOURCE=alpha. alpha cal factor confirmed sensor_cal_s[3]=0.994 (~1.0; keep 1.0, within
noise). NOTE: in that run the GENTLE 'yaw' phase gave bad alpha (r=0.23) — a DATA fluke (centroid s0 hit -5.0,
marker off-center in the early phase), NOT a code regression (yawagg in the same run/code was perfect).