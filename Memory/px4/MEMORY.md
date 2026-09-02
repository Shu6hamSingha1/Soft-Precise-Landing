# Memory Index — PX4/Gazebo phase (Phase 2: SITL + img_data + controller.py)

> Per-subfolder index to avoid cross-chat push collisions. The PX4 chat appends NEW
> entries here (../MEMORY.md is the slim auto-loaded CORE — cross-cutting rules only; shrunk 2026-07-02). Topic files for new PX4 work live in this
> folder. Cross-cutting findings go in ../shared/. MATLAB work -> ../matlab/.

> ⭐⭐ [2026-08-31→09-01 SESSION — perception-mode STATIONARY cross-marker landing NOW WORKS for IC1-5](project_20260831_perception_mode_landing.md) — root cause was a STALE `CROSS_ALPHA_0` (90.23° → re-derived to `radians(0.58)` from 5 fresh phased flights, commit `b963e207`): slope-sign flip + a baked-in mount rotation that `_getVirtualPts`'s `[y,-x]` swap already handles. Post-fix gate `test_data/ICValidation/20260831-144626` (n=5): IC2/3/4 ~0.10 m mean, 3/5 precise each (all were 100% TARGET_LOST before); IC1 4/5 good; IC5 4/5 ≤0.14 m, 3/5 precise. Curated IC1-5 videos+datasets committed to `test_data/Final/` (`b89f66b8`, see [[reference_final_landing_recordings]]). Also this session: run() list(int) crash fix, origin-free arm-pixel selection, tdV2 flow-freeze false-fire fix, `_dtheta_correction` removed (→ joint-QP `CBF_AZ_COST_GAIN`), VDS `s_dot_meas` KF glitch gate. **⛔ CBF `Rz_p90b` "fix" `ebb8093c` was REVERTED (`4d7bc210`) — a clean matched A/B proved it regressed EVERY IC (IC2 5/5 ≤0.19 m → 11 m fly-off); at the FoV-box edge it rotates/sign-flips the lateral restoring cmd (8-13°/2% → 55-120°/50%). `validate_cbf.py` 13/13 was misleading. The 90° SMC↔CBF convention question stays OPEN — needs a coupled `Rz_p90b`+`CBF_LW_ROT`+joint-QP rework validated IN SITL, never synthetic-only.** **Rover phase starts from main @ `4d7bc210`.** #1 OPEN stationary blocker (likely relevant to rover terminal): IC5's 1/5 miss is terminal-overfill — LK lateral flow `h_y` corrupts *coherently* (smooth 0→+1.3 ramp, ~150 corners still "tracked", no gate fires) once `MARKER_EXTENT_PX` saturates the frame below ~alt 0.5 m → middle-loop `sigma=h−h_d` explodes → attitude/thrust kick. NEXT: new chat for ROVER moving-target landing experiments (entry: [[project_moving_target_prep]], docs/MOVING_TARGET_PREP.md, [[project_moving_rover_landing_works]]).
> ⭐⭐ [2026-09-01/02 SESSION — MOVING rover_cross perception landing DIAGNOSED (0/5 static, all moving reps stall/fly). NOT a code regression](project_20260901_rover_cross_perception_diagnosis.md) — flat `cross_marker` IC1-5 still land dead-centered on current HEAD; `controller.py`/`cross_marker_perception.py`/`cross_marker_detector.py` have ZERO `WORLD`/rover conditionals. Failure = what the shared code is FED. **Cluster A (offset IC2/3/5): oblique-view detector collapse** (`centroid_mismatch` from alt 5 m — rover wheels + 3 m-marker overhang edge over the 0.6 m platform). **Cluster B (IC1/4): terminal overfill loom collapse, descent stalls ~1.1 m** (detector fine; same as the stationary #1 blocker, +0.6 m higher). Scene causes: (1) two-instance camera warm-up ~7 s frozen-grey feed — ABSORBED by `TARGET_IS_VISIBLE`, not blocking; (2) platform box `<diffuse>` 0.25→**0.6 FIXED** (V 76→153, was passing `inRange(V≤100)`; helps IC1/IC4 → dead-centered); (3) = Cluster B. **Control-side terminal stall (moving): single-frame centroid spike `s_x` 0.16→0.91→0.16 → `dh_d`±11 → `theta_desired` 92° → CBF FoV clip → joint-QP `CBF_AZ_COST_GAIN=5.0` folds ~5 m/s² UP into `I_a[2]` → `B_T`→0 → stall.** ⛔ Corrections: the terminal `h_z` loom "inversion" is FAITHFUL (corr +0.89 vs the run's OWN GT), a symptom not cause; raw reduced-lstsq `Tz` does NOT fail at overfill in `CROSS_BG_FLOW=1` (cond≈1.2, agrees w/ GT); `rho_fov` is NOT the hard bound (`p_10=center/focal` is); moment-loom is wrong for cross-marker but was NOT the IC5 cause (detector never locked). **Committed `8ed004ad`:** `CROSS_S_JUMP_GATE`(0.35, DEFAULT ON — kills the spike class, removes the early stall, does NOT fix the landing), `CROSS_ADAPT_GATE`(DEFAULT OFF — CLAHE+adaptiveThreshold; detOK 49→83% on rover_cross IC2 but 27% of extra dets are >20 px off GT = confidently-wrong fits, DO NOT BAKE), `CROSS_EXTENT_FROM_LINES`(OFF), `tools/validate_detector_gt.py` (offline GT-scored detector harness). See [[feedback_cross_detector_contrast_not_darkness]].
> ⭐ [Cross detector must key on CONTRAST-to-background, not darkness](feedback_cross_detector_contrast_not_darkness.md) — 2026-09-01 (user): `inRange(V≤100)` fails on ANY dark object near the marker — PROVEN in a controlled empty-world test (one dark box → IC2 detOK 100%→10%, flight can't start; `cross_marker_clutter.sdf` kept for the fix work). Right design: contrast-based + polarity-agnostic segmentation → perpendicular intensity-profile stroke validation (ridge/valley vs step) → geometry-first confirm. `tools/validate_detector_gt.py --set test_data/DetectorFrameset` is the GT-scored harness (6 sets: flat/clutter/rover). **2026-09-02 (user-measured root cause of Cluster A): not the V≤100 gate — `_isolate_marker_by_shape` scored `-area` and picked a diagonal platform-shadow-bar slice (fill 0.35) over the thin cross (fill 0.07), 78/128 frames. FIXED `ee858086` with a fill-ratio band [0.02,0.25]: flat unchanged, rover_IC4 detOK 78→88%, rover_IC2 45→59%; clutter still open (box+shadow+cross MERGE into one component after the Hough dilate).**
> ⭐⭐⭐ [PRE-FLIGHT CHECKLIST — the analysis mistakes Claude repeats on this project](feedback_recurring_analysis_mistakes.md) — 2026-09-02 (user-requested). ~85 of ~200 memory files record a correction; the same shapes recur. 8 classes, each with dated instances + the ONE check that catches it: (1) log↔log TIME ALIGNMENT (`np.interp` CLAMPS outside the GT window — 38% of one run scored against frozen values, detOK 49% vs a true 18%); (2) pairing assumed from directory NAMES (confirm by mtime + matching a plotted trace); (3) reference-frame/offset — **on the rover `uav.z-target.z ~ 0.49` MEANS LANDED** (this one condemned 34 real landings, then fooled me again an hour later); (4) metric sampled at the wrong instant with no absolute gate; (5) confounded / NON-OVERLAPPING comparisons sold as mechanisms; (6) one-sided metrics — a catch rate without its false-positive rate, a log line without its BASE RATE (`Disarming denied` = 97.4% of genuine landings); (7) stale derived docs quoted instead of source; (8) unverified baselines under concurrent sessions. **⚠ Meta: recording a trap does NOT prevent re-committing it — run the checks BEFORE concluding.**
> ⭐⭐⭐ [CLUSTER A ROOT CAUSE: `centroid_mismatch` is 79% of cross-marker rover failures](project_20260901_rover_cross_perception_diagnosis.md) — 2026-09-02: the 9 static-offset perception `rover_cross` runs detect at **0.0% from 5 m down to 1 m** (live `Detection Status`; `FEATURE_IS_VISIBLE` tracks it) → controller flies OPEN-LOOP the whole descent → lateral never closes (2.9→2.3 m) → drops 0.8-1.5 m/s onto the ground BESIDE the pad. Disabling ONLY that gate: rover_IC2 **28.5→95.9%**, clutter_IC2 **50→97.5%**, flat bit-identical, median centroid err flat-to-better — so it is rejecting GOOD fits. ⚠ not free: rover within-0.15 95.9→76.2%. Mechanism: the gate checks the fitted intersection against the **MASK PIXEL CENTROID**, which merged platform structure corrupts; the `ee858086` fill band and the compact-blob strip both miss it because the rover contaminant is an ELONGATED bar (aspect 3.9-5.1). **FIX LANDED `f49f567f`: `CROSS_CENTROID_SPAN_RESCUE` (DEFAULT OFF)** — legacy check as fast path, and only on failure ask whether the intersection projects INSIDE both fitted arms' inlier spans (clean scenes never reach it → provably inert; flag-off reproduces legacy exactly). rover_IC2 28.5→**94.8%**, clutter_IC2 50→**87.7%**, flat bit-identical. ⚠ buys RECALL not accuracy (rover within-0.15 95.9→75.9%) → needs an n≥5 SITL gate. ⛔ The obvious alternative — validate against the arms' inlier CENTROID — was tried and is WORSE (28.5→**11.0%**); asymmetric arm spans put that mean off the junction. Don't re-try. ⛔ Also corrects this session: 0/18 static-offset runs landed ON the pad (altitude alone never proves a landing on a RAISED pad — test the footprint), and the Mode-B ~0.5 m stall is CONTROL-side, not perception (measured loom tracks own GT at +0.76/+0.81).
> ⭐⭐ [GT-FEEDBACK ABLATION: static rover_cross DOES land — the DETECTOR is the blocker](project_20260901_rover_cross_perception_diagnosis.md) — 2026-09-02: `PLASMC_GT_FEEDBACK=1` on static `rover_cross` lands dead-centred (**3/3 genuine landings**, `min_alt` 0.487-0.51 = **0-0.01 m above the 0.50 m platform**, xy 0.007-0.010). So the platform is landable and the perception-mode failure is the detector — confirmed independently by flat-vs-clutter at matched obliquity (one dark box: detOK 100→60%). **⛔ CORRECTS this file's "Cluster B = terminal overfill" attribution** — GT-FB reaches 0.51 m fine and its loom comes from truth, so overfill cannot bite; the perception-mode `rover_IC4` stall is downstream of the detector. ⚠ only 3 GT-FB runs, all centred — an offset-IC (IC2/3/5) GT-FB sweep is NOT yet run and is where the perception failure lives. Platform constant, verified empirically: settled rover landings rest at **rel-z 0.487** (mode 0.5, n=12) because the target pose is the rover BASE, not the surface. ✅ `Test_Videos/montage_rover_static_ic1*.mp4` ARE correct (an earlier claim that they were mislabelled is RETRACTED) — built from `Landing_Test/Thu Aug 27 00-21-14`, a genuine 0.487 m landing. ⛔ Two traps behind that error: never infer a video↔run pairing from directory names (check mtimes + match the plotted trace), and on the rover **`|rel. position| ~ 0.5` MEANS LANDED**, not airborne.
> ⭐⭐ [ARCHIVE RE-SCORE: 13.4% of all 'precise' landings were scored MID-AIR](project_20260902_archive_rescore_false_precise.md) — 2026-09-02: re-scored all 4446 saved runs against a SURFACE-relative altitude floor (flat 0.20 m / rover 0.70 m — a legit landing on the 0.4 m rover platform sits ~0.48 m above the rover target-pose origin, so 34 runs were rescued by getting this right). **159 of 1192 `precise` verdicts are unsupported**: 21 ABORTED (a centered IC starts above the marker → `xy_err`~0 at t=0) + 138 whose GT log stopped MID-DESCENT (112 still descending, median last alt 0.486 m — a touchdown detector false-latching at altitude). Bounded to **Jul 19-23 and Aug 22-24**; `ICValidation/20260828` (27/0), `20260829` (103/0) and the post-alpha0 gate `20260831-144626` (13→12, its one flag marginal at 0.211 m) are CLEAN — the IC1-5 stationary result STANDS. `test_data/Final/IC5` is the published casualty ("precise 0.058 m" scored at 0.422 m alt → Final is 1/5, not 2/5, see [[reference_final_landing_recordings]]). **FIX IMPLEMENTED `d6610ea7`**: `NOT_LANDED` branch gating precise/soft on the LOWEST altitude above the landing surface (<=0.20 m), + a surface-agnostic `min_alt<=0.15*start` backstop (0 FP but fires 0 times today — insurance only), + `terminal_state_ok`/`not_landed_reason` recorded, + `tools/rescore_softprecise.py` (non-destructive). Surface height REUSES `PLASMC_GT_MARKER_DZ` (0.0 flat / 0.5 rover) — grep for an existing world constant before adding one: a flat floor would have flagged EVERY rover landing as NOT_LANDED. ⛔ REJECTED after measuring: `|dz/dt|` rest gate (catches 152/159 but rejects **58% of LEGIT landings**) and IMU contact-spike (no separation, 27.0% vs 26.7%). No perception/control change; `coord_descent_tune.py` is the one consumer that closes a loop on the flag. STILL OPEN: campaign/.ods/manuscript SP rates not re-scored.
> ⭐ [Offline detector-replay gotchas: recorded frames carry a debug OVERLAY; detOK% inflated by ground frames](feedback_detector_offline_replay_gotchas.md) — 2026-09-02: `CROSS_RING_OVERLAY_DBG` (default ON) bakes ~3-5% drawn flow points into recorded PNGs — **live detection unaffected**, but offline replay is contaminated and **15.5-20.9% of the `CROSS_ADAPT_GATE` mask is overlay halo** (legacy dark gate 0.8-3.7%). Measured NOT to explain adapt's ~7-8 pt accuracy deficit (survives inpainting) but shifts absolutes ~3-8 pts → re-record eval sets with `CROSS_RING_OVERLAY_DBG=0`. Also: headline detOK% is dominated by post-touchdown ground frames (rover_IC2 133/310 below 0.7 m at 98.5% vs **20% at 4-6 m**) — judge on the 4-6 m ACQUISITION band; and hand-paired `runs`+`--frames` scores frames past GT end against clamped values (38% on one run: true 18% read as 49%) → use `--set`.
> ⭐ [Verify what your A/B BASELINE actually contains before believing a null result](feedback_ab_baseline_verify_concurrent_commits.md) — 2026-09-02: another session committed the fill-band fix (`ee858086`, default ON) 3 min before this session's A/B ran, so the "baseline" arm was already fixed → a real fix measured as 0% delta and got wrongly retracted. With the true legacy (`CROSS_ISOLATE_FILL_HI=1.0`) it is rover_IC2 44.5→59.0%, rover_IC4 77.8→87.9%. Rules: re-check `git log` right before an A/B (a null result is the trigger), pin control arms by explicit revert env not "the default", echo the live constant per arm. Plus: check a recording's mtime against OUT-OF-REPO scene-fix `.bak_*` timestamps before treating it as current.
> [Perception touchdown-detect V2 IMPLEMENTED](project_20260830_perception_touchdown_detect_broken.md) — 2026-08-30: `_touchdownDetectV2` in controller.py, cross-marker perception only, `PLASMC_TD_V2=1` default (`=0` reverts). No `|s_e_n|` gate. 3 paths: overfill (N_corn collapse+extent), backstop (extent+centroid frozen 3s), flow-freeze (bg px-disp high→low, catches off-marker settles). Unit-tested 3/3, NOT SITL-validated. IMU `_impactDetector` (|a|>50) + PX4 LandedState remain the hard-contact backstops.
> ⛔ [Perception-mode touchdown-detect is BROKEN — blocks perception-mode landing](project_20260830_perception_touchdown_detect_broken.md) — 2026-08-30: both paths in `_touchdownDetect` unvalidated (GT-FB bypasses via `_td_gt_*`). Replay on 25 clean GT-FB landings: loom-independent 2nd path (`_td_ext_only`) FALSE-FIRES at altitude in 23/25 (`_extentTouchdownProximate` uses running-max, ~always true on a monotone descent); loom-spike 1st path never fires in 22/25. Added `PLASMC_TD_EXT_ONLY` env (default on) to gate the 2nd path; with it off, nothing latches touchdown at all. Fix = re-base proximity on frame size + loosen flattened window; revisit 1st-path h_z criterion.
> [Cell-ID bug FIXED + IC5-VALIDATED — center-bridge now engages on the real `CROSS_BG_FLOW=1` default](project_20260824_ic5_angle_clustering_and_hang_investigation.md) — 2026-08-29: added a cell-ID-independent `_prev_flow_uid` match key in `cross_marker_perception.py`; `_tryCenterBridge` matches by uid not cell_id. A/B at real defaults: IC5 **20/20 SP (2×n=5) with bridge=10 vs 12/18 bridge=0** — FIRST candidate in this investigation to survive a confirm sweep, with a mechanism (37-70 bridge accepts/rep, `lt2_angle_clusters` spike suppression). IC1/2/4 bridge=10 = 9/9 SP clean. **IC3 n=5 A/B (re-run clean): 4/5 vs 4/5 bridge-off — NEUTRAL, no regression** (the earlier 1/3 + 16.5m fly-away did NOT reproduce — infra flake). IC2-5 gate effectively CLEARED. Code UNCOMMITTED. **Recommendation: land the uid fix AND flip `CROSS_CENTER_BRIDGE_FRAMES` default 0→10** (pending user go-ahead).
> ⛔⛔ [GT touchdown detector false-fired at altitude in an entire 11-run IC/motion batch; fixed + exposed a real kappa-ratchet](project_20260828_ic_motion_batch_gt_touchdown_and_kappa_ratchet.md) — 2026-08-28: don't trust ANY "on-platform" GT-feedback result from 2026-08-26/27 (the touchdown fix era) without re-checking against the depth-ceiling fix (`PLASMC_TD_GT_MAX_DEPTH`, default 0.25m); IC1 rover_cross now shows a real w_u-saturation→kappa-ratchet divergence (tune-plasmc mode 11), unexplored for this gain set — video batch BLOCKED on this
> [A relative-flatness check alone never proves a terminal state; pair with an absolute-value gate](feedback_relative_flatness_needs_absolute_gate.md) — general lesson from 2 consecutive GT-touchdown-detector bugs (2026-08-26, 2026-08-28)
> [Don't disable cast_shadows / move sunUTC to fix shadow-on-marker perception issues](feedback_reject_disable_cast_shadows.md) — 2026-08-25, user rejected; real drones cast real shadows, harden perception instead
> ⛔ [Overlay tool now logs real detection artifacts, not offline reconstruction](project_20260825_overlay_detection_artifact_logging.md) — CONTRADICTED 2026-08-27: live `cross_marker_perception.py::getLogData()` does NOT expose "Line Points I/J"/"Stub Points"/"Flow Points Prev/Curr Px" as of that date — verify directly before trusting this line; every overlay checked 2026-08-26/27 used the offline re-detection fallback
> [Chase-cam bumped to 1280x960](project_20260826_chasecam_resolution_bump.md) — fixed blurry montage video; validated no down-cam fps impact (rover_cross.sdf, outside repo)
> [Camera dropped 640x480→320x240 (fx 270→135), frame rate + h/texture design gap](project_20260827_framerate_and_h_texture_investigation.md) — 2026-08-27: restores MATLAB-native scale, fixes <30Hz process_frame() rate (n=3: ~15-23Hz→~38Hz) but detect-rate degraded + sensor-cal NOT recalibrated (BLOCKING for real use); ⭐ h never used textured background (design-intent gap, s=geometry/h=surface-texture-flow) — FIXED+VALIDATED same day: extent-ROI + line-exclusion + multiscale GFT, fed through the REAL `_solve_jacobian`, gives strong real GT correlation (r=0.66-0.76) when texture grain is coarse enough to survive the camera resolution (fine grain: weak, a resolvability limit not an algorithm defect)
> ⛔ [Img_Data vs Ground_Truth clock-skew avoidance advice is now PARTIALLY SUPERSEDED](../feedback_imgdata_gt_clock_skew.md) — 2026-08-27: a validated direct fix exists (`Img_Data['Time'] - gt['Start Time']`, confirmed Control_Data['t'] shares this same clock) — don't just avoid the comparison, see the file's own correction note before assuming "index-correlating is invalid" means "don't compare these logs at all"

> **⛔⛔ HARD RULE, RE-STATED 2026-08-25 AFTER A REPEAT VIOLATION: default every landing
> test to `MARKER_TYPE=cross WORLD=cross_marker`. ArUco (`MARKER_TYPE=aruco`/default) is
> COMPARISON-ONLY — use it ONLY when the user explicitly asks for an ArUco run.** This
> rule already existed ([[feedback_aruco_perception_scope]], [[project_marker_roadmap_gt_ablation]],
> since 2026-08-23) but was buried near the bottom of this file and got violated anyway,
> repeatedly, across an entire session (2026-08-24/25 IC5 fly-away/CBF-margin investigation
> — every single test launch defaulted to bare ArUco with no `WORLD`/`MARKER_TYPE` override,
> including after the mechanism was already confirmed general-not-cross-marker-specific,
> where there was no remaining reason to stay on ArUco at all). Lesson: a correctly-worded
> rule buried mid-file does NOT survive a long session's accumulated momentum — once a
> command pattern is copy-pasted into repeated launches, nothing re-triggers a check
> against it. **Before EVERY landing-test launch, silently re-verify: does
> this command include `WORLD=cross_marker MARKER_TYPE=cross`? If not, is the user's most
> recent instruction an explicit ArUco request? If neither, add the cross-marker override
> before running, don't launch first and rationalize after.** See
> [[feedback_aruco_perception_scope]] for the full history and rationale.

> **⭐⭐⭐ 2026-08-25 SESSION WRAP-UP — READ THIS FIRST, then follow its links.**
> Committed (`d8b06cf`, `a87ac00`): cross-marker hw coast+freeze KF + touchdown rolling
> window + corner-join Hough filter, all validated under GT-feedback only. **First
> perception-mode test found a NEW, unfixed bug**: `TOUCHDOWN-DETECT` false-fired at
> **3.75m altitude** (not ground) — real perception noise can spike `h_z` positive for 3
> frames at ANY altitude; GT-feedback validation never exercised this because GT `h_z` is
> noise-free by construction. Same rep also hung past 180s with SITL CONFIRMED clean
> (weakens the earlier "hang = concurrent-session contamination" theory). IC5
> angle-clustering finding is DOWNGRADED/unconfirmed (likely contaminated by a concurrent
> session's own SITL use). `model.sdf` ground-collision height is DELIBERATELY at true
> ground contact — user directive, ongoing work, do NOT revert without new instruction.
> Full detail + ordered next-steps: [[project_20260825_session_wrapup_touchdown_hardening]].**
> ⚠ UPDATE: an uncommitted `HW_FROZEN` fix for exactly this false-touchdown bug now sits in
> the working tree (`controller.py`/`cross_marker_perception.py`/`img_data.py`) as of a later
> point the same day — excludes frozen coast+freeze-KF frames from the touchdown spike-streak
> window. NOT committed by this entry's session; check its validation status before assuming
> it's done.

> **⭐⭐⭐ 2026-08-25 SESSION WRAP-UP (separate thread, same day) — dtheta/CBF_MARGIN_RESERVE
> investigation. Full detail: [[project_20260825_dtheta_cbfmargin_session_wrapup]].** Traced
> IC5 perception failures to 3 distinct causes: (1) `dtheta`'s self-defeating attitude-history
> loop through `cbf2_filter`'s `th_curr` (confirmed, UNFIXED), (2) the uncapped-`dtheta`
> correction fly-away (band-aid mitigated via `PLASMC_DTHETA_AZ_CAP`+`PLASMC_DTHETA_HREF`
> together, NOT root-fixed), (3) a genuine perception FoV-margin gap — marker corners
> physically exit frame under real perception at IC5, confirmed on BOTH marker types, absent
> under GT-feedback. Attempted fix `CBF_MARGIN_RESERVE` (proactive Phase-1 margin reserve):
> mechanistically sound, confirmed engaging, but a CLEAN n=3 sweep found NO evidence it helps
> (reserve=1.0 actually showed a HIGHER TARGET_LOST rate than default). NOT baked. ⚠ that
> result is ArUco-only (see the hard rule above — this thread violated it repeatedly) and has
> NEVER been checked on cross-marker.

> **⛔ HARD RULE (2026-08-25): check for concurrent SITL use/other active `claude` sessions
> BEFORE launching or force-killing any PX4/Gazebo process.** Multiple `claude` sessions can
> run on this machine simultaneously — caught live when a kill loop during launch-flake
> troubleshooting had no `grep -qa claude /proc/$p/cmdline`-style ownership guard on the
> `px4_sitl`/`landing_test.py`/`MicroXRCEAgent` kills (only `gz sim` kills had it, copied
> from the launcher scripts' own `stray_clean` pattern). See
> [[feedback_check_concurrent_sitl_before_launch]] for the full checklist.**

> **⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (implemented) — `CBF_JOINT_QP`: solves the visibility box
> DIRECTLY for I_a (not theta) INSIDE `cbf2_filter` itself, superseding `PLASMC_AZ_JOINT`'s
> downstream-patch approach.** Derivation: `theta = P@I_a[:2]/a_z` (`P` = the same forward
> rotation already used), so the box's Jacobian w.r.t. `I_a[:2]` directly is `M=Lw2@P/a_z` --
> the SAME alternating-projection algorithm runs unchanged, just on `I_a[:2]`, interleaved
> with a sphere projection (`|I_a+g*e3|<=A_CAP`, full 3-vector) each outer iteration, so
> `a_z` and lateral are genuinely solved TOGETHER, not sequentially. **Offline: 200 trials
> confirmed MACHINE-PRECISION (9.16e-16) equivalence to the theta-based path when not
> saturating; 0 sphere violations in 200+ trials.** **Caught a SECOND instance of the exact
> same blowup bug class found in `PLASMC_AZ_JOINT`'s first draft** (500-trial extreme-input
> stress test hit `theta_cone=3.52 rad`) — the sphere bounds thrust MAGNITUDE, not the
> derived angle RATIO. Fixed with the same `a_z`-aware sanity clip
> (`arccos(a_z_final/A_CAP)`) validated before, re-confirmed bounded (max 1.563 vs pi/2)
> across 1000 extreme trials with equivalence still exact. `tools/validate_cbf.py` 12/12
> throughout (default path unaffected). **SITL smoke test (n=3, IC5): 2/3 SP, worst miss
> 1.22m — the mildest result of ANY variant tested this session**, no dtheta outliers, only
> a mild kappa rise (1.42, not a ratchet) on the one miss. n=3 only, not yet n>=5 confirmed,
> but the strongest current candidate given both the result AND the more principled
> architecture (joint solve in the barrier itself, not a patch). **Process lesson (now
> doubly confirmed): any a_z/deliverability mechanism in this codebase needs its OWN
> explicit angle-sanity bound — bounding thrust magnitude alone is not sufficient — and
> extreme/edge-case offline stress-testing (not just realistic-range checks) is what
> catches this class of bug before SITL.** Full data:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (BAKED) — `CBF_JOINT_QP` is now the DEFAULT
> (env default flipped 0->1 in `cbf_visibility.py`; still requires `A_CAP`, which
> `controller.py` always passes, so this is what actually runs there now).**
> `controller.py`'s downstream fixed-hover `theta_cap` re-clip (the old `PLASMC_AZ_JOINT`
> block) is now SKIPPED whenever the joint QP ran (gated on the same `CBF_JOINT_QP` env,
> default true) — that block's fixed-cap re-clip would otherwise fight the joint QP's own
> `a_z`-aware clip (redundant, or wrongly tighter when `a_z` sits below hover).
> `PLASMC_AZ_JOINT`/fixed-cap path stays available for `CBF_JOINT_QP=0` A/B. **Re-validated
> post-flip: `tools/validate_cbf.py` 12/12 (no `A_CAP` in that harness -> exercises the old
> path, unaffected). SITL (`run_ic_validation.sh IC_LIST=IC5 N_REPS=3`,
> `PLASMC_GT_FEEDBACK=1 CBF_HZ_AWARE_DRIFT=1`, `CBF_JOINT_QP` now the unset/default-on
> path): 2/3 SP, xy_err 0.018/0.018/1.574 — matches the pre-flip validated number exactly.**
> Process note: two earlier "regression" smoke tests this session were false alarms — one
> ran the ArUco pipeline (`cbf_visibility_aruco.py`, a SEPARATE untouched copy; MARKER_TYPE
> defaults to `aruco`, not `cross` — `CBF_JOINT_QP` only lives in the cross-marker file), the
> other ran the right pipeline but omitted `PLASMC_GT_FEEDBACK=1` (the standard IC-validation
> harness config) — always match the exact env/launcher (`run_ic_validation.sh` +
> `PLASMC_GT_FEEDBACK=1` + `WORLD=cross_marker MARKER_TYPE=cross`) a prior result used before
> treating a new number as a regression.
>
> **⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 — `lt2_angle_clusters` fail_reason breakdown now
> PERMANENTLY logged** (`cross_marker_perception.py`'s `_fail_reason_log`, exposed as
> `"Fail Reason"` in `Img_Data.npy` — previously only derivable via one-off scripts reading
> `get_diag_log()` per investigation, per [[feedback_cross_marker_detection_flicker]]'s "Not
> done" item). Re-confirms (does NOT newly discover) the 2026-08-25/26 finding:
> `lt2_angle_clusters` dominates every IC5 rep's misses and is the ONLY fail_reason that
> scales with outcome (6.1% best rep → 20.4% the one miss, this n=5 batch; up to 68% in the
> worst historical reps) — every other reason stays flat regardless of outcome. Also found:
> the z_v-rejection miss branch (`process_frame`) never calls `_log_frame_data` at all, so
> those frames are invisible to EVERY log in the class, not just this new one — not fixed,
> real gap if that fraction turns out material. Full data:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (GATED) — full IC1-5 n=5 confirm of the
> `CBF_JOINT_QP` default: IC1-4 perfect (5/5 SP each, sub-3cm), IC5 4/5 SP (best full-gate
> IC5 result to date, single 1.81m miss).** Config: `PLASMC_GT_FEEDBACK=1
> PLASMC_DTHETA_HREF=1 CBF_HZ_AWARE_DRIFT=1`, `CBF_JOINT_QP` at its new default (unset->on),
> via `run_ic_validation.sh IC_LIST="IC1 IC2 IC3 IC4 IC5" N_REPS=5`. Zero regression outside
> IC5; IC5 itself improved over every prior isolated n=3 test this thread. This combination
> is now the strongest validated full-gate config. Full table:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]].
>
> **⭐⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (implemented, superseded by CBF_JOINT_QP above) —
> `PLASMC_AZ_JOINT`: a real bug caught+fixed
> pre-merge, validated SAFE but LOW PRACTICAL IMPACT.** First draft fully skipped the
> hover-assumed `theta_cap` angle clip, not realizing it does double duty (deliverability
> bound AND the only thing preventing a degenerate QP output from reaching the attitude
> command undamped) — SITL smoke test hit `theta_cone=21.68 rad` (nonsensical) and 0/3 SP.
> **Fixed**: never skip the angle clip; make its bound `a_z`-aware instead
> (`arccos(a_z_current/A_CAP)`, always finite 0..pi/2, exactly matches the old constant at
> hover, tightens correctly as `a_z` moves away from hover). Re-test: 2/3 SP, no
> catastrophic failures, `theta_cone` confirmed bounded throughout. **But the joint
> constraint barely engages (0-1/600-1400 frames per rep) — `a_z` never nears `A_CAP` in
> these failures, consistent with the earlier physics correction (the ratchet isn't a
> saturation effect).** Implemented, safe, principled (worth keeping on its own
> correctness merits), but NOT the fix for the remaining IC3/IC5 gap — that likely needs
> either the `th_curr` self-defeating-loop fix or a Phase-2 minimum-authority floor (see
> the detection-outage finding below), not more `a_z`-budget work. Full data:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (later still) — TWO INDEPENDENT triggers for the IC3/IC5
> kappa-ratchet, not one.** Traced `IC5_rep4` (large `dtheta_az` outlier, 11.15 rad, but
> landed clean) vs `IC5_rep5` (ignited) from the `CBF_HZ_AWARE_DRIFT` confirm sweep — run
> WITHOUT `DTHETA_HREF`, so the `h_ref` leak mechanism below can't be the cause here. Real
> discriminator: both reps track identically to t~2.0s then bifurcate — rep5's `s_e_n_y`
> diverges well BEFORE its dtheta spike even appears (the spike is a late SYMPTOM, not the
> trigger). Root cause: `cbf2_filter` fell into Phase-2 (decode failed) in both, but rep5 hit
> a genuine ~5s (88% of flight!) near-total detection outage from an already-marginal
> last-known position, freezing `theta_cone` at EXACTLY 0 for 99.3% of that window (Phase-2's
> `effective_margin` clamped to 0 — the CBF working AS DESIGNED, conservative, not a defect).
> Zero lateral authority for 5s while off-center → `s_e_n` grows unbounded → ratchets once
> authority returns. Rep4's outage was short (2s) from a healthy last-known position
> (`theta_cone` stayed near-ceiling, 0.766). **This connects to the detection-reliability
> thread (`lt2_angle_clusters`/oblique fragility), NOT a CBF-design or `dtheta`/`h_ref`
> issue — the joint `a_z`-QP redesign (below) would NOT have prevented this specific
> failure.** Candidate mitigation, not yet built: a minimum-authority floor on `theta_cone`
> during Phase-2 (never let it hit exactly 0), trading a little conservatism for avoiding a
> total lateral deadlock — a different lever from the drift-model fix or the joint-`a_z`
> work. Net: TWO real, independent, still-open triggers for the same ratchet symptom. Full
> data: [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐⭐⭐ 2026-08-29 (later same day) — `CBF_HZ_AWARE_DRIFT` prototype (default off):
> fixing the CBF's own drift model (constant-velocity `dft=tau*d` → closing-rate-aware
> exponential, driven by the scale-free `h_z` loom proxy, `cbf_visibility.py`) substantially
> reduces the IC3/IC5 ratchet WITHOUT `DTHETA_HREF` at all.** n=5 IC3+IC5 confirm (GT-FB):
> IC3 4/5→**5/5 SP**, IC5 3/5(worst 5.46m catastrophic)→4/5(worst **1.71m**, `TARGET_LOST`
> fallback catches it, no hard impact). Combined 7/10→9/10 SP, zero regression, `tools/
> validate_cbf.py` still 12/12, exact backward-compat at `h_z=0`. **NOT fully fixed** — the
> one remaining IC5 miss still shows the ratchet (milder: `a_u_y` to 72 vs -152/+378).
> User's parallel proposal (not yet built): bring `a_z` into the SAME constrained QP as
> `a_x`/`a_y`, since `theta_max = arccos(a_z/A_CAP)` means growing `a_z` (the old dtheta/
> `h_ref` "extra lift" approach) physically SHRINKS deliverable lean at a fixed thrust
> ceiling — it was never reliably going to help. Full data + next steps:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐⭐ 2026-08-29 (CORRECTED same day) — IC3/IC5 gate failure ROOT-CAUSED: NOT a new
> lateral-drift bug, it's the kappa-RATCHET (failure mode 11) reignited by
> `PLASMC_DTHETA_HREF=1` modulating a PRE-EXISTING, CORRECT `h_ref`→lateral coupling.**
> `project_20260828_kappa_ratchet_campaign` confirmed the ratchet resolved in the current
> baked config — but never tested with `DTHETA_HREF=1` on. Confirmed via `Control_Data.npy`:
> `kappa_y` 0.17→4.3 (IC5) / 0.08→10.8 (IC3), `sigma_y` diverging past -9, `a_u_y` to
> -152/+378 — the classic ratchet signature. **Mechanism, corrected after user pushback**:
> `controller.py:2390`'s `h_d_ff = (h_ref_eff - dot(cross_ws,e3)) * s[:3]` — multiplying
> `h_ref_eff` into the FULL `s[:3]`, not just z — is NOT a bug; it's the exact, intentional
> MATLAB-ported design (`visualControl_IBVS_adaptive.m:665-666`, `h_rd` also multiplies the
> full `V_s(1:3)`), present since 2026-06-01, ~3 months before `DTHETA_HREF` existed. What's
> actually new (2026-08-24) and risky is `DTHETA_HREF`'s `dtheta_href_g` — a NEW time-varying
> gain on `h_ref_eff`, driven by `dtheta_az`, which is itself UNCAPPED (only the gain-scaled
> `dtheta_correction` is capped at 2.0) and can spike to outlier magnitudes (8.175 rad
> measured) right at the FoV edge (IC3/IC5-specific geometry) — collapsing `dtheta_href_g`
> transiently and injecting a lateral perturbation, THROUGH the correct pre-existing
> coupling, at exactly the moment `sigma` is most vulnerable, igniting the ratchet. IC1/IC2/
> IC4 don't trigger it — gentler viewing geometry, `dtheta_az` rarely spikes that hard there.
> **NOT YET FIXED — fix target is `dtheta_az`/`dtheta_href_g`'s per-cycle bound (cap or
> rate-limit), NOT the `s[:3]` coupling** (an earlier draft of this finding wrongly proposed
> restricting `h_ref_eff` to `s[2]` only — that would break the validated MATLAB coupling for
> every other caller, not just this one). **Do NOT bake `PLASMC_DTHETA_HREF=1`
> unconditionally** — safe at IC1/IC2/IC4, unsafe at IC3/IC5 until a fix lands.
> Cross-referenced into both `project_20260828_kappa_ratchet_campaign` (caveat added) and
> `project_20260824_crossmarker_offcenter_convergence_wall` (distinguished — that one is
> kappa leaking DOWN/bounded, this is kappa RATCHETING UP/unbounded, don't conflate). Full
> mechanism: [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐ 2026-08-28 (superseded misdiagnosis, kept for history) — IC1-5 GATE for
> `PLASMC_DTHETA_HREF=1`: target mechanism CONFIRMED
> fixed, ZERO regression at IC1/IC2/IC4, but IC5 STILL FAILS via a different, unfixed
> lateral-drift mechanism — do not describe IC5 as solved.** Full 25-rep gate (n=5/IC): IC1
> 5/5 SP (0.003m), IC2 5/5 SP (0.019m), IC4 5/5 SP (0.023m) — all clean, no regression. IC3
> 4/5 SP (1 miss, 1.99m). **IC5 only 3/5 SP, worst miss 5.46m — the largest xy_err of the
> whole investigation.** Checked all 3 misses (IC5×2, IC3×1): **zero show the `ASCENDING`
> cap-pinning signature** (`descent_anomaly_cause='N/A'` in all 3, pinned-run max 12 cycles
> vs the 55-74 catastrophic threshold) — so the fix is doing exactly what it targets. IC5's
> 5.46m miss instead traced to a genuinely different mechanism: GT altitude descends cleanly
> the WHOLE flight (no climb), but a rapid LATERAL drift starts ~t=7s (y: +0.07m→-4.55m by
> t=11s, ~1.1 m/s sustained divergence) — unrelated to `dtheta_az` (pinning stayed mild).
> **Net: `PLASMC_DTHETA_HREF=1` is safe to bake re: IC1/IC2/IC4 regression and does fix its
> target mechanism, but IC5 (and now IC3) has a real, still-unsolved terminal lateral-drift
> failure this investigation hasn't root-caused yet — combined IC5 data across all
> DTHETA_HREF testing is 8/12 SP (67%), 0/12 ASCENDING events, but real tail risk up to
> 5.46m remains.** Next step: root-cause the lateral-drift mechanism (check against
> [[feedback_terminal_root_lateral_zeta_r]] / [[feedback_terminal_smc_actuator_wall]] for a
> possible shared cause) before considering IC5 closed. Full data:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐⭐⭐ 2026-08-28 (earlier same day, IC5-only isolated sweep) — `PLASMC_DTHETA_HREF=1`
> confirmed to eliminate the catastrophic dtheta cap-pinning mechanism at IC5 in isolation**
> (n=7 combined: 0/7 `ASCENDING` anomalies vs 2/8 without the fix, max pinned-cycle count 8
> vs 55-74). Superseded/extended by the IC1-5 gate entry above, which adds the IC1-4
> regression check and reveals IC5's remaining lateral-drift problem. Two of three sweep
> attempts that day hit contamination from a concurrent session's `kr_rp.sh` IC-sweep
> (port/gRPC launch collisions, not flight-data corruption) before a clean run succeeded —
> see [[feedback_check_concurrent_sitl_before_launch]].
>
> **⭐⭐⭐⭐ 2026-08-26 — IC5 catastrophic mechanism ROOT-CAUSED: it's the ALREADY-DIAGNOSED
> `dtheta` az-visibility-filter defect (project_20260824_dtheta_ic5_flyaway_rootcause), NOT
> primarily the angle-clustering/Hough bug below (that's a downstream SYMPTOM). Both
> catastrophic reps' `Ground_Truth.npy` reads `descent_anomaly_cause='ASCENDING'`,
> `target_lost=False` — a genuine closed-loop instability, not the
> `CBF_CORNERS_STALE_ABORT`/open-loop fallback originally assumed.
> `Control_Data.npy`'s `dtheta_correction(t)` shows BOTH bad reps pin at the 2.0 m/s^2 cap
> for 55-74 cycles in just the first 2s of flight (vs 0-16 in clean reps), incl. a 26-cycle
> continuous pinned burst at t=0.75-1.0s — a sustained near-continuous maximum-extra-lift
> injection right at launch, driven by IC5's steep initial viewing angle sitting the CBF's
> visibility cone near its ~0.766 rad/44deg ceiling from frame 1. **Verified the causality
> direction, not just correlation**: at post-fix rep5's worst pinned burst, cross-marker
> Detection Status was 100% `'ok'` — the `lt2_angle_clusters` domination only appears ~4s
> later, after the dtheta burst has already destabilized the trajectory. So: dtheta burst
> destabilizes FIRST, perception degrades SECOND as a consequence, not a cause. **This
> reframes why `a87ac00` only partially helps** (below): it hardens the downstream
> perception symptom, not the upstream control-law defect. The real fix
> (`PLASMC_DTHETA_HREF=1`, breaks the `th_curr` self-defeating feedback loop that lets the
> correction sustain instead of settle) already exists in `controller.py` but defaults OFF
> pending its own n>=5 validation — every sweep in this whole investigation ran with the
> actual root-cause fix disabled. **Next real step: validate `PLASMC_DTHETA_HREF=1` at IC5,
> n>=5, isolated** — not further perception hardening. Full trace:
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section).
>
> **⭐⭐⭐ 2026-08-26 — IC5 angle-clustering finding (the downstream-symptom half of the
> above): real bug, `a87ac00` HELPS but does NOT fix it (n=5 confirm sweep completed; an
> earlier n=3 "already resolved" reading, and a same-day "falsified" verdict before that,
> were BOTH too optimistic small-sample reads — caught by the user twice, corrected same
> session).** Chain: (1) a first clean re-run (n=3, current HEAD) landed 3/3 tight, wrongly
> read as "no bug" — confounded, since `cross_marker_detector.py` changed (`a87ac00`,
> corner-join Hough filter) the morning after the original failure, so "clean" meant
> different code too, not isolated contamination. (2) Swapped in the EXACT pre-fix detector
> code, re-ran isolated n=3 — reproduced the original signature exactly (93%/20%/57%
> detect-ok, `lt2_angle_clusters` dominant in the bad rep 370/621=60% of misses, one 2.76m
> hard-impact failure) — confirmed the fragility is real, not contamination. (3) Current
> HEAD (`a87ac00` applied) then showed 0/3 `lt2_angle_clusters` failures — read as "already
> fixed", but n=3 was too small. (4) **A proper n=5 confirm sweep on current HEAD
> reproduced the SAME catastrophic failure**: rep5 hit 30% detect-ok, `lt2_angle_clusters`
> 41% of misses (239/580), PX4 hard-impact landing at 4.78m — nearly identical
> severity/signature to the pre-fix failure. **Combined post-fix (n=8 across 2 sweeps): SP
> rate 33%→62.5% (real improvement), but `lt2_angle_clusters` recurred in 2/8 reps incl. one
> catastrophic hard impact.** `a87ac00` roughly halves the failure rate, does not eliminate
> the mechanism — **do not describe IC5 as fixed.** Full trace + all raw data (3 sweeps):
> [[project_20260824_ic5_angle_clustering_and_hang_investigation]] (top section is the final
> corrected finding; the earlier "resolved"/"falsified" sections are both superseded but
> kept for the process lesson — small-n clean sweeps at a probabilistic/threshold failure
> mode are systematically prone to false-negative reads, twice in this same investigation).
>
> **⭐ 2026-08-25 — original (now-falsified) IC1-5 GT-feedback sweep at TRUE ground contact:
> 4/5 SOFT+PRECISE (IC1-4), IC5 (2,2,3 ENU — low altitude + large lateral offset, steep
> viewing angle) failed intermittently via TARGET_LOST/hard impact.** Provisionally
> root-caused to `cross_marker_detector.py`'s angle-CLUSTERING step (not the same failure
> class as the near-touchdown shadow/Hough-line-COUNT issue fixed the same session) —
> `lt2_angle_clusters` dominated fail reasons (up to 68%), hover-settle alpha std ranged
> 5.4-133° across identical-IC reps (vs <5° for other ICs). `CBF_CORNERS_STALE`
> (kappa-freeze) ruled out with certainty — hard-bypassed under GT-feedback since
> 2026-08-19 (this ruling-out is UNAFFECTED by the falsification above). A separate one-off
> "hung past 180s" rep could not be reproduced after cleaning ~489 leaked `fastrtps_*`
> `/dev/shm` files (from repeated `kill -9` SITL teardowns) — most likely an infra artifact,
> not a control-code bug, though not proven.

> **⭐⭐ 2026-08-25 (LATEST, supersedes the 08-24 entry below) — model.sdf is DELIBERATELY
> at TRUE ground contact (`-0.2195`) as of 2026-08-25, user directive, not a leftover test
> state to "restore."** Purpose: understand + improve the landing height itself (the
> 0.487m-raised collision floor from 2026-08-09 was a workaround for the texture-blur
> crossover, not a fix — this phase is about closing that gap for real, not re-hiding it).
> **Do NOT revert to the raised version without an explicit new user directive** — earlier
> guidance in this file ("check live state, don't assume") undersold this: it's not just
> unknown/flip-floppy, it's the current INTENTIONAL target config. Perception-mode
> (non-GT-feedback) cross-marker work under this config will hit the texture-blur ceiling
> (~0.79m crossover vs ~0.34m camera-to-marker at true ground) — that's the open problem to
> solve, not a bug to work around by re-raising the floor.
>
> **⭐⭐ 2026-08-24 — the drone's ground-collision geometry
> (`~/PX4-Autopilot/.../x500_base/model.sdf`, OUTSIDE this repo) was deliberately raised
> +0.5m on 2026-08-09 for camera standoff; this raised floor, not a control-law bug, was
> why GT-feedback cross-marker landings appeared to "stall"/"not touch down" near 0.5m
> alt — reverting to true ground (`model.sdf.bak_before_legext_20260809`) gave a clean
> first-try SOFT+PRECISE GT-FB landing, min_alt=-0.01m. Same session:
> 3 cross-marker touchdown/perception hardening fixes, all uncommitted — Hough
> corner-join shadow filter (`cross_marker_detector.py`), hw coast+freeze KF ported
> from ArUco's already-validated fix (`cross_marker_perception.py`, replaces a
> `zeros(6)`-on-miss that reintroduced [[feedback_kf_frozen_during_marker_loss]]), and
> a rolling-window touchdown streak (`controller.py::_touchdownDetect`, no-op by
> default). Full trace + IC1-5 GT-FB-at-true-ground sweep status:
> [[project_20260824_touchdown_groundcontact_and_perception_hardening]].**

> **⭐ 2026-08-24 — cross-marker montage videos rebuilt with a new `rover_cross`
> world/model (outside this repo), plus `tools/overlay_image_features.py` (s/h/alpha
> HUD overlay on recorded down-cam video) and a real bugfix in
> `cross_marker_detector.py::_robust_fit_line`: it computed Huber outlier weights
> then silently discarded them every iteration (never actually excluded
> contamination). Found this by visually diagnosing WHY the overlay's alpha arrow
> was wrong near touchdown — the drone's own cast shadow overlaps the marker's stub
> in the final ~10 frames, causing a ~183° disambiguation flip then a stale-held
> freeze while `Detection Status` falsely stayed `"ok"`. Fix now genuinely prunes
> outlier points (arm AND stub) before they feed `s`/`alpha`; validated offline
> (same raw frames) and via a fresh live SITL landing (no flip, correctly reports
> `"miss"` instead of a silently-biased center on the worst frame). Full trace:
> [[project_20260824_cross_marker_montage_overlay_robustness]];
> alpha-is-not-pixel-frame gotcha: [[feedback_cross_marker_alpha_not_pixel_frame]]; texture
> history tangent (marker's own texture never removed, just kept low-contrast + flow-sampler
> access narrowed): [[feedback_cross_marker_texture_history]].**

> **⭐ 2026-08-24 — CBF architecture reference written after THREE mistaken framings
> this session: (1) conflating `rho_fov`/`d_min_fov` with the real barrier, (2)
> proposing to tune deprecated `rho_fov_0`/`rho_fov_inf`, (3) building a Phase-2
> ground-truth check that passed 13/13 while a negative control proved it had ZERO
> discriminating power (zeroing out the `radius` input still passed clean) --
> reverted rather than ship a false-confidence check. `tools/validate_cbf.py`
> CONFIRMED 12/12 passing as-is (real barrier math, `CBF_LW_ROT` 90°-fix, Phase-1
> byte-identical ArUco/cross-marker; cross-marker's `radius`-derived `delta2`
> doesn't even affect the Phase-1 bound, so cross-marker's live barrier is exactly
> as verified as ArUco's). Three real, STILL-OPEN gaps: (1) manuscript's
> `δ_m=15px/f` margin inset is NOT implemented in code; (2) ArUco's own Phase-2
> rewrite has zero automated test coverage; (3) cross-marker's Phase-2
> `radius`-based margin has NEVER been validated against ground truth, and doing
> so properly needs a real position/velocity rollout under bounded acceleration
> over the `tau` horizon, not a one-shot attitude-substitution trick (that's what
> failed the negative control). **Read
> [[reference_cbf_visibility_architecture]] BEFORE reasoning about CBF correctness,
> proposing any visibility-related fix, OR trusting a new correctness check without
> running a negative control on it first.**

> **⭐ 2026-08-23 — GT-feedback SP regression ROOT-CAUSED AND FIXED (ArUco AND
> cross-marker, same bug): `PLASMC_TD_SPIKE` default 0.5 was ~16x larger than the
> real touchdown `h_z` signal (peaks ~0.031) and could never latch, so the descent
> never disarmed and ran into a Gazebo ODE physics-engine auto-sleep + 25s watchdog
> abort at ~0.487m. Reverted to 0.0 (486f713's actual historical value). Confirmed
> via n=5 TD_SPIKE sweep (hard cliff: 0/15 SP at 0.1/0.3/0.5, 5/5 at 0.0) AND a
> wholesale `controller.py`+`cbf_visibility.py`+`img_data.py`+`gt_feedback.py`
> revert-and-rerun to the 486f713 SP-era snapshot (5/5, identical altitude to
> sub-mm). Post-fix: ArUco GT-FB IC1-5 = 24/25 SP, cross-marker GT-FB IC2 = 5/5 SP.
> Fixed in `b2a0194`/`87b8896`. SUPERSEDES [[project_20260817_crossmarker_descent_stall_investigation]]'s
> "kappa-leakage fixed-point" framing as the CAUSE (the fixed point is real, just not
> why landings failed). Full trace + 4 other falsified hypotheses (Z_REG double-count,
> VDS_KF_Q, TERMINAL_COMMIT, FEATURE_PTS_FRESH integrator gate):
> [[project_20260823_td_spike_regression]].

> **⭐ Standing reference: `PX4_Gazebo/px4_autopilot_overrides/` (git-tracked) is a
> snapshot+restore point for every `~/PX4-Autopilot` file this project has modified
> (camera/marker/world SDFs, airframe configs) — that repo is external/untracked by
> this project's git, so a reset/reinstall would silently lose those edits otherwise.
> Update it whenever a PX4-Autopilot asset changes. See
> [[reference_px4_autopilot_overrides_backup]].** ⚠ STALE as of 2026-08-09 — does
> not yet reflect the hi-res `cross_marker.png` texture swap; re-snapshot before
> relying on it for a PX4-Autopilot restore.

> **🔧 2026-08-12 — cross-marker flow-computation ARCHITECTURE investigation,
> CLOSED with an honest negative result: neither fix resolves the hard
> landing.** Fixed a real `getFPS()` init/crash bug (`gz_subscriber.py`,
> IMPLEMENTED); IMPLEMENTED the dt/frame-pairing fix (removed
> `self._prev_gray`/`_prev_frame_t`/`_prev_quat`/`_prev_angvel` entirely,
> `dt` now pinned to native camera rate ~0.016s, was 4-8x inflated during
> terminal-descent dropouts); IMPLEMENTED moment-loom+MAD-outlier-rejection
> for `Tz` (overrides `sol[2]`, joint solve unchanged per user directive).
> **Both validated as REAL, individually-correct fixes (no crashes, dt
> confirmed clean, moment-loom matches hand-prototyped numbers exactly) —
> and BOTH separately re-tested live and CONFIRMED to NOT fix the hard
> landing.** Sign-flips still occur post-dt-fix (one instance actually
> worse than pre-fix); moment-loom recovers isolated-outlier cases but
> fails when the point cloud is majority-corrupted (which the touchdown
> window's point scarcity makes common) or when point count drops below
> its own minimum threshold. **Root cause is structural: too few reliable
> tracked points at extreme close range — neither fix adds new
> information, both only better-use whatever points already exist.** Real
> next steps: more/better close-range points (structural marker-geometry
> problem) or a different close-range signal source entirely (depth-aware
> fallback, untried) — not further refinement of point-set processing.
> RE-CONFIRMED gyro-derotation is structurally necessary (full 6-col cond
> 9-103 vs derotated 4-col's 1.7-4.6); **RETRACTED a wrong claim** — the
> intended comparison target is the **nested TEXTURED ArUco marker** (the
> live `aruco.sdf` world's single `arucotag` model: small tag nested in a
> big tag, textured background, one physical plate) — NOT a spread
> multi-marker board, and it does NOT avoid the Wx/Wy aliasing via spatial
> spread (disproven directly: its own achieved corner spread and condition
> number are comparable-to-worse than the cross-marker's) — its real
> advantage is ~2.5x lower per-point correspondence noise (exact ID-decode
> vs LK-tracked), confirmed across full datasets. Don't re-cite "ArUco
> board spread" framing for this comparison. Full trace:
> [[project_20260812_cross_marker_flow_architecture_investigation]].
>
> **✅ 2026-08-10 RESOLVED (was the 08-09 "sign flip" theory below — that
> theory is FALSE, disproven, not just superseded):** the claimed "Tz
> sign-flips around 2m altitude" mechanism never existed. Pooling all 12
> available cross-marker landing flights (per-flight demeaned) gives the
> TRUE 1-2m-band correlation as **+0.23 — positive, same sign as every
> other band.** The original 5/5-negative finding was a ~3%-chance unlucky
> cluster from one back-to-back-launched batch; a follow-up 3-flight batch
> showed the opposite sign. Mechanism: the "landing" validation maneuver is
> a near-constant-vz linear descent, so a narrow-altitude-band correlation
> there is noise-dominated and its sign is close to a coin flip per flight
> — see [[feedback_correlation_needs_pooling]]. **Don't re-derive a
> sign-flip fix from this.** The catastrophic negative Hz R² near touchdown
> is still real but is better explained as a near-zero-true-variance R²
> artifact, not a sign problem — check bias/noise floor, not sign, next.
> Full trace (read bottom-up, top sections describe the disproven theory):
> `PX4_Gazebo/docs/HANDOVER_cross_marker_hz_signflip_20260809.md`.
>
> **2026-08-10: `CROSS_FLOW_CENTER_EXCLUDE_FRAC` 0.35→0.55 tried (radial-
> leverage idea, since Hz's per-point signal is linear in radial distance)
> and REVERTED** — a narrow z-phase-only check looked promising but the
> whole-flight fit showed a net regression across Hx/Hy/Hz alike (point
> starvation from the more aggressive exclusion). See
> [[feedback_whole_flight_fit_before_adopting]] for the methodology lesson.
>
> **⚠ CLOSED 2026-08-10: user explicitly accepted the 0.9→0.8 z-phase
> correlation gap as tolerable — do not reopen or re-propose bisection work
> on this unless the user raises it again.** History kept below for
> reference.
>
> The cross-marker's raw-signal regression (was OPEN, now closed)
> (z-excitation-phase Tz-vs-GT correlation 0.90-0.96 pre-08-05 archived data
> → 0.78-0.82 current, confirmed real via apples-to-apples n=5-per-side).
> 4 of 5 candidates from the 08-04/05 change cluster TESTED 2026-08-10 via
> proper n=3 A/Bs (each temporarily reverted then restored), none give a
> clean fix: color-gate threshold RULED OUT (breaks detection under current
> line-width), camera Z offset RULED OUT (.15→.20 revert: no change),
> tracking-based ROI addition RULED OUT (disabled: no change), line-width
> halving INCONCLUSIVE (full-width hi-res mask gave -0.03/+0.60/+0.94 —
> much noisier, not adopted). Mount-yaw+90° TESTED 2026-08-10 (matched
> SDF+code revert, physically self-consistent, 100% detection ok-rate
> confirmed the match was valid): +0.775/+0.803/-0.193 — inconclusive, one
> sign flip, not pursued further. All 5 candidates from the cluster now
> tried; none give a clean fix. Recommendation: stop bisecting without a
> new cheap idea — may be a combined/nonlinear effect, not one cause. Also
> verified 2026-08-10: perception/GT timestamp sync is sound (in-range gap
> median 0ms, 95th pct 4ms) — the low-R² findings above are not a sync
> artifact. See
> [[project_cross_marker_hz_regression_bisection_20260810]].
>
> **Project focus as of 2026-08-10:** active work is cross-marker only;
> ArUco is comparison-baseline-only, not itself under investigation. See
> [[project_20260810_cross_marker_focus]].
>
> **2026-08-11 — ArUco IC1-5 comparison baseline established** (n=2/IC,
> `run_ic_validation.sh`): all 10/10 landed, 0 fly-aways; IC1 excellent
> (0.021m mean xy, precise); IC2-5 classic off-center pattern (mean xy
> 0.98-1.39m, no precise, mostly soft) — somewhat better than the last
> full gate on record. See
> [[project_20260811_aruco_ic1to5_comparison_baseline]].
>
> **2026-08-11 — `run_aruco_landing.sh` gained a `WORLD=` override** (was
> hardcoded to `PX4_GZ_WORLD=aruco`), enabling the FIRST-EVER real
> closed-loop landing test against the cross-marker pipeline
> (`WORLD=cross_marker MARKER_TYPE=cross`). First flight: perception clean
> (100% detection) but a HARD landing (541 m/s² impact, 4.27 m/s touchdown).
> **ROOT-CAUSED same session, CORRECTED after a methodology check** (compare
> against proper `Z_REG`-regularized, time-synced GT via
> `_compute_gt_flow_zreg`, not just the controller's own desired reference —
> `compute_gt_signals` can't see this at all, no Z_REG, hard-NaNs below 1m).
> Two problems: (1) a real GT-confirmed sign flip at ~1.3m altitude
> (perceived +0.27 vs GT -0.8), (2) the DOMINANT issue — perceived `h_z`
> stays pinned ~-0.2 through the terminal second while true GT `h_z` grows
> to -5.70 near touchdown (`1/(z+Z_REG)` amplification, same mechanism
> `Z_REG` exists to regularize for ArUco) — perceived signal has nowhere
> near the dynamic range to track it, so the controller never brakes hard.
> kappa/s_e_n/CBF stayed nominal throughout. A rate-limiter alone would NOT
> fix problem #2; real candidates are a depth-aware fallback near the
> ground or re-deriving the cal for that regime specifically. **This
> misdiagnosis (first pass compared perceived h_z only against the
> controller's own h_d_z, not GT) is now codified as a hard rule + skill:
> [[feedback_diagnose_against_gt_not_internal_reference]] /
> `PX4_Gazebo/.claude/skills/diagnose-flight-data/SKILL.md` — invoke before
> any future "the perception was wrong" conclusion from recorded data.**
> Full procedure + diagnostic trace:
> [[reference_cross_marker_headless_flight_testing]] /
> `PX4_Gazebo/docs/HANDOVER_cross_marker_headless_flight_testing_20260811.md`.

> **🟢 2026-08-07d — Hz's variance source FOUND AND FIXED, R^2
> DOUBLED (0.22->0.48), DEPLOYED.** Pure analysis of already-collected `Flow
> Diag Log`s (no new flights needed to find it): `_sample_flow_points` (GFT
> corner search) only fires on cold-start or when the tracked pool drops
> below `RESAMPLE_TRIGGER=10` — between those events the SAME point set
> tracks forward via LK indefinitely. 4/5 pre-fix runs showed a perfectly
> CONSTANT `n_kept` for their ENTIRE ~500-frame z-phase (zero resamples) —
> whichever corner subset got picked once near takeoff just persisted the
> whole flight. Hz/Wz's raw `_fill_A` columns are position-WEIGHTED
> (`[-x,-y]`/`[-y,x]`), so that one-time draw's exact locations
> disproportionately set the WHOLE flight's Hz/Wz quality, while Hx/Hy
> (position-INDEPENDENT columns) stayed robust — explains the large,
> previously-unexplained run-to-run variance that made the 08-07c color-gate
> and camera-Z bisections (below) inconclusive; neither of those parameters
> touches this mechanism at all. **Fix:** `RESAMPLE_PERIOD_S=1.0`
> (`CROSS_RESAMPLE_PERIOD_S`) forces the existing top-up-refresh logic
> periodically, independent of pool size. Validated n=5 (flake rate
> unusually high this session): R^2 Hx=0.69 Hy=0.76 **Hz=0.48** Wz=0.50 vs
> the prior deployed Hx=0.73 Hy=0.79 Hz=0.22 Wz=0.53 — Hz MORE THAN DOUBLED,
> everything else within normal noise (0.03-0.06 vs typical 0.1-0.2+
> swings), no detection-quality cost. DEPLOYED into
> `CrossMarkerPerception.__init__`, superseding the peripheral-bias-only
> cal. Closes the variance-source thread; color-gate/camera-Z stay formally
> inconclusive but are now low priority. Wx/Wy/Wz's separate structural
> collinearity ceiling is untouched — still needs a bigger marker plate.
> User decision: only pursue a full PHYSICAL drone scale-up (real coupled
> mass/inertia/rotor-arm/thrust, not a cosmetic mesh trick) if this thread
> hits a dead end specifically needing more camera height.
> [[feedback_cross_marker_radial_spread_ceiling]] [[project_cross_marker_pipeline_20260801]]

> **🔴 2026-08-07b — Hz's real cause found: a genuine RAW-SIGNAL
> REGRESSION in the 08-04/05 camera/geometry cluster, not an intrinsic ceiling
> like Wz's. Perspective-divide-noise (the last suspect from the entry below)
> RULED OUT too — `Z_V Log` shows grazing rays never happen in calibration
> flights (min z_v >=0.888 always).** Re-checked the 08-03 memory's claim
> "raw Hz-vs-GT correlation is strong, r=0.93-0.96" — now FALSE, was stale:
> current data gives z-phase raw `h2`-vs-GT r=0.07-0.67 (mean ~0.5) vs
> 0.90-0.96 on the archived pre-08-05 recordings
> (`calibration_data/output_cross_stale_pre20260805/`), direct same-maneuver
> before/after. Confirmed the intermediate pre-peripheral-bias batch was
> already degraded too (r=0.21-0.88, high variance) — rules out peripheral-
> bias itself as the cause, isolates it to the earlier camera/geometry
> cluster (mount yaw+90deg, Z offset .20->.18/.15, line width halved,
> color-gate 20->100, tracking ROI, axis-sign-flip). Leading hypothesis
> (code-derived, NOT flight-tested yet): the 08-05 color-gate loosening
> `V<20->V<100` (needed because the halved line width left too few pixels
> for line detection at the old threshold) now admits large amounts of
> POSITION-correlated anti-aliased edge noise into the GFT mask, which
> disproportionately corrupts `_fill_A`'s position-WEIGHTED Hz `[-x,-y]`/Wz
> `[-y,x]` columns while leaving Hx/Hy's position-INDEPENDENT `[1,0]`/`[0,1]`
> columns comparatively immune — also explains why peripheral-bias (which
> biases toward the noisiest, farthest-from-center pixels) never moved Hz.
> Next test (not yet run): tighten the color gate back toward V<20-60,
> relying on `_reject_blobby_components`'s shape gate alone for ghost
> defense (already confirmed sufficient per the 08-05 comment).
> [[feedback_cross_marker_radial_spread_ceiling]] [[project_cross_marker_pipeline_20260801]]

> **🔴 2026-08-07c (LATEST) — Hz bisection (color gate + camera Z-offset)
> BOTH inconclusive at n=2, swamped by ~0-0.8 run-to-run variance bigger
> than any tested parameter's likely effect; separately CORRECTED a
> mislabeling — the "ghost" from the 2026-08-02 entry is the drone's REAL
> landing legs, not a mirrored render duplicate; and confirmed no SDF-only
> lever exists to widen the leg gap.** Color gate: V<60 (2 runs) 0.005/0.449,
> V<20 (2 runs) 0.745/0.557 but with UNSTABLE detection ok-rate (90%/58%,
> below the 95% cal-quality gate) — reverted to V<100, not adopted. Camera
> Z-offset: tested Z=.20 (2 runs, isolated via the pre-existing
> `model.sdf.bak_before_campos_20260805_010737` backup) got 0.791/0.017 —
> same huge spread. Camera restored to live Z=.15 immediately. User
> clarified Z=.15 is a DELIBERATE tradeoff (keeps landing legs out of the
> downward FoV), not a revertible regression — reverting Z is not an
> adoptable fix regardless of Hz correlation unless the legs' footprint is
> separately widened. Investigating that surfaced the ghost correction:
> dumped raw on-ground disarmed frames
> (`apps/diag_raw_image_dump.py`/`calibration_data/diag_raw_ghost_check/`)
> and confirmed the top/bottom-margin artifact is two distinct real
> leg-strut+skid shapes (front/back leg pairs) matching the SDF's own leg
> COLLISION geometry, not a duplicated drone silhouette — the "mirrored
> ghost of the drone's own body" description was a misdiagnosis, likely
> conflated with the separate, actually-real marker-reflectivity bug from
> the same 08-02 session. Existing ghost-defense code in
> `cross_marker_detector.py` is unaffected (built against the real
> artifact, only the mental model was wrong). Checked whether the leg gap
> could be widened via SDF: NO — `x500_base/meshes/NXP-HGD-CF.dae` is ONE
> combined mesh for the whole airframe (no separate leg link/pose); the
> leg SDF entries with independent poses are physics-only collision boxes,
> invisible to the camera. Needs real `.dae` mesh editing, no config path —
> not attempted per user instruction. Net: Hz's real cause is STILL open;
> both suspects tested this round were inconclusive, not confirmed or
> ruled out, due to insufficient n. [[feedback_cross_marker_radial_spread_ceiling]]
> [[project_cross_marker_pipeline_20260801]]

> **🟢🟡 2026-08-07 — cross-marker peripheral-bias cal DEPLOYED (Hx/Hy win);
> Wz's collinearity mechanism confirmed directly; z-excitation-amplitude tested and
> RULED OUT as Hz's cause; Hz itself still unexplained (superseded by 08-07b above).** Continues the 2026-08-06
> entry below (kept for its contamination-disproof + hardware-comparison history).
> Deployed: re-derived the peripheral-bias cal isolated from the 08-05 camera/axis
> changes — Hx 0.55->0.73, Hy 0.63->0.79 (BEATS the pre-08-05 baseline 0.70/0.71),
> Hz flat at 0.22, Wz roughly flat 0.57->0.53 (a wash, not a regression) — pasted into
> `CrossMarkerPerception.__init__` (LIVE as of 2026-08-07). Directly confirmed Wz's
> "near-collinearity" symptom with real numbers: raw `Opt Flow Ang Vel` correlation
> matrix has two near-zero eigenvalues (0.0015, 0.0021) — raw `h1`(Ty)/`w0`(Wx)
> correlate r=0.98-1.00 in EVERY phase (not just where co-excited), because
> `_fill_A`'s Wx column `-(1+y^2)` stays ~constant at this marker's achievable `y`
> spread, numerically aliasing Ty's constant column — a structural, spread-fixable-
> only-by-a-bigger-marker degeneracy, not a fit-methodology issue. Tested the other
> leading suspect (z-phase excitation amplitude, `CALIB_AMP_Z` 1.2->2.2): real ~4x
> raw-SNR gain (confirmed via achieved-altitude-vs-commanded tracking: PX4 attenuates
> the commanded sine ~2.3-2.7x at 0.5Hz) but Hz only moved 0.22->0.25 (noise-level)
> while Hx/Hy both DROPPED (0.73->0.68, 0.79->0.75) — the same ArUco-era over-drive
> tradeoff a 21-day-old memory had flagged but never applied. NOT deployed. Net: Hz's
> real cause is still open (both spread and amplitude now ruled out) — only untried
> idea left is `_getVirtualPts` perspective-divide noise on near-grazing rays.
> [[feedback_cross_marker_radial_spread_ceiling]] [[project_cross_marker_pipeline_20260801]]

> **🟡 2026-08-06 — cross-marker Hz/Wz calibration weakness: contamination
> hypothesis DISPROVEN (bug in own diagnostic script), radial-spread fix shipped a
> software-only point-selection bias (not just marker size) that fixed Hx/Hy but left
> Hz UNCHANGED — root cause of Hz specifically still open (superseded by 08-07 above,
> kept for the contamination-disproof + hardware-comparison detail).** Chain: (1) an
> apparent z-phase GT contamination finding traced to a misaligned ad-hoc script
> (didn't apply `compute_gt_signals`'s own `valid`-mask before truncating
> `gt['Phase']`); reverted the resulting purity-gate addition to
> `tools/derive_cross_marker_cal.py` (it made Wz worse — no real contamination to
> remove). (2) `_sample_flow_points` (`cross_marker_perception.py`) edited to bias
> `goodFeaturesToTrack` toward the marker's arm/stub tips (central-disk exclusion,
> `CROSS_FLOW_CENTER_EXCLUDE_FRAC=0.35`) + a frame-boundary exclusion margin
> (`CROSS_FLOW_BOUNDARY_MARGIN_PX=20`, corners near the true edge drop out mid-track
> under lateral/descent motion, biasing the survivor pool back toward center). (3)
> Re-recorded 4 valid runs, re-derived: radial spread genuinely increased (Radial
> Diag Log mean p90 0.113->0.184), Hx/Hy improved substantially (0.55->0.73,
> 0.63->0.79), but **Hz stayed EXACTLY 0.22** and Wz got marginally worse. Also
> checked hardware (ArUco) output-cal for comparison: OPPOSITE profile (Hz=0.64
> strong, Hx/Hy=0.07-0.08 weak from real-camera noise) — only Wz is weak on both
> platforms; "Hz/Wz are universally hardest to observe" is not a cross-platform law.
> [[feedback_cross_marker_radial_spread_ceiling]] [[project_cross_marker_pipeline_20260801]]

> **⏸ 2026-08-04 (LATEST) — rover/ArUco moving-target thread PAUSED**, superseded by the
> cross-marker work below (same investigation, different chat). Marker-agnostic carryovers
> for whatever marker sticks: the kappa-ratchet/staleness-freeze principle, MARKER_LOSS_GRACE's
> 1.0s-too-long-for-a-moving-target issue (unfixed), the flow-coupled-coast KF architecture
> point (validated partial fix), the theta_cone/CBF tilt-not-arresting-growth question (found
> but not investigated: a rover rep's downward camera lost the ENTIRE ground scene, not just
> the marker, after tilt climbed steadily past 11deg — ring-flow/observer fixes would NOT have
> helped THIS failure), GT-FB-vs-real-perception independence, and the rover_trajectory.py
> Linear/Circular speed-scaling bug. ArUco-specific work (PlanarFeatureMap multi-slot/overflow,
> the 5 ported fixes' implementation details) is moot if the new marker doesn't use a similar
> homography-map architecture. [[project_20260731_moving_target_real_perception_chain]]

> **🟢 2026-08-03 — cross-marker Wx/Wy confirmed genuinely unrecoverable
> (not just untested); Hz/Wz gap vs ArUco root-caused to point radial spread.** Ported
> rollexc/pitchexc excitation phases into `record_cross_marker_calibration.py` (were
> missing — Wx/Wy had only ever seen incidental x/y-phase tilt); dedicated-excitation
> raw-vs-GT correlation came back -0.15/-0.09 (near-zero, wrong-signed). Root cause: `_fill_A`'s
> Hz/Wz columns are linear in point (x,y), Wx/Wy's are QUADRATIC — both need radial spread
> the marker doesn't achieve (measured max|x|,|y| ~35-50% of frame half-extent, every
> phase). Tried both fixes: loosening the `roi_frac_y=0.65` ghost-defense crop (now
> env-overridable, `CROSS_ROI_FRAC_Y`) to 0.90/1.00 — ghost defense held (99.7-99.9%
> ok-rate, `_reject_blobby_components` is the real defense now) but Wx/Wy only reached a
> noisy +0.1..+0.26, x-spread barely moved (footprint-limited, not ROI-limited). Lowering
> `CALIB_TAKEOFF_HEIGHT` 2.7m->2.4m->2.0m — 2.4m no improvement, 2.0m broke detection
> outright (ok-rate 25%, marker overflows frame). **Ceiling is the marker's physical
> footprint (3.0m plate), not a recorder/detector parameter** — untried lever is a bigger
> plate. Wx/Wy=0 confirmed correct, not a gap. [[feedback_cross_marker_radial_spread_ceiling]]
> [[project_cross_marker_pipeline_20260801]]

> **🟢 2026-08-03 — cross-marker Hx/Hy weakness RESOLVED: two real bugs in the
> raw h,w/s computation, not the calibration methodology.** (1) Missing V-frame
> gravity-leveling — `cross_marker_perception.py` computed flow from raw un-leveled camera
> pixels with zero attitude compensation, while GT is in the tilt-compensated V-frame
> `img_data.py` always projects through (`_getVirtualPts`); ported it in, sourced the quat
> from `Image_Node.getQuaternions()` (synced to frame capture, not a separately-polled
> `FC.getQuat()`). (2) dt/staleness mismatch: dt came from the outer polling clock
> (advances every call) while the LK "previous frame" state only advances on successful
> detections — after any dropout (every run had some), the next good frame divided a
> multi-frame displacement by a one-frame dt, spiking all six solved params. Fixing (1)
> alone showed NO improvement (Hz even regressed) — (2) was masking (1)'s real benefit.
> Together: raw Hx/Hy-vs-GT correlation went from ~0.01-0.10 (every test before) to a
> consistent 0.74-0.87. Re-derived + pasted cal (5 clean runs): R^2 Hx=0.70 Hy=0.71
> Hz=0.42 Wz=0.52 (vs ArUco's 0.75/0.75/0.79/0.71 — Hx/Hy now within 0.05, Hz/Wz remain
> the gap); centroid sx/sy inter-run spread ~7-10% (TIGHTER than ArUco's own live cal's
> ~19-24%). Wx/Wy still untested (no rollexc/pitchexc phases ported to the cross-marker
> recorder yet). Cal NOT yet re-validated against independent multisine data.
> Process lessons: [[feedback_missing_vframe_leveling_port]],
> [[feedback_dt_staleness_after_detection_dropout]]. [[project_cross_marker_pipeline_20260801]]

> **🔴 2026-08-02 — cross-marker calibration found NOT to generalize on
> independent multisine validation (near-zero/negative R^2) → root-caused to a
> `resolution[::-1]` transpose bug in `CrossMarkerPerception.__init__`'s `self.center`**
> (swapped cx/cy, corrupting the h,w Jacobian solve + centroid `s` geometry). This is the
> SAME mistake `img_data.py` made and reverted in June 2026 (see that file's own comment)
> — repeated fresh because the fix only ever lived as a comment there, not shared/tested
> code, and `cross_marker_perception.py` deliberately duplicates that math standalone.
> Point starvation, ill-conditioning, and temporal misalignment were all ruled out first
> (new `_flow_diag_log` diagnostics: n_kept, cond(A), in-sample LSTSQ residual) before the
> geometry bug was found. Fixed: raw Hz-vs-GT correlation 0.014→0.652, Wz stayed strong at
> 0.75 on independent data; Hx/Hy improved only slightly (0.04/0.05→0.06/0.10) — STILL
> WEAK, unexplained, next thing to chase before the cal can be trusted. The DERIVED cal
> pasted 2026-08-02 (below) predates this fix and should be treated as unreliable —
> re-derive after the Hx/Hy weakness is resolved. New tools:
> `apps/record_cross_marker_validation.py` + `tools/validate_cross_marker_flow.py` (GT-direct
> R^2 check against the live cal, closes the io-calibration skill's train/validate loop).
> Process lesson: [[feedback_duplicated_math_diff_check]].
> [[project_cross_marker_pipeline_20260801]]

> **🟡 2026-08-02 — cross+stub marker output calibration DERIVED and PASTED**
> into `CrossMarkerPerception.__init__` (`_sensor_cal_hw`/`_sensor_cal_s`, replacing the
> identity placeholder). Root-caused two Gazebo rendering bugs along the way: the marker
> plate's PBR material was defaulting to reflective (mirroring the drone onto itself —
> fixed via explicit metalness/roughness), and a pre-existing camera-render ghost (mirrored
> drone duplicate in the outer frame margin, present in BOTH aruco and cross_marker worlds,
> previously unnoticed since ArUco's small centered tag never reached it) was corrupting
> detection whenever the 3m plate's arms reached the margin. Layered fix in
> `cross_marker_detector.py` (position-based anisotropic ROI + a new shape/extent-based
> `_reject_blobby_components`) cut the worst-case run from 22.9% to 69.5% detection ok-rate
> and made the typical run ~92%+, though full run-to-run consistency (99%+) wasn't fully
> recovered — some residual SITL/render stochasticity looks irreducible without deeper
> engine investigation. Derived cal: R^2 Hx=0.40 Hy=0.49 Hz=0.34 Wz=0.36, centroid scale
> still ~2.4x inter-run spread — usable starting point, NOT yet validated against
> independent multisine/landing data (io-calibration skill's train/validate discipline).
> [[project_cross_marker_pipeline_20260801]]

> **🟢 2026-08-01 — decode-free cross+stub marker pipeline built, committed
> (916aa53, pushed), as an ArUco alternative.** Standalone (`cross_marker_detector.py` +
> `cross_marker_perception.py`), NOT routed through `img_data.py`/`IMG_PROCESSOR` — no
> PlanarFeatureMap, no marker handover, no board layout (all ArUco-specific patches for
> problems this marker's design doesn't have: decode-or-nothing dropout, multi-marker
> range extension). Computes the manuscript's actual `s`/`alpha`/`h,w` (not the project's
> ArUco-shaped conventions — `s`=homogeneous centroid, `alpha`=unweighted 2nd-moment over
> real pixels, `h,w`=image-Jacobian solve over plate-wide Shi-Tomasi points). Wired into
> `controller.py` via `MARKER_TYPE=cross`; single-point visibility CBF reuses
> `cbf_visibility.py` unmodified. Validated offline + live (ground/5m/7m hover, partial
> tilt). **NOT ready for closed-loop descent**: `CrossMarkerNode` only implements the
> interface subset the hover path touches (no terminal-kick/ring-loom-fusion — will
> AttributeError if exercised). [[project_cross_marker_pipeline_20260801]]

> **🟡 2026-07-31 — real-perception (non-GT-FB) moving-target still fails; root-caused
> to a coast/flow disconnect + implemented a fix (UNVALIDATED).** After the five 2026-07-30
> fixes fully solved the rover case under GT-feedback (n=3 Linear 0.034-0.167m, matching
> historical), the SAME test with GT-feedback OFF (real image+IMU driving the primary SMC
> signal) failed badly (n=3, all TARGET_LOST, impacts 57-311 m/s^2, xy_err ~1-1.4m) --
> confirming perception itself (not just CBF-authority-starvation) needs hardening for a
> moving target. Traced to: ~1.4-1.8s of genuine total corner loss near the deck (all 3 reps)
> during which `s`'s predict-only coast (`img_data.py:_kf_step`) extrapolates using its OWN
> weak, already-decayed position-derived rate state, fully decoupled from optical flow `h`
> (which stayed accurate the whole time via a separate KF). `MARKER_LOSS_GRACE` (1.0s) is
> working as designed but is tuned for stationary-target brief dropouts, not moving-target
> multi-frame loss. FIX implemented (`_kf_feat_update`, flow-coupled coast: inject h_x/h_y as
> the coast rate for xc/yc) -- VALIDATED PARTIAL: outcome tracks coast duration exactly (0.62s
> coast -> 0.044m, matching historical; 2.33s/6.09s coasts -> 1.12m/1.75m, still bad). Helps but
> insufficient for multi-second outages; next lever is an independent corner-free source for h
> itself (FLOW_FUSE_RING/PLASMC_CENTROID_RATE, both baked off, not re-examined for moving
> targets yet). [[project_20260731_moving_target_real_perception_chain]]

> **🟢 2026-07-31 — five 2026-07-30 perception/kappa fixes VALIDATED, no IC1-5
> regression.** Parallel Windows/hardware session ported 5 fixes (self-heal, CBF_CORNERS_STALE
> + feature_fresh AND-gate, held-out validation-reset, confidence-weighted loop-closure,
> kappa/integral freeze on staleness — the last is the direct, hardware-confirmed fix for the
> a_u-explosion/kappa-ratchet this session traced by hand on the rover). IC1 n=3 + IC2-4 n=3 +
> IC5 n=8: no crashes, no regression signature; IC5's one 19.5m outlier checked against 447
> historical reps (p95-p99, not new) — [[project_20260730_five_fixes_ic_validation]].

> **🔴 2026-07-30 — rover moving-target baseline is STALE, 0/3 re-test.** Re-ran the
> documented 2026-07-02 rover command (GT-FB, Linear, ROVER_SPEED_MULT=0.3) that was 3/3
> on-platform (rel-lat 0.044-0.28m) — n=3+1 re-test on 2026-07-29/30 is 0/4, all missing the
> platform by 0.7-4.3m (inconsistent magnitude, no target_lost/perception-loss/descent_anomaly
> flags — tracking-lag issue, not a hard failure). Multiple bake sessions since 07-02 (dense-
> recovery, FLOW_FUSE_RING=0, MARKER_KLT_RELAX_GATE=1, s-extrap fix, 07-09) shifted the default
> gains; rover parity never re-validated against them. [[project_rover_baseline_stale_2026-07-30]]

> **🔴🟢 2026-07-28 — accidental month-long PLASMC_SINGLE_MARKER=1 default found
> +reverted; exposed+fixed a dead-code shape bug; new residual gap found.** Chasing an
> elevated TARGET_LOST/UNKNOWN rate traced to `_single_marker` defaulting ON since commit
> `758dcb2a` (2026-06-24) -- meant for a different, uncommitted experiment, never reverted,
> silently active against the nested board in EVERY standard gate/A-B since (incl. all of
> 2026-07-26/27/28's go-around/DESCENT_ANOMALY/dense-recovery work). Reverting to default-off
> exposed a real, previously-dead-code bug (board-mode LK correspondence fallback assigned a
> (4,1,2)-shaped RHS into an actually-(4,2) `all_pts_1[sl]`) that crashed every rep -- both
> fixed together, commit bf64c08. Post-fix n=25 gate: zero crashes, TARGET_LOST 60%->44%,
> IC2-4 precision markedly better. **New gap found (not fixed):** `getFailureCause()`
> (DRIFT_OFF/OVERFLOW) is itself single-marker-only logic, gated on `_single_marker` -- now
> ALWAYS returns UNKNOWN in board mode (confirmed: 11/11 post-fix TARGET_LOST reps all
> UNKNOWN). Needs a board-mode-native rewrite, not just un-gating. Recent bakes (go-around
> removal, DESCENT_ANOMALY, dense-recovery) were validated under the WRONG default -- mechanisms
> likely still sound but don't cite their absolute numbers as current-baseline without a
> re-check. [[project_single_marker_default_mismatch_20260728]]

> **🟢🟢 2026-07-28 — dense-homography recovery BAKED default-on.** Isolated A/B
> (n=25 off / n=24 on, IC1-5) closes out the 2026-07-07 "mixed, don't bake" status:
> TARGET_LOST 60%->46%, mean xy 1.11->0.92m, DESCENT_ANOMALY 1->0, UNKNOWN causes 11->6.
> Improved IC1-4; IC5 alone slightly worse on TARGET_LOST rate (still improved on mean xy) --
> consistent with IC5's structural full-FoV-exit issue being outside what a partial-view
> recovery can fix. `PLASMC_DENSE_RECOVER` default "0"->"1", commit 381f669.
> [[project_dense_recover_ab_20260728]]

> **🟢🟢🟢 2026-07-27 — go-around REMOVED (reverted per user); new DESCENT_ANOMALY
> failure classification added.** TARGET_LOST goes straight to open-loop leveling again, no
> retry -- post-loss data is diagnostic-only per user directive. New: oscillating (>=4 vertical
> sign-reversals/3s) or net-ascending (>0.5m above running best altitude) during closed-loop
> approach is now its own sticky failure tag (`DESCENT_ANOMALY`), independent of touchdown
> xy/vel -- catches fly-aways the old endpoint-only classification mislabeled as plain FAIL.
> Live-validated on IC1_rep2's known marker-switch fly-away. Commit fca2c86.
> [[project_goaround_removed_descent_anomaly_20260727]]

> **🟢🟢 2026-07-27 — go-around regression found+fixed (SUPERSEDED, go-around since removed); IC5 marker-loss confirmed
> structural, not a bug.** The unconditional go-around retry (07-26, below) held the vehicle's
> drifted lateral position while climbing then retried from the SAME geometry that caused the
> loss — for IC5 this burned both attempts on an immediate repeat, stuck-near-start-altitude
> rate went 0.9%→70% (3/321 vs 7/10). Fixed: retry now gated on genuine altitude progress since
> the last loss (`LANDING_GO_AROUND_MIN_PROGRESS_M`, default 0.75m); commit 7b01231, validated
> live. IC5 still loses the marker almost immediately on every closed-loop attempt (centroid at
> the FoV edge from frame one — large offset/short runway forces the correction itself to push
> the marker out before any descent) — confirmed pre-existing/structural, not fixable by
> go-around/leveling/the progress gate. [[project_go_around_progress_gate_20260727]]

> **🟢🟢🟢 2026-07-26 — Three more fixes committed, closing out the day's perception/
> failsafe work: (1) decode-staleness confidence decay on map_confidence (5341e14, reconciled
> with Pi's independent same-day fix); (2) FLOW_LAT_REDUCED runtime attitude-rate gate
> (dcec397), closing the "needs runtime gating on real attitude rate" gap flagged since
> 2026-07-10/11 -- validated n=20 no fly-aways, IC5 A/B shows no causal link to IC5's own
> variance; (3) TARGET_LOST leveling + bounded go-around (ce513dc) -- the marker-lost fallback
> used to freeze whatever tilt the vehicle had (not level it), producing ballistic ejections
> (62m/15.7m/s, confirmed via live roll/pitch trace); now actively levels AND attempts a
> bounded climb-to-5m recovery before committing to open-loop descent, with target_lost STICKY
> from the first genuine loss regardless of recovery outcome (per explicit user correction --
> go-around is a data-collection aid, not a failure-reclassification). None of these fully
> solve IC5 (still open, pre-existing large-offset/short-runway/velocity-momentum tension) but
> each closes a distinct, previously-open mechanism. Also produced a full perception-pipeline
> migration handover doc for the Pi/hardware port (architecture, platform-agnostic vs.
> platform-specific classification, full Gazebo/Pi divergence history).
> [[project_flowlatreduced_targetlost_gate_20260726]]

> **🟢🟢🟢 2026-07-23/24 (LATEST) — THREE independent raw-decode plausibility gaps found+fixed via
> IC1/IC2 fly-away traces, same family, three different consumers: (1) centroid `ds` outlier-hold's
> `_s_prev` never reset on a marker-loss gap (only its flow sibling was) -- froze `s` post-
> reacquisition then dumped a compounded jump, detonating kappa/a_u (IC1 9.28m fly-away, commit
> 9075a97); (2) the LK-correspondence path's raw decode (`aruco_pts_1`) had ZERO plausibility
> check -- a spurious-but-cleanly-tracked wrong marker fed straight into s/kappa (IC2 41.7m fly-
> away, commit 9f0c490); (3) the centroid-rate observer's raw decode (`aruco_pts_0`, DEFAULT-ON,
> independent of LK) ALSO had zero check -- spurious decodes during a real attitude tumble fed
> theta to 3080 + alternating h_x (IC2_rep3, commit c97202d). All three fixed by reusing/extending
> `_planarMapPredictionPlausible`; all validated n=5 clean on their triggering IC + n=3 regression
> on IC1/3/4 (no fly-aways either round; IC3/IC4 each landed SOFT+PRECISE post-fix-3). The h_x/h_y
> observer formula itself was independently re-derived from first principles and confirmed CORRECT
> (matches `_fill_A`'s convention exactly, incl. its w_z sign flip) -- purely a garbage-in bug.
> IC5's short flights are its own IC geometry (short-runway canary), left untouched. IC2's
> remaining non-SP rate is TWO SEPARATE PRE-EXISTING issues (terminal close-range marker-loss
> forcing a blind touchdown; baseline lateral kappa-ratchet imprecision on non-loss attempts) --
> NOT caused by or fixed by any of the three gates above; out of scope, already catalogued.**
> [[project_ic1_ds_guard_gap_reset]] [[project_ic2_ic5_20260723_investigation]] [[project_ic2_observer_plausibility]]
>
> **🟢 2026-07-25 FOLLOW-UP — the `UNKNOWN` marker-loss cause (traced above) root-caused + fixed
> (commit 2a5c21f): PlanarFeatureMap's rescue gate was structurally blocked during a genuine total
> zero-corner dropout, NOT because the map lost track (`map_confidence` stayed healthy 0.54-0.58
> the whole outage) but because `RESCUE_GATE_MARKER_AWARE=1`'s `self.confidence` collapses to a
> hard, PERMANENT 0.0 after just 3 zero-corner frames via `_rigid_fail_streak` decay, and can't
> recover until corners return (by which point `MARKER_LOSS_GRACE` has already expired). Added
> `PlanarFeatureMap.primary_zero_corners` so the rescue gate falls back to `map_confidence`
> specifically when there's genuinely no shape data to distrust, while a deforming-but-present
> marker (the 2026-07-19 fix's actual target) still gates on the marker-aware confidence.
> Validated n=5 IC1/2/3/4/5, no new failures attributable; IC2 in particular now zero fly-aways +
> 1 SP (mean xy 0.29m). IC5 landed with zero fly-aways too but only SOFT-only (xy_err=2.80m,
> precise=False) -- IC5 has NOT achieved SP this session or, apparently, ever; its 2.83m offset at
> only 3m altitude (half the runway of every other IC) is a structural geometry/bandwidth
> constraint, not a bug -- see [[feedback_rescue_gate_zero_corner]]'s correction note.**
>
> **🟢🟢🟢 2026-07-25 (LATEST) — SAME-DAY, INDEPENDENT confirmation on Pi hardware of a related but
> DISTINCT PlanarFeatureMap bug: confidence-lockup (commit b420d3a, ported from Pi, Pi copy itself
> left untouched). Once a homography fit fails (resid_px=inf), frame_to_map freezes and refill
> poisons new points through it forever -- confidence/map_confidence can pin at exactly 0.0
> PERMANENTLY, even long after the marker is cleanly re-tracked (no existing recovery path).
> Found live on Pi (confidence pinned 14+s while roi_hits=30/30) AND independently in a Gazebo
> historical scan (16/1788 past reps, 8 coinciding with TARGET_LOST incl. 2 catastrophic
> fly-aways -- one of which is the previously-UNRESOLVED 55.78m IC1_rep5 from
> [[project_ic1_kappa_leakage_drift_20260721]]). Self-heals via a TIME-based (not frame-count --
> cross-platform rate mismatch) degenerate-streak watchdog + full reset/re-bootstrap. Also ported
> a Pi RANSAC-bounds tuning fix (ransac_max_iters=200/confidence=0.98, measured better AND faster).
> Reconciled with this session's earlier primary_zero_corners fix. **CORRECTED same session (user
> caught it): validated n=18 across all 5 ICs -- 9/18 clean, 2 real fly-aways + 1 SITL flake (NOT
> "zero fly-aways" as first claimed here). Both fly-aways traced CONTROL-led/TARGET_LOST (not
> perception-spurious-decode) -- normal SITL variance surfacing the already-catalogued lateral-
> control family, not a regression from this merge. Fix correctness itself unaffected -- see the
> full correction in [[feedback_planar_map_confidence_lockup]]. Deep traces of both fly-aways
> (IC1_rep2 = same marker-switch-triggers-real-tumble mechanism as IC2_rep3, via the still-open
> FLOW_LAT_REDUCED gap; IC2_rep5 = terminal marker-loss under real residual velocity from IC2's
> fast offset-closing) in [[project_ic1rep2_ic2rep5_flyaway_traces_20260725]].**

> **🟢🟡 2026-07-21/22 — Long perception-pipeline session (img_data.py): baked 8 fixes to the
> decode<->map override/KF chain (fda359f..ce881f4), n>=5-validated clean on the ICs that
> previously flew away; traced (NOT fixed) a residual 55.78m IC1 fly-away to kappa LEAKAGE
> draining the adaptive gain under a real, growing error — see
> [[project_ic1_kappa_leakage_drift_20260721]]** (full mechanism, same family as
> [[feedback_dont_conclude_lag_floor]]/[[feedback_kappa_clamp_bandaid]]). Perception-side fix
> chain (all committed, see commit messages for detail, not restated here): alpha KF sin/cos-pair
> sub-filter (wrap bug), CENTROID_FROM_MAP/ALPHA_FROM_MAP override-vs-KF ordering bug, source-aware
> `cal_s`, map-authoritative-once-accepted (decode is cross-check only), confidence-scaled KF `r`
> for map-sourced AND decode-KLT-fallback samples, ds-outlier-hold/KF ordering bug, windowed-rate
> plausibility gate. Next session: check `P` (leakage)/funnel decay-rate (`_gamma`) tuning against
> `IC1_rep5` (test_data/ICValidation/20260721-141516, t=50-54s).

> **🔴🟢 2026-07-17 (LATEST) — CBF small-marker preference/overflow-driftoff/handover wiring session: found+fixed a live s[2]-homogeneous-decay bug (610,997 a_u spike) THEN found+fixed its deeper root cause (decay-to-zero position desyncing from kappa, replaced with kf_predict), a map_confidence RANSAC-outlier blind spot, a CBF corner-source flicker regression, plus gyro-seeded KLT (unvalidated). Skill `/planar-map-perception` rewritten to cover all of this; dead `_img_feature_param_real` buffer + several stale comments cleaned up.**
> Full arc: (1) CBF cone-clamp had no staleness gate ([[feedback_cbf_staleness_and_rigidity_confidence]], `FEATURE_IS_STALE`→corrected to `FEATURE_PTS_FRESH` after user pushback — that flag has zero rescue-awareness). (2) CBF now prefers the SMALL marker slot (more tilt headroom) independent of the big-priority flow pipeline, classifies OVERFLOW(span)/DRIFT-OFF(one-sided) off its own FoV margin, pulls back via tightening cbf2_filter's OWN p_10 (not a new-signed force), and signals (not forces) a HANDOVER_LATCHED path via `apps/landing_test.py` bridging `CBF_OVERFLOW`+`SMALL_SLOT_CONFIDENT`. (3) **REGRESSION FOUND+FIXED**: the small-slot preference had zero hysteresis → IC4 target_lost=DRIFT_OFF (theta_cone<0.05 for 45% of frames) that wasn't there before; fixed with 5-frame-persistence-on hysteresis (`CBF_SMALL_SLOT_ON_FRAMES`), confirmed IC4 recovered. (4) **`map_confidence` RANSAC-outlier blind spot found+fixed**: IC5's marker had held-out reprojection error swinging 0.3-199px (chaotic — a KLT drift/decode-triggered-correction cycle, not smooth) while `map_confidence` stayed 0.72-0.99, because `resid_px`/`track_conf` are computed ONLY over RANSAC inliers. Fixed with a per-slot `inlier_fail_streak` (tracked-but-rejected persistence, occlusion-safe) folded into both `map_confidence` and `get_slot_confidence`. (5) **CRITICAL: `s[2]`-homogeneous-decay bug found+fixed, THEN SUPERSEDED by its root cause** — [[feedback_s2_homogeneous_decay_bug]] — a live regression check surfaced `a_u=610,997` (single-frame spike); part 1 forced `s[2]=1.0` (fixed the symptom). A SECOND live regression check then found the deeper problem: decaying position to exactly `[0,0]` during a coast (the design `s[2]` was patched on top of) fabricates a "centered, zero error" reading that can desync from `kappa` (ratcheted then froze reading "converged", producing a 2,976 `a_u` spike on the real-error snap-back). User: *"the decay-to-zero behavior... I never approved it. We have kf_predict."* Fixed: the ENTIRE polyfit+decay extrapolation for `s` was REMOVED, replaced with the feature KF's own predict-only step (`self._kf_feat_x[:,0]`) — validated, `a_u` 2,976→12.3. (6) **Gyro-seeded KLT added** ([[feedback_gyro_seeded_klt]]) — `PlanarFeatureMap.update(gray, quat_R=...)` now seeds KLT's search with a rotation-compensated prediction instead of the zero-motion prior, running every frame (even through marker-loss, since the FC quat doesn't need a decode) — targets the IC5 drift mechanism directly. **NOT YET SITL-validated.** (7) Post-session consistency pass: deleted the now-dead `_img_feature_param_real`/`_real_t` buffer (only consumer was the removed polyfit) and fixed several comments left stale by the above (wrong call-site direction, references to removed code) — see `/planar-map-perception` skill, fully rewritten same day.
>
> **🟢 2026-07-16 — PlanarFeatureMap override path had zero plausibility check → IC1 142m fly-away; fixed with shared position+size rejection gate on BOTH override and rescue.**
> [[feedback_planar_map_plausibility_gate]] — split rescue gate (map_confidence) from override gate (confidence, stricter); added `_planarMapPredictionPlausible()` shared helper (FoV-position + size-ratio REJECT, not clip — a `±5.0` clip was numerically wrong for this camera's real ~±1.2 FoV bound). Also clarifies `V_aruco_norm[0]`=genuine ArUco decode vs `V_aruco_norm[1]`=KLT-tracked (not a re-decode). IC1-5 n=1 validation done: no fly-away, but IC2/IC5 hit `DRIFT_OFF`.

> **🟢🟢 2026-07-10/11 (LATEST) — TOUCHDOWN_LOOM UNBLOCKED (4 fixes baked) + KF-refit cal merged; n=5 IC1 4/5 precise but 0/5 soft (new gap found).**
> Traced why `TOUCHDOWN_LOOM` (loom-inversion touchdown detector, baked 2026-06-29) never fired on IC1: (1)
> `FLOW_LOOM_SIGN_GUARD` (default-on since 2026-06-22, clamps consumed loom `h_z<=0`) structurally prevented the
> detector's `h_z>0` condition — [[feedback_loom_sign_guard_blocks_touchdown_detect]]. (2) Even after removing the
> guard, the corner-flow KF was NEVER STEPPED during a marker-loss gap (`_kf_update()` only ran on real-data
> frames) — `h_z` froze at its exact last value indefinitely, structurally unable to invert no matter how long the
> descent continued. Fixed: KF `dt` decoupled (state-prop stays capped 0.1s, uncertainty-growth now uses the TRUE
> elapsed gap, `KF_DT_UNC_MAX=2.0`) + a predict-only coast mode stepped every miss-frame —
> [[feedback_kf_frozen_during_marker_loss]]. (3) Flow-KF now also resets on a primary-marker-ID switch (mirrors
> the existing 2026-07-04 loom/centroid reset, previously missing for this KF). Root-caused the FULL IC1 terminal
> a_u-explosion chain end-to-end: KLT blackout (real gyro-confirmed 2.8-7.5 rad/s attitude rate, invisible to the
> frozen h_z) → relock switches small→big nested marker → `MARKER_EXTENT_PX` 51.7→334.1px in one frame →
> `FLOW_LAT_REDUCED=1` drops `w_xy` columns (assumes level target) → real rotation misattributed into `h_xy`,
> amplified by `r²` via the bigger marker's far corners → `c`-term → θ (1.76→912) → κ ratchet (1.73→3.69) → a_u
> 1400+ — [[project_ic1_terminal_kick_root_cause_chain]]. `FLOW_LAT_REDUCED`'s model-misspecification itself is
> NOT fixed (needs runtime gating on real attitude rate, flagged not implemented). A TEXTURED-MARKER experiment
> (ring-flow LK survival ~2x) was tried + FALSIFIED (net landing regression, ArUco primary-decode interference) +
> reverted — [[feedback_textured_marker_falsified]]; a purpose-built coarse-square alternative exists,
> untested. A parallel session found+fixed a Savgol-vs-KF calibration/filter mismatch (cal fit on raw signal,
> applied at runtime to KF-filtered state) and this session merged the resulting KF-refit `_sensor_cal_hw/_s`
> values in. **n=5 IC1 (all fixes + refit cal): 4/5 precise (0.002-0.038m — dead-center), 0/5 soft** — velocity at
> touchdown clustered right at/above the 0.2 m/s threshold (systematic, not noise); rep5's 0.723m/s outlier
> checked and confirmed the SAME mechanism (heavier tail), not distinct.
> [[project_touchdown_detect_velocity_gate_gap]] — detector has no velocity/magnitude check, sign+persistence
> only; NEXT fix target.
>
> **⛔ CORRECTION (2026-07-11, separate session):** the "n=5 IC1 (all fixes + refit cal): 4/5
> precise, 0/5 soft" result above is very likely PHANTOM — the predict-only KF coast, decoupled-dt
> uncertainty growth, and marker-switch `_kf_x`/`_kf_feat_x` reset it describes did NOT exist
> anywhere in `src/img_data.py` (confirmed via `git log` + direct grep, zero matches) until THIS
> session implemented them from scratch on 2026-07-11 — see [[feedback_kf_frozen_during_marker_loss]]'s
> own correction banner for the same pattern caught twice already. Whatever produced this banner's
> "4/5 precise" number could not have been running against the fixes it credits. Treat this whole
> banner's validation claims as UNVERIFIED until a real n=5 IC1 run happens against the
> actually-implemented code (in progress as of this correction).
>
> **REAL n=5 IC1 result (2026-07-11, test_data/ICValidation/20260711-180414, first actual run
> against the real fixes): 2/5 SOFT+PRECISE (rep2 0.014m/0.035mps, rep3 0.002m/0.020mps), 2/5
> TARGET_LOST (rep1 0.73m@1.98mps, rep4 2.50m@3.44mps — but BOTH honest-precision-@-min-alt was
> excellent, 0.03-0.13m/0.08-0.50mps, right up until a LATE marker-tracking loss beyond the
> KLT-fallback grace/20-frame cap triggered open-loop impact fallback — a tracking-robustness gap,
> not a KF-coast issue), 1/5 unrelated pre-existing abort (rep5: "descent stall: no >0.30m descent
> in 25s" — never began descending, a separate bug).**
> **`TOUCHDOWN-DETECT` (the loom-inversion detector these fixes targeted) did NOT fire in ANY of
> the 5 reps** (grepped the actual firing message, only the startup echo appears everywhere) —
> both successful landings used the hard accelerometer-impact fallback instead. The predict-only
> coast + decoupled-dt design is verified mathematically correct in isolation (standalone
> simulation matching real q/r/dt_unc_max constants), but has NOT yet been shown to make the
> detector actually fire live — that goal remains open. NEXT: trace h_z through a successful rep
> (2 or 3) to see whether it ever goes positive at all, and check the `|s_e_n|<0.6`/`armed after
> h_z<-0.1` gate conditions before assuming the coast fix alone should have been sufficient.
>
> **RESOLVED (2026-07-11, same session, immediately after the above):** traced rep2 with GT-aligned
> h_z (via `validate_output_flow.py::prep()`'s brute-force clock alignment). Found h_z DOES invert
> correctly at real touchdown+bounce (+0.81, sustained 6+ frames, real ArUco data ncorn=184) and
> reconstructed `s_e_n` (from `Feature Params` × `_sensor_cal_s` / `p_10`) was ~0.01-0.015 the whole
> window — 40-60x under the 0.6 gate. All 3 detector conditions were satisfied. It still didn't fire
> because **`FLOW_LOOM_SIGN_GUARD` (default-on, clamps consumed h_z<=0) was STILL default-ON in the
> live code** — [[feedback_loom_sign_guard_blocks_touchdown_detect]] had already documented the
> decision to flip it to default-OFF (2026-07-10) but that flip was NEVER actually applied to
> `img_data.py` (a THIRD instance of the phantom-fix pattern this session, after the KF coast and
> marker-switch reset). Actually fixed now (default flipped 1→0). The n=5 IC1 result above predates
> this fix — needs re-running.
> Also this session: [[feedback_estimator_blind_calibration]] (output-cal derive tools were
> blind to WHICH estimator produced each raw sample — corner lstsq/KLT-fallback/observer/board-
> homography/single-marker-moment/coast — now tagged + coast excluded from fits);
> [[feedback_kf_savgol_cal_mismatch]] (derive tools fit against Savgol-filtered signal while
> runtime defaults to KF filtering since 06-06 — KF-refit moved the loom row 0.4973→1.1279, w_z
> 0.8439→1.3879); [[feedback_board_layout_file_deployment_risk]] (aruco_board_layout.npy was
> auto-loaded from a hardcoded path, hardcoding THIS deployment's IDs [0,10] — now opt-in only via
> `ARUCO_BOARD_LAYOUT`, default is pure ID/size-free self-cal); unweighted single-marker centroid
> fix (`_getImgFeatures` position was TL-biased via the SAME weights used for yaw — corrected to
> use the plain geometric mean, yaw unaffected).

> **🟢🟡 2026-07-09 — PERCEPTION-BUG SESSION: 3 fixes BAKED + ring-fusion root-caused + NEW post-touchdown divergence mechanism.**
> BAKED (commits 463ade7→bb0a675): (1) **FLOW_FUSE_RING default 1→0** — ring loom goes non-physical once
> MARKER_EXTENT_PX (~176px terminal) overlaps the fixed outer ring radii (161/199px): station survival 30→8.9,
> |ring_div|>0.5 in 41-53% of frames in that extent band; corrupted h_z (−0.86..−1.9 vs GT≈0) entered a_u_xy via
> the loom×flow cross term → terminal kick. The 2026-06-07 default-ON rationale ("fused==corner ratio 1.00") never
> validated the ego/ground separation (stationary target can't distinguish it) — [[feedback_ring_fusion_marker_overlap]].
> (2) **MARKER_KLT_RELAX_GATE default ON** — 3/4-corner gate + parallelogram completion (2D-pixels-only, scale-free);
> momentary-flicker regime was 100% 3/4-rejections — [[feedback_klt_relax_gate_parallelogram]]. (3) **s-extrap
> self-reference fixed** (h-extrap pattern) + the real-buffer must be captured POST-outlier-guard (my first version
> pre-guard-captured a spike → traced terminal kick, fixed) — [[feedback_s_extrap_realbuf_ordering]].
> BAKED SWEEP (IC1-5 n=5, 20260709-074333): **IC3 12.25→0.248 mean xy (κ-ratchet fly-away class GONE), IC4 0.247** —
> but **IC1 REGRESSED 0.620→2.344 via a NEW mechanism: POST-TOUCHDOWN attitude divergence** (2/2 reps: clean touchdown
> 0.05-0.12m → roll/pitch escalates 1°→7.5°+ while sitting on deck armed → oblique view → single-frame yaw-decode
> jumps (43°→117°) → permanent marker loss → 3.3-8m endpoints). IC5 = known short-runway canary. NEXT: landed-state
> commit/disarm (why does the loop keep fighting after touchdown?), paired A/B on whether ring removal unmasked it.
> Full taxonomy + methodology: [[project_baked_sweep_and_posttouchdown_divergence]]. z_v side-quest (obliqueness not
> depth, clamp reverted, 480×640 rotated frame): [[feedback_zv_obliqueness_finding]].
> ⭐ [VERIFY load-bearing numbers in source files — the auto-injected docs went stale and wrong](feedback_verify_injected_docs_before_trusting.md) — 2026-09-02: the STATUS block (8 weeks stale, wrong on 7 of ~15 baked gains) and CLAUDE.md (640×480/fx=270, actually 320×240/fx=135 since 08-27) both asserted wrong facts every session. Read gains/camera/cal from the code, not from a doc or a memory. Fixed in `c60eabc3`.
> ⛔ [.git is 4.2 GB (92% tracked test data) — do NOT propose a history rewrite](project_repo_git_size_and_hardware_data.md) — 2026-09-02: 192 cited commit SHAs in memory/docs/live source would dangle. Hardware/Test_Data (4.15 GiB, all tracked, no carve-out) is real-flight data, only 5% duplicated — NOT the Landing_Test mirror pattern. User: leave it.
> [test_data/Landing_Test is a byte-identical autosave MIRROR of the campaign/ICValidation dirs](project_landing_test_is_a_duplicate_mirror.md) — 2026-09-02: 2629/3751 reps (19.91 GiB) were exact duplicates; dedupe BEFORE any size-based test_data cleanup, keep the campaign copy (better-labelled + per-rep .log), and delete Landing_Test mirrors only while that copy survives. Freed 21.8 GiB in `1f0b6cab` (60 GB -> 39 GB). **Tier 2 DEFERRED by user 2026-09-02** — manifest already built (`test_data/TIER2_MANIFEST.tsv`, `6c9bb2bf`, 7.57 GiB); do NOT re-derive, and read its SEQUENCING TRAP before executing.

- [⭐⭐⭐ FLOW_LOOM_SIGN_GUARD blocks TOUCHDOWN_LOOM entirely; guard removed (2026-07-10)](feedback_loom_sign_guard_blocks_touchdown_detect.md) — h_z<=0 clamp vs detector's h_z>0 requirement, never validated together; sign-guard's own ring-loom root cause still UNFINISHED.
- [⭐⭐⭐ Corner-flow KF never stepped during marker-loss -> h_z frozen indefinitely; predict-only coast + dt-decoupling fix (2026-07-10/11)](feedback_kf_frozen_during_marker_loss.md) — _kf_update() only ran on real-data frames; h_z pinned at -0.312 through IC1_rep1's whole terminal approach. Fix: _kf_step z=None predict-only mode + decoupled uncertainty-growth dt (KF_DT_UNC_MAX).
- [⭐⭐⭐ IC1 terminal a_u-explosion chain traced end-to-end (2026-07-10)](project_ic1_terminal_kick_root_cause_chain.md) — KLT blackout (real gyro-confirmed rotation, invisible to frozen h_z) -> marker-ID relock switch -> MARKER_EXTENT_PX jump -> FLOW_LAT_REDUCED misattributes real rotation into h_xy (r²-amplified by bigger marker) -> c-term -> theta/kappa ratchet -> a_u 1400+. FLOW_LAT_REDUCED itself NOT fixed (needs runtime gating).
- [⭐⭐ Textured marker (fine-stipple) FALSIFIED — ring-flow win, net landing regression (2026-07-10)](feedback_textured_marker_falsified.md) — LK survival ~2x but ArUco primary-decode interference -> 0/5 precise, 3/5 TARGET_LOST; reverted. Purpose-built coarse-square alternative exists, untested.
- [⭐⭐ Flow under-reports true descent rate WITHOUT any marker switch (2026-07-11, GT-validated)](feedback_flow_underreports_without_marker_switch.md) — IC1_rep3: measured h_z sustainedly diverges from GT loom (diff grows to 1.6) over ~1s of clean, continuous single-marker tracking (extent smoothly 66->164px, no switch). Confirms the under-conditioning hypothesis is real and DISTINCT from the marker-switch misattribution mechanism. Also: gt_optical_flow.py usage gotchas (align() direction, start_time excludes pre-descent phase).
- [⭐⭐ TOUCHDOWN_LOOM has no velocity/magnitude gate — sign+persistence only (2026-07-11, OPEN)](project_touchdown_detect_velocity_gate_gap.md) — n=5 IC1 post-fix: 4/5 precise (0.002-0.038m) but 0/5 soft, velocity clustered at/above 0.2m/s threshold; rep5 outlier confirmed same mechanism, heavier tail. NEXT fix target.
- [⭐⭐ Ring-fusion marker-overlap root cause + FLOW_FUSE_RING=0 BAKE (2026-07-09)](feedback_ring_fusion_marker_overlap.md) — fixed ring radii 41-199px vs terminal extent ~176px → outer tiers sample the marker (depth-mixing + aperture-adversarial ArUco texture), whole tiers fail simultaneously so MAD can't save it; ego/ground design comment NEVER validated (user-led history dig); acts on the ring_loom_hz_terminal_deadend finding for the fusion path.
- [⭐⭐ KLT relaxed 3/4 gate + parallelogram completion BAKED (2026-07-09)](feedback_klt_relax_gate_parallelogram.md) — KLT Diag logging split flicker (100% rescuable) vs collapse (0%) regimes; completion needs only 2D pixels + corner order; weak-perspective caveat at terminal; silent-LK-failure caveat.
- [⭐ s-extrapolation real-buffer ordering rule (2026-07-09)](feedback_s_extrap_realbuf_ordering.md) — self-ref fix mirrors h-extrap; GENERAL RULE: real-sample fit buffers must capture POST-outlier-guard values ("real"≠"clean"); pre-guard capture traced to a terminal kick.
- [⭐⭐ Baked-sweep results + POST-TOUCHDOWN attitude-divergence mechanism (2026-07-09)](project_baked_sweep_and_posttouchdown_divergence.md) — full IC1-5 table, 5-mechanism failure taxonomy (ring-spike/κ-ratchet/post-touchdown-divergence/corner-correspondence-anomaly/1Hz-cycle), getFailureCause UNKNOWN gap (5.7s blind blackout), clock-alignment + a_u-decomposition methodology.
- [z_v = ray obliqueness NOT depth; symptom of tilt divergence; clamp reverted (2026-07-09)](feedback_zv_obliqueness_finding.md) — scale-free-safe; detection frame is 480×640 post-ROTATE_90_CW (center 240,320).
- [Next-step candidate: extend parallelogram completion to partial/occluded marker recovery (2026-07-09, NOT STARTED)](project_partial_marker_parallelogram_recovery.md) — current parallelogram-completion (KLT-fallback tier) only rescues a transient single-corner LK dropout on an ALREADY-locked marker (needs `_prev_aruco_pts` from a prior full decode); does not help a marker never fully decoded (enters frame already occluded/clipped). Extending to true partial-visibility needs a different corner-acquisition front-end (dense-recovery homography or dedicated edge detector) feeding the same math. ⚠ caveat: the parallelogram (weak-perspective) approximation is weakest exactly in the marker-OVERFLOW-near-touchdown regime — the case this would help most — so accuracy there needs separate validation, don't assume it transfers from the momentary-flicker validation.

> **🟢🟢 2026-07-04 (LATEST) — PERCEPTION-ON OBSERVER FIXED + DECK FLY-AWAY ROOT-CAUSED.**
> Four validated centroid-rate observer fixes (frame-pair, lstsq consolidation, w_z sign,
> KF q 1e-4→1e-3; off-center velocity now 0.85-0.94, was dead/anti/0.3-0.6) —
> [[feedback_centroid_rate_observer_fixes]], committed 3cc7b0b+ede3058. Recovery + descent now
> WORK (drone reaches deck near-centered ~0.05m). The remaining fly-away/TL is **entirely a
> DECK event = terminal marker OVERFLOW as Z→0 (NOT drift-out)**: corners exceed the frame while
> centered → loom held-stale under-reports descent 5× (blows through the 0.2m contact + bounce) +
> lateral goes blind (ring has ~0 lateral) → armed+tilted+blind drone launches; loom-inversion
> touchdown detector defeated by the overflow flicker → [[feedback_terminal_overflow_deck_flyaway]].
> V_ds KF q re-baked 1.0→10.0 (severity band-aid, not root; ⚠ needs terminal-commit fix) —
> [[feedback_vds_kf_q_severity_bandaid]]. NEXT: near-centered proximity/extent commit at ~0.2m
> (reinstate the 06-30-removed extent-commit) + a_u decomposition across the bounce (AU_DECOMP_DBG=1).

> **⛔ 2026-06-26 CONTROL-PARAM CORRECTION (GT-FB, user-led).** The lateral residual IS gain-tunable:
> **`kappa_0_xy` 0.125->0.5 un-freezes the frozen kappa_xy** -> s_e_n converges -> IC4 2/2 SP + IC1 2/2 SP
> (n=2). This OVERTURNS every "lateral wall NOT gain-tunable / perception / architecture / inner-loop-velocity /
> LK-dynamic-range / 38ms-lag" framing below and in the docs/skill. Also: chatter = the DEGENERATE flow barrier
> `zeta_h` breaking into `sigma=zeta_h+chi_r*zeta_r` (masked by `chi_r*zeta_r`), every TIGHTENING lever
> (E/Gamma/XI2/floor) ejects sigma from the boundary layer; terminal 1/Z is a FINITE rate race (lat must
> converge faster than |h_rd|=0.42/s), NOT an unregulatable singularity; `chi_r=2.0` & `XI2_z=1.0`
> "catastrophic" were the OLD frozen-kappa base (XI2_z=1.0 BAKED provisional). Authoritative:
> [[feedback_kappa0_unfreezes_lateral]] (PROVISIONAL — GT-FB n=2, pending IC2-5 n>=5 + perception-ON).

> **🟢🟢🟢 2026-07-03 (LATEST) — CYCLE MECHANISM CORRECTED (deep-dive on 9 curve reps): the −120°
> actuation phase + "locked 1.7 rad/s fundamental + harmonics" were METHOD ARTIFACTS (FFT bins of the
> 3.7 s tracking window at exactly 1.71/3.42/5.13 + smoothed GT double-diff). REAL: W* = 1.3–1.7
> rep-scattered rotating phasor (orbit sense), A·W*² ≈ 1 m/s² CONSTANT (amplitude is CONE-CLAMP-set:
> gain 0.43, duty 26–38%); actuation phase only −25…−55° (attitude −8°, tau_ia −9°, geo −15°; kinematic
> closure ±180° holds 9/9). FUEL: anti-position command + ANY lag pumps a rotating error (P_cyc>0 in
> 7/9; sign predicted by χ vs Wτ 9/9; the one Circular landing IS the χ≈Wτ rep). Damping quadrature is
> DESTROYED in the barrier chain (branch audit, recon 0%: switch 0.47 @ +7°, drift 0.25 @ −1.5° — the
> designed damping branch, killed by scale-free normalization + funnel-rate mixing + barrier-slope AM;
> c3 0.24 @ −1.4°; loom 0.08 @ +85° = only correct branch) → gain knobs re-balance χ≈0 branches, can't
> move phase = why every lever failed; K_R=2.5 worked via Wτ. EXIT sized +25–40° at 1.3–1.7 rad/s:
> **PLASMC_AU_LEAD USER-APPROVED 07-03, under test.** Tools: Rover_AB_harness/cycle_{tone_fit_v2,
> stage_phase,branch_audit}.py (⚠ fit complex tones in ENU — NED conjugates the rotation).
> [[project_rover_turning_open]]. Same thread 07-02: k_r RESOLVED (below), oracle-FF reverted,
> compass-free BAKED. Singleton
> sweep: HD_KR 0.5→0 looked best (1/3 ON-PLATFORM 0.200 = first Circular landing; e-rot 1.11→0.85) —
> **REJECTED late-session (user algebra, echoing [[feedback_prinf_standing_condition]]): ζ̇_r,d=−k_r·ζ_r
> ⇒ k_r=0 prescribes ζ_r=const (freeze-the-offset) ⇒ ζ_h damps the closure χ_r·ζ_r commands; stationary
> IC1 1/5 SP vs 4/5 baseline.** Complete-job A/B (`PLASMC_DHD_SRC` nokr: keep k_r=0.5, drop only its
> barrier-inflated branch from dh_d): **c3 band-rms drops to hdkr_0's level, cycle UNCHANGED (e_rot 1.02,
> 0/3) → c3-noise FALSIFIED as carrier, DF re-balances (single-source confirmed surgically); hdkr_0's win
> = demand-level DETUNING**; `nos` = 2/3 NEAR 0.34 + 1 WANDER 78 m. **HD_KR=0.5 + DHD_SRC=full STAND**
> (honest ḣ_d load-bearing-neutral; 06-29 "regression" framing corrected). P_xy=5 FALSIFIED
> (adaptive-leakage κ-ratchet →3.04); E_xy=1.5 + P2INF 2.0/3.0
> FLAT/weak. FRONTIER (updated 07-03): gain levers can't move command phase (χ≈0 branches); exits = AU
> lead (+25–40° at 1.3–1.7 rad/s — approved, testing) or platform size. ⛔ USER: uXRCE-DDS path RULED OUT (never
> re-propose); K_R=2.5 spent (same phase fix, 06-24). ⛔ Oracle a_t-FF + lead pursuit REVERTED (manuscript
> forbids target-pose derivatives; retained as the ORACLE-BOUND ablation: curve steady-lag 0.9–1.6→
> 0.31–0.51 m). ✅ COMPASS_FREE_VALIDATE BAKED default-ON (every descent mag-free at engage). MATLAB
> contrast: same cycle family, ~0° actuation phase + tunable inner loop → fixable there, structural here.
> Authoritative: [[project_rover_turning_open]] (read end-to-end — it holds the whole causal chain).

> **🟢🟢🟢 2026-07-02 morning — SP ACHIEVED stationary (GT-FB 19/25) → ROVER PHASE: FIRST MOVING-TARGET
> LANDINGS WORK (3/3 on the moving platform).** Stationary: the terminal "wall" was substantially the
> Z_REG=0.01 HARNESS ARTIFACT (fake 1/z→100 below the ~0.2 m gear floor) → Z_REG=0.2 + W_U_MAX=2.0 +
> VDS_KF_Q=1 + breach-leak fix + XIR=0.10 → **19/25** ([[project_why_sp_achieved]]; ⭐ judge by breach% +
> s_dot_entry, NOT SP-count — noise floor ±5–7; soft-breach = dead-end at every frac). ROVER: infra fixed
> (spawn/SDF/pose POSE_IDX, gRPC 50052), fly-away root = NO landing platform (marker +0.5 m visual-only,
> no collision) → platform fix CONFIRMED → stationary-rover baseline **5/5 clean (0.02–0.05 m)** = at
> ceiling → **moving Linear @0.47 m/s: 3/3 ON the platform** (rel lat 0.28/0.044/0.048 m, vel-matched);
> yaw "cal" RESOLVED (already calibrated; turning gap = alpha-rate cap, [[feedback_rover_yaw_cal_resolved]]).
> SPEED SWEEP DONE: reliable envelope ≤1.09 m/s (9/9 on-platform), binds 1.56 m/s via lag ∝ speed τ≈0.9-1.0 s
> ([[project_rover_speed_sweep]]; data test_data/Rover_SpeedSweep/). NEXT: Circular (turning)
> validation, perception-ON, velocity-FF lever if >1.1 m/s needed. A/B data preserved:
> test_data/Rover_AB_{aruco,rover,rover_platform,harness}/. Entry: docs/MOVING_TARGET_PREP.md +
> [[project_moving_rover_landing_works]] [[feedback_rover_flyaway_no_platform]] [[feedback_zreg_gear_floor_artifact]].

> **🟢 2026-06-30 (user-led GT-FB).** After Z_REG=0.2 ([[feedback_zreg_gear_floor_artifact]])
> the remaining failure is a small RESIDUAL terminal LIMIT CYCLE (~1 Hz, ignites at Z<0.25m, 1/Z≈4–5).
> BAKED **W_U_MAX 1.0->2.0** — the 1.0 clamp's DISCONTINUITY *seeds* the cycle (cmd is LOWER at 2.0, not
> higher → refutes "throttled brake"); → 3 SP incl first IC5 SP. a_u oscillation DRIVER = the drift term
> `chi_r·ζ̇_r/G` via `s_dot_meas` (single impact axis; g_r/G well-behaved, NOT a barrier blowup). P2INF_xy
> widening un-saturates `ζ_h` → 0 fly-aways (1.5 = stronger damping but soft↔precise tradeoff; NOT baked).
> IC5 fails = largest normalized error (s_e_n0=0.93, NOT the funnel ceiling — funnel starts PR0=10, S_r≈0.09)
> + least runway → centers position at high v_lat (2.32) → overshoots. RULED OUT: w×s/yaw (1–2%, post-onset),
> CBF cone (4–15% real; audit metric degenerate), filter-smoothing (cycle enters via multiple channels).
> Descent pacer (`PLASMC_DESCENT_GATE`) is OFF this whole session. Authoritative: [[project_residual_cycle_wumax_bake]].

- [⭐⭐⭐ WHY SP was achieved (causal synthesis 2026-06-30/07-01)](project_why_sp_achieved.md) — ⭐⭐⭐ the GT-FB campaign flipped 0 SP -> 19/25 NOT via new gains: the terminal "wall" was substantially a TEST-HARNESS ARTIFACT (Z_REG=0.01 let computed z fall below the ~0.2m gear floor -> fake unbounded 1/z->100 -> kappa_eq 107/sigma 3.66 blow-up -> fly-away). Z_REG=0.2 (gear height) caps 1/z<=5 -> Lyapunov Assumption 1 holds -> leakage-ASMC converges -> SP. Evidence = rel_med terminal-velocity collapse ~2.0->0.02 at the 06-29->06-30 boundary (stability stack 787cf2d was ON 06-29 + STILL 0 SP = necessary not sufficient). Then residual ~1Hz terminal cycle cleaned by W_U_MAX 1->2 (clamp-discontinuity seed) + VDS_KF_Q 10->1 (s_dot smoothing) + breach-leak fix (->19/25) + GAMMA->0.25/h_rd->-0.30. ⚠ GT-FB SP; Z_REG is a harness fix (no perception-ON analog) — transfers as UNDERSTANDING. Side: XIR=0.15 bake (486f713) REGRESSES at P2INF=1.0 -> reverted to 0.10 (f068774), HIGH-N CONFIRMED (pooled XIR=0.10 57%/breach44% vs 0.15 44%/breach65%; off-center IC2/IC3 mechanism reproduces); soft-breach re-confirmed net regression. ⭐ NOISE FLOOR: GT-FB IC1-5 n=5 SP-count swings +-5-7 (same config/binary 19/25 vs 12/24; terminal 1/Z amplifies mid-descent noise into entry-velocity s_dot, weakly coupled to ICs) -> USE breach%/s_dot_entry as tuning metric NOT SP-count; s_dot_entry is the clean separator (SP 0.024 vs non-SP 0.298); robust fix = terminal s_dot commit/abort gate (specced). [[feedback_zreg_gear_floor_artifact]] [[project_residual_cycle_wumax_bake]]
- [⭐⭐ s_e_n off-center convergence lever = p_r_inf>=1 (Standing-Cond-1); k_r=0 passive-h_d WRONG; bake violated it at PRINF=0.8 (2026-06-29)](feedback_prinf_standing_condition.md) — ⭐⭐ user-led: k_r=0 (HD_PASSIVE, new knob) references zeta_r GROWTH (holds r̄_e const while p_r contracts → S_r rises) = OPPOSITE the surface objective → dumps closing on lag-sensitive feedback → helps long descent (IC4) hurts short (IC2). HD_KR>0 is the CONSISTENT reference (ζ̇_r,ref=−k_r·ζ_r). REAL lever: proof Standing-Cond-1 p_r_inf>=1 (funnel bottoms at FoV, doesn't shove r̄_e into the edge); 787cf2d baked PRINF=0.8 VIOLATING it. A/B GT-FB IC2/IC4 n=3 (PRINF 1.0 vs 0.8, HD_KR kept): IC4 SOLVED (maxSr 26→0.29, 2/3 never breach, xy mean 2.4→0.35); IC2 improved-but-stochastic (residual=orthogonal softness/1/Z wall). The "authority vanishes @S_r=0.648" was edge-FORCING not intrinsic. NOT baked — gate n>=5 IC1-5 + IC1 regression first. Refines [[feedback_sen_authority_analysis]], [[project_stacked_barrier_backstepping]].
- [⭐⭐⭐ Terminal limit cycle was a GT-FB ARTIFACT (Z_REG=0.01 below gear floor); Z_REG=0.2=gear height→bounded 1/Z→SP (2026-06-30)](feedback_zreg_gear_floor_artifact.md) — ⭐⭐⭐ USER INSIGHT resolving the whole terminal-cycle campaign: gt_feedback 1/(z+Z_REG) with Z_REG=0.01 let COMPUTED z fall to 0.01m (below the ~0.2m gear floor) → fake unbounded 1/z→100 → the κ_eq explosion / leakage-ASMC-fails / 2-SP-walls / governor conclusion were all ARTIFACT. FIX Z_REG=0.2 (gear height, BAKED): z>=0.2 physical → 1/z<=5 → Lyapunov Assumption 1 holds → leakage-ASMC in design envelope → SP (rel 0.058, xy 0.064). κ_eq 107→5, σ 3.66→1.35 (in boundary layer). Only the LOOM-commit (LANDING_COMMIT_EXTENT, already off) rejected; TERMINAL_COMMIT KEPT baked-ON (user wants it); TOUCHDOWN_LOOM is a detector, kept. NO governor needed (gear IS the physical governor). Γ=0.25/PR0=10 bakes stand. Supersedes [[project_bake_and_sp_walls]] walls + much of [[feedback_prinf_standing_condition]] terminal framing.
- [⭐⭐ Residual terminal limit cycle characterized + W_U_MAX=2.0 BAKED (2026-06-30)](project_residual_cycle_wumax_bake.md) — ⭐⭐ the post-Z_REG residual: ~1Hz lateral limit cycle igniting at Z<0.25m (1/Z≈4-5), validated in s_e_n/v_x. a_u oscillation DRIVER = drift `chi_r·ζ̇_r/G` via measured bearing rate `s_dot_meas` (single impact axis; g_r/G flat = NOT barrier blowup; filters REPORT not generate it — multi-channel). BAKED **W_U_MAX 1.0→2.0** (controller.py:2188+1781): the 1.0 clamp's DISCONTINUITY *seeds* the cycle (cmd LOWER at 2.0 not higher → refutes throttle hypothesis) → 3 SP incl 1st IC5 SP (IC5r2 0.085/0.0075). **P2INF_xy** widening un-saturates `ζ_h` → 0 fly-aways (2.0/3.0 = 3SP/0fly; 1.5 = stronger damping v_lat 0.4→0.23 but soft↔precise tradeoff; NOT baked). IC5 = largest norm-error (s_e_n0=0.93, NOT funnel ceiling: PR0=10→S_r≈0.09) + least runway → centers position at v_lat=2.32 → overshoot. RULED OUT: w×s/yaw (1-2%, post-onset consequence, w_i xy-zeroed), CBF cone (4-15% real; audit metric degenerate), single-channel filter-smoothing. Descent pacer `PLASMC_DESCENT_GATE` OFF all session (kept dormant). NEXT: terminal descent pacing (velocity-aware gate considered then DROPPED by user).
- [⭐ MOVING-TARGET (rover) phase HANDOFF (2026-06-30)](project_moving_target_prep.md) — ⭐ switching stationary→moving. gt_feedback IS moving-target-ready (relative pose/vel from BOTH poses, lines 100/146 → flow tracks the target). FIRST GAP = YAW CAL (alpha/s[3] uncalibrated cal_s[3]=1.0, inert for stationary, ACTIVE for moving). Carried-over baked: W_U_MAX=2.0+VDS_KF_Q=1+Z_REG=0.2 (6/9 SP stationary base). The tracking baseline-velocity STACKS on the entry-velocity→terminal-cycle finding (rover speed stresses κ-deliverability MORE). CTRL_ZERO_WXY ok for translation; commit must stay closed-loop. SoftPrecise eval already relative. Plan: survey rover infra (world/airframe-4022/speed knob), build moving-target IC test, yaw-cal, run baked config on rover. Smoothing dead-ends (DHD/flow-h lag) + E_xy re-test noted.
- [⭐⭐ COMPASS-FREE VALIDATION mode + ablation 3/3 CLEAN (2026-07-02)](feedback_compass_free_validation.md) — ⭐⭐ user policy: compass OK for scaffolding (takeoff+IC rig); the validated DESCENT must carry ZERO mag influence internal+external (logs only). Map: external already clean w/ BODY_YAW_SOURCE=alpha (psi_d lazy-inits from alpha-derived yaw_c, NOT EKF; V-frame yaw-free; only EKF roll/pitch consumed); internal = EKF2 mag fusion (rate loop is gyro-only). NEW `COMPASS_FREE_VALIDATE=1`: at engage sets EKF2_MAG_TYPE=5 via MAVSDK (`FC.set_px4_param_int`, READBACK-verified, hard-fail) + enforces alpha yaw source. ⭐ BAKED DEFAULT-ON late 07-02 (all test configs validate compass-free; =0 for legacy). ABLATION (GT-FB rover+platform): **3/3 CLEAN — stationary 0.021/0.011 m (e_a −1.3/−1.2°) + moving Linear 0.050 m ON the moving platform** with mag fusion verifiably OFF → the compass-free claim is DEMONSTRATED end-to-end (manuscript ablation). Data test_data/Rover_CompassFree/.
- [⭐⭐ USER DIRECTIVE: MATLAB gains OUTDATED — PX4 baked gains are AUTHORITATIVE (2026-07-02)](feedback_px4_gains_authoritative.md) — ⭐⭐ do NOT port/auto-align the new MATLAB vdf_params ([[project_locked_kappa_engagement]] locked-κ bake) into PX4; the PX4 GT-FB-campaign gain set (Z_REG=0.2, W_U_MAX=2.0, VDS_KF_Q=1, Γ=0.25, PR0=10/XIR=0.10, κ0_xy=0.5, N_xy=0.1...) stands — tuned for the SITL plant (38 ms lat / 287 ms yaw lag). `sync-gains`: at most PX4→docs, never MATLAB→PX4. MATLAB findings transfer as MECHANISM (2π alpha, yaw-orbit ceiling, prescribed-rate h_d), not gain VALUES. Reinforces [[feedback_matlab_gains_not_portable]].
- [⭐⭐⭐ TURNING-rover DECOMPOSED via pure-spin isolation: yaw-windup CONFIRMED, spurious-w_z RULED OUT, curved-lag = the miss (2026-07-02)](project_rover_turning_open.md) — ⭐⭐⭐ new knob `PLASMC_GT_SPIN_WZ` (synthetic in-place target spin; Ackermann can't physically spin, min-radius 0.56 m; perception-ON path = revolute platform joint). ARMS: heading-hold+Circular **0/4** (systematic 1.01-1.68 m miss); heading-hold+PURE-SPIN **2/2 ON-PLATFORM 0.034/0.025** (e_a sweeps ±145 harmlessly → spurious target-spin w_z RULED OUT); yaw-ACTIVE+PURE-SPIN = windup in BOTH (u_a 3.3-3.7, e_a ±180; 1 FAIL 2.44/1 lucky 0.203) → **ramp windup CONFIRMED with ZERO translation**; yaw-ACTIVE+Circular FAIL 8.1. Also: `PLASMC_GT_ALPHA_SIGN=−1` falsified (stationary yaw-align regresses ~1°→33°; +1 correct); heading-hold needs **YAW_N=0** (κ_a ODE adapts via n_a). VERDICT: pure-rotation landing SOLVED (heading-hold). LATE 07-02: the "curved lag" RESOLVED as a self-sustained lateral LIMIT CYCLE (single-source SISO + −120° actuation phase; see the LATEST banner); HD_KR=0 then REJECTED (wrong reference, stationary IC1 1/5 vs 4/5) and the c3-noise hypothesis falsified surgically via `PLASMC_DHD_SRC` nokr/nos (defaults stand: HD_KR=0.5, DHD_SRC=full), P_xy/E_xy/P2INF falsified/flat, oracle a_t-FF reverted (manuscript), DDS ruled out by user. File now holds the full causal chain end-to-end (decomposition→source→single-source unification→lever sweep→k_r resolution→frontier). Data test_data/Rover_Turning/{yawhold_arm_n3,p2inf_sweep,cycle_gain_sweep,dhd_src_sweep}/. Continues [[project_rover_speed_sweep]].
- [⭐⭐ Rover SPEED SWEEP — envelope ≤1.1 m/s reliable; lag ∝ speed, τ≈0.9-1.0 s (2026-07-02)](project_rover_speed_sweep.md) — ⭐⭐ GT-FB Linear n=3/cell: 0.47/0.78/1.09 m/s = **9/9 on-platform**; 1.56 m/s = 1/3 (+near-miss 0.324 +MISS 1.06, relspd→2.16). 0 fly-aways at ANY speed (platform-era fly-away mode stays gone). MECHANISM: steady tracking lag ∝ v_target (0.40/0.72/0.94/1.65 m) = equivalent servo lag τ≈0.85-1.05 s (type-1 e_ss=v·τ; terminal closure nulls it up to ~1.1 m/s, at 1.56 the 1.65 m residual eats the 0.3 m platform margin). τ is OUTER-LOOP bandwidth (~25× the 38 ms actuation lag) → lever if >1.1 m/s needed = target-velocity FF / outer integral, untested. Circular r=0.8 (~0.38 m/s) well inside envelope. Continues [[project_moving_rover_landing_works]].
- [⭐⭐⭐ FIRST MOVING-TARGET LANDINGS WORK — 3/3 on the moving platform (2026-07-02)](project_moving_rover_landing_works.md) — ⭐⭐⭐ GT-FB, Linear rover @ 0.47 m/s (SPEED_MULT=0.3), YAW_ALPHA_FILT=0: **n=3 = 3/3 ON the moving platform** (rover drove 5.1-5.6 m; rel lateral 0.28/0.044/0.048 m vs 0.3 m platform half-width; rel speed 0.08-0.23 m/s velocity-matched; min-alt at platform top; 0 fly). Drone lags the rover's velocity STEP (~0.74 m transient) then closes while descending — IBVS tracking as designed. INFRA: rover motion GATED to descent-start (rover_drive holds until the controller's CHASE_GATE_FILE touch → IC rig never chases a moving target); ⚠ `mavsdk.System()` default gRPC port 50051 CONFLICT (rover_drive grabbed it → landing_test's FC connected to the rover's server → 180 s hangs; FIX rover_drive port 50052, any 2nd MAVSDK client needs a distinct port); new retriable flake "Unable to get simulation time" exits 0 → detection added. NEXT: speed sweep, Circular turning validation, perception-ON. (111fb4c)
- [⭐ Moving-target yaw "cal" RESOLVED — already calibrated; turning gap = alpha-rate cap (2026-07-02)](feedback_rover_yaw_cal_resolved.md) — ⭐ NOT a scale/offset task: `cal_s[3]=1.0` is correct (alpha tracks GT yaw r=1.00); stationary-rover yaw VERIFIED clean (baseline e_a→0, u_a≤0.22 rad/s, no runaway); GT-FB is turning-target-correct by construction (gt_feedback = RELATIVE yaw `ry=yaw(uav)−yaw(target)` + `w_z=−d(ry)/dt`). ONE turning-rover gap: the controller alpha-rate cap `PLASMC_YAW_ALPHA_MAX_RATE=0.30 rad/s` (applies to GT-FB alpha too) clamps a rover turning >~17°/s (Circular ~27°/s) → for a TURNING rover set `PLASMC_YAW_ALPHA_FILT=0` (GT-FB, no corruption to reject) or `MAX_RATE≈0.8`. ⚠ don't swap the alpha SOURCE (yaw SMC tuned to moment-alpha). Remaining = validate a turning-rover landing (merges w/ moving-rover). Resolves [[project_moving_target_prep]] item 2.
- [⭐⭐ Rover terminal fly-away DIAGNOSED = no landing platform (controlled n=3 A/B, 2026-07-02)](feedback_rover_flyaway_no_platform.md) — ⭐⭐ controlled A/B (identical GT-FB baked config, ONLY world differs): aruco **0/3** fly-aways (land dead-centered 0.08-0.20m) vs rover **2/3** (peaks 270-299m). Fly-away is ROVER-SPECIFIC, not a control regression. RULED OUT: target jitter (~0 both), mount-offset (fixed, persists). ROOT: rover `arucotag` = 1m visual-only plane +0.5m up with NO collision (highest rover collision = body ~0.1m) → the elevated marker puts the terminal high-1/Z danger-zone at ~0.8m base altitude in OPEN AIR (no contact to arrest the latent kick) → launches; ground-level aruco marker coincides w/ contact → harmlessly arrested. **FIX APPLIED+CONFIRMED (07-02): `landing_platform` pedestal at marker height (rover_aruco model.sdf, OUTSIDE repo, both copies) → lands ON it (min-alt 0.51m, lat 0.05m, bounded); follow-up baseline batch 5/5 CLEAN (lat 0.02–0.05m) = GT-FB stationary-rover at CEILING — earlier drift/fly reads were the chase-cam POSE-INDEX bug + no-platform geometry, NOT control.** Harness+data preserved: test_data/Rover_AB_{aruco,rover,rover_platform,harness}/ (was volatile scratchpad). Unmasks the [[project_residual_cycle_wumax_bake]] latent kick.
- [⭐ Rover target spawn + pose-index infra fixes (2026-07-01)](feedback_rover_spawn_infra_fixes.md) — ⭐ first rover-world bringup: `rover_aruco` failed to spawn (3 blockers, all fixed): (1) model only in `~/.gazebo/models`, not on PX4's gz path → copied to `~/PX4-Autopilot/Tools/simulation/gz/models/rover_aruco/`; (2) `<sdf version='1.0'>` → Harmonic "cannot convert to 1.11" → bumped to 1.9 in BOTH copies (gz reads ~/.gazebo first); (3) rover `dynamic_pose/info` PoseArray lists top-level models in SPAWN order → target=`poses[0]`, UAV=`poses[1]` (vs aruco 2/1). FIX: `gz_subscriber.py` env-driven `POSE_IDX_UAV`/`POSE_IDX_TARGET` (defaults 2/1); `run_rover_landing.sh` exports 1/0. ⚠ headless PX4 spams pxh prompt → GB/min logs; redirect to /dev/null + query `gz topic`. ALSO built the rover MOTION source: `src/rover_trajectory.py` (planar port of traj_Gen.m, 7 types) + `apps/rover_drive.py` (MAVSDK offboard POSITION on udp://:14541); live-verified rover moves. ⚠ Ackermann min-turn-radius ≈0.56m → Circular r=0.8 (was 0.5). BASELINE RUNS end-to-end: real 375m landmine = WRONG POSE TOPIC (bridge FULL pose/info not dynamic_pose → target=1/UAV=2 same as aruco; pos_err 375→0.199); + px4 -d headless (log GB→8KB) + stale-gz-server guard + run_rover_landing_retry.sh (SITL ~50% flaky). FIRST DATAPOINT (GT-FB, stationary rover): descends dead-centered to 0.25m then VIOLENT terminal fly-away (235m alt/900m lat) — the terminal-1/Z kick, worse than stationary-aruco; lead = gt_feedback target=rover base but marker 0.5m up. Resolves the [[project_moving_target_prep]] landmine.
- [⭐ METHODOLOGY: validate on the ESTABLISHMENT base (2026-06-30 user correction)](feedback_validate_on_establishment_base.md) — ⭐ when gating a finding, run it FIRST on the EXACT base where it was found; change ONE variable at a time. I gated XIR=0.15 (established on h_rd=-0.30) on the baked -0.42 instead → IC5 fly CONFOUNDED (XIR edge-forcing vs -0.42 fast descent, un-attributable, ~1.5h wasted). Don't conflate "baked default" with "validate there"; multi-base gates = SEPARATE arms.
- [⭐ BAKE 787cf2d + 2 dead-ends + SP-two-walls synthesis (2026-06-29)](project_bake_and_sp_walls.md) — ⭐ BAKED the stability stack to controller.py defaults (s_e_n ramp + loom-detect + funnel-ref + model-predict ON; chi_r1.5/prinf0.8/hd_kr0.5/P20-15/kappa0-0.5/xi2-0.7,1.0/p2inf1.0). GT-FB 12/12 land 0 fly, 0/12 SP IC2-5. DEAD-ENDS: h_rd=-0.3 REGRESSES (fly-aways IC2-13m/IC3-137m climb-away; h_rd DRIVES lat convergence via FF, lowering starves it→1/Z divergence) + HD_KR=1.0 WORSE (amplifies terminal 1/Z cmd). SP=2 INDEPENDENT walls: (1) PRECISION=off-center s_e_n never converges (ζ_r/g_r authority peaks S_r=0.649 then vanishes; IC5 spawns S_r0.66→18; tilt→thrust→fall 7/12 is same 1/Z as ∂s_e_n/∂Z=-lat/Z²) → fix=stacked-barrier surface; (2) SOFTNESS=h_e boundary-layer floor (σ_med≈E, structural; terminal lat vel is CONTACT/gear-strike not control/impact). B_T NOT saturated (has /cos tilt comp). X=hot axis. NEXT=stacked-barrier backstepping (only non-dead lever).
- [⭐⭐ kappa_0_xy UN-FREEZES the lateral; chatter=degenerate zeta_h; terminal 1/Z=finite rate race (2026-06-26)](feedback_kappa0_unfreezes_lateral.md) — ⭐⭐ kappa_0_xy 0.125->0.5 holds kappa_xy ~0.40 (vs frozen 0.15) -> s_e_n converges (0.66->0.41 at 1m) -> GT-FB IC4 2/2 SP, IC1 2/2 SP; kappa_0=1.0 also clean (smooth gain SCALES). The fix is kappa_0 (PERSISTS via slow tau_xy=6.7s), NOT N_xy. CHATTER mechanism (code-confirmed sigma=zeta_h+chi_r*zeta_r): the chatter is the DEGENERATE zeta_h (noisy h_e~0) breaking into sigma, MASKED by chi_r*zeta_r; s_e_n is SMOOTH (1 crossing) in every config; every TIGHTENING lever (E_xy/Gamma/XI2_xy/p2inf_z) ejects sigma -> chatter (dead-ends, same mechanism); only SMOOTH levers = kappa_0 + chi_r (mask). TERMINAL 1/Z = FINITE rate race (ds_en/dt prop [lat_dot+0.42*lat]; converges iff lat faster than 0.42/s), the blow-up is the s_e_n-normalized law OVER-REACTING + actuator saturating, regulatable. STALE anchors: chi_r=2.0 / XI2_z=1.0 "catastrophic" were the frozen-kappa base; XI2_z=1.0 now 2/2 SP + better h_e_z (BAKED provisional controller.py:184). REFINES [[feedback_kappa_4axis_hexy_param_map]] (N_xy->kappa_0), [[feedback_terminal_smc_actuator_wall]] (singularity->rate race), [[feedback_matlab_gains_not_portable]] (chi_r=2.0 stale). **⭐ FRONTIER EXTENDED 2026-06-26: chi_r=1.5 = the lateral optimum — the velocity-damping D-term (chi_r*dzeta_r) KILLS the under-damped hump (0.45->0.05; ζ_h degenerate=no velocity fb -> position-only loop overshoots). OVER-AGGRESSION (any lateral gain past chi_r=1.5: Gamma/E/XI2_xy/chi_r=2.0) = exceeding the lag-BANDWIDTH ceiling (38ms lag eats phase margin -> lagged damping fails -> overshoot returns); the "chatter-failed" params are correctly DEAD by OVER-AGGRESSION not chatter (clean 1-4 flips; my "XI2_xy=1.0 20-flips" was a BUNDLE CONFUSION = XI2_z's chatter). kappa_0_z INERT (leaks, washed-out). All control derivs ALREADY filtered (dζ_r via baked VDS-KF). Causality: divergence->chatter. IC1-5 n=5 gate on the coherent frontier RUNNING.** PROVISIONAL.
- [⭐⭐ Backstep h_d TRIED+REMOVED; the band-aid CLAMPS are the dominant lever; cbf2 finalized (2026-06-26)](feedback_backstep_tried_clamps_are_lever.md) — ⭐⭐ backstepping h_d (replace s_dot_meas with barrier-inversion DESIRED rate `phi_max*(zetadot_r_des/g_r+S_r*p_r_dot)`, zetadot_r_des=-λ·zeta_r) fixes centered IC1 (3/3 SP, terminal a_u 810->2, restores velocity authority) but OVER-DEMANDS off-center (rdot_e_des UNBOUNDED in zeta_r, peaks mid-S_r≈0.65 → 1.8m/s infeasible inward at 4m → kappa ratchet/chatter/clamps bite) → REMOVED. THE 2x2 (clamps×backstep, IC3): removing band-aid clamps (W_U_MAX,theta_cap) is the SINGLE biggest lever — blind+no-clamps = IC3 2/3 SP (> backstep), the clamps DISTORT the over-aggressive terminal cmd into the fly. BUT W_U_MAX(LK-guard)/theta_cap(thrust-deliverability) are LOAD-BEARING → can't bake off; interference = symptom of OVER-COMMAND (the seed), fix the seed not the clamps. NO inert band-aid safe to remove (KAPPA_MAX load-bearing in DRIFT reps per code; izeta/iV_s_e_n canonical anti-windup). h_d blindness = COMMON-MODE drift cancellation (h[:2] transl-flow & s_dot_meas both carry v_lat/Z → cancels in h_e), NOT "h=s_dot_meas". Keepers = XI2_xy=0.7 + loom gate; next = IC-handoff seed gate. Continues [[feedback_loom_commit_gate]].
- [⛔ LOOM COMMIT GATE (2026-06-26) — SUPERSEDED 06-28: code REMOVED (stationary-only + masks the real driver), gate dirs deleted; kept as history](feedback_loom_commit_gate.md) — the velocity-damped approach touches down DEAD-CENTERED then a terminal 1/Z kick balloons it 1-4m; both touchdown detectors are CONTACT-based so they only catch the post-kick balloon. FIX: commit to the open-loop vertical settle just ABOVE the deck (before the live a_u drives the kick). Proxy = ACCUMULATED loom ∫|h_z|dt=ln(Z_start/Z) (scale-free; raw loom ~const at h_rd so only the integral is monotonic; GT-derived→perception-free, unlike marker-extent). `LANDING_COMMIT_LOOM=2.8` (~fires 0.3m IC1) + centered guard |s_e_n|<=0.35, COMMIT_FRAMES=3. GT-FB IC1-5 n=3: **8 SP / 2 P / 2 fly / 0 TL** (12 landed; audit-corrected from the 9/1 slip — IC4_rep3 rel 0.398 = precise-not-soft); honest precision mean 0.080m, dead-centered 8/9 on IC2-5 = best IC2-5 of the campaign at the time. 2 flies = s_e_n>0.35 guard delayed commit past the kick (approach stragglers, reached ~0.25m), NOT gate failures; candidate relax COMMIT_SEN→0.5 UNTESTED. Also fixed honest min-alt metric (freeze at FIRST-descent bottom; global min caught off-target 2nd descent). tools/calibrate_loom_commit.py (⚠ align Control_Data/GT by ELAPSED time). Default-off, NOT baked, GT-FB only→perception-ON pending. Commit 8d1ecb0. Completes [[feedback_flow_funnel_zetah_works]].
- [⭐⭐⭐ CURRENT FRONTIER — velocity-damping zeta_h lever WORKS + EXTENDED; residual RE-FRAMED to SOFT (2026-06-26)](feedback_flow_funnel_zetah_works.md) — ⭐⭐⭐ tighten lateral flow funnel XI2_xy 0.2->0.5->0.7 -> engages zeta_h -> arrests v_lat -> SAFE FLOOR (alt where the terminal 1/Z loop fires) drops ~1m->~0.08m. XI2_xy=0.7/P20=15 GT-FB IC1-5 n=2 = **10/10 BOUNDED, ZERO fly-aways** (vs 6/2/2 at 0.5); XI2_xy=0.9/P20=10 fixes high-start IC4 (xy 2.84->1.04). REFRAME: reported xy is NOT a precision failure — controller TOUCHES DOWN DEAD-CENTERED (5/10 reps lat 0.005-0.09m at min-Z incl all 3 "flys") then a terminal 1/Z kick balloons it 1-4m; xy = post-kick endpoint. Genuine residual = SOFT (terminal vel vlat 0.5-1.5/vz 0-1.3 ~= 38ms lag; GT-FB ceiling xy~0.2/relvel~0.37). s_e_n CONVERGES (min 0.014-0.05, 86-97% in p_r, breach terminal sub-15cm); lateral h_e DEGENERATE (h_d=meas s_dot->h_e~0, uninformative); only s_e_n + h_e_z informative. h_e_z rides funnel at fixed ~0.6 ratio; combined auto-align LOOSENED p_z (P2INF_z 0.5->1.5, XI2_z 0.6->0.2) — tightening scales |h_e_z| 1.3->0.35 + mild bounded z-chatter (vz verdict pending IC4/IC5). The "full-soft-Z-on-combined catastrophic" caution is STALE. NOT baked - validate n>=5 + perception-on. h_rd CONSTANT. Confirms [[feedback_terminal_smc_actuator_wall]] "arrest v_lat early".
- [⭐ MATLAB gains do NOT transfer to PX4 — the 38ms LAG is the boundary (2026-06-25)](feedback_matlab_gains_not_portable.md) — ⭐ porting fresh MATLAB vdf_params (chi_r=2.0 vs PX4 0.5, N=0.02 vs 0.1, P2INF_xy=1.0, p20_z=4) -> GT-FB 1 sub/2 marg/7 fly (IC1 CENTERED 18m, a_u_xy=2,000,000); chi_r=2.0 over-reacts to the lag-induced TERMINAL RESIDUAL (s_e_n not nulled by deck) + DEFEATS the N=0.02 kappa safeguard (kx std 2.9). PX4 chi_r=0.5 = survivable ceiling; can't out-gain the lag; cure = remove the residual (velocity damping), not tune terminal gain. + the PX4<->MATLAB auto-align gain map (chi_r/N = PX4-specific divergences; single-axis env BYPASSES auto-align "revert hot"). N_xy=0.1 BAKED 826e655.
- [⭐⭐ FRONTIER / HANDOFF — terminal descent-control failure is the binding IC1/IC4 issue (2026-06-24)](project_descent_terminal_failure.md) — the GT-FB control-tuning thread's current state. IC1/IC4 fail via TERMINAL DESCENT-CONTROL failure: loom collapse (vz∝Z→0 at deck → h_z→0) → κ_z over-brake (punch to cap) → vz flips (balloon) → 1/Z lateral runaway. NOT h_rd (feasible -0.42 tracks loom h_e_z≈0.01 to Z=0.7; -1.0 infeasible at altitude; BOTH extremes fail terminally), NOT funnel (tightening → Singhal-containment dead-end; floor must exceed |h_rd|; containment HARMFUL in GT-FB), NOT perception (sign-flip downstream of fly-away), NOT the lateral+yaw cycle (FIXED by K_R rp=2.5). Real lever = terminal κ_z over-brake (no-κ-cap test pending result). ⚠ENV TRAPS: PLASMC_KAPPA_MAX_Z (NOT KAPPA_MAX_Z — ignored all session, cap ran at 3.0); single-axis PLASMC_GAMMA/XI2 bypasses VDF auto-align→X/Y revert hot. Baseline (h_rd=-0.42,VDF,K_R=2.5,GT-FB)=3/5 sub. Full state/dead-ends/next/infra in file. Continues [[project_gt_feedback_control_tuning]].
- [⭐ NEXT TASK — GT-feedback control tuning (2026-06-23, fresh chat)](project_gt_feedback_control_tuning.md) — feed V-frame GT s (centroid bearing) + h (flow) to the controller (bypass perception), tune gains for s_e_n→0 AND h_e→0 on CLEAN signals → isolates CONTROL from PERCEPTION; then re-introduce perception. Established this session: NEITHER converges (s_e_n plateaus ~0.4=Task1 fail; h_e bounded ~0.4 + grows terminally b/c terminal flow SATURATES — all estimators 5-10x under deck loom, SCALE-CHANGE limit, LK params inert). Compute GT s/h via tools/gt_optical_flow.py, inject behind PLASMC_GT_FEEDBACK in img_data/controller, GT poses from gz_subscriber. s/h are image-space → control law stays scale-free (only sensor idealized). First steps + files/frame-gotchas in file.
- [Single-marker: rank-deficiency was BOARD-specific — single-LARGE-marker world WORKS (2026-06-23)](feedback_single_marker_rank_deficiency.md) — ⭐⭐ RESOLVED: on the nested board PLASMC_SINGLE_MARKER LOST to momentsz (lateral rank-deficiency, R² 0.07) — but that was the board's SMALL markers. Switched world to a single ~1m ArUco (arucotag.png id0, model.sdf .bak_board) + RE-CAL in-world (R² Hx/Hy 0.63/0.62 = LARGE marker well-conditioned; cond ∝ angular extent s_phys/Z). n=5 MATCHED cal: 2/5 sub-meter (0.34,0.38m), escapes the dead-end (0/5), PARITY w/ momentsz. Cal now single-marker-specific (board cal .pre_singlemarker_cal_bak). Remaining = SHARED terminal ceiling (vz launches 4/5 + 1/5 stochastic 19m), NOT single-marker. Whole earlier thread (was "FULLY CLOSED") (single 1/5 vert 0/5 sub vs momentsz 0/5 vert 2/5 sub). Single near-axial marker cond 15-60 (p90 270-471) vs multi ~11 (spread-set ~s_phys/Z). MOMENT loom (FLOW_LOOM_DECOUPLE, −½d(lnM)/dt = direct scalar, NO pinv) sidesteps rank-deficiency for LOOM (continuous |Δh_z|~0) but lateral h_x,h_y still pinv-rank-deficient → SCALE BIAS (not jitter: single lat_noise 0.126<momentsz 0.212) → drift → 1/Z → vz launch. Board nested b/c no single marker fills FoV across 10m→0.4m (~4.5 octaves). FLOW_LOOM_DECOUPLE = bakeable vertical fix (vert 5/15→1/15). FULLY CLOSED via 4 angles: GFT NEGATIVE (point-starvation, lat_noise 0.126→0.173, reverted); rcond=3e-2 (lat_noise 0.232→0.051 but |h_lat| 0.437→0.179 attenuation, still loses); GT-flow split = h_y R² 0.74→0.07 CORRELATION COLLAPSE (cal-/frame-independent, survives yaw-invariant |h_lat| R² 0.34 vs board 0.76) → re-cal won't rescue. Side: global ~−14° V-frame yaw offset on ALL arms. OPEN: single-LARGE-marker WORLD (SDF) untested.
- [V-frame flow = DE-ROTATED VIRTUAL POINTS; pure_div fixed to the virtual plane; V-frame ring correct but closed-loop NEGATIVE (2026-06-22)](feedback_virtual_plane_loom_ring.md) — MATLAB computes ALL flow on de-rotated virtual points (no LK/rings/warp); PX4 corner flow ports it faithfully (corr 0.87 = the GT-accurate consumed signal). pure_div (ring Singhal loom) was computed in the REAL image plane (GT corr ~0) → fixed to VIRTUAL (corr 0.65, BAKED default-on, ~inert: EKF fallback ~1% duty, 0% terminal). PLASMC_RING_VFRAME (default-off) makes the consumed ring loom GT-accurate 0.58→0.84 but closed-loop n=15 NEGATIVE (vertical launches 5/15 unchanged, terminal vz worse) → rings = a down-weighted safety net done correctly, NOT the loom fix; the vertical launch is κ-amplified terminal perception, not loom accuracy.
- [Lateral wall RESOLVED = gain-parity bug, combined surface WORKS (2026-06-21)](../project_current_state.md) — ⭐⭐ the PX4 lateral wall was NOT a fundamental perception/terminal-1/Z limit — it was the combined-barrier mode running the OLD back-mapped soft-config gains (GAMMA 2.0/KAPPA0 0.5/XI2 0.6, 3-5× too hot) → a_u over-aggression → fly-aways. FIX = auto-align combined gains to MATLAB VDF-ASMC vdf_params (22cc732). BAKED DEFAULT-ON: PLASMC_COMBINED_BARRIER=1 + VDS/DHD/DW-KF + chi_r=0.5 + h_rd=-0.42 + PLASMC_YAW_ALPHA_FILT=1. IC2 n=5: 4/5 sub-meter, s_e_n→0, 1/5 terminal-1/Z. OPEN: final IC2-5 gate. Corrects [[feedback_combined_surface_divergence]] + [[feedback_lateral_wall_anti_restoring_au]] (both had wrong "perception-gated" / "terminal-1/Z fundamental" verdicts). Records NC100-106.

- [Test-record system + all-runs SP scan (2026-06-23)](reference_test_record_system.md) — ⭐ tools/build_test_record.py scans test_data/ (276 cfg/3137 reps) → test_record_runs.json + test_record.tsv (GREP THIS, not the ods) + docs/TEST_RECORD.md; parameter_record.ods restructured to 7 sheets (NewCal_Notes/All_Test_Runs/Genuine_SP_Reps added; NC R0 log-blob rescued). KEY: 18 "full SP" but 2 frozen-GT false + many genuine ones are b9*/b10B open-loop-HANDOFF INVALID → only trustworthy closed-loop SP = 2 CoordDescent + a few SPCampaign b13/b14/b2A. restructure_*.py is ONE-SHOT; refresh_scan_sheets.py idempotent (rebuilds both auto sheets).
- [KAPPA_MAX_Z=0.03 prevents Z bounce — GT-FEEDBACK ONLY, NOT production (2026-06-24 CLOSED)](feedback_gtfb_kappa_z_bounce.md) — ⭐ G_z=99.69 (XI2_Z=1.0) → switching=367m/s² → bounce. KAPPA_MAX_Z=0.03 fixes GT-FB. DO NOT bake: production NC150-152 showed 3/3 IC1 fly-aways (weakens Z noise rejection). Production κ_z stays 0.03-0.05 naturally via leakage.
- [GT-feedback lateral orbit ceiling: xy≈0.2m, rel_vel≈0.37m/s — LAG-LIMITED (2026-06-24 CLOSED)](feedback_gtfb_lateral_orbit_divergence.md) — ⭐ κ=0.03 near-marginal: n=3 median xy=0.202m, rel_vel=0.370m/s (orbit sometimes soft/usually not). κ=0.01 regressed (barrier breach). κ=0.05 inert. 38ms lag ceiling, not gain-tunable. Production: only KAPPA_MAX_Z applies (GT-FB-specific).
- [⭐⭐⭐ ROOT CAUSE FOUND — GT-FB w_z SIGN BUG drove the lateral divergence (2026-06-25, user-led)](feedback_gtfb_wz_sign_bug.md) — ⭐⭐⭐ gt_feedback.py fed w_z=+alpha_dot but true rotational flow w_z=-alpha_dot=-psi_dot_b (user's lead "alpha_dot=-w_z"; VALIDATED perception lstsq w_z corr -0.91 vs IMU body yaw rate). Wrong sign flipped h_d's cross(w,s) rotation-FF + old c-term's omega_dot×s/2w×h → spurious ANTI-RESTORING feedforward → drone flies OUT at altitude (IC4 offset 2.5→6.4m). FIX w[2]=-_slope: IC4 converges MONOTONE 2.5→0.7→0.2→0, lands 0.21 (worst 8m fly FIXED). GT-FB ONLY (perception already correct → production unaffected). Explains why ALL session caps failed (bounded OUTPUT of a sign-flipped FF). Mode-1 altitude-divergence FIXED; mode-2 terminal deck-launch still present (isolated). Corrects "c-term mis-derived/use clean form" — old c-term FINE, omega INPUT was flipped. Confirm n=3 then bake.
- [⭐⭐ TERMINAL FLY-AWAY = positive-feedback LOOP THROUGH THE FLOW (2026-06-25)](feedback_terminal_launch_flow_loop.md) — ⭐⭐ motion→big flow h (h_lat 11→133)→c-term FF explodes→a_u_xy (100s-1000s)→max tilt→thrust law saturates (full throttle)→LAUNCH 3-8m→more motion→runaway. z stays bounded (per-axis θ); launch is LATERAL→TILT→THRUST-SAT coupling (command-decoupling ≠ actuator-decoupling). c-term is the UNGATED amplifier (switching tamed to ±4 by κ-cap+per-axis θ → c-term=94% of a_u_xy; ω̇×s dominant, |ω̇×s| 0 mid-descent but 40-266 terminal even on clean reps). SINGLE-SUB-TERM CAP = WHACK-A-MOLE: PLASMC_CTERM_DWS_MAX=10 → a_u_xy still 449 (shifted to loom×flow=−(h·ê3)h=234; all sub-terms fed by same flow h); retry 8/10 sub BUT 3/10 still launched 5-7m. DEAD-END knob (default-off). FIX = break loop at OUTPUT (COMMIT_AU_MAX caps a_u_xy<thrust-sat) or SOURCE (tight s_e_n convergence→small terminal flow), NOT the middle. h_rd CONSTANT.
- [⭐⭐ PER-AXIS THETA = first PROVEN terminal fix (2026-06-25): 9/10 sub, 0 fly, 0 stall](feedback_theta_per_axis_decoupling.md) — ⭐⭐ replaces shared scalar ‖θ‖_F with per-axis θ_k=‖row_k(θ)‖=sqrt(v_k²+1) in switching term + κ-ODE (PLASMC_THETA_PER_AXIS, default-off, controller.py 5 edits). LYAPUNOV PROOF (Drafts/PER_AXIS_THETA_PROOF.md): UUB PRESERVED (same V/φ₁/ϑ) — disturbance enters σ̇ row-wise |(θd̄)_k|≤θ_k·d̃ so θ_k is the TIGHT bound, ‖θ‖_F=conservative special case; cross-cancellation holds per-axis. EMPIRICAL (GT-FB IC1-5, bundle 022346): terminal |a_u_z| stays 3.6-7.8 even when |a_u_xy| explodes to 2995 (vs shared-θ FLY where lateral spike detonated z to 92-320 collateral) → z DECOUPLED → no balloon → lands before lateral spike throws it → 0 fly. Lateral a_u_xy STILL spikes (lateral runaway NOT cured, isolated remaining root). First PROVEN non-band-aid fix; confirm n≥3 then bake. Implements [[feedback_terminal_root_lateral_zeta_r]]
- [⭐⭐ TERMINAL ROOT RE-FRAMED = LATERAL zeta_r drives shared theta, NOT altitude (2026-06-25 axis-by-axis dig)](feedback_terminal_root_lateral_zeta_r.md) — ⭐⭐ GT-FB Control_Data dig (yaw->alt->lateral): YAW exonerated (e_a +-8deg, no cycle). ALTITUDE = universal SYMPTOM (terminal loom spike h_z->-1.8 at <12cm is INHERENT to h_rd*Z, present even in CLEAN reps; z over-brake is COLLATERAL). LATERAL = ORIGINATOR: shared theta tracks the lateral POSITION barrier zeta_r (at theta peak zeta_r=5.2 vs loom zz=0.7 on worst flys); s_e_n=lat/Z 1/Z-amplifies + descent positive-feedback (s_e_n_dot⊃+|h_rd|s_e_n) -> near deck can't null -> zeta_r->p_r floor -> theta DETONATES all axes -> fly. CLEAN survives = stays centered til last 12cm; FLY = enters terminal with offset (IC1 early drift, IC3 kick at 0.8m). CORRECTS [[project_descent_terminal_failure]] (causality was BACKWARDS — not loom->kz->balloon->lateral). Fix = lateral zeta_r/s_e_n, h_rd CONSTANT; kappa caps are downstream band-aids
- [κ-clamp is a BAND-AID; tune control so it rarely triggers — + gate-wide proof KAPPA_MAX=0.03 clears the fly-aways (2026-06-25)](feedback_kappa_clamp_bandaid.md) — ⭐ USER GUIDANCE: KAPPA_MAX caps bound the switching-term OUTPUT (no PX4 sat) but don't stop it WANTING to over-command → use as diagnostic/scaffold, NOT final fix; goal = κ stays natural (~0.03) + sat()<1 through the terminal so the clamp is inert. EMPIRICAL: GT-FB IC1-5 gate KAPPA_MAX=(.03,.03,.03)+XI2_Z=1.0+VDF+K_R=2.5 = FIRST gate-wide clearing of IC1/IC4 terminal fly-aways (max xy 1.04m vs baseline 7.8/11.7m; 8/10 sub-meter, 4 sub-0.25) BUT cap is binding (not backstop) → NEW failure no-descent hover-stall (κ_z 0.03 starves descent-init from κ0_z=0.25) + soft still unmet (lag). Follow-up KAPPA_MAX_Z→0.08. Real levers = E/XI2/P2INF/N/P/θ-blowup/loom-governor. [[feedback_clamps_during_tuning]]
- [IC3-5 failure root = TERMINAL LOOM SIGN-FLIP (perception, not control); IC_YAW_TARGET=alpha DEAD-END (NC153-160)](feedback_ic_yaw_target_fix.md) — ⭐⭐ IC3 real root (traced Control+Img_Data): descent rates IDENTICAL to IC2 ("70% faster" RETRACTED, time-align artifact); IC3 dies in last ~0.4m: at Z≈0.42m the 1m marker OVERFLOWS FoV (corners off-frame, centroid still centered |s|=0.19) → aruco decode lost (Nflow 89→0) → fusion EKF (FLOW_FUSE_RING=1) falls back to RING loom → ring swings + near deck → fused loom FLIPS −0.31→+0.99→+1.83 → "marker receding" → upward-thrust balloon + s_e_n breach → fly-away. PERCEPTION failure, not gain/anti-restoring. Fix lever (later): clamp h_z≤0 / gate ring-loom when aruco N=0. Alpha fix NEUTRAL IC3, CATASTROPHIC IC4 (90° alias at 7m). GT-FB IC2-5 gate launched to tune control sans glitch.
- [⭐⭐ THE fly-away CAUSE via controller objectives: Task-1 (s_e_n) SUCCEEDS, Task-2 (h_e) FAILS at deck (2026-06-25)](feedback_sp_task2_terminal_limit_cycle.md) — ⭐⭐ s_e_n converges+bounded to the deck (position control WORKS); h_e_lat spikes 0→3.27 FIRST (before s_e_n, before launch) b/c h=v_rel/Z diverges (v_rel doesn't reach 0, 38ms lag breaks the soft-touchdown Cor.1) → unbounded h_e → GROWING LIMIT CYCLE (bounce alt 0.1↔9.8m). Clean vs fly = RACE (touchdown vs h_e spike). ONE mode (mode-1 altitude-divergence was a binning artifact). Supersedes c-term-as-cause. Lever = soft condition (arrest v_rel early), h_rd CONSTANT.
- [⭐⭐ TERMINAL = SMC convergence failure at an ACTUATOR-BOUNDED wall + loom cycle = boundary-layer saturation (2026-06-25)](feedback_terminal_smc_actuator_wall.md) — ⭐⭐ sigma breaches E b/c deliverable authority = g·tan(60°)=17 m/s² (a_u_xy commands 703, 2% deliverable) vs UNBOUNDED 1/Z disturbance → NOT tunable (more κ/Γ/θ_cap fail/backfire). Lateral θ explodes 4→859 (flow h_xy=v_lat/Z 1/Z; c-term loom×flow SCALES it ×bounded-loom, NOT 1/Z²) → tilt 46° → steals vertical thrust → loom h_e_z breach (s_e_n & h_e_z coupled via TILT). Loom cycle = sat(σ_z/E_z) saturation; E_z=3 KILLS it (E_z=0.5 baked TOO TIGHT; σ_z=3.66 = barrier ceiling at S_MARGIN clamp; E_z=4/P2INF_Z=2 closes residual). Fix = arrest v_lat EARLY, not the SMC reaching.
- [⭐ KAPPA frozen on lateral (N_xy too slow, τ=33s) + h_e_xy parameter map + velocity-damping lever (2026-06-25)](feedback_kappa_4axis_hexy_param_map.md) — ⭐ κ-ODE τ=1/(N·P): lateral N=0.02→τ=33s≫7s descent → κ_xy FROZEN ~0.12 (no adaptive robustness; z N=0.1 τ=2s + yaw n_a=1 τ=0.5s ADAPT). FUNNEL=performance (bounds h_e_xy), SMC=convergence (drives it) — distinct. Flow funnel p2_xy too loose (9-35) → ζ_h only 7% of σ_xy → velocity UNDAMPED → terminal h_xy explosion. Full param map (P2INF/XI2/χ_r/Ω/Γ/E/κ/N/P roles). Lever = tighten flow funnel (engage ζ_h) + raise N_xy (safe on clean reduced-solve h_xy) to wake κ_xy. → BOTH DONE: N_xy=0.1 BAKED (826e655, τ 33s→7s, κ_xy adapts std 0.23); the velocity-damping (flow funnel) lever CONFIRMED in [[feedback_flow_funnel_zetah_works]]. ⚠ N_xy=0.1 alone REGRESSES (woken κ amplifies the terminal: κ chases κ_eq=θG|σ|/P, θ∝h_xy∝1/Z, explodes 0.42→13.5; frozen N=0.02 was a SAFEGUARD); it's safe ONLY with the velocity damping (which keeps h_xy/κ_eq from exploding).
- [⭐ Reduced-solve lateral flow h_xy (FLOW_LAT_REDUCED=1 baked) — drop w_xy cols, σ_min→σ_max, 206× cleaner (2026-06-25)](feedback_lateral_flow_reduced_solve.md) — ⭐ h_xy is the σ_min mode of the full 8×6 (degenerate w/ tilt w_xy, 0.4° principal angle) → noise-amplified (corr-GT 0.1-0.66). FIX: drop w_xy cols (V-frame leveled → level-target w_xy≈0) → h_xy largest-σ (cond 14→2). Offline corr 0.2-0.3→0.5-0.65. BAKED FLOW_LAT_REDUCED=1 + FLOW_TARGET_LEVEL=1 (=0 full-solve fallback for tilting deck). ⚠ REQUIRES paired RECAL: cal h_x/h_y rows are a w_xy RECOMBINATION → break under reduced solve (~3× under-read) → re-run output-cal w/ FLOW_LAT_REDUCED=1 → diagonal rows. GT-FB unaffected; production regresses until recal.
- [✅ DONE — obsolete test-data cleanup, BOTH phases (2026-06-26)](project_obsolete_cleanup.md) — (A) combined-barrier gain-parity-bug era (pre-22cc732) + (B) NC-falsified. Phase-1: 101 dirs/2.46GB deleted (0dbbd51); Phase-2: 843 pre-cutoff Landing_Test reps (a428b9d+abb74a6, 6.3→2.2GB). 3.27GB archived at ~/spl_obsolete_archive/ (OUTSIDE repo — the only recovery copy); genuine SP unchanged. ⚠ index said "IN-PROGRESS/NOTHING deleted" until 2026-07-02 (caused a false re-run request); 07-02 re-check: both manifests clean vs disk (843 DELETE gone/381 KEEP present), both archives root-count verified, and BOTH executors now refuse an empty-set --execute (guard added). Plan/history: docs/OBSOLETE_CLEANUP_HANDOFF.md

- [⭐⭐ FINALIZED terminal-kick DESIGN (approach+commit) — 2026-06-28](project_terminal_kick_commit_design.md) — full spec in docs/TERMINAL_KICK_COMMIT_DESIGN.md. ROOT = residual lateral VELOCITY v_res, COMMANDED (h_d=measured ṡ copies flow corr 0.90 → h_e≈0 → never says "stop"), 1/Z-normalized → s_e_n breach → ζ_r→∞ → kick. Position does NOT discriminate (precise |s_e_n| 0.163 vs failed 0.159); velocity does (|ds_e_n| 0.061 vs 0.252). FIX (2 parts): (1) APPROACH HD_FUNNEL_REF=1+HD_KR=k_r (pure funnel-ref INERT=prescribes S_r const, drops p_r·Ṡ_r; k_r restores it; ⚠ HD_KR needs FUNNEL_REF=1 or silently ignored) + P2INF_xy~0.12 + maybe lower χ_r; (2) COMMIT at MARKER_EXTENT≥400px (≈Z0.5m, start-height-INDEP) = zero ζ_s + ring handoff + 1-way latch. Discriminator: |s_e_n| small AND h_z<0 AND |ds_e_n| small (settled, held N) → case(b) commit; off-center/ds_e_n>0 → case(a) abort+re-ascend. Dropped ||h_xy|| for |ds_e_n| (centroid-robust, catches zero-crossing). ⚠ "h_e_xy=rotation-compensated velocity" FALSIFIED (rot FF only 19% of h); p_2inf bounds h_e_xy, works at center by h_d→0 not rotation removal. h_rd FIXED. DEFERRED: k_r/P2INF sweeps, case(a) module, ring-handoff test. Builds on [[feedback_terminal_kick_commit_vs_live]].
- [⭐⭐ TERMINAL-KICK: loom-commit (stationary) vs funnel-ref/k_r (moving/rover) — 2026-06-27](feedback_terminal_kick_commit_vs_live.md) — the lateral 1/Z kick = position barrier ζ_r blowing up at the funnel edge → infeasible a_u → max tilt → launch. σ already ensures ζ_r→0 (χ_r is the dial; explicit k_r is redundant back-mapping that re-adds G_s⁻¹ starvation + dh_d blow-up). REFRAME: open-loop loom-commit (level+const-thrust freeze-and-fall) is STATIONARY-only (8/25); for the MOVING rover it lands where the target WAS → must track live to touchdown → the funnel-ref+k_r depth-free terminal recovery (k_r=0.3 fixes IC4 gate-off 1.18→0.248 via vlat-IN drift-recovery) ARE the rover solution; gate-OFF = the moving regime. ⛔ funnel-ref BAKED default-on REGRESSES stationary gate (8/25→1/25); un-bake for stationary, keep env-gated for rover. h_rd FIXED.
- [⭐ HD_FUNNEL_REF baked ON but ζ_h STILL degenerate — mechanism OPEN (2026-06-30)](feedback_hd_funnel_ref_zeta_h_degenerate.md) — ⭐ VALIDATED: HD_FUNNEL_REF=1 baked (controller.py:888; the "default-off/moving-target-candidate" framing is STALE). h_d's rate = POSITION funnel-ref (S_r·dp_r−hd_kr·ζ_r/g_r), NOT measured s_dot → "h_d uses measured rate" REFUTED. BUT empirically h_e=−0.4519 EXACTLY const (IC4r2) → ζ_h degenerate → funnel-ref NOT un-degenerating ζ_h as claimed → surface has NO velocity damping → terminal cycle grows undamped in ζ_r. WHY const-despite-funnel-ref = OPEN (need h_d_noS/_hd_rate/s_dot_meas logging). Also corrects: κ "frozen by containment" wrong (0.30<1, κ_y/z adapt); releasing κ won't aid recovery (N=1.0 runaway).
- [⛔ IDEA 1 soft funnel-breach = DEAD-END at every frac (2026-07-01 frac sweep)](project_soft_breach_idea1.md) — ⛔ PLASMC_SOFT_BREACH frac sweep (XIR=0.10 base, GT-FB, IC4/IC5 n=5): NO frac reaches OFF (8/10) — frac 0.05/0.15/0.30/0.50 → 4/9·7/10·7/10·6/10, and κ_xy RISES as frac SHRINKS (OFF 0.62→0.05 1.07) → the "smaller frac bounds κ" hypothesis REFUTED. ROOT: the terminal σ-breach is the FLOW barrier ζ_h pinning (NOT position; S_r≈0 converged) because under HD_FUNNEL_REF h_d→0 once s_e_n converges so h_e≈h=v_lat/Z = raw 1/Z residual velocity = GENUINE+SUSTAINED. Soft-breach's flow arm shares the SAME if/else gate as the protective κ-freeze (contained[idx], controller.py ~1451→~1648) → enabling it removes the freeze → κ ratchets on the persistent breach. Symptom-side handling can't fix an un-arrested-velocity (cause-side) problem; only live lever = P2INF_xy/χ_r velocity damping or the 38ms lag. Keep default-off. Also: W_U_MAX=2.0 INERT (0% duty), P2INF_z reduction DEAD (1.5=19>0.5=17>1.0=12; vz lag-set). [[project_why_sp_achieved]]
- [⭐⭐ HANDOFF: terminal softness = 1/Z-gain × 38ms-lag LIMIT CYCLE; soft-breach REGRESSES (2026-06-30)](project_handoff_terminal_oscillation.md) — ⭐⭐ NEXT-CHAT START. Baked {h_rd=-0.30, XIR=0.15, P2INF_xy=1.0} committed 486f713 same day; XIR→0.10 reverted f068774 07-01 (P2INF=1.0 A/B winner 7/9). CONTAINMENT/SOFT-BREACH (Idea 1) = confirmed NET REGRESSION (7/10→4/10): position-containment INERT (wide funnel never breaches 0.95), flow branch un-freezes the PROTECTIVE κ→runaway; default-off, revert-or-leave. Terminal softness wall = lateral v_lat RING (1/Z loop gain × 38ms lag → phase margin→0 near deck → limit cycle intensifying as Z→0; phase-gated touchdown). LEVER=the 38ms lag (gain knobs can't move phase margin). c-term −h_z·h_xy = 1/Z² PHANTOM (v_z·v_lat/Z², tiny real v) → poisons θ → κ can't handle (feedforward not switching, un-deliverable); fix=cterm cap at source. Two walls: IC4 softness (ring), IC5 precision (short runway). Diagnostic logging kept.
- [⭐ INVARIANT: the moment loom must BYPASS `_sensor_cal_hw` row 2 at EVERY emitting site (2026-07-17, fixed 00ba40d)](feedback_moment_loom_cal_bypass_invariant.md) — ⭐ with FLOW_LOOM_DECOUPLE=1 (baked) `h_z` is ALREADY the calibrated scale-free vz/Z; cal row 2 (raw-pinv→cal map) over-scales it ~7% AND re-injects the lateral/rotational coupling the decoupling removed. ASYMMETRY found+fixed: the observer branch (LK failed, marker decoded, img_data.py ~:2579) applied cal row 2 while the LK-succeeded branch (~:2450) bypassed it — same quantity (both build row 2 as `_hz=_loom_dec` ~:2044), two treatments → loom scale+coupling silently changed with WHETHER LK SURVIVED the frame, and it reached CONTROL (that branch feeds the fusion EKF, default-on) on every LK-dropout-at-altitude. Hides because the cal multiply reads locally-correct at each site and the BYPASS is the surprising line → any NEW h_z producer reintroduces it. Correct sites (all bypass): ~:2457, ~:2592, getter :3765/:3769; coast is safe VIA the getter (don't pre-calibrate it). NOT flight-validated (code-reading fix) → judge on h_z continuity ACROSS LK-dropout frames, not SP count (n=5 underpowered vs ±5-7 noise floor). Context: this was the ONLY real defect from a full s/alpha/h cal audit — `cal_s[3]=1.0` (alpha) is LOAD-BEARING not a placeholder (see [[project_yaw_calibration_pending]] ✅RESOLVED), and planar_map_rescue's error is homography DRIFT not gain (unfittable by β=σ_GT/σ_raw; the plausibility gate is what protects it).

## Legacy flat-file index (pre-2026-06-19 topic files in `Memory/` root — migrated 2026-07-02 from the legacy MEMORY.md)

> Hooks below are verbatim from the legacy index; files live in `Memory/` (NOT this folder).
> ⛔ Dropped as superseded/contaminated-era (files remain on disk with their own stamps):
> feedback_convergence_ordering, feedback_coord_descent_sp_lucky_ic, feedback_descent_softness, feedback_dh_d_overload_lpf, feedback_dterm_outer_funnel_analysis, feedback_fov_cone_clamp_deadlock, feedback_ic2_lateral_gain_chain, feedback_instability_mechanism, feedback_krp_pz_ic2to5_regression, feedback_lateral_wall_anti_restoring_au, feedback_newcal_tuning_results, feedback_optic_flow_underreports_root, feedback_phase1_matlab_baseline, feedback_phase3_ic_robustness, feedback_phase4_sensor_noise, feedback_plasmc_two_task_framework, feedback_sen_authority_analysis, project_current_state, project_stacked_barrier_backstepping.

- [MATLAB pinv tol=0.01 loom finding: mechanism transfers, number doesn't (2026-06-19)](../feedback_pinv_tol_loom_scaling.md) — ⭐ literal tol=0.01 INERT in PX4 (σmin~0.077>0.01; numpy rcond RELATIVE, MATLAB pinv tol ABSOLUTE). VERIFIED on a CENTERED descent (validation_data/loom_descent): regularizing the rank-deficient loom dir DOES help — close-range loom RMSE 0.78→0.42 at lstsq rcond=3e-2 — but it's a SPIKE/MAGNITUDE tradeoff (kills phantom-loom spikes, attenuates magnitude ratio 0.90→0.66, corr flat). PX4 lever = FLOW_LSTSQ_RCOND (default 1e-3; added to img_data.py). NOT baked — SITL-validate
- [Pyramidal LK is INERT — availability not dynamic range (2026-06-19)](../feedback_pyramidal_lk_inert.md) — ⭐ NEGATIVE: maxLevel 3/4/5 + winSize 21-51 change corner flow by ~0 in every GT-speed bin (n=2 fly-aways); when LK tracks, raw lateral flow is PROPORTIONAL to GT (~1.0-1.3) to 3 m/s. The combined-barrier 0.5× under-report is an AVAILABILITY artifact (only 46-63% frames yield flow; controller holds stale flow on dropouts; track-rate 90%@5m→22-41%@1.7m = close-range decode breakdown). Pyramidal LK NOT the lever; real lever = decode/track robustness. Tool tools/tune_lk_dynamic_range.py
- [Combined surface WORKS in PX4 (2026-06-21; "perception-gated" verdict was wrong)](../feedback_combined_surface_divergence.md) — ✅ the combined sliding surface RESOLVES the PX4 lateral wall; the 2026-06-19 "perception-gated, does NOT beat the wall" verdict was a MISDIAGNOSIS (the real cause was over-hot back-mapped gains in combined mode; VDF-ASMC gains + chi_r=0.5 → IC2 4/5 sub-meter). NOW BAKED DEFAULT-ON. The 2026-06-18 CONVERGENCE insight (combined surface = unified fix for PX4 wall + MATLAB option-b) was correct. [history] the 2026-06-18 MATLAB push d69ed78 (c̃_h option-(b) BLOCKED at IC5: corrected feedforward "lacks the closing authority the funnel needs", s_e_n breaches p_s 46%/→6×) is the SAME old-form root as the PX4 lateral wall — BOTH codes run the old back-mapped form (SEN_FUNNEL→ds_d→h_d; σ=ζ_h+Ω∫ζ_h) while the manuscript+[[project_stacked_barrier_backstepping]] already specify the combined surface. So the already-designed combined surface resolves BOTH (unblocks option-b + closes the wall + aligns code↔paper). Holds the PRECISE PX4 controller.py code-level map (purely-lateral change; ζ_r=_zeta_s[-1], ζ̇_r=smooth4(_dzeta_s_deque); zeta_aug/chi_dzeta_aug 3-vecs; drop ds_d from h_d; χ_r>0). ⚠ MATLAB code OFF-LIMITS (Windows). Impl moved to a fresh chat (NOT done here)
- [Sliding surface CAN'T stack position+flow barriers (rel-degree)](../feedback_sliding_surface_relative_degree.md) — ⭐ REJECTED σ=ζ_h+λζ_s: position s is rel-deg-2 from a_d, flow h is rel-deg-1 → ζ_s rel-deg-2 (ζ̇_s has no a_d) can't sit in a first-order surface; AND ζ_h=barrier(h_e)≠ζ̇_s. The existing CASCADE (position funnel→desired flow h_d→flow ASMC) IS the correct backstepping for the rel-deg-2 position. Valid combined surface needs ζ_s AND its rate ζ̇_s, not ζ_h
- [CBF cbf2 implementation (2026-06-14)](../feedback_cbf_lw_rotation_bug.md) — ⭐ current CBF approach (port/reuse this): pure src/cbf_visibility.py::cbf2_filter — camera-plane QP over body tilt. L_eff=L_w@[[0,1],[-1,0]] (lean→rotation coupling), θ_d=Rz(−ψ)a_xy/a_z, alternating-projection QP onto L_eff rows, post-QP θ_cap, two-phase δ; apply θ*→rd3=[−Rz(ψ)θ*,1] DIRECTLY as desired attitude (thrust from loom). Env CBF_LW_ROT/CBF_RD3_DIRECT default-ON. Validated: offline tools/validate_cbf.py 13/13 + SITL tools/analyze_cbf_visibility.py (judge by VISIBILITY not fly-away — fly-away is control, CBF can't fix). Demo notebooks/cbf_validation.ipynb; port map docs/CBF_SEN_MATLAB_PORT.md
- [Descent perception is at its ceiling (2026-06-13)](../feedback_descent_perception_ceiling.md) — ⭐ NEGATIVE result: tuning LK (corner+ring win/lvl), KLT, ArUco params (incl maxMarkerPerimeterRate), deblur/sharpen do NOT improve corner survival or decode. On a CLEAN centered descent decode 88-100% + corners to deck + marker never fills FoV + frames sharp (Gazebo = no motion blur). Failures are FLY-AWAY/GEOMETRY induced (lateral wall), not params. Loom ACCURACY = calibration: _sensor_cal_hw[2,2] 1.0744→1.2988 (validated cal/GT 0.89→1.07, corr 0.993; LK-dyn-range nonlinearity caveat). Clean-descent recorder = record_output_validation landing + IMG_RECORD_RAW. Tools: tune_lk_survival.py, tune_aruco_decode.py
- [Tuning trajectory — full campaign timeline](../reference_tuning_trajectory.md) — chronological arc of every trial (G#/NC#): what varied, why, outcome; cal-regime epochs; pairs with tune-plasmc skill. (Timeline ends ~NC49; later eras: px4/MEMORY.md top banners — project_current_state is historical)
- [PX4_Gazebo/docs/ design notes](../reference_docs_folder.md) — ⭐ PLASMC_TUNING_GUIDE.md is the tuning entry point (auto-injected via SessionStart hook); + cbf2/parity/perception/analysis/sh-patterns docs; indexed in CLAUDE.md
- [SITL calibration lessons](../feedback_calibration_lessons.md) — failsafe handling, run-until-5-valid, offline-vs-runtime savgol trade-off
- [SO(3) body ω: quaternion-diff](../feedback_so3_quaternion_omega.md) — NEVER np.gradient(W_R_B)+skew (over-reports ω_z 2.27×); drop-in code in file
- [Image-center bug was plotter cell-38 only](../feedback_image_center_bug.md) — (320,240)→(240,320); img_data.py center was ALWAYS correct — don't "fix" it
- [V-frame g-sign bug in _getVirtualPts (FIXED)](../feedback_getvirtualpts_g_sign.md) — g=R@[0,0,1] should be R.T@…; amplifies tilt-induced spurious s
- [Applied sensor_cal provenance](../reference_cal_data_provenance.md) — from calibration_data/output/; refreshed 2026-06-07 to all-13 corner + transfer ring; KEEP output/
- [V-frame flow-check RHS yaw-only + V-frame model & naming (consolidated 2026-06-18)](../feedback_vframe_rhs_yaw_only.md) — zero V_w_tug[:,:2]; image stamp is correct fps (fps-fix was a red herring). NOW INCLUDES (folded from outputcal_flow_validation_vframe): frame model (camera=body-FRD; V=gravity-leveled rotz(yaw), R_V_from_body≠I under tilt) + B_/V_ naming convention (ed27641) + OPEN SEMANTIC FLAG (cal mixes V-for-h / body-for-w channels)
- [Ring depth-mixing rationale FALSIFIED](../feedback_ring_depth_mixing_falsified.md) — ring CAN be calibrated (board coplanar+V-leveling ⇒ uniform Z); loom value = robustness
- [Centroid filter default savgol→KF](../feedback_centroid_kf_default.md) — savgol(13) added 110ms lag + 2× noise; revert via IMG_FEATURE_FILTER=savgol
- [Validation apps run, amplitudes too hot](../project_validation_runs_status.md) — multisine drops marker / flips drone open-loop; tune VAL_MS_AMP_* down
- [No-descent NOT the EKF](../feedback_ekf_default_breaks_descent.md) — FLOW_FUSE_RING=1 exonerated (corner-only hovers identically); hover is a shared descent regressor
- [Ring-flow cal M_ring applied via transfer mode](../project_ring_flow_calibration.md) — vs calibrated corner; Wx/Wy=0, Wz coarse; ring_cal≈corner_cal
- [Aggregator must use -B_w_ug (=B_w_tug)](../feedback_aggregator_signed_btug.md) — for axes 3-5 (target-rel ω); signed Pearson scale (FIXED)
- [Compass-free yaw sign](../feedback_compass_free_yaw_sign.md) — BODY_YAW_SOURCE=alpha needs BODY_YAW_ALPHA_K=-0.949 (euler[2] anti-corr w/ alpha); first non-divergent compass-free landing
- [Image wx/wy ARE observable (OVERTURNED)](../feedback_wxy_unobservable_imu_fusion.md) — "unobservable" was an under-excitation+V-frame artifact; but uncalibrated → keep CTRL_ZERO_WXY for stationary
- [PX4/Gazebo key paths](../reference_px4_gazebo_paths.md) — PX4, mono_cam SDF, venv, ROS 2 bridges, cal scripts (in-repo paths predate the scripts/tools/apps reorg)
- [Impulse-response lag (clean)](../feedback_impulse_response.md) — pitch rate-loop 38ms (30 MAVSDK + 8 PX4); supersedes the old 168ms loop-latency figure; uXRCE-DDS is the lag lever
- [Marker detection breakdown = DECODE-fail, marker IN-FoV (2026-06-10 quantified)](../feedback_marker_detection_stale.md) — 9/12 TLs had 4/4 corners in-FoV (ArUco can't decode, NOT geometric loss); nan-quat = marker-LOST sentinel; TRIGGERS the κ-runaway; corners ARE stored → KLT corner-track + use genuine data on loss (don't nan/extrapolate); corners-based CBF
- [MC_*RATE_P runtime tuning DEAD](../feedback_mc_rate_p_dead.md) — breaks SITL preflight; leave 1.0; reduce lag via uXRCE-DDS or airframe edit
- [KLT marker-fallback](../feedback_klt_marker_fallback.md) — MARKER_KLT_MAX_STEPS=20 bridges ArUco failures; 4.5× lower zero-rate; default ON
- [V_YAW_SOURCE=alpha REMOVED](../feedback_v_yaw_source_alpha.md) — marker-aligning V zeros yaw feature → open-loop yaw; compass-freeness goes on the CONTROL side
- [Compass yaw drift under aggressive maneuvers](../feedback_compass_yaw_drift.md) — EKF yaw drifts 30-46° → BODY_YAW_SOURCE=alpha (default); =gt in analysis
- [GT vel/ω noise is bridge jitter](../feedback_gt_noise_uniform_dt.md) — uniform-dt interp + adaptive sgf (2Hz) before gradient; apply sgf AFTER interp
- [Img_Data vs Ground_Truth CLOCK SKEW](../feedback_imgdata_gt_clock_skew.md) — index-correlating is INVALID; use GT co-sampled features (alpha tracks yaw r=1.00)
- [GT optical-flow — CORRECT computation + tool](../reference_gt_optical_flow.md) — ⭐ ALWAYS use `tools/gt_optical_flow.py` (compute_gt_flow) for "check X vs GT"; ports the notebooks. 4 mistakes to NEVER repeat: fabricated linspace dt, raw-gradient on jittery GT, fractional-index alignment, wrong depth. Align via gt['Start Time']. Loom=vz/Z in V-frame
- [Frame conventions (PX4 NED/FRD vs Gazebo ENU/FLU)](../reference_frame_conventions.md) — conversion matrices, MAVSDK frames, quaternion delta-from-spawn
- [Input-cal yaw lag 5× roll/pitch](../feedback_input_cal_yaw_lag.md) — ωx/y 52-61ms r=0.92; ωz 287ms r=0.76; don't expect IBVS yaw bandwidth to match translation
- [Input-cal B_T is Newtons](../feedback_input_cal_thrust_units.md) — FC slope thrust_norm=0.738-B_T/42.3 (closed-loop gain →~1.00)
- [DDS lag-fix blocker — ⛔ CLOSED by user 2026-07-02, never re-propose](../feedback_dds_lag_fix_blocker.md) — uXRCE-DDS rate path built+impulse-validated but landing blocked on rclpy "context invalid"; user ruled the path OUT (lag/phase limits → operational spec instead)
- [Landing target design + best landing](../project_landing_target_design.md) — single-marker rank deficiency → multi-marker board → inner-cluster (0.675m best ever); single cal M can't span descent
- [Plotter cal un-baking is ill-posed](../feedback_plotter_cal_unbaking.md) — evaluate candidate cals via raw logs (cell 8), not by un-baking 6x6 from post-cal logs
- [IC1 hard-impact = ds_d touchdown spike](../feedback_dsd_touchdown_spike.md) — 1/Z spike, not perception; DH_D_MAX=5 eliminates it but LATER RESTORED to 50 as physics-guard (see trajectory); mechanism still valid
- [Test data at PX4_Gazebo/test_data/](../project_test_data_cleanup.md) — moved 2026-06-02; SP reps git-tracked; closed-sweep raw deleted
- [Yaw calibration pending](../project_yaw_calibration_pending.md) — ✅ RESOLVED 2026-07-02: cal_s[3]=1.0 is CORRECT (alpha tracks GT yaw r=1.00); moving-rover "yaw cal" dissolved — turning gap = controller alpha-rate cap, not cal (px4/feedback_rover_yaw_cal_resolved)
- [Moment-yaw is canonical](../feedback_moment_yaw_canonical.md) — yaw SMC tuned to moment alpha (π-period, [4,3,2,1], -0.9379); geometric swap regressed+reverted; don't swap
- [IC2-5 yaw runaway = COMPASS DRIFT at start](../feedback_yaw_compass_drift_ic_start.md) — EKF drifts ~77° at descent start, not alpha; fix the test rig (servo GT yaw), not the controller
- [θ_norm: contained downstream, NOT eliminated (2026-06-10)](../feedback_theta_norm_klt_drift.md) — ~99% cross(dw,s); 2 sources (off-screen-KLT [guarded] + DOMINANT frame-jump dw artifact); κ-cap+P-leakage contain it; dw source-fix REVERTED dead-end
- [Lateral κ-runaway = WRONG h_d from off-screen VIRTUAL centroid (2026-06-10, GT-verified)](../feedback_lateral_kappa_runaway.md) — controller uses VIRTUAL s (tilt-leveled), CBF uses ACTUAL; under big tilt _getVirtualPts z_v→0 divide reprojects an in-FoV feature OFF-SCREEN → cross(w_i,s) fabricates h_d≈-8 (ds_d≈0, h physical, matches GT) → breach → κ_xy 7.26; UNBOUNDABLE by gain (κ_xy uncapped); TRIGGERED by ArUco decode-loss. FIX: clamp PHANTOM s only (#1 marker-LOST extrapolation clip, PLASMC_FEAT_FOV_CLIP ON) — NOT the GENUINE in-FoV s (#2 global _getVirtualPts clamp PLASMC_VIRT_GUARD REGRESSED: clamped genuine s→under-correction→far drift 29/91m, OFF). Real lever = keep marker DECODED (KLT corner-track). Re-test FeatClip_EZ_IC1 pending
- [cbf2 QP design](../feedback_cbf_theta_cap.md) — theta_cap post-QP; two-phase δ (Phase 1 centroid-only, Phase 2 ramp on decode-fail). NB theta_cap later moved OUT of cbf2_filter to the caller (d3dfbaa)
- [Use GT yaw NOT e_a for yaw outcome](../feedback_use_gt_yaw_not_ea.md) — e_a misleads both ways (marker-fill corrupts alpha); tools/gt_yaw_analysis.py
- [Run SITL headless](../feedback_sitl_headless_run.md) — HEADLESS=1 + taskset 6-15 + retry wrapper; failure-modes table in file
- [SITL reliability fixes](../feedback_sitl_reliability_fixes.md) — lockstep race=CPU starvation (taskset -c 6-15 + renice); IC-yaw limit cycle → IC_YAW_SERVO_DMAX_DEG=0.3
- [Tuning restarts under 8-run cal](../project_tuning_campaign_newcal_reset.md) — refreshed all-13 2026-06-07; 3 cal regimes; run_knob_sweep.sh; never pass LANDING_OUT_BASE to run_ic_validation.sh
- [Target-visibility CBF design](../project_cbf_visibility_design.md) — L_omega camera-plane tilt-QP; FUNNEL_MODE=cone0|cbf1|cbf2; only bites with THETA_FLOOR<60
- [Flow-underreport brief GT-FALSIFIED](../feedback_flow_underreport_brief_falsified.md) — flow honest at altitude (ratio ~1); real gap = LK dynamic range + detection loss
- [Terminal touchdown blocked by close-range loom over-report → RING-LOOM FIX (2026-06-11)](../feedback_terminal_descent_loom_overreport.md) — descent tracks loom perfectly to ~1m; below ~0.5m corner loom over-reports ~4× → z-SMC over-brakes → BALLOON; ring divergence is the cleaner terminal loom. PROTOTYPE RING_LOOM_NCORN=12 (corner loom-row R→1e6 at low n_corn, ring carries vertical; corner keeps lateral): honest n=5 post-fix: mean 12.2, 2 TL LATERAL fly-aways (vertical-only fix can't touch them); 2 clean touchdowns (0.62@0.24, 1.44@0.44); vertical mechanism real, NOT a net win — not baked. NameError had masked the fly-aways by truncating flights. LESSON: check flight_s>0 + non-empty GT before trusting summary rows. Opposite of failed FLOW_NCORN_SWITCH
- [aggregate_calibration[_phased].py methodology](../reference_aggregate_calibration.md) — phased default; signal-floor per-axis CIs; signed cal reveals ω_x/y negative
- [⭐ Perception-ON baseline (2026-07-03): config transfers stably (0 fly/25) + sub-meter IC1-4, target_lost on ALL (mid-descent feature staleness) + IC5 never acquires (FoV-edge marker); binding gap = perception CONTINUITY not control](project_perception_on_baseline.md)
- [⭐ Nested marker switch (2026-07-03): single 1m -> nested 0-small_10-big.png 2m (big ID10 7-10m, small ID0 regenerated smaller ratio 10.63 -> detected to 0.08m for touchdown margin); IMG_MARKER_PRIORITY=big NEW default (biggest marker for observability); NEEDS output re-cal; concentric=no corner spread (board would fix flow rank-deficiency)](project_nested_marker_switch.md)
- [⭐ Centroid-rate observer: 4 validated fixes (2026-07-04)](feedback_centroid_rate_observer_fixes.md) — frame-pair (quats[1]→quats[0]) + lstsq consolidation + w_z sign (_oz=−_wv[2], fixes h_y anti-corr) + KF q 1e-4→1e-3; off-center velocity 0.85-0.94 (was dead/anti/0.3-0.6). Committed 3cc7b0b+ede3058
- [⭐ Deck fly-away/TL ROOT = terminal marker OVERFLOW not drift-out (2026-07-04, ⚠ NOT universal, see project_dense_recovery_and_failure_tagging)](feedback_terminal_overflow_deck_flyaway.md) — corners exceed frame while centroid centered (0.95>0.89 edge); loom held-stale under-reports descent 5× → blow through 0.2m + bounce; lateral blind (ring=~0 lateral); loom-inversion touchdown defeated by flicker. Recovery+descent WORK; failure is all at the deck. ⚠ 07-07: at least one verified fly-away was DRIFT-OFF at alt~2.5m, not overflow — check altitude at Ncorn->0 case-by-case, don't assume; use getFailureCause() to auto-tag.
- [V_ds KF q 1.0→10.0 re-bake (2026-07-04): q is a fly-away SEVERITY band-aid, not the root](feedback_vds_kf_q_severity_bandaid.md) — 06-30 terminal-osc rationale stale (PR0=10 handles 1/Z); q=10→7-31m deck fly-aways, q=1→~1m TL; restores off-center low-lag; ⚠ needs terminal-commit fix or env-revert to 1.0
- [⛔ DEAD-END (RECONFIRMED 07-07 w/ CLEAN signal): ring loom for h_z at terminal](feedback_ring_loom_hz_terminal_deadend.md) — 3 ring bugs found+fixed (arm-mask, station-pairing, fake-zero sentinel; keep the fixes). Even with the loom now CLEAN+verified (ring_hz≈marker_hz≈GT), a fly-away STILL occurred, launching >1s AFTER the loom-ring went silent -> the terminal fly-away is an INDEPENDENT lateral/bounce mechanism, not a loom problem. Keep ring-commit/loom-ring OFF; don't chase h_z fixes here. WINNER: observer+DESCENT_GATE (5/5 deck, 0.055-0.095m, 0 fly). Also: ad-hoc GT scripts must feed GTFeedback continuously (stateful; gap-feeding inflates std ~2x).
- [⭐ AGREED terminal velocity-handover design (2026-07-04, ⛔ ACTION superseded — see ring_loom deadend)](project_terminal_velocity_handover_design.md) — retarget PLASMC_TERMINAL_COMMIT: keep marker s (position, moving-target-OK), switch flow h_xy+h_z→RING, gated on d(logM) handover THEN centered; fixes the ACTUAL roots (corrupted velocity) vs TERMINAL_COMMIT's zeroing zeta_r (position, wrong signal); trigger=handover not extent (flicker-robust)
- [⭐ h-extrapolation BAKED default-ON (2026-07-07)](feedback_h_extrapolation_baked.md) — PLASMC_H_EXTRAP=1 replaces 2026-05-13 hard-zero; the old failure was 3 fixable bugs (self-reference, no decay, fit-through-clipped-samples), not extrapolation itself being unsafe. Validated: 0/3 misses vs baseline 1/5 (GT xy 2.7m). ⚠ s/_img_feature_param has the SAME unfixed self-reference bug — next target, NOT yet fixed.
- [Dense-recovery (RANSAC homography) + failure-cause auto-tagging (2026-07-07)](project_dense_recovery_and_failure_tagging.md) — PLASMC_DENSE_RECOVER unified-staleness-gate implementation, validation MIXED (do not bake); getFailureCause()->DRIFT_OFF/OVERFLOW/UNKNOWN now auto-tags TARGET_LOST. Also: recurring SIGSEGV rc=139 is a pre-existing SITL infra flake, not a code-change symptom.
- [⚠ _savgol_predict suspected in a 23.93m fly-away, NOT confirmed (2026-07-07)](feedback_savgol_predict_suspect_flyaway.md) — default-ON model-based h spike-reconstruction is CONTROL-AFFECTING (feeds a_u directly, not just logging); false-triggers frequently on dt-floor-amplified noise (confirmed); the specific catastrophic jump (h 0.02->1.25 in one step, invisible in raw/KF perception) not yet reproduced live. SAVGOL_PREDICT_DBG=1 available.
- [⛔ CBF extent-fix n=5 sweep FALSIFIES the n=1 smoke-test optimism (2026-08-13), root-caused to lateral-convergence-rate NOT the CBF (2026-08-14)](../project_20260813_cbf_extent_fix_followup.md) — 0/25 precise, 1/25 soft; raw pre-CBF I_a_raw(t) direction itself degrades/reverses before the CBF binds (checked directly, CBF confirmed NOT the cause). Also: ArUco regression PASS, ring-sampling origin_ratio INCONCLUSIVE n=1, validate_cbf.py had 2 stale-oracle bugs (now fixed)
- [CBF radius split (commit 27410ed, 2026-08-14): eliminates fly-away outliers, doesn't touch lateral-convergence root cause](../project_20260814_cbf_radius_split_and_hardware_check.md) — IC3/IC4 max xy_err 28.7m/14.8m -> 2.9m/3.4m after the split; precise/soft still 0/25,1/25. Also: Hardware confirmed ArUco-only (zero cross-marker code, extent bug N/A there); NaN bug confirmed STILL PRESENT in Hardware/scripts/controller.py; cross-session reminder file created at Hardware/docs/PENDING_NAN_FIX_PORT.md (not yet pushed)
- [⛔ SELF-CORRECTION (2026-08-15): SP WAS achieved with GT-FB on ArUco (56-76%!), terminal 1/Z was a Z_REG=0.01 harness artifact not an unsolved control limit](../project_20260815_crossmarker_gtfb_sp_investigation.md) — see [[project_why_sp_achieved]]; I wrongly claimed the opposite earlier this session from an incomplete parameter_record.ods search. Current cross-marker defaults match most of the winning config (XIR/PR0/W_U_MAX/Z_REG); VDS_KF_Q reverted to 10 for a separate reason. n=1 cross-marker GT-FB IC2 failed (xy=1.28) but noise floor is +-5-7/25 — n=10 rerun in progress, don't trust n=1 either direction
- [⭐ Descent-stall at alt=0.487m: marker-type-AGNOSTIC kappa-leakage fixed point, ArUco escapes it only 1/6 too (not "ArUco is fine"); numerical-noise + CBF-radius-cap hypotheses both TESTED AND FALSIFIED; VDS_KF_Q=1 still untested](../project_20260817_crossmarker_descent_stall_investigation.md) — n=10 cross-marker GT-FB sweep found a repeatable stall (4-9/10 depending on config) at the SAME altitude to 3 decimals; proved via exact-equilibrium match it's NOT numerical noise (a "fix" made it WORSE, reverted); raising CROSS_CBF_RADIUS_CAP_PX 100->130->160 had ZERO effect (falsified); ArUco hits the identical fixed point, only escaped 1/6 reps under current defaults (not 56-76% like the historical campaign) -> points at VDS_KF_Q=10 vs historical 1 as the likely gap, NOT a marker-type difference. Also: fixed a real theta(t) logging bug (was logging the inert legacy scalar under default PLASMC_THETA_PER_AXIS=1); corrected a wrong d_min_fov/rho_fov-vs-real-CBF-barrier(p_10) mental model (rho_fov is legacy, FROZEN at rho_fov_0 by default, disconnected from cbf2_filter's actual Phase-1 bound); ring-sampling's modest fix (n=5 validated, target_lost 100%->60%) traced to the SAME lateral-convergence root cause via a new Ring Active Log diagnostic (guard correctly declines when off-center). theta(t) fix + Ring Active Log + video-overlay tool UNCOMMITTED at session end. NEXT: test VDS_KF_Q=1 first
- [⭐⭐⭐ ArUco is now COMPARISON-ONLY; active track = cross-marker + GT-feedback; next milestone = GT-ablation test, then ArUco not needed at all (2026-08-23, user directive)](project_marker_roadmap_gt_ablation.md) — stop running perception-mode ArUco test flights to chase ArUco-specific bugs (see [[feedback_aruco_perception_scope]]); any ArUco finding is informational/comparison data only
- [Terminal kappa-ratchet/actuator blow-up traced to CBF_CORNERS_STALE's 30-frame-streak guard not tripping against detection FLICKER (2026-08-23)](project_20260823_kappa_ratchet_detection_flicker.md) — IC2 ArUco flight: raw ArUco decode froze 0.93s at touchdown (Img_Data.npy corners byte-identical), but cbf_corners flickered 0<->valid each tick (coast-hold cascade re-validating stale corners), so _cbf_corners_none_streak never reached 30 -> kappa's freshness guard never tripped -> kappa 0.78->2.07 integrating against frozen s_e_n -> a_u exploded 4 orders of magnitude. Raw-freeze trigger + specific flip-flopping cascade tier + fix NOT yet found.
- [Cross-marker raw detection is structurally flicker-prone (Hough re-derived from scratch every frame, zero hysteresis); CBF_CORNERS_STALE windowed-fraction fix + opt-in KLT center-bridge landed 2026-08-23 (commit 0431eab)](feedback_cross_marker_detection_flicker.md) — root cause + both fixes; center-bridge default OFF pending cross-marker+GT-FB validation. DISTINCT from the offcenter_convergence_wall finding below -- don't conflate.
- [⭐⭐ Cross-marker IC2 off-center y-axis non-convergence (xy_err 2.3-4.0m) — bisected pre-existing, NOT introduced by 2026-08-23's commits (2026-08-24); ⛔ "never-solved kappa-leakage wall" framing RETRACTED same day, see below](project_20260824_crossmarker_offcenter_convergence_wall.md) — bisected via isolated git worktree at 87b8896 (pre-change): xy_err 2.27-2.40m there too, confirming pre-existing. GT-verified: ux converges+overshoots, uy stays flat/drifts away; controller's own s_e_n[1] matches GT (not a perception bug); kappa[1] DECREASES (0.50->0.27) while s_e_n[1] stays large. Detection Status was 'ok' 450/450 frames this rep -- confirmed INDEPENDENT of the detection-flicker issue above.
> ⛔⛔ **MAJOR CORRECTION (2026-08-24): this is NOT the original 0.648-anti-restoring-authority kappa-leakage wall — that gap was already diagnosed+FIXED 2026-06-29** ([[feedback_prinf_standing_condition]], funnel-shape lever `PLASMC_PR0_X/Y=10.0` + `PLASMC_XIR_X/Y=0.10`, ALREADY BAKED as controller.py defaults, `controller.py:461/464`). Proof: an IC5 cross-marker run under `PLASMC_GT_FEEDBACK=1` (same IC, same baked config, same session) landed precise+soft (`xy_err=0.066m, rel_vel=0.083`) — `s_e_n` converges 0.82→~0.15-0.20 and STAYS converged, no wall at all. The kappa-decay-while-error-persists signature seen in EVERY perception-mode run (IC2 and IC5) but ABSENT under GT-FB on the identical config means the real, still-open gap is **perception-specific** — real detection noise/flicker breaks the "S_r stays tiny" invariant the funnel-shape fix relies on, something never checked since that fix was validated (that whole investigation was GT-FB-only). NOT a fundamental control-law limitation; a perception-robustness gap.
> **RESOLVED same day: [[project_20260824_ic5_perception_fov_margin_gap]]** — isolated `dtheta` as a confound (`PLASMC_DTHETA_AZ_GAIN=0`), compared perception-mode cross-marker (2 reps) AND ArUco (1 rep) vs GT-FB, same IC5. **Reproduces on ArUco too — NOT cross-marker-flicker-specific.** Real mechanism found directly in the ArUco log: `ArUco lost — KLT fallback active` (8x) then repeated `KLT corners left image bounds — stopping fallback` — the marker's corners GENUINELY exit the camera frame (a real geometric FoV event, `MARKER_EXTENT_PX` held flat ~1.8s = stale/decode-failure signature), not subtle noise-on-`s_e_n`. GT-feedback structurally cannot experience this (no imaging chain at all) — that's why the identical baked control law works under GT-FB and fails under real perception at this IC. Directly connects to [[project_20260824_dtheta_az_filter_self_defeating_feedback]]: `dtheta`/CBF fires 65-98% of frames at IC5 because it's genuinely fighting a thin FoV margin, and even so sometimes loses. This is a perception/CBF-margin gap, NOT a controller-gain or funnel-shape problem (those are already solved for whatever the imaging chain can actually deliver). TARGET_LOST fires ~3-3.6s, well before the funnel-shape law's needed ~5-9s convergence window. **Follow-up (2026-08-25): IC2/IC3 checked, same mechanism confirmed but NOT catastrophic there** — identical `KLT corners left image bounds` events occur (IC2: 2x, IC3: 52x) but cluster near touchdown of a full 7-7.5s flight (`min_alt≈0`, `xy_err` 0.30/0.43m, benign/expected terminal perception loss) rather than mid-flight. Root distinction is WHEN, not WHETHER: IC2/IC3 start at 5m (same 2m/2m offset as IC5) so the margin only tightens near the ground, already-late where it's harmless; IC5 starts at 3m so the same squeeze happens almost immediately (~3s, still 2-3m up) where it's catastrophic. Not an IC5-exclusive bug — the same universal geometric mechanism, just triggered far earlier by IC5's lower starting altitude.
> **FIX implemented + A/B'd (2026-08-25): [[project_20260825_cbf_margin_reserve_fix]]** — `CBF_MARGIN_RESERVE` (default 0.0=unchanged, `validate_cbf.py` still 12/12) added to BOTH `cbf_visibility.py`/`cbf_visibility_aruco.py`: Phase 1's box bound was centroid-only by design (`m2=phi_max`, delta2/footprint only used reactively in Phase 2 after decode already failed) — this proactively reserves a fraction of the marker's own measured footprint in Phase 1 too. IC5 ArUco A/B (perception, dtheta=0 confound removed): reserve 0.0 and 0.5 both hit `TARGET_LOST`. **Extended to n=3/arm (2026-08-25): INCONCLUSIVE, not the confident win first reported.** reserve=0.0: 2/3 TARGET_LOST; reserve=1.0: 1/3 TARGET_LOST but that ONE failure was xy=22.46m, the single worst result of the whole investigation — real variance in BOTH arms. IC1/IC2 regression spot-check (n=1, reserve=1.0): no obvious regression (IC1 SOFT+PRECISE 0.083, IC2 SOFT-only 0.190); IC3/IC4 not checked. Mechanism confirmed genuinely engaging (diverging trajectories/theta_cone, not a no-op) and correctly targeted. **⛔ That first sweep was CONTAMINATED (concurrent SITL session active the whole window, `test_data/Rover_AB_harness/ic5_hang_chase_*`) — CLEAN re-run (n=3/arm, confirmed no concurrent SITL before every launch) REVERSES the direction: reserve=0.0 1/3 TARGET_LOST, reserve=1.0 2/3 TARGET_LOST.** No clean evidence this fix helps; weak hint it may cost more than doing nothing. NOT baked (left at default 0.0). Correctly-targeted lever, unproven benefit — a real n>=5 clean sweep (and checking a milder reserve, since 1.0's max conservatism may itself be the problem) would be needed before revisiting.
- [⭐⭐⭐ dtheta az-visibility-filter is SELF-DEFEATING via an attitude-history feedback loop through cbf2_filter's th_curr -- CONFIRMED (2026-08-24) with the direct theta_safe/theta_desired ratio (1.25->0.95, gain 5->40); not a gain-tuning problem, A_CAP saturation ruled out with hard data](project_20260824_dtheta_az_filter_self_defeating_feedback.md) — PLASMC_DTHETA_AZ_GAIN sweep {5,10,20,40} n=3 IC5 cross-marker: correction magnitude scales correctly with gain (4x range, I_a[2] shift -0.6->-2.75) and dtheta fires 65-98% of frames, but xy_err/rel_vel show NO improving trend. Root cause: BOTH dtheta and theta_cone (CBF's own solved lean magnitude) shrink monotonically with gain (12.08->6.69deg) -- extra lift changes realized attitude history, which feeds back into cbf2_filter's th_curr (current-tilt) reference next cycle, shrinking the QP's own solved theta_safe. Follow-up same day: logged `theta_desired(t)` to Control_Data, reran IC5 gain 5 vs 40 -- theta_safe/theta_desired ratio shrinks 1.25->0.95 confirming the QP delivers a shrinking FRACTION of an already-shrunk ask as gain rises (not just a smaller ask). The mechanism undermines the exact lateral authority it exists to protect. Fix needs to break the loop (anchor th_curr against DESIRED not realized attitude, or decouple) -- NOT YET IMPLEMENTED. `PLASMC_DTHETA_AZ_GAIN` left at placeholder default (10.0). IC2's earlier positive A/B trade-off (better xy_err, worse rel_vel vs the old loom-margin approach) likely reflects IC2's much rarer dtheta firing (~20% vs IC5's ~65-98%) limiting how much this loop bites there. Whether the OLD loom-margin approach shares this vulnerability is still unverified. **Same-day follow-up implemented: [[project_20260824_dtheta_href_continuous_compensation]]** — `PLASMC_DTHETA_HREF` adds a continuous (exponential, no dead-zone) `h_ref` compensation upstream of `cbf2_filter` + a crossfaded direct `I_a[2]` term (bridges the several-cycle `h_ref` propagation lag, then fades). Mechanism confirmed firing correctly; `theta_safe/theta_desired` ratio moved toward/above 1 (1.25→1.70 low-gain, 0.95→1.03 high-gain) — less over-suppression. NOT yet validated at n>=5. Default-OFF.

> ⛔ **CORRECTION same day: the IC5 smoke-test failures are NOT the off-center wall** — [[project_20260824_dtheta_ic5_flyaway_rootcause]] traced the actual trajectories: a genuine catastrophic fly-away (3m→7-10m altitude runaway climb + 20-28m lateral divergence in ~9s), caused by `dtheta`'s near-continuous (65-98%) engagement at IC5 injecting an UNCAPPED, cumulatively-growing extra-lift correction every cycle (measured: correction magnitude itself grows 0.2→1.5-1.9 m/s² over the first 2s) — an integral-windup-shaped defect in the raw `dtheta` correction, independent of and NOT introduced by the v3 `PLASMC_DTHETA_HREF` work (identical fly-away confirmed present in the pure pre-v3 baseline). IC2 comparison (gain=10, baseline vs v3): `dtheta` duty cycle only 8-20% there (vs IC5's 65-98%), `href_g` stays ~1.0 (barely engages), NO fly-away (zmax flat ~5.0m), both land with the real off-center-wall-level `xy_err` (~2.0-2.3m) — confirms the wall is real and IC2-visible, just not what caused the IC5 failures. Not yet fixed: needs a cap on `dtheta`'s cumulative/rate contribution to `I_a[2]`, separate from the `th_curr` self-defeating-loop fix. **Band-aid implemented + tested (NOT a root fix — user-corrected same day): [[project_20260824_dtheta_cap_flyaway_fix]]** — `PLASMC_DTHETA_AZ_CAP` (default 2.0 m/s², clamps the final per-cycle correction). Cap ALONE insufficient (still ASCENDING, peak alt 5.9m — reduced severity, not fixed; a bounded-but-nonzero bias sustained 89% of frames for 5+s still integrates into real climb). Cap + v3 crossfade (`PLASMC_DTHETA_HREF=1`) TOGETHER suppress the visible ASCENDING symptom (`descent_anomaly=False`, monotonic 3.0m→0.49m descent) by clamping+fading the correction's OUTPUT — but do NOT address why `dtheta` runs chronically high in the first place; per [[feedback_clamps_during_tuning]] this is a band-aid, not a fix. Landing STILL fails at IC5 (`target_lost=True`, `xy_err≈9.4`, drift to -6.3/+6.7m). **Traced (same day): this residual failure shows the same signature as the IC2 case** ([[project_20260824_crossmarker_offcenter_convergence_wall]], now CONFIRMED at IC5 too, not IC2-specific) — `kappa[xy]` decays 0.50→0.33/0.31 while `s_e_n` never converges (0.86 pinned, then grows past the ~1.0 funnel bound right before TARGET_LOST). ⛔ RETRACTED same day: this is NOT the old "never-solved" kappa-leakage wall (that's already fixed, baked, and confirmed clean under GT-FB on this exact IC/config) — it's a perception-mode-specific gap; see the correction on the IC2 entry above.
- [Kappa-ratchet campaign 2026-08-28: baseline pass IC1-5 clean, ratchet already resolved](project_20260828_kappa_ratchet_campaign.md) — 25/25 SP GT-FB, kappa_xy frozen at 0.5, 0% breach; no knob sweep needed
- [⭐ Guaranteed SP across IC1-5 (GT-FB) — synthesis: why every prior failure, cumulative fixes](px4/project_20260828_guaranteed_sp_synthesis.md) — 25/25 SP n=5/IC; ⚠ GT-FB only, real-perception still open (cal accuracy + oblique-low detection + off-center convergence wall)
- [test_data/Final/ — curated IC1-5 landing recordings (committed b89f66b8)](px4/reference_final_landing_recordings.md) — per-IC montage + onboard/chase cams + full dataset/; config main@4d7bc210 perception-mode cross-marker; IC2 & IC5 precise (0.065/0.058m), IC1/3/4 IMG_RECORD-perturbed misses (clean baseline = ICValidation/20260831-144626)
