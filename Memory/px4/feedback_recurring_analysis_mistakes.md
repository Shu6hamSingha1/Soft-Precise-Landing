---
name: feedback_recurring_analysis_mistakes
description: "PRE-FLIGHT CHECKLIST of the analysis mistakes Claude makes REPEATEDLY on this project, each with dated instances and the one check that catches it. Recurring classes: log-to-log time alignment; pairing assumed from directory names; reference-frame/offset (rover rel-z 0.5 MEANS landed); metric sampled at the wrong instant; confounded or non-overlapping comparisons; one-sided metrics; stale derived docs trusted over source; unverified baselines under concurrent sessions. ADDED 2026-09-16/17/18 (sections 10-19): STATIONARY and ROVER launchers both default to ArUco not cross-marker (WORLD/MARKER_TYPE, and ROVER_MODEL for the rover one, silently unset -- the canonical IC2-5 gate script run_ic_validation.sh included -- verify via Img_Data.npy's own KEYS, not the launch command: FEATURE_IS_VISIBLE/Fail Reason/MARKER_EXTENT_PX = cross-marker, Centroid Map Raw/Ring Opt Flow*/Alpha Map* = ArUco); harness stale-directory false-success (a crashed rep silently re-read the prior rep's output); the STIMULUS changed -- verify the scenario the test drives, not just the code (ROVER_TRAJ=Circular silently stopped driving a circle and cost a whole session); mechanism inferred from an observational log-diff and reported as a finding (2 of 3 such claims refuted by the first controlled test); out-of-repo state -- a worktree reconstructs the CODE not the EXPERIMENT, so validate bisect endpoints; n=25 deltas quoted without the baseline own spread (identical baseline spans 17-20/25, so <3/25 is noise); mtime on archived test_data is not the run date; pixel quantities compared across the 640x480->320x240 change; inherited env defaults are not a controlled variable on a shared worktree. ~85 of ~200 memory files record a correction. Run these BEFORE concluding, not after."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 5f1d366c-f4b6-4a4f-9d5b-05c93b9a480f
  modified: 2026-09-02T00:00:00.000Z
---

**Written 2026-09-02 at user request**, after a single session re-committed several of these.
**~85 of ~200 memory files contain a CORRECTION / RETRACTION / SUPERSEDED / FALSIFIED note.**
The same handful of shapes recur. This is the checklist, not a confession list.

> ⚠ **The meta-lesson, and the reason this file exists:** recording a trap does NOT prevent
> re-committing it. On 2026-09-02 I derived the rover platform constant empirically, wrote it
> into memory *as a trap to avoid*, and then fell into it about an hour later while reading a
> chart. Knowing the fact is not the control; **running the check before concluding is.** The
> common shape of every failure below is: reach a conclusion, then read weak evidence as
> confirmation.

## 1. Log-to-log TIME ALIGNMENT (the most repeated one)

Different logs are on different clocks and different windows. Correlating by INDEX, or
assuming coverage, is invalid.

- 2026-06-04 [[feedback_imgdata_gt_clock_skew]]: `Img_Data` (perf_counter, ~83 Hz) vs
  `Ground_Truth` (mission-relative, ~125 Hz), Δorigin ~29.8 s, non-overlapping ranges.
  Index-correlating falsified an "alpha doesn't track yaw" conclusion (real r=1.00).
- 2026-08-27: that file PARTIALLY SUPERSEDED — a validated direct fix exists
  (`Img_Data['Time'] - gt['Start Time']`).
- **2026-09-02, hit again in a new form:** `validate_detector_gt.py` aligns frames to the tail
  of `Img_Data` and interpolates GT with `np.interp`, which **CLAMPS** outside the GT window
  instead of rejecting. On one hand-paired run **117/308 frames (38 %) fell past the end of GT**
  and were scored against frozen touchdown values — reporting detOK 49 % where the truth was 18 %.

**CHECK:** print both series' `t0..t1`, sample rate and length, and the OVERLAP, before any
comparison. Filter to `t_g[0] <= t <= t_g[-1]`. Prefer the curated `--set` path over
hand-pairing.

## 2. PAIRING assumed from directory/file names

> ⚠ See also **§13** — the MIRROR of this one: there the directory NAME was right and the
> mtime was wrong. Neither is authoritative; cross-check against recorded content.


- 2026-09-01: `validate_detector_gt.py`'s own docstring warns the data-dir ↔ `_raw`-dir pairing
  is MANUAL (data dir lags ~13 s).
- [[reference_final_landing_recordings]] records that IC1-4 chase↔dataset links rest on
  save-time adjacency + visual confirmation only.
- **2026-09-02:** I claimed `montage_rover_static_ic1*.mp4` showed a non-landing because I
  paired it to `Cross_Marker_Montage_Rover` on directory-name similarity. WRONG — mtimes showed
  the montage was written 61 s after `Rover_Static_IC1_Montage/rep1.out`, a genuine 0.487 m
  landing. The filename had named the right directory all along.

**CHECK:** confirm a pairing by **file mtimes** AND by matching a plotted/recorded trace against
the candidate run's data. Never by name similarity.

## 3. REFERENCE FRAME / OFFSET confusion

The single richest source of wrong conclusions in this project.

- **Rover:** the target pose is the rover BASE (z~0.02), not the landing surface. The platform
  top is 0.50 m. **`uav.z - target.z ~ 0.49` MEANS LANDED**, not airborne. Verified empirically:
  settled rover runs rest at rel-z **0.487, mode 0.5, n=12**.
  - 2026-09-02 instance A: a first-pass archive audit using a flat 0.20 m floor condemned **34
    genuine platform landings**.
  - 2026-09-02 instance B: an hour after writing (A) down, I read a montage's
    `|rel. position| -> 0.5` as "hovering above the plate". It meant landed.
- [[feedback_image_center_bug]]: the centre bug was in the NOTEBOOK, not `img_data.py`.
- The `_resolution = (480, 640)` / `center = (240, 320)` pairing in `img_data.py` is correct for
  the POST-`ROTATE_90_CW` frame — don't "fix" one without the other (CLAUDE.md).
- The 90° SMC↔CBF `Rz_p90b` convention: a synthetic-validated "fix" (13/13) regressed every IC
  in SITL and was reverted (`4d7bc210`).

**CHECK:** for any height/position claim, state the datum explicitly and verify it against runs
with a KNOWN outcome before interpreting.

## 4. Metric sampled at the WRONG INSTANT / no absolute gate

- [[feedback_false_sp_frozen_gt]] (2026-06-10): frozen→origin-reset GT gives `xy_err`~1e-21 and
  trips `precise`.
- [[feedback_relative_flatness_needs_absolute_gate]]: a "stopped changing relative to its own
  history" test proves local stationarity, NOT a terminal state — learned from **two consecutive**
  touchdown-detector bugs (2026-08-26, 2026-08-28).
- 2026-09-02 [[project_20260902_archive_rescore_false_precise]]: **159 of 1192 (13.4 %)** archived
  `precise` verdicts were computed mid-air; `xy_err` was read at whatever instant the control loop
  exited. Fixed `d6610ea7`.

**CHECK:** does the evaluation instant correspond to a VERIFIED terminal state, gated on an
absolute quantity — not just a relative/flatness one?

## 5. Confounded or NON-OVERLAPPING comparison presented as a mechanism

- [[feedback_historical_cal_confound]] (user, 2026-06-03): ~2000 reps' "lag is the floor"
  conclusions were confounded by a 2-13× broken output calibration.
- [[feedback_correlation_needs_pooling]]: Pearson on a near-constant true signal is
  noise-dominated; pool across flights before trusting the SIGN.
- [[feedback_sensitivity_sweep_methodology]]: n=1 is noise.
- **2026-09-02:** I asserted a "~30° obliquity cliff" in detection from two datasets whose angle
  ranges **barely overlap**. Where they DO overlap (30-45°) they disagree completely (rover_IC4
  100 % vs rover_IC2 31 %) — angle was confounded with altitude. Retracted.

**CHECK:** do the compared groups overlap on the proposed explanatory variable? Is there a
matched pair (same scene, one variable changed)? The flat-vs-clutter pair was valid *because*
it was controlled; the IC2-vs-IC4 comparison was not.

## 6. ONE-SIDED metric — a rate quoted without its complement

- **2026-09-02:** recommended a `|dz/dt|` terminal-rest gate citing its CATCH rate (152/159)
  without measuring its FALSE-POSITIVE rate. It rejects **58 % of LEGITIMATE landings**. Wrong,
  retracted, recorded in [[project_20260902_archive_rescore_false_precise]].
- **2026-09-02:** read `Disarming denied: not landed` as PX4 contradicting a touchdown latch.
  Base rate over 2396 paired runs: **97.4 % of genuine landings**. It is routine noise.
- 2026-09-01: `validate_cbf.py` 13/13 synthetic passes were "misleading" — the fix regressed
  every IC in SITL.

**CHECK:** a detection rate needs its false-positive rate; a log message needs its BASE RATE in
the negative class; a synthetic pass needs a real-data confirmation.

## 7. Stale DERIVED doc trusted over source

- CLAUDE.md itself warns the sensor-cal block "will be stale; read the live values in
  `img_data.py`".
- 2026-09-02 [[feedback_verify_injected_docs_before_trusting]] (another session): the
  auto-injected STATUS block and CLAUDE.md both asserted wrong gains/camera facts.
- [[project_20260825_overlay_detection_artifact_logging]] is stamped CONTRADICTED — verify
  directly before trusting its line.

**CHECK:** quote load-bearing numbers from source files, never from a doc, a memory, or an
injected block. Memories reflect what was true when written.

## 8. BASELINE not verified (multi-session repo)

- 2026-09-02 [[feedback_ab_baseline_verify_concurrent_commits]]: another session committed the
  exact fix under test 3 minutes before my A/B ran, so the "baseline" arm already contained it.
  A real fix measured as a 0 % delta and I wrongly retracted the root cause.
- Sibling: [[feedback_check_concurrent_sitl_before_launch]] — 3+ `claude` sessions commit to this
  repo concurrently (verified: `session_01QgSdJd`, `01Jtuav4`, `01D2gjuv`, plus mine, all on
  2026-09-02).

**CHECK:** `git log --oneline -5` immediately before an A/B and again before writing up a NULL
result; pin the control arm by explicit revert env; echo the live constant per arm. **A null
result is the trigger to re-verify the baseline, not to conclude the mechanism was wrong.**

## 9. Acting on a memory entry a LATER code change killed

Memory is append-mostly: when a mechanism is removed from the code, a NEW entry gets written
and the OLD one is usually left standing. Read in isolation, the old entry still reads like a
live instruction.

- **2026-09-03 instance:** `px4/MEMORY.md` line ~284 said *"Next real step: validate
  `PLASMC_DTHETA_HREF=1` at IC5, n>=5, isolated"* (a verified 2026-08-26 causality finding:
  dtheta destabilised first, perception degraded second). I recommended exactly that. But
  `_dtheta_correction` had been REMOVED on 2026-08-31 (`e110b8a7`) and folded into the joint QP
  as `CBF_AZ_COST_GAIN` — `grep -c _dtheta_correction src/controller.py` == 0. **Both facts were
  in the same index file**, the newer one 277 lines above the older, and only the newer said so.
  The user caught it. (Same session, same shape: `p_s`/`ζ_s` cited as live when the design had
  moved authority into `ζ_r`; `PRINF` questioned when the code comment says it is inert by design.)

**CHECK:** before acting on any memory recommendation naming a symbol, `grep` that symbol in the
source FIRST. If it is gone or default-off for a different reason, the entry is stale.

**AND WHEN YOU FIND ONE: stamp the OLD entry, don't just add a new one.** An unstamped superseded
entry will produce the same wrong recommendation for the next session. Historical findings
(causality, mechanism) can stay valuable while the named fix and next-step are dead — say which
is which in the stamp.

## 10. MECHANISM inferred from an observational LOG-DIFF, reported as a finding

**2026-09-09→16 session: THREE confident mechanism claims, TWO refuted by the first
controlled test that touched them.** Each looked strong observationally — matching
magnitudes, high correlations, agreement with prior campaign analysis — and each was wrong.

- **κ-ratchet HF pump** (AU_LEAD stationary regression). Log-diff evidence: `I_a_raw` peak
  4.7→14.6, κ peak 0.21→0.41, κ-growth +0.08→+0.37, `corr(lead-delta, κ)` −0.01→+0.42, RATIO
  clamp pinned at exactly 0.5·|I_a_raw|. **Refuted by a GT-FB A/B** (no regression, κ *decays*
  in both arms) — all of it was a driven response to amplified sensor noise, not self-excitation.
  [[feedback_aulead_stationary_regresses]]
- **"The visibility-QP rewrite killed the curve limit cycle."** Evidence: 4/4 gain reverts null
  (13/13 clean), old cone rotated the command 0.345-0.473 rad at 100 % duty vs the new QP's
  0.058-0.106 / 0-16 % active, AND the original campaign had independently named that cone
  *"the DF that caps growth"*. **Refuted by a worktree at the pre-rewrite commit** — old cone
  live at July magnitude, still no cycle. [[project_20260916_curve_qgate_revalidation]]
- The one that SURVIVED its test: "AU_LEAD is a perception-noise amplifier" → the QGATE fix
  worked. Note what distinguishes it — it was framed as a prediction and then tested.

**The check:** a log-diff yields a HYPOTHESIS. Before writing it as a finding, name the
controlled experiment that would falsify it, and run that. Correlated magnitudes in one arm
are not a mechanism; only an intervention is.

## 11. OUT-OF-REPO state — a worktree reconstructs the CODE, not the EXPERIMENT

**2026-09-16:** bisecting Jul→Sep for what killed the curve cycle. Endpoint validation first:
the **exact July commit `edb546f0`** that produced 27/27 cycling reps was re-run on a worktree
**today → NO-CYCLE (median |e_rot| 0.13 vs +0.58…+1.10)**. Same commit, opposite behaviour ⇒
the cause is outside git, and a bisect would have falsely converged on the earliest commit.

This project's behaviour depends on state git does not hold: the **camera SDF**
(`~/PX4-Autopilot/.../mono_cam/model.sdf` — 640×480→320×240 on 2026-08-27), Gazebo world and
marker assets, PX4 version, host load.

**The check:** before any bisect or "commit X changed behaviour" claim, **verify the old
commit still reproduces the old behaviour.** If it doesn't, stop — the answer is not in the
history.

## 12. n=25 deltas quoted without the BASELINE'S OWN SPREAD

**2026-09-09→12:** three independent runs of the IDENTICAL perception baseline (no config
change whatsoever) gave **20/25, 19/25, 17/25** — pooled 56/75 = 0.747. **Run-to-run spread is
3/25 (12 pct pts) with nothing changed.** I reported "18/25 beats the 17/25 baseline" as an
improvement; Fisher p = **1.000**.

Resolution at n=25: ≤8/25 is unambiguous; ~13/25 is marginal (p≈0.05); ≥16/25 is
indistinguishable from baseline.

**The check:** run ≥2 baseline repeats before believing any gate/knob result, and quote a
Fisher p, never a bare count delta. Also beware the sibling error: slicing to the ICs that
worked (an "IC1-4 recovered" that was 13/25 overall, still p=0.046 below baseline).
[[feedback_session_20260909_12_audit]]

## 13. mtime on archived test_data is NOT the run date

**2026-09-16:** built an `e_rot` timeline dated by file mtime; it showed the curve cycle still
alive on 2026-08-07, narrowing the search window to Aug 7→Sep 9. Those directories are named
**`Fri Jul  3 ... 2026`** — July runs whose mtime a later copy had bumped. Re-dating from the
directory NAME collapsed the window back to the original Jul 3→Sep 9 (no moving-target data
exists in between at all).

**The check:** parse the run timestamp out of the directory name (`%a %b %d %H-%M-%S %Y`);
use mtime only as a fallback and say so. Note this is the MIRROR of §2 — there the names lied
and mtime was right; here mtime lied and the name was right. **Neither is authoritative;
cross-check against recorded content** (`Control_Params`, `Img_Params`) whenever the date matters.

## 14. PIXEL-domain quantities compared across the 2026-08-27 resolution change

**2026-09-16:** predicted from archived July reps that the AU_LEAD quality gate would transmit
**<1.5 %** and kill the curve — by dividing July-era `MARKER_EXTENT_PX` by **today's**
`frame_min=240`. July ran 640×480 → `frame_min=480`. True July fill was 246/480 = **0.51**
(gate OPEN), not 1.02 (gate closed). **Off by exactly 2×**, and the live run measured 0.33-0.56.

**The check:** any px quantity (`MARKER_EXTENT_PX`, centre, fill fractions, `rho_fov`, cal
matrices) must be renormalised across 640×480→320×240. Read `Img_Params.txt` from the run
itself. GT-derived metrics (`e_mean`, `e_rot`, `osc_std`, touchdown lat) are
resolution-independent and safe.

## 15. Inherited DEFAULTS are not a controlled variable on a shared worktree

**2026-09-09:** my GT-FB A/B ran arms 27 min apart without pinning `CBF_DRIFT_TAU`. A peer
session re-baked its default **between them** (`6701d143` 21:40 →0; arm A 22:13; `827933b5`
22:27 →0.15; arm B 22:40), so **arm A ran 0.0 and arm B ran 0.15** — and I had told the peer
the harness pinned it. It didn't. One arm of that A/B is permanently confounded.

**The check:** pin EVERY non-default env var explicitly in the harness, and verify from the
run's own `Control_Params.resolved` afterwards — not from what you believe the default is.
Related harness bug from the same thread: `env NAME=VAL -u OTHER` silently runs `-u` as the
COMMAND; all `-u` flags must precede every `NAME=VALUE`. That silently no-op'd a whole arm.

## 16. The STIMULUS changed — "is the experiment even the same?" (the one that ate a whole session)

**2026-09-17, the root cause of an entire multi-hour confusion.** I chased "what killed the
curved-target limit cycle" through FOUR controlled experiments — a 4-arm gain-revert sweep, a
pre-rewrite worktree, a 640×480 camera restoration, and an aborted commit bisect — and
produced three confident mechanism claims, all wrong. **The cycle had not been fixed. It was
never being driven.**

`ROVER_TRAJ=Circular` stopped driving a circle on **2026-07-03 12:13**, commit **`b816fea0`**:
`ROVER_CIRCLE_R` **0.8 m → 10 m**, with the code's own comment saying *"only the path
curvature drops ~12x"*. Measured from the GT target path: July **0.86 m radius / 234° swept**
vs today **9.4-12.1 m / 11-24°** — a straight line. Restoring `ROVER_CIRCLE_R=0.8` on
unmodified HEAD brings the cycle straight back (median |e_rot| 0.70, r_fit 0.88 m, one rep
missing by 7.20 m).

**Every "no cycle" result I had was a null stimulus, not a null effect.** The controller was
never the variable.

**The check — do this BEFORE any A/B on a dynamic scenario:** measure the STIMULUS from the
recorded data and confirm it matches the reference experiment. For a moving target that is
four numbers off `Target Pose`, and it costs seconds:
- fitted path radius (algebraic circle fit) and **arc swept** (unwrapped angle about the fit
  centre) — `pathlen / (r_fit · swept) ≈ 1` only tells you the fit is self-consistent, it does
  NOT tell you the path is curved; a 12 m/20° arc scores 1.0 too. **Read the radius.**
- target speed and angular rate; check `v ≈ wz·r` closes.
- ⚠ do NOT use accumulated heading change as the curvature proxy — numerically differentiating
  a stair-stepped GT pose makes a straight path score a huge "turn" (mine read 50 rad on a
  straight line and pointed the opposite way).

**⚠ THE SHARPER FAILURE MODE (peer session, 2026-09-17, worth more than the rest of this
section): the check EXISTED and was NON-ROBUST — which is more dangerous than skipping it,
because it manufactured false confidence.** The peer *did* test trajectory comparability
before pooling July with September. Their metric was heading sweep from
`unwrap(atan2(dy,dx))` on numerical gradients — and it returned **16-26 rad for the September
arms**, i.e. it reported the near-straight paths as turning MORE than July's real circles,
because on a straight path the heading is pure noise and `unwrap` accumulates spurious 2π.
They had explicitly flagged that metric as noisy earlier in the same session and then let it
license the pooling anyway. I independently hit the identical trap (50 rad of "turn" on a
straight line). **A circle fit was equally cheap and would have caught it instantly in both
cases.**

⇒ "I checked" is not the bar. **A check built on a differentiated noisy signal can invert the
answer.** For any geometric property, prefer a FIT over an accumulated derivative: circle fit
(Kåsa) for curvature, total displacement for travel, endpoint angle about a fitted centre for
sweep. Cross-validated: two sessions, same wrong metric, same wrong conclusion, and one
cheap robust metric (r_fit: July 0.87-0.88 m vs Sep 9.9-13.9 m, n=30) settled it outright.

**The deeper failure:** an experiment is code + parameters + **scenario**. Worktrees and git
bisect reconstruct the first two. Section 11 says "a worktree rebuilds the CODE, not the
EXPERIMENT" and I still only checked out-of-repo *assets* (camera SDF) — never the
*trajectory the test was driving*, which was in-repo the whole time and env-overridable.

**Corollary for endpoint validation (§11):** when a historical endpoint fails to reproduce,
that means EITHER out-of-repo state OR **a mis-placed anchor** — I picked the July anchor by
date (`--before "2026-07-03 23:59"` → `edb546f0` at 23:30) which sat **11 hours AFTER** the
12:13 change, then concluded "not in the repo". Anchor to the DATA's own timestamp (the run
directories were named `Fri Jul  3 11-14…11-48`, i.e. pre-noon), not to the end of the day.

## 17. Harness infers "run succeeded" from a POST-RUN directory listing alone

**2026-09-17:** a multi-rep A/B harness detected the newest output directory with
`d=$(ls -dt "$out"/*/ | head -1)` AFTER each rep and treated any non-empty result as success.
One rep's SITL launch crashed on attempt 1 (non-retriable, no output written) — but the
directory query still returned the PREVIOUS rep's still-newest directory, so the harness
silently re-analyzed old data under the new rep's label. Two "reps" printed identical numbers
before this was noticed.

**The check:** snapshot the newest-directory query BEFORE the run starts, and after the run
compare the new query against that snapshot — `before=$(ls -dt ... | head -1)` pre-run,
`[ "$d" = "$before" ]` post-run means NO new data, not `[ -z "$d" ]` alone (which only catches
the case of zero directories ever having existed, not a failed run that left old ones behind).
Same family as §2/§13 (don't trust a directory listing without a positive freshness check) but
the failure mode is different: not stale content pairing, but a harness that can't tell "this
rep produced nothing" from "this rep succeeded."

## 18. ROVER launcher defaults to ArUco, not cross-marker — a whole session ran the wrong world

**2026-09-17:** `scripts/run_rover_landing.sh` defaults to `WORLD="${WORLD:-rover}"` and
`ROVER_MODEL="${ROVER_MODEL:-rover_aruco}"` — the plain ArUco rover — with `MARKER_TYPE`
unset (ArUco perception). None of that session's curve-validation harnesses
(`r08_confirm.sh`, `curve_qgate_ab_r08.sh`, `cycle_isolation`, `curve_worktree_confirm.sh`)
set `WORLD`/`ROVER_MODEL`/`MARKER_TYPE`, so **every rover curve test that session — the
`ROVER_CIRCLE_R` discovery, the whole QGATE re-validation, the cycle-isolation sweep, the
`d380901c` worktree test — silently ran on the ArUco rover**, not cross-marker, even though
the project's live default is cross-marker (`feedback_recurring_analysis_mistakes` §
"Also worth screening for", 2026-08-25 entry: the same trap already happened once for the
STATIONARY launcher's `WORLD=cross_marker MARKER_TYPE=cross` rule).

This is subtler than the stationary case: under `PLASMC_GT_FEEDBACK=1` most channels
(`s`,`h`,`h_z`,`yaw`,`w_z`) are synthetic and marker-agnostic, so a GT-FB run "looks" fine
regardless of marker — nothing errors, nothing looks obviously wrong. But
`PLASMC_AU_LEAD_QGATE`'s extent term reads **live** `MARKER_EXTENT_PX` from the real
perception pipeline REGARDLESS of `PLASMC_GT_FEEDBACK` (GT-FB replaces s/h features, not the
detector), so any gate/threshold tuned or validated under GT-FB can still be silently
marker-specific.

**The check:** for ANY rover-scenario run — GT-FB included — explicitly pass `WORLD=rover_cross
ROVER_MODEL=rover_cross MARKER_TYPE=cross` (the launcher's own header comment names this
exact triple) unless the ArUco rover is deliberately the target. Grep a harness for these
three vars before trusting its output; their absence means the launcher default silently
applied. Same rule, same enforcement gap, as the stationary `WORLD=cross_marker
MARKER_TYPE=cross` rule below — but now proven to also bite the MOVING-target launcher and
its own set of harnesses.

## Also worth screening for

- 2026-09-02: the recorded `_raw`/frames PNGs carry a **drawn debug overlay**
  (`CROSS_RING_OVERLAY_DBG` default ON) — live detection is unaffected but offline replay is
  contaminated. [[feedback_detector_offline_replay_gotchas]]
- Headline detOK%/SP% are inflated by post-touchdown ground frames — read per-ALTITUDE bands.
- 2026-08-25: a correctly-worded HARD RULE buried mid-file (`WORLD=cross_marker
  MARKER_TYPE=cross`) was violated across an entire session once a command pattern got
  copy-pasted. Rules need to fire at the point of action, not sit in a file.

## 19. STATIONARY launcher also defaults to ArUco, not cross-marker — sibling to §18

Continues §18 (rover launcher, `4ba07bb8`, found ~1 hour earlier the same day by another
session). The identical defect exists on the **stationary** path:

- `src/controller.py:73`: `MARKER_TYPE = os.environ.get("MARKER_TYPE", "aruco")` — default
  is literally `"aruco"`.
- `scripts/run_landing.sh:23`: `WORLD="${WORLD:-aruco}"` — same default, unchanged since WORLD
  became overridable (2026-08-11).
- The 2026-09-03 rename commit (`99367421`) **asserts in its own message** "WORLD/MARKER_TYPE
  are env-driven and the standing rule makes every run cross-marker" — that claim does not
  match the code and never has. It is the origin of the false belief.
- `scripts/run_ic_validation.sh` — the canonical IC2-5 gate script — **never sets
  WORLD/MARKER_TYPE**. Every run through it silently uses ArUco unless the caller's shell
  happens to have them exported.

**Cost this time:** an IC1-5 gate for a genuine code fix (`96271ba6`) came back 0/25 precise
and looked like a catastrophic regression. A same-day OLD-vs-NEW A/B (correctly following
§10's "validate the bisect endpoint" rule) showed OLD failed identically — which is what
correctly stopped the fix from being blamed, but the *actual* cause (wrong marker/detector,
not environment drift) required one more level of digging: comparing `Img_Data.npy` KEYS
between the "good" and "bad" bundles. `FEATURE_IS_VISIBLE`/`Detection Status`/`Fail
Reason`/`MARKER_EXTENT_PX` are logged only by `cross_marker_perception.py`; `Centroid Map
Raw`/`Ring Opt Flow Ang Vel`/`Alpha Map Raw` only by `img_data.py` (ArUco). The "good" Sep-12
baseline had the former; the "bad" Sep-17 runs had the latter — proving the marker/detector
itself differed, not just the environment. ArUco's sensor cal is documented in CLAUDE.md as
NOT recalibrated for 320x240 (stale since 2026-07-17 at 640x480/fx=270) — exactly enough to
explain a marker-alive collapse identical in both code versions.

**Check, added to the standing checklist:** for ANY stationary landing run, gate script
included, verify `WORLD=cross_marker MARKER_TYPE=cross` is actually set — don't trust a
launcher's rename-commit comment or a script's filename. Fastest verification is NOT to grep
the launch command (a shell export won't show there) but to **check the resulting
`Img_Data.npy`'s own keys**: `FEATURE_IS_VISIBLE`/`Fail Reason`/`MARKER_EXTENT_PX` = cross-
marker; `Centroid Map Raw`/`Ring Opt Flow*`/`Alpha Map*` = ArUco. This is authoritative
because it's recorded by whichever module actually ran, unlike an env var that may have been
set in a shell you can't see.

**Fix needed** (not yet done): `scripts/run_ic_validation.sh` should set
`WORLD=cross_marker MARKER_TYPE=cross` explicitly rather than relying on caller-shell state,
matching the fix direction `4ba07bb8` recommends for the rover launcher.
