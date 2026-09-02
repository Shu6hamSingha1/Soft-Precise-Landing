<!--
  PLASMC PARAMETER-TUNING GUIDE — the single entry point for any tuning / landing-diagnosis work.
  A SessionStart hook (.claude/settings.json) injects the STATUS block below into every session and
  tells the assistant to read this whole file before tuning. KEEP THE STATUS BLOCK CURRENT — update it
  (and the rest) at the end of every tuning round. This file is the orientation layer; it points to the
  deep docs (§11) rather than duplicating them.
-->

# PLASMC Parameter-Tuning Guide — START HERE

<!-- STATUS:BEGIN -->
## ⏱ STATUS (last updated 2026-09-02 — keep current)

> **Scope note.** This block is injected into every session, so it stays SHORT: current state +
> standing facts + pointers. Session narratives live in the memory topic files linked from
> `px4/MEMORY.md` — do not paste them back in here. Anything below marked ⚠ is unverified or
> known-stale; read the named source file for authoritative values.

### Where the project is
- **🟢 STATIONARY cross-marker perception landing WORKS for IC1-5** (2026-08-31/09-01). Root cause
  of the prior 100% TARGET_LOST was a stale `CROSS_ALPHA_0` (90.23° → re-derived `radians(0.58)`,
  commit `b963e207`). Gate `test_data/ICValidation/20260831-144626` (n=5): IC2/3/4 ~0.10 m mean,
  3/5 precise each; IC1 4/5; IC5 4/5 ≤0.14 m. Curated IC1-5 videos+datasets in `test_data/Final/`.
  → [[project_20260831_perception_mode_landing]], [[reference_final_landing_recordings]]
- **🔴 MOVING rover_cross landing does NOT work yet** (2026-09-01/02): 0/5 static, all moving reps
  stall or fly off. **NOT a code regression** — flat `cross_marker` IC1-5 still land dead-centered
  on the same HEAD, and `controller.py`/`cross_marker_perception.py` have ZERO rover conditionals.
  The failure is what the shared code is FED. Two clusters:
  - **A (offset IC2/3/5): oblique-view detector collapse** — `centroid_mismatch` from ~5 m (rover
    wheels + the 3 m marker overhanging the 0.6 m platform).
  - **B (IC1/4): terminal overfill loom collapse**, descent stalls ~1.1 m (detector fine; same as
    the #1 stationary blocker, +0.6 m higher).
  - Control-side terminal stall: single-frame centroid spike `s_x` 0.16→0.91→0.16 → `dh_d` ±11 →
    `theta_desired` 92° → CBF FoV clip → joint-QP folds ~5 m/s² UP into `I_a[2]` → `B_T`→0 → stall.
  → [[project_20260901_rover_cross_perception_diagnosis]]
- **#1 OPEN blocker (both scenarios): terminal overfill.** Below ~0.5 m (stationary) / ~1.1 m
  (rover), `MARKER_EXTENT_PX` saturates the frame and LK lateral flow `h_y` corrupts *coherently*
  (smooth 0→+1.3 ramp, ~150 corners still "tracked", no gate fires) → middle-loop `sigma = h − h_d`
  explodes → attitude/thrust kick.
- **Active thread: cross-marker DETECTOR robustness.** Must key on **CONTRAST-to-background, not
  darkness** — `inRange(V≤100)` fails on ANY dark object near the marker (proven: one dark box →
  IC2 detOK 100%→10%). GT-scored offline harness: `tools/validate_detector_gt.py` (`--set`,
  `DetectorFrameset` eval set). Latest fix: fill-ratio band on `_isolate_marker_by_shape`
  (`ee858086`). → [[feedback_cross_detector_contrast_not_darkness]]

### Standing facts (verify in-file before relying on any of these)
- **Platform:** PX4 SITL + Gazebo Harmonic, `x500_mono_cam_down`. **Down-cam is 320×240,
  fx=fy=135, hfov=1.74** (dropped from 640×480/fx=270 on 2026-08-27 to recover frame rate; same
  hfov, so normalized coords and PLASMC gains are unchanged and now match MATLAB `Constants.m`
  exactly). Verified live in `mono_cam/model.sdf` + `img_data.py:49` on 2026-09-02.
  ⚠ **`CLAUDE.md` still says 640×480/fx=270 in 3 places — that is stale.**
- **⚠ TWO perception paths with SEPARATE calibrations — do not mix them up:**
  - **cross-marker** (`src/cross_marker_perception.py:797`) — **RECALIBRATED for 320×240 on
    2026-08-28** from GT-FB landing recordings. Near-diagonal; trustworthy for a nominal ~5-6 m
    descent (leave-one-out R² h_z +0.95..+0.99, h_x/h_y +0.85..+0.94). **KNOWN GAP: degrades
    above ~6 m** (IC4 @7 m holds out weakly, twice) — the h-block scale is not altitude-constant.
  - **ArUco** (`src/img_data.py:186`) — cal is from **2026-07-17 at 640×480/fx=270 and was NOT
    redone** for 320×240. The file's own comment says this is **STILL BLOCKING** for real landing
    validation. So any ArUco comparison run right now is on a stale cal.
- **⛔⛔ HARD RULE — default every landing test to `WORLD=cross_marker MARKER_TYPE=cross`.** ArUco is
  COMPARISON-ONLY; use it only when the user explicitly asks. The launcher's own default is still
  `WORLD=aruco`, so this must be passed EXPLICITLY on every launch — re-verify the command carries
  it before each run. (This rule was violated repeatedly across a whole session in 2026-08-24/25.)
  → [[feedback_aruco_perception_scope]]
- **Baked defaults (`src/controller.py`)** — every value below re-read from the file 2026-09-02
  (x/y/z where per-axis): `K_rp=3.0 · K_ri=0.1 · K_rd=0.5 · gamma_s(XIS)=0.5 · PSINF=0.35 ·
  XI2=1.0/1.0/1.0 · P=2.5/2.5/5.0 · P2INF=2.5/2.5/0.5 · E=0.8/0.8/0.5 · Γ=2.0/2.0/1.0 ·
  Ω=0.1/0.1/0.1 · N=0.1/0.1/0.1 · KAPPA0=0.5/0.5/1.0 · KAPPA_MAX_Z=3.0 · KAPPA_MAX_XY=30 ·
  W_U_MAX=2.0 · YAW_PSID_RATE=1.0 · DH_D_MAX=50 · BODY_YAW_SOURCE=alpha · cbf2 · SEN_FUNNEL=1`.
  Notes: `P2INF_xy` is `pa()`-based 1.5 **rebaked to 2.5 at controller.py:466-467** when the env var
  is unset (2026-08-28) — the effective default is 2.5. `KAPPA_MAX_XY` 1e6→30 is a 2026-08-19
  hardware-parity port, **not yet Gazebo gate-validated**.
  > ⚠ **The 2026-07-09 block this replaces was WRONG on 7 of these** — it listed `P=1.5/1.5/5`,
  > `XI2=0.6/0.6/0.6`, `E=0.8/0.8/0.1`, `Ω=0.1/0.1/0.025`, `N=0.02/0.02/0.1`, `W_U_MAX=1.0`, and
  > omitted `KAPPA_MAX_XY`. Treat any tuning reasoning that leaned on those numbers as suspect.
  > Re-verify rather than trust this list after any bake:
  > `grep -oE 'pa\("[A-Z0-9_]+", *[0-9.e+, ]*\)' src/controller.py`
  ⚠ `KR` and `h_rd` appear in the old list but I could not locate their live defaults — check the
  file directly before relying on them.
- **Cross-marker-era knob defaults** (verified 2026-09-02): `CROSS_BG_FLOW=1 ·
  CROSS_CENTER_BRIDGE_FRAMES=10 · CROSS_S_JUMP_GATE=0.35 · CROSS_ADAPT_GATE=0 (do NOT bake — 27%
  of its extra detections are >20 px off GT) · PLASMC_TD_V2=1`.
- **⛔ CBF `Rz_p90b` "fix" (`ebb8093c`) was REVERTED (`4d7bc210`)** — a matched A/B showed it
  regressed EVERY IC (IC2 5/5 ≤0.19 m → 11 m fly-off). `validate_cbf.py` 13/13 was misleading.
  The 90° SMC↔CBF convention question stays **OPEN**; any rework must be validated IN SITL, never
  synthetic-only.
- **Hard rules:** never run SITL yourself unless the user explicitly asks for a `HEADLESS=1` run in
  this conversation (GUI-bound); use **default params** (ask before any non-default env var);
  **n≥5** per cell; **IC2-5 gate** before defaulting a gain/controller change (perception fixes are
  exempt); tune **per-axis**; reject on a SINGLE failed landing.

### Recent bakes / reverts (one line each — details in the linked memories)
- `ee858086` (09-02) fill-ratio band on `_isolate_marker_by_shape`; `8ed004ad` (09-01) detector
  robustness scaffolding + `tools/validate_detector_gt.py`.
- `b963e207` (08-31) `CROSS_ALPHA_0` re-derive — the fix that made perception-mode landing work.
- `4d7bc210` (08-31) revert of the CBF `Rz_p90b` change (see above).
- (08-29) cell-ID-independent `_prev_flow_uid` match key + `CROSS_CENTER_BRIDGE_FRAMES` 0→10:
  IC5 20/20 SP vs 12/18 with bridge off.
- (08-30) `_touchdownDetectV2` (`PLASMC_TD_V2=1`) — 3 paths (overfill / backstop / flow-freeze).
  ⚠ unit-tested only where it landed; IMU `_impactDetector` + PX4 LandedState remain the hard
  backstops. → [[project_20260830_perception_touchdown_detect_broken]]
- (08-27) camera 640×480 → 320×240; (08-28) cross-marker cal re-derived for it.

### Pre-August history
The ArUco-era tuning log (2026-06-10 → 07-09: `FLOW_FUSE_RING=0`, `MARKER_KLT_RELAX_GATE=1`,
s-extrapolation, the Z_REG GT-FB harness artifact, the lateral-wall/velocity-damping thread, the
combined-barrier gain-parity bug) has been moved out of this always-injected block. It is preserved
in git history (`docs/PLASMC_TUNING_GUIDE.md` before 2026-09-02) and indexed in `px4/MEMORY.md`.
⚠ Treat all of it as ArUco-era and pre-320×240: conclusions there may not transfer to the
cross-marker pipeline.
<!-- STATUS:END -->

## §0 — What this is & how to use it
This is the orientation layer for PLASMC tuning. At session start the hook gives you the STATUS block; for any real tuning or failure-diagnosis work, **read this whole file**, then pull live data (§2) and open the relevant deep doc (§11). Update §STATUS + the relevant section when a round finishes.

## §1 — The system in one paragraph
PLASMC lands a quadrotor by image-based visual servoing, **scale-free & depth-free** (no altitude/metric in the control law — hard constraint). **Outer loop** (Python PID): normalized pixel error `s_e_n` → desired optic-flow `h_d`. **Middle loop** (Python adaptive SMC): drives measured optic flow `h` to `h_d` through a log-barrier performance funnel with an adaptive gain `κ`. **Yaw** (Python SMC on the image `alpha` feature — compass-free). **Inner loop:** PX4's geometric controller (we ship body-rates + thrust over MAVSDK; PX4 does actuator allocation). NED frame throughout.

## §2 — How to gather the current data (run these to orient)
- **Parameter record (canonical):** `test_data/Landing_Test/parameter_record.ods`. `pandas` is absent — use odfpy:
  ```python
  from odf.opendocument import load; from odf.table import Table,TableRow,TableCell; from odf.text import P
  def cell(c): return " ".join(str(p) for p in c.getElementsByType(P)).strip()
  def row(r):
      o=[]; [o.extend([cell(c)]*int(c.getAttribute("numbercolumnsrepeated") or 1)) for c in r.getElementsByType(TableCell)]; return o
  doc=load("test_data/Landing_Test/parameter_record.ods")
  for t in doc.spreadsheet.getElementsByType(Table):       # sheets:
      print(t.getAttribute("name"))                        #   PX4_NewCal_Record (NC1-49 = current R3 era, the one to read)
      for r in t.getElementsByType(TableRow): print(row(r))#   PX4_Gain_Record (G1-60 = R1/R2 history) · MATLAB_Test_Record · Removed_Parameters
  ```
  Each NC/G row now carries **`Bundle` + `Timestamp`** provenance columns → maps a trial to its `test_data/` data.
- **Test data:** named bundles `test_data/<Name>/<YYYYMMDD-HHMMSS>/rep<N>/` (+ a `summary.tsv` per bundle) AND raw per-landing autosaves `test_data/Landing_Test/<Day Mon D HH-MM-SS YYYY>/`. Each rep dir: `Control_Params.npy` (the gains that ran), `Ground_Truth.npy` (`UAV/Target Pose`, `SoftPrecise`), `Control_Data.npy`, `Img_Data.npy`, `Telemetry_Data.npy`.
- **Memory (auto-loaded index):** `~/.claude/projects/-home-shubham-Soft-Precise-Landing/memory/MEMORY.md`. Key files: `reference_tuning_trajectory` (the whole campaign arc), `reference_docs_folder`, and the `feedback_*` lessons.
- **Code defaults:** `src/controller.py` (the `pa()` / `os.environ.get` block, ~lines 120–175) is the source of truth for baked values.
- **Git history:** tuning milestones are in the commit log (`git log --oneline -- PX4_Gazebo/src/controller.py`).

## §3 — Cal regimes (the master variable)
| Regime | When | Trust |
|---|---|---|
| R0 (broken) | pre-June | ~2000 reps at 2–13× mis-scale. **Confounded.** Only lag timing (38/52–61/287 ms) survives. |
| R1/R2 (multisine) | Jun 2–4 | the "28% SP @10cm" (G46) is a **hypothesis**, not a result. |
| **R3 (honest)** | Jun 5–10 | **the only trustworthy data** (NC1–49). |
A number from one regime does **not** transfer to another — the #1 historical analysis error.

## §4 — Parameter inventory
Knobs are **direct per-axis values** `PLASMC_<PARAM>_{X,Y,Z}` (all `*_SCALE` factors removed 2026-06-03). Current baked defaults are in §STATUS. For the full per-parameter role/empirical analysis, read **`docs/PARAMETER_ANALYSIS.md` §3**; for the complete env-knob table + sweep methodology, the **`tune-plasmc` skill**. Groups: Outer PID + outer funnel (`KP, KI, KD, XIS/gamma_s, PS0, PSINF, DH_D_MAX, TAU_DS`); middle SMC (`XI2, P20, P2INF, OMEGA, GAMMA, E, N, P, KAPPA0, KAPPA_MAX`); yaw (`YAW_*, KR_YAW, PSID_RATE, TAU_UA`); visibility (cbf2 CBF — `THETA_FLOOR, RHOFOV*`); inner (`KR_*, W_U_MAX`); image (`IMG_FEATURE_FILTER, MARKER_KLT_MAX_STEPS, ARUCO_*, BODY_YAW_SOURCE, CTRL_ZERO_WXY`); landing-test (`LANDING_REF_RAD_OPT_FLOW, LANDING_IC_*`).

## §5 — Failure modes & the κ-runaway explosion chain
Almost every catastrophe is one chain (detail in `PARAMETER_ANALYSIS.md` §2):
```
lateral drift → outer PID windup (h_d ≈ K_rp·s_e_n/G_s) → loom funnel can't track within p_2inf
   → barrier ζ→∞ → |σ|>E → κ-ODE runaway → a_u blowup → hard impact / fly-away / TARGET_LOST
```
Bounding rules: **P** bounds κ cleanly (`κ_eq∝1/P`); **E** bounds κ but softens tracking (one-knob-one-job: P for κ, E for stiffness); **N, Γ** can't fix it (math-excluded); **cbf2 masks it** (if the CBF bites in normal ops the control law is failing); **funnel width = gain** (never widen a funnel for a transient). The **current binding limit is stochastic LK/ArUco perception** (~2 m/s dynamic-range ceiling → 1–2 TARGET_LOST per 5 reps even at a perfect IC) — *not* gain-tunable. Lag (38 ms roll / 287 ms yaw) is the **last** explanation, not the first.

**θ_norm is contained, not eliminated.** θ_norm (= ‖Θ‖_F, drives `dκ/dt`, ~99% `cross(dw,s)`) still **spikes** (~480 baked, up to 1226 if E_z is narrowed) from a frame-jump `dw` artifact + close-range perception. The κ-cap (`KAPPA_MAX_Z=3`) + P-leakage contain it **downstream** — at the baked config it never even ran κ away. **Do not source-fix it via a `dw` rewrite** (tried 2026-06-10, net-negative: `dw` feeds the load-bearing `c` feedforward → *more* κ-runaways). Detail: `PARAMETER_ANALYSIS.md` §2.

## §6 — Methodology (the rules — violate at your peril)
1. **n ≥ 5** per cell — n=1 is noise (debunked repeatedly). 2. **IC2-5 gate** (`run_ic_validation.sh`, never pass it `LANDING_OUT_BASE`) before changing any default — IC1 wins routinely regress off-center. 3. **Per-axis**, not uniform (image-x is 1.39× hotter than y). 4. **One knob, one job.** 5. **Reject a value on a single TL/crash** (100% reliability needed; a single failure reveals the binding constraint — fix that first). 6. **Clamps are band-aids** — tune the bare law, re-engage protective clamps after. 7. **Sanity-check every SP** against its trajectory (§8). 8. **Use defaults; ask before non-default env vars.** 9. **Don't run SITL** — prepare the command, the user runs it.

## §7 — Known dead-ends & winners (R3)
**Dead-ends (don't retry):** `KI≥2` / `KI=0.35` · `KP≥13` · `KP=12 + E_XY=2.5` · `W_U_MAX>1.7` · `E_Z≥1.5` · `N_XY=0.05` · `N_Z=0.05` alone · `tau_ua=0.3` · `P_XY=3` · `TAU_DS=0.05` · `K_rd=0` alone (needs gamma_s=1.0) · `gamma_s=0.1` · `KR_YAW≠2` · `YAW_OMEGA=1` · `YAW_GAMMA=1` · RHOFOVINF/THETACAP terminal sweeps (frontier mapped) · `MC_*RATE_P>1` via MAVSDK · enlarging the marker · sensor-cal refresh via `aggregate_calibration.py`.
**Winners / baked:** `K_rd=0 + gamma_s=1.0` (the 1.32 m stack) · `KAPPA0_Z=1.0 + KAPPA_MAX_Z=3.0 + κ-freeze` (descent bootstrap) · `P=5/5/5` (κ bound) · yaw 2π-wrap + conditional `ie_a` integration · cbf2 · `BODY_YAW_SOURCE=alpha`.

## §8 — Data-integrity gotchas
- **Cal regime** (§3) — check it before trusting any number.
- **False SPs (frozen GT).** The `SoftPrecise` flag can fire on a degenerate GT: if `UAV Pose` freezes at the IC then resets to the origin, `xy_err`→~1e-21 trips `precise` with no real landing. **There is currently no verified SP in the saved R3 data** (the only sub-10 cm rep was this artifact — `Landing_Test/Wed Jun 10 01-22-38`, marked FALSE). Before trusting an SP: did the drone **descend** (min alt→~0)? Is `xy_err` physical (~0.02–0.06), not ~1e-21? Is the pose non-frozen? See memory `feedback_false_sp_frozen_gt`.
- **Provenance:** the `.ods` carries `Bundle`/`Timestamp`; some Jun-9/10 trials are loose autosaves bundled retroactively (`GammaS_sweep_n25`, `BootstrapFix_n21`, …).

## §9 — Diagnostic tools (`tools/`)
`analyze_explosion_chain.py <rep>` (first-exploding state + causal chain) · `analyze_saturation_audit.py --glob/--events` (limit duty cycle + per-event reason; the standard batch diagnostic) · `analyze_timeseries.py` (per-channel corr with xy_end) · `diagnose_failure_cause.py <rep>` (perception-vs-control at onset) · `analyze_impulse_response.py` (per-axis lag) · `analyze_sigma_compare.py` (MATLAB-vs-PX4 σ).

## §10 — Workflow for a new tuning task
1. Read §STATUS + this guide; pull the latest `.ods` (§2). 2. **Identify the failure mode** (§5) — if it's stochastic perception or lag, a knob won't fix it. 3. Pick **ONE** parameter; sweep **3–5 values** spanning [0.5×,2×], **n≥5** at IC1 (write the command, the user runs `run_knob_sweep.sh`). 4. Analyze (§9); reject on a single failure (§6). 5. If promising at n≥5 → **IC2-5 gate**. 6. **Update** the `.ods` (with provenance), the relevant memory, this §STATUS block, and `PARAMETER_ANALYSIS.md` if a default changes.

## §11 — Deep-dive pointers
- **`tune-plasmc` skill** (`/tune-plasmc`) — the tuning playbook: full env-knob inventory, sweep methodology, the 8 failure modes, known-bad/known-good configs, the diagnostic chain.
- **`docs/PARAMETER_ANALYSIS.md`** — comprehensive per-parameter analysis + the explosion chain + dead-ends + SP-flag integrity (honest-cal rewrite).
- **memory `reference_tuning_trajectory`** — the connected campaign timeline (every trial G#/NC#: what varied, why, outcome).
- **`docs/CONTROLLER_PARITY.md`** (MATLAB↔Python diff + intentional divergences) · **`docs/FUNNEL_CBF_DESIGN.md`** (cbf2) · **`docs/PERCEPTION_FLOW_FINDINGS.md`** (perception layer + LK dynamic-range lever) · **`docs/SH_REFERENCE.md`** (bash patterns).
- **`docs/TUNING_HISTORY_ARUCO_ERA.md`** — the 2026-06-10 → 07-09 dated tuning log, moved out of the STATUS block on 2026-09-02 so it stops being injected into every session. ⚠ Entirely ArUco-era and pre-320×240; read as history, not current state.
- **`parameter_record.ods`** — the raw quantitative record (4 sheets).
- **memory `MEMORY.md`** — the full index of `feedback_*`/`project_*` lessons.
