# Memory Index — CORE (auto-loaded every session)

> **MIGRATED 2026-07-02 (was the frozen legacy index, 238 lines/58 KB — it exceeded the
> auto-load budget and was truncated at session start).** The live per-phase indexes are
> **`px4/MEMORY.md`** (PX4/SITL phase — read its top banner first), **`matlab/MEMORY.md`**
> (MATLAB sim + manuscript, incl. the migrated writing-convention block), and
> **`shared/MEMORY.md`** (cross-cutting). Legacy pre-06-19 topic files remain flat in
> `Memory/` — their index hooks now live in the per-phase LEGACY sections; when hunting any
> topic, `grep -il <topic> Memory/ -r`. The full pre-shrink index is in git history (d01c5c0).
> This file keeps ONLY the cross-cutting rules below — one line each, ≤~200 chars; new
> entries go in the per-phase indexes, never here.

> **⛔ Standing correction (2026-06-26, condensed):** any pre-06-21 claim that the PX4 lateral
> wall is "perception / architecture / inner-loop-velocity / LK-dynamic-range, not gain-tunable"
> is SUPERSEDED (gain-parity bug + velocity damping + the Z_REG=0.01 GT-FB harness artifact;
> stamped in the files). Live truth: `px4/MEMORY.md` top banner.

## Hard rules & cross-cutting methodology
- [Always use default params](feedback_use_defaults.md) — ask before any non-default env var; stale comments (e.g. REF_RAD=-0.70) may not apply under current cal
- [Sim platform: MATLAB done, PX4/Gazebo active](project_simulation_platform.md) — disambiguate "the simulation" between the two
- [Dual-platform development](user_platform.md) — user works from Ubuntu AND Windows; verify live env before assuming paths
- [Terminal newline = Alt+Enter (Cursor)](feedback_terminal_newline.md) — don't remap Shift+Enter / Ctrl+J
- [PX4/Gazebo Python lives in git project](feedback_px4_python_location.md) — copy from ~/ws/, edit copies; never edit ~/ws/ in place
- [Don't suggest thrust+torque refactor](feedback_thrust_torque.md) — rejected; stay in rate-mode MAVSDK
- [Always validate IC2-5 before defaulting](feedback_ic_validation.md) — IC1 wins regress off-center; run_ic_validation.sh mandatory
- [Sweep methodology](feedback_sensitivity_sweep_methodology.md) — n=1 is noise; trust direction-of-effect; validate singletons at n≥5 before stacking
- [Historical record CAL-CONTAMINATED](feedback_historical_cal_confound.md) — ~2000 pre-June reps at 2-13× broken cal; "lag is the floor" confounded; timing numbers survive. The meta-memory that supersedes the contaminated phase/precision/coord conclusions
- [HARD CONSTRAINT: scale-free & depth-free](feedback_scale_free_depth_free.md) — no Z/altitude/metric in control law or mode switching, ever; truth only for test setup+eval
- [Fix saturation CAUSES not limits](feedback_fix_causes_not_limits.md) — analyze_saturation_audit.py every batch; funnel width=gain (never widen to make room)
- [Per-axis tuning (not uniform)](feedback_per_axis_tuning.md) — image-x 1.39× hotter than y; impact axis = explosion axis every rep (bulk numbers are a flagged contamination-correction record)
- [Log PX4 runs in parameter_record.ods](feedback_parameter_record_logging.md) — odfpy + backup before edit; git-tracked via carve-out
- [Clamps are band-aids](feedback_clamps_during_tuning.md) — disable during tuning, re-engage after; _ie_a_clamp REMOVED→conditional integration; A/B/C audit in file
- [False SP — frozen-GT artifact + no genuine R3 SP](feedback_false_sp_frozen_gt.md) — SoftPrecise can fire on a frozen→origin-reset GT (xy_err~1e-21); 1 found (BootstrapFix rep6); NC47e "0.03m SP" unverified; sanity-check SPs vs trajectory
- [TESTING RULE: reject on a SINGLE failed landing](feedback_reject_on_single_failure.md) — stop on first TL/crash; a single failure reveals the binding constraint — fix that first
- [METHODOLOGY: don't conclude "lag floor"](feedback_dont_conclude_lag_floor.md) — while a safety-net masks under-tuned control; one knob one job (P=κ-bound, E=stiffness)
- [Fix shared MATLAB+Python bugs in Ubuntu](feedback_shared_issue_fix_in_ubuntu.md) — 2026-06-14; issues in both codebases get corrected on the SITL-validated Python side, not tuned blind in MATLAB
- [Test new formulation before updating manuscript](feedback_test_new_formulation_before_manuscript.md) — 2026-06-16; new (blended-surface) formulation tested in control_formulation.tex + code FIRST; VALIDATED 2026-06-16 → gate CLEARED (manuscript port unblocked, still need user go-ahead); principle holds for future unvalidated changes
- [Show drafts before applying for non-trivial prose rewrites](feedback_drafts_before_apply.md) — Locked 2026-05-10; for multi-sentence rewrites / per-axis explanations / theorem-justification additions, present draft in chat first and wait for confirmation; mechanical fixes (single-word swap, `;`→`.`) still apply directly
- [Shift before delete](feedback_shift_before_delete.md) — Move content to supplement first; only deduplicate in a second user-approved pass
- [Backup discipline](feedback_backup_discipline.md) — Merged: code/config → Obsolete/<subdir>/<name>_vN.ext before EVERY edit (git-show-HEAD fallback for tracked files); active tex → Drafts/<name>_vN.tex STRUCTURAL edits only (sentence-prose waiver 2026-06-10); Literature_Review_Baselines.xlsx edited in place (exception)

## Where everything else went (2026-07-02 shrink)
- PX4-phase topic hooks → `px4/MEMORY.md` § "Legacy flat-file index".
- Manuscript/MATLAB hooks (naming epochs, prose rules, comparison history) → `matlab/MEMORY.md` § "Legacy manuscript-side index".
- Superseded/contaminated-era lines were DROPPED from the indexes; the topic files stay on disk with their own ⛔ stamps: feedback_convergence_ordering, feedback_coord_descent_sp_lucky_ic, feedback_descent_softness, feedback_dh_d_overload_lpf, feedback_dterm_outer_funnel_analysis, feedback_fov_cone_clamp_deadlock, feedback_ic2_lateral_gain_chain, feedback_instability_mechanism, feedback_krp_pz_ic2to5_regression, feedback_lateral_wall_anti_restoring_au, feedback_newcal_tuning_results, feedback_optic_flow_underreports_root, feedback_phase1_matlab_baseline, feedback_phase3_ic_robustness, feedback_phase4_sensor_noise, feedback_plasmc_two_task_framework, feedback_sen_authority_analysis, project_current_state, project_stacked_barrier_backstepping.
