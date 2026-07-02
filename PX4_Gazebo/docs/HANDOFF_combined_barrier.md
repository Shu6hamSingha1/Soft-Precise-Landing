# HANDOFF — Combined-Barrier PX4 validation (2026-06-19; ✅ RESOLVED 2026-06-21)

> **⚠ STALENESS STAMP (2026-07-02 audit) — HISTORICAL; even the 06-21 FINAL VERDICT below has dated parts:**
> (1) its "OPEN: final IC2-5 gate" was overtaken by events (GT-FB campaign → Z_REG gear-floor fix →
> **19/25 SP across IC1-5** → rover phase; see `px4/MEMORY.md` top). (2) The listed bakes were re-baked
> later: `chi_r` 0.5→**1.5**, `XI2_xy`→0.7, `PRINF`→1.0, `HD_FUNNEL_REF/HD_KR` ON (787cf2d, 06-29);
> `h_rd` −0.42→**−0.30**, `P2INF_xy`→1.0 (486f713); `XIR`→0.10 (f068774). (3) The REJECTED-list anchor
> "SOFT-CONFIG Z on combined = CATASTROPHIC 5/5 fly" is a **STALE anchor** (measured on the pre-velocity-
> damping over-hot base — do not cite as binding). (4) `project_current_state.md` is itself historical;
> live state = `px4/MEMORY.md` top + the tuning-guide STATUS.

## ✅ FINAL VERDICT (2026-06-21) — COMBINED SURFACE WORKS; the wall was a GAIN-PARITY BUG
**The 2026-06-19 "does NOT beat the wall — perception-gated" verdict below was a MISDIAGNOSIS.**
- **Real cause:** PX4 combined-barrier mode was running the OLD **back-mapped soft-config gains**
  (GAMMA 2.0, KAPPA0 0.5, XI2 0.6) — **3–5× too HOT for the combined surface** → `a_u` over-aggression
  → fly-aways. The "ṡ under-reports ~0.5× → under-braked" reading was the over-hot gains amplifying the
  residual, NOT the binding limit.
- **Fix (commit 22cc732):** auto-align combined-mode gains to the MATLAB VDF-ASMC `vdf_params`
  (GAMMA 0.4375/0.5, KAPPA0 0.125, XI2 0.2) + `chi_r=0.5` (PX4-specific, vs MATLAB 0.85, for lateral-velocity arrest).
- **Result (IC2 n=5, manuscript gains):** **4/5 sub-meter** (|xy| 0.72–0.99, td_lat med 0.92), `h_e`
  100% bounded, `s_e_n`→0 (~96% inside `p_r`, mild terminal-1/Z breach ~1.05 in the last 0.3 m),
  vz_med 1.6, yaw e_a ~0.9°; **1/5 stochastic terminal-1/Z fly-away** (residual, not the wall).
- **Now BAKED DEFAULT-ON:** `PLASMC_COMBINED_BARRIER=1` + `PLASMC_VDS_KF=1` + `PLASMC_DHD_KF=1` +
  `PLASMC_DW_KF=1` + `chi_r=0.5` + `h_rd=-0.42` + `PLASMC_YAW_ALPHA_FILT=1`.
- **REJECTED levers (do NOT re-try):** P2INF chase-lag, terminal a_u-cap (COMMIT_AU_MAX), yaw KF,
  Omega_a/Gamma_a>0.5, SOFT-CONFIG Z on combined (CATASTROPHIC 5/5 fly — z not orthogonal to the
  combined lateral surface).
- **OPEN:** the final IC2-5 gate (config tuned/validated at IC2; must confirm IC3/IC4/IC5 hold).
  Records NC100–106. Authoritative state: `Memory/project_current_state.md` (2026-06-21) +
  `Memory/feedback_combined_surface_divergence.md` (corrected verdict at top).

⬇️ **Everything below is the SUPERSEDED 2026-06-19 perception-gated reasoning — kept for history.**

## ⛔ [SUPERSEDED 2026-06-21] VERDICT (2026-06-19) — COMBINED SURFACE DOES NOT BEAT THE WALL (perception-gated)
- **GT-check (decisive):** the ṡ feeding the blended `h_d` under-reports the dominant lateral velocity —
  ~0.8× while slow, **~0.5× during the high-velocity overshoot** (centroid-rate 0.48 ≈ LK 0.52, both
  saturate = LK dynamic-range ceiling, not filter noise). Under-braked → overshoot.
- **IC2=(2,2,5) A/B, n=5 each** (`scripts/run_cb_ic2_ab.sh`, 15/15 ran, 0 flake):
  backmap median fin_lat **13.55** (5/5 TL) · combined χ_r=0.85 **13.58** (5/5 TL, SAME wall) ·
  chir χ_r=1.3 **24.81** (5/5 TL, WORSE — κ-runaway 5–9). s_e_n converges first (min 0.07–0.59) then
  diverges to 1.5–2.3 in every rep → structurally sound, brake-starved.
- **[WRONG] Conclusion:** combined surface is structurally right but blocked by the inner-loop velocity front-end
  (same wall as the back-map). **χ_r↑ rejected.** Real lever = perception (KLT/pyr-LK) = architecture.
  `controller.py` stands as paper↔code alignment (default-off), NOT a wall-breaker. See
  `Memory/feedback_combined_surface_divergence.md` (verdict at top). Below = the pre-verdict state.
  — *This conclusion was overturned 2026-06-21: the A/B above ran the OLD over-hot back-mapped gains
  in combined mode (the gain-parity bug); with VDF-ASMC gains the fly-aways vanish (see FINAL VERDICT).*


> Continuation point for a fresh chat. `MEMORY.md` + `CLAUDE.md` auto-load; the freshest detail lives in
> `Memory/feedback_combined_surface_divergence.md` (recall it). This file is the one-page state.

## The thrust
Validate the **COMBINED-BARRIER sliding surface** (the manuscript's combined surface, §III) ported to PX4
`src/controller.py`, to fix the **lateral wall** (drone converges then overshoots → TARGET_LOST). The position
barrier `ζ_r` enters σ directly (`σ_k = ζ_h_k + χ_r·ζ_r_k`), replacing the SEN back-map (which had the
`G_s⁻¹→0` demand-starvation). Manuscript design + Lyapunov done; MATLAB impl `a152479` (25/25 SP).

## Code state (DONE, uncommitted)
- **`PX4_Gazebo/src/controller.py`** — 9 edits, env-gated **`PLASMC_COMBINED_BARRIER` (default-off)**, aligned to
  MATLAB `a152479` "blended sliding surface". py_compile clean; default-off byte-identical; combined-math finite.
  - ζ_r on FoV-normalized `s_e_n` (= s_e/p_10; p_10 IS φ_max) with its OWN funnel `p_r` (1.2→1.0 FoV-consistent,
    ξ_r=0.10) — NOT the SEN `p_s`. Built in `_updatePerfFunc` + `_updateImgFeatureParam`.
  - `h_d = [s_dot_meas; 0] + transport + descent` (blended; measured ṡ, NO back-mapped ds_d). `s_dot_meas` =
    `smooth4(finite-diff of s_e[:2])`. `dh_d` differentiates `h_d_noS` (drops s̈; κ absorbs it). New member `self._h_d_noS`.
  - σ lateral = ζ_h+χ_r·ζ_r, descent = ζ_h+Ω_z·∫ζ_h; `chi_zeta_aug=[χ_r·ζ̇_r; Ω_z·ζ_h3]` into Θ/u_eq/a_v.
  - Baked combined defaults: `χ_r=0.85`, `p_r∞=1.0`, `p_2∞_xy=0.5` (auto in combined mode). Knobs:
    `PLASMC_CHI_R_X/Y`, `PLASMC_PR0_X/Y`, `PLASMC_PRINF_X/Y`, `PLASMC_XIR_X/Y`.
- **Backups:** `src/controller.py.bak` + `src/controller.py.HEAD.bak` — **DO NOT COMMIT these.**
- **NOT committed** — pending SITL validation. Commit `controller.py` only (exclude `.bak`) when the user says.

## First SITL result (IC1, combined ON, headless smoke, n=1) — the wall RECURS
Run dir: `test_data/CombinedBarrier_smoke/Fri Jun 19 00-11-24 2026/` (Control_Data.npy = all signals).
Code SITL-STABLE but **flew away: xy=6.87 m, vel=4.84 m/s at IC1** (centered — back-map lands this). Combined
confirmed engaged (`p_inf=[0.5,0.5,0.5]`). From the authority/breach diagnostic:
- **s_e_n converges (min 0.01@1.3s) then diverges** (final 2.06, max 4.19); breaches p_s@5.9s (38%), p_r@8.1s
  (20%). Converge-then-overshoot. → authority converges it but **can't keep it bounded**.
- **h_e bounded in p_h the whole flight (0% breach)** — but **trivially**: blended `h_d` makes `h_e≈0` by
  construction → the flow funnel is **decoupled** from position. Don't trust h_e-boundedness as progress.
- σ→8.6, κ_y→4.54, |a_u_xy|→1076 m/s² (anti-restoring, corr(a_u,−s_e_n)≈−0.5).
- **Root hypothesis:** the combined `h_d` depends on the MEASURED ṡ; SITL's LK flow **under-reports velocity** →
  `h_d` under-commands the brake → overshoot. **Same inner-loop velocity wall, now inside the combined surface.**
  MATLAB (accurate ṡ) = 25/25; SITL = perception-gated.

## Next steps (open)
1. **GT-verify the ṡ under-report** — measured flow vs `tools/gt_optical_flow.py` (compute_gt_flow). **Decisive.**
2. **Cleaner ṡ:** the PX4 port uses `smooth4(finite-diff)`; the MATLAB has a Savitzky-Golay derivative
   (`CB_SDOT_FILT`) giving a clean ṡ + s̈ from one fit — **not ported**. Add it if ṡ-noise compounds.
3. **χ_r↑** (force ζ_r authority earlier) + the **IC2 A/B** (n≥5; the wall is at IC2=2,2,5 — IC1 already shows it).
4. If perception-gated is confirmed: the combined surface is structurally right but blocked by the **inner-loop
   velocity front-end** → real lever = perception (KLT corner-track / pyramidal LK) = architecture, not control.

## How to run SITL (user authorized running it; headless)
From `PX4_Gazebo/`, env is free, 16 cores:
```bash
HEADLESS=1 PLASMC_COMBINED_BARRIER=1 INITIAL_DRONE_ENU=2.0,2.0,5.0 \
  LANDING_OUT_BASE=$PWD/test_data/CombinedBarrier_IC2 \
  taskset -c 6-15 bash scripts/run_aruco_landing.sh        # IC1=0,0,5 ; IC2=2,2,5
```
~3–4 min/run, ~50% flake (lockstep race) — use `scripts/run_aruco_landing_retry.sh`. Combined engaged ⇔
`Control_Params['p_inf'][:2]==0.5`. GT has `Pose` objects (use the gt tool, not raw float()).

## Conventions (load-bearing)
- **MATLAB code OFF-LIMITS** (Windows-owned) — PX4-side only. The MATLAB combined-barrier is `a152479`.
- **Back up before editing any file** (user directive 2026-06-18).
- Default params; ask before any non-default env var. n=1 is noise — confirm at n≥5 before concluding.
- Memory is now git-tracked in `Memory/` (symlinked from ~/.claude); cross-machine via the public repo.
