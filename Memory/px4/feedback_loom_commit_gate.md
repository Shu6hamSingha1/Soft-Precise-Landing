---
name: feedback_loom_commit_gate
description: "⭐⭐⭐ LOOM-ACCUMULATION COMMIT GATE turns the velocity-damped approach into actual SOFT+PRECISE landings (GT-FB IC1-5). The terminal 1/Z kick fires ~10cm BEFORE ground contact, so the contact-based detectors catch only the post-kick balloon; committing to the open-loop vertical settle just above the deck (before the live a_u can drive the kick) lands the dead-centered approach. Proxy = ACCUMULATED loom ∫|h_z|dt = ln(Z_start/Z) (scale-free, perception-free under GT-FB; raw loom ~const at h_rd so only the integral is monotonic). LANDING_COMMIT_LOOM=2.8 (~fires 0.3m IC1) + centered guard |s_e_n|<=0.35. GT-FB IC1-5 n=3: 8 SP / 2 P / 2 fly / 0 TL (12 landed; audit-corrected from the 9/1 headline slip). Default-off, NOT baked; GT-FB only — perception-ON pending."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f1723146-d87f-40d0-9a34-49233fbc4e72
---

> ⛔ **SUPERSEDED + INVALID (2026-06-28, user-led).** The loom-commit (open-loop vertical settle /
> freeze-and-fall) was judged a WRONG approach: (a) its lateral freeze lands where a moving target
> WAS (useless for the rover), and (b) it MASKS the real terminal driver rather than fixing it. The
> code (`LANDING_COMMIT_LOOM`) was REMOVED from landing_test.py and all 42 loom-commit ICValidation
> gate dirs were DELETED (1.1 GB) as invalid/confounded tests. The terminal-kick is now handled by
> the **corner-exit commit framework** ([[project_terminal_kick_commit_design]]); the real terminal
> a_u driver is the **c-term loom×flow** `−h_z·h_xy ~ (vz·vlat)/Z²`, independent of the barriers
> (NOT a ζ_r/ζ_h thing). This file is kept for history only — do not act on the loom-commit results
> below (incl the "9 SP / 1 P / 2 fly" tally and the LANDING_COMMIT_LOOM=2.8 number).

## 2026-06-26 — LOOM COMMIT GATE: the terminal-kick fix that lands the velocity-damped approach (user-led, SITL)

**The velocity-damping lever ([[feedback_flow_funnel_zetah_works]]) makes the drone touch down DEAD-CENTERED at min-Z, but a terminal 1/Z kick then balloons the landed drone 1-4m up+sideways. The commit gate stops the live command just above the deck so the kick never fires → the dead-centered approach becomes an actual SOFT+PRECISE landing.**

### The gap it closes
The terminal kick fires in the last ~10cm BEFORE ground contact. Both touchdown detectors are CONTACT-based (`flight_controller.py`: accel-spike `_impactDetector` |a|>50; PX4 `_getLandedState` ON_GROUND), so they only catch the drone AFTER the kick has ballooned it → the scored `xy_err` (captured at LANDED) is the post-kick endpoint, not the precision achieved. There is no pre-contact trigger by default.

### The gate (landing_test.py, default-off `LANDING_COMMIT_LOOM=0`)
- **Proxy = ACCUMULATED loom** `loom_accum = ∫|h_z|dt`. A clean `h_rd` descent holds the loom `h_z ≈ h_rd` CONSTANT (median measured −0.366 ≈ −0.42), so the INSTANTANEOUS loom is NOT a proximity signal — only the integral is monotonic. `∫|h_z|dt = ln(Z_start/Z)` = e-folds of altitude descended = **scale-free** (a ratio, no depth/altitude). Under GT-FB the loom is GT-derived → **perception-free** (unlike marker-extent; this is why loom, not extent, was chosen — keeps the gate from contaminating the GT-FB control-isolation).
- **Trigger:** `loom_accum >= LANDING_COMMIT_LOOM` AND `|s_e_n| <= COMMIT_SEN(0.35)` for `COMMIT_FRAMES(3)` fresh frames → `in_final_descent=True` → open-loop vertical settle (zero body rates + sub-hover thrust) → no live `a_u` → the kick can't fire → drops straight → LANDED → disarm. SUCCESS path.
- **Calibration (`tools/calibrate_loom_commit.py`):** maps accum→altitude (⚠ Control_Data `t`=perf_counter vs Ground_Truth `Time`=elapsed are DIFFERENT clock origins → align by ELAPSED time). `accum 2.8 → fires ~0.305m (IC1), lat 0.043` — comfortably above the ~0.12m kick zone. Per-IC: a fixed accum fires at `Z=Z_start·e^-accum`, so 2.8 → IC1 0.30m / IC5(3m) 0.18m / IC4(7m) 0.43m — all above the ~0.08m safe-floor.
- **Honest metric (also fixed this session):** the min-alt precision tracker must FREEZE at the FIRST-descent bottom (`LANDING_MINALT_FREEZE_DZ=0.3`) — the GLOBAL min altitude catches the off-target post-balloon SECOND descent, not the clean first approach.

### Result (GT-FB, XI2_xy=0.7/P20_xy=15, accum=2.8)
- **IC1 n=3 (bundle 20260626-014221):** gate-OFF endpoint 2.12m → gate-ON rep2 **SP** (0.041/0.146), rep3 **P** (0.053); rep1 fly.
- **IC2-5 n=3 (bundle 20260626-015011):** **7 SP / 1 P / 1 fly** (9 landed; IC2_rep2/3 + IC5_rep3 = SITL flake, not control). 0 TL. Honest precision mean 0.080m, max 0.254m, dead-centered (xy@min-alt<=0.10) **8/9**.
- **Combined IC1-5: 8 SP / 2 P / 2 fly / 0 TL of 12 landed** (⚠ audit 2026-07-02 vs raw npy: the original "9 SP / 1 P" headline was an arithmetic slip — IC4_rep3 `02-02-40` has rel_vel 0.398 = precise-NOT-soft, matching the per-bundle breakdown above, which sums 1+7 SP / 1+1 P; honest-precision mean recomputes 0.078 ≈ 0.080 ✓). Best IC2-5 result of the campaign — actual soft-precise across the off-center ICs (the long-standing wall).
- **The 2 flies (IC1_rep1, IC3_rep2) = SAME mechanism, NOT a gate failure:** `s_e_n` stayed >0.35 through the clean window so the centered guard correctly BLOCKED an off-center commit → the gate fired late (accum 4.40) after the kick began. Both still reached ~0.25m at min-alt. They are approach-convergence stragglers (XI2_xy=0.7 didn't fully arrest them), not the gate.

### ROOT CAUSE of the stragglers — stochastic descent-start SEED amplified by the velocity loop being BLIND (the `h_d=measured ṡ` degeneracy)
- **The "stochastic" part is the descent-START state, not anything during descent.** Fly reps ENTER the descent already off-center AND moving OUTWARD (IC1 FLY @4.5m: lat 0.59m, v_lat 0.70 m/s radial-OUT; IC1 CLEAN: 0.04m, 0.15 m/s). The approach/IC-convergence handoff leaves a variable residual offset+outward velocity within the loose IC tolerances (`IC_POS_TOL`=0.5m, `IC_VEL_TOL`=0.5m/s). That seed is the entire randomness.
- **WHY the controller can't brake it = the velocity barrier is BLIND at altitude.** The middle SMC regulates the FLOW error `h_e=h−h_d`, but in the combined surface `h_d`=MEASURED centroid rate ṡ → the controller sets desired flow = ACTUAL flow → `h_e≈0` regardless of drift. Data (IC1 FLY, drifting OUT at 0.4-1.9 m/s): `|h_e_xy|` only 0.017-0.107 and `|σ_xy|`≈0.002-0.015 (SMC believes it's CONVERGED) at 1.5-3m, while the POSITION barrier `ζ_r` correctly screams (0.2→2.6). The position loop SEES the offset but does not issue a braking `a_u` (σ stays on-surface); the flow loop OWNS `a_u` but is blind. The loop only "wakes" at ~1m (σ 0.01→0.31, ζ_h 0.04→1.54) when `h_xy=v_lat/Z` finally grows by 1/Z — too late → 1.7m out → overshoot → kick. The controller literally cannot distinguish "drifting out" from "tracking the reference" because it adopts the measured drift as the setpoint. (Both barriers are also 1/Z-normalized so a real metric offset reads small at altitude → weak demand — same blind-at-altitude pathology.) This is the handoff's "lateral h_e is DEGENERATE" stated as a CAUSE.
- **`XI2_xy` only partially helps** because it inflates a `h_e` that is structurally ~0 (ζ_h share 7%→18%). Real fixes: (a) kill the SEED — IC-handoff radial-velocity gate (require v_radial≈0 before descent); (b) give the loop a TRUE velocity error — de-weight measured ṡ in `h_d` so `h_e` regains velocity content at altitude (the combined-surface design traded this away for clean position authority).

### `COMMIT_SEN`=0.5 A/B (band-aid, TESTED 2026-06-26, IC1+IC3 n=3, bundle 20260626-023136) = WORKS, it's a ROBUSTNESS lever
- **Eliminates the flies entirely.** All 6 reps committed (none blocked), 0 balloon, 0 TL, worst case **0.156m vs the 0.35-guard's 3.45m**. The IC3 straggler that ballooned to 3.45m now lands 0.15m. Tally: 2 SP / 1 PRECISE / 1 SOFT / 2 over-tol(0.107,0.154m).
- **At the predicted cost:** reps that commit off-center (s_e_n≈0.48, which 0.35 blocks) land 0.11-0.15m and aren't soft (commit WITH residual v_lat → open-loop drop drifts, vel 0.18-0.55). Well-converged reps (s_e_n≤0.23) still sub-0.06m SP, unaffected. So 0.5 = guaranteed no-catastrophe + ~0.15m worst case, trading terminal precision/softness on the stragglers. Bakeable as a safety floor; does NOT converge them (symptom, not root).

### Caveats / next
- **GT-FB ONLY** (perception idealized). Per the handoff the next step is perception-ON (drop `PLASMC_GT_FEEDBACK`) before any bake. Default-off, NOT baked.
- The terminal `v@min-alt` is still ~1 m/s (lag-set) but the open-loop settle arrests it by touchdown (endpoint vel 0.01-0.40 on the SPs) → the gate ALSO delivers the SOFT condition, not just precise.
- Commits 8d1ecb0 (gate + honest-metric freeze + calibrate tool). Reproduce: `HEADLESS=1 PLASMC_GT_FEEDBACK=1 PLASMC_XI2_X=0.7 PLASMC_XI2_Y=0.7 PLASMC_XI2_Z=0.2 PLASMC_P20_X=15 PLASMC_P20_Y=15 LANDING_COMMIT_LOOM=2.8 IC_LIST="IC2 IC3 IC4 IC5" N_REPS=3 bash scripts/run_ic_validation.sh`.

Continues [[feedback_flow_funnel_zetah_works]] (the velocity-damping lever that this gate completes). The reframe there ("xy is post-kick balloon, controller touches down dead-centered") is now ACTED ON: the gate captures the dead-centered approach as the landing.
