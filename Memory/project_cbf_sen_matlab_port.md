---
name: project_cbf_sen_matlab_port
description: CBF (cbf2) + SEN_FUNNEL ported from PX4 Python into MATLAB Phase-1 controller; gotchas + status
metadata: 
  node_type: memory
  type: project
  originSessionId: 5ef5a2d7-329a-4539-b101-9f7e6204c84a
---

Ported the target-visibility CBF (cbf2) + SEN_FUNNEL from PX4 (`PX4_Gazebo/`) into the
MATLAB canonical single-run `MATLAB/Multi_init_cond/visualControl_IBVS_adaptive.m`
(2026-06-14). Brief: `PX4_Gazebo/docs/CBF_SEN_MATLAB_PORT.md`. User chose **replace
outright** (cone clamp + outer PID deleted, no A/B flag). Backup at
`Obsolete/Multi_init_cond/MATLAB/visualControl_IBVS_adaptive_v2.m`.

**Files:** NEW `MATLAB/Common/cbf2_filter.m` (pure port of `src/cbf_visibility.py`,
state passed as a struct in/out since MATLAB has no mutable dict). Controller edits:
SEN_FUNNEL replaces outer PID; CBF replaces cone-clamp; Fix B builds `rd3` from `th_safe`.

**Load-bearing gotchas (re-derived for MATLAB frames):**
- `K.p_10` in Constants.m is axis-SWAPPED `[res(2);res(1)]/2f=[0.889;1.185]` (= [y-half;x-half]).
  The CBF box needs correctly-ordered `phi_max=[res(1);res(2)]/2f=[1.185;0.889]` matching C_nP rows.
  Kept K.p_10 for `s_e_n` normalization (legacy parity); separate `phi_max_cbf` for the CBF.
- `C_nP` is ALREADY centre-relative (range ±res/2) → tangent = C_nP/f, center=0, no subtraction.
- MATLAB projection (`C_nP=f·(I_R_C'(P−cam))[1:2]/[3]`) is algebraically identical to Python's
  pinhole, so the **`L_w·M` sign (M=[0 1;−1 0]) TRANSFERS**: confirmed numerically 5.6% (fix) vs
  216% (identity) in MATLAB's exact frame. See [[project_kinematics_correction_2026_06_11]].
- ZOH=3 (dt=0.01, img@33Hz): CBF called every step on held C_nP; drift/loom FD gated on a
  `refresh` flag with `dt_img=ZOH*dt` so previous-value trackers span the true image interval.
- Fix B thrust: `T_cd = m·|I_a_cd_filt(3)|/rd3(3)` (vertical demand from descent loop, direction
  from th_safe). Falls back to `−I_F/|I_F|` on Phase-2 decode-fail (th_safe=[]).

**Verification:** mirrored validate_cbf.py tests 2/4/5 against my MATLAB transcription in Python:
barrier predicted-feature in box 4e-16, minimal-intervention 2e-16, no-strangle 0%. Test 3
(TRUE feature) 0.024 vs 0.02 thr = known one-step linearization slack, not a bug. User declined a
full validate_cbf.m (already SITL-validated). **Judge by visibility (marker in frame under tilt),
NEVER landing xy** — fly-away is separate control tuning the CBF doesn't own.

**Framework comparison (2026-06-14):** full MATLAB↔Python control-law diff verified intact after
the port. Middle loop (p_2/S_2/zeta_2/G_2/sigma/c/Theta/kappa-RK5/u_sw/u_eq), yaw ASMC, desired-flow
V_h_d, I_a LPF, R_d, SO(3) — all still exact ports (Python cites MATLAB line numbers). SEN izeta
clamp 5.0 matches Python `_izeta_clamp`. The pull added two outer-loop governors Python-only:
`tau_ds` LPF (0.05s) + `DSD_LAT_MAX` lateral cap (default off). **User chose NOT to add either to
MATLAB** (SITL-noise/async protections; clean MATLAB perception doesn't need them) — do not re-add.
Python-only middle-loop protections (Singhal outlier containment, kappa-freeze/kappa_max, dh_d ±50)
are by-design Category-B, NOT MATLAB gaps. D1 (MATLAB rotz-only accel transform) is the *correct*
side. phi_max axis order [1.185;0.889] matches C_nP axes + validate_cbf.py (Python's opposite _p_10
= img_data transposed-pixel convention, internally consistent).

**MATLAB VALIDATED + Fix B ROOT-CAUSED & CORRECTED (2026-06-14, R2025b, Circular, IC[2,2,-5]).**
The z-limit-cycle (descend to ~0.29m, bounce to ~1m, never close last 9cm) was a **MATLAB-only T_cd
translation bug**, NOT inherent to Fix B and NOT present in PX4. My Fix B set
`T_cd = m*|I_a_filt(3)|/rd3(3)` using the COMMANDED tilt cosine; thrust acts at the ACTUAL (lagging)
attitude -> vertical force = T_cd*cos(actual) = m*|I_a_z|*cos(actual)/cos(commanded) > demand during
slew -> early climb (alt rose to 5.17) -> descent never commits. **PX4 divides B_T by cos(euler) of
the MEASURED attitude (controller.py:1284-1286), so its vertical force = demand exactly, always — PX4
never had this.** FIX: divide by the ACTUAL cosine R33=I_R_C(3,3), not rd3(3). Bisection (noiseless):
original lands 8.57; CBF+SEN+buggyFixB=limit-cycle; CBF+SEN no FixB=lands 8.57; CBF+SEN+correctedFixB
(rd3 from th_safe, T_cd via R33)=lands 8.34 soft+precise, CBF bound EXACT on attitude, visibility PASS
(Phase-1 100%, tilt 15deg, corner<=91/120). **ADOPTED corrected direct-th_safe as default** (PX4-
faithful, uses th_safe directly in geometric tracking — user's intent). TRADEOFF/OPEN: direct path
drops the tau_ia LPF on the ATTITUDE cmd (kept for T_cd vertical only) -> less noise absorption; 1
noisy seed landed HARD (xy .254, v_rel .852, not soft/precise) vs noiseless soft+precise. Single noisy
run unreliable [[project_comparison_noise_variance]] but mechanism real. DECISION (user 2026-06-14):
KEEP corrected direct-th_safe as default.

**RESOLVED — LPF moved BEFORE the CBF (Option 1, 2026-06-14).** The noise-vs-exact-bound tradeoff is
gone. Principle: the CBF is a PROJECTION that re-imposes the hard FoV bound on whatever desired lean it
is given, so filtering its INPUT cleans noise WITHOUT smearing the bound; filtering its OUTPUT (theta_safe
or the post-CBF accel = old Fix B) mixes safe leans across moving per-step boxes -> smear. Impl: tau_ia
LPF (alpha_ia) now applies to I_a_cd BEFORE the CBF; the CBF operates on I_a_cd_filt; one filter serves
both theta_unsafe (lateral) and T_cd (vertical); raw I_a_cd(:,idx) still logged. R_d still builds rd3
from th_safe directly (exact bound) + T_cd via actual R33. RESULTS (Circular IC[2,2,-5]): noiseless
lands 8.51s soft+precise (xy .006); WITH NOISE lands 32.49s SOFT+PRECISE (xy .009, v_rel .186) vs the
LPF-removed direct path's HARD xy .254/v .852. Visibility intact: Phase-1 100%, tilt 16°, corners <=94/120,
centroid <=91/120 (bound NOT smeared). Port FINAL: cbf2_filter.m + SEN_FUNNEL + direct-FixB(actual-R33
T_cd) + LPF-before-CBF, all default. Other rejected LPF spots: theta_unsafe (~=Option1, lateral-only),
theta_safe (smears=Fix B bug), w_u rate-cmd (inner-loop phase lag).

**COMMITTED** cd3c330 (pushed). **NOISY MULTI-IC SWEEP done** (sweep_cbf.m, canonical 5 ICs x noiseless+4
noisy, Circular): noise robustness CONFIRMED recovered — noisy soft+precise 14/20, landed 18/20. Solid:
IC1[0,0,-5] 4/4, IC4[2,2,-7] 4/4, IC2[2,2,-5] 3/4. Two weak spots (BOTH separate from LPF placement):
(1) IC5[2,2,-3] FoV-fails even NOISELESS — low-alt(3m)/high-lateral(2.83m), marker subtends large angle;
likely REGRESSION from replacing the corner-guarding cone clamp (rho_fov 15px inset) with the centroid-
guarding CBF: CBF keeps centroid in (its guarantee) but strict fov_fail checks ALL corners. Fix options:
relax fov_fail to match CBF centroid guarantee, or add corner inset to CBF box. (2) IC3[2,-2,-5] softness
degrades under noise (SEN_FUNNEL tuning, §9). **CORNER-BASED CBF (2026-06-14, user direction).** PX4 guards the board CENTROID because corners
aren't reliably detected at altitude (multi-marker board); MATLAB has EXACT analytical corners, so
cbf2_filter now constrains EVERY corner to the FoV box (per-corner L_w + anchor, alternating projection
over all 2N rows), not the centroid+two-phase-delta. phi_max then = the strict fov_fail box (res/2f).
Needed a 15px INSET (phi_max_cbf=([res/2]-15)/f, matches old rho_fov_0) for slack vs the one-step
linearization + attitude-slew lag — without it IC5 grazed 120.7/120. WITH inset: IC5[2,2,-3] noiseless
LANDS 7.41s soft+precise (was FoV-fail; old cone-clamp landed it 7.38s -> regression FIXED); IC2 unchanged
8.51s. Two-phase delta now vestigial (kept for the C_nP-empty Phase-2 fallback). Re-sweep running.

**IC3 BLOW-UP ROOT-CAUSED -> N_z fix (2026-06-14).** The corner-CBF noisy divergence (IC3[2,-2,-5]
xy->275) is a kappa_z RUNAWAY (the documented §6 fragility). Caught one (diag_ic3.m) + decomposed the
kappa ODE dkz/dt=Theta*N_z*G_zz*|sz|-N_z*P_z*kz: at low alt (~1m) a pixel-noise spike on he_x/he_y
couples (c-term) into he_z -> vertical funnel breaches (S_zz->0.95 clamp) -> G_zz(0.9->9.1) AND
sz=zeta_z(0.4->3.66) BOTH max simultaneously (both ∝ residency) -> kz_eq=Theta*G*|s|/P ~120-230. Theta
stays MODERATE ~18 (NOT dh_d/c-term driven; P_z=5 is protective). The parameter that ignites it:
**N_z=0.05** sets the chase RATE — dkz/dt~0.41/step vs 0.16 at N_z=0.02 -> kz escalates within the brief
transient breach. PX4 ALREADY fixed this: CONTROLLER_PARITY.md:61 reduced N_z 0.05->0.02 "kappa_z
adaptation too fast on noisy flow". MATLAB kept 0.05 (clean synthetic flow); corner CBF's noise exposed
it. FIX APPLIED: K_ctrl.N z 0.05->0.02 (single-run only; NOT mirrored to run_simulation/Comparison —
they lack the corner CBF trigger + Comparison locked). Re-sweep running. kappa has NO clamp in MATLAB
(PX4 KAPPA_MAX_Z=3.0) — available as a hard backstop if N_z alone insufficient. Corner CBF noise-averaging
(smooth C_nP) still an option to reduce trigger frequency. Artifacts: diag_ic3.m, temp_diverge.mat.

**N_z=0.02 RE-SWEEP + IC5 ROOT-CAUSED (2026-06-14).** N_z=0.02 worked: IC3 1/4->4/4 noisy s+p, noisy
s+p 13->16/20, ZERO kappa_z-runaway diverges. Remaining failure = IC5[2,2,-3] (hardest IC: 3m alt +
2.83m lateral), 3/4 noisy fly-away (xy->77-180). Clamp audit on a caught IC5 divergence (diag_ic5.m,
temp_ic5.mat): NOT a kappa runaway (all kappa tiny <1, S_2 funnel 0% breached -> N_z fix held). Dominant
saturation = SEN_FUNNEL: S_s residency at +-0.95 **62.5%** + izeta_s at +-5 **61.5%**. IC5's initial
s_e_n >> p_s_0=1.2 -> S_s clips -> back-mapped PPC goes gentle + G_s^-1∝p_s collapses as funnel contracts
-> DEMAND STARVATION -> can't close offset -> fly-away. This IS the documented FUNNEL_CBF_DESIGN §9
"SEN_FUNNEL needs tuning". Levers (ranked): funnel SIZING (p_s_0 wider / gamma_s slower) > §9 HARD
outlier-containment (force s_e_n inside bound on breach, mirror the h_e funnel's Singhal containment
controller.py:688-692) > K_rp/ri/rd retune (izeta_s windup -> K_ri). Gains alone can't fix a saturated
funnel (G_s^-1->0). **USER DECISION (2026-06-14): SEN_FUNNEL/IC5 correction MOVED TO UBUNTU/Python** —
same issue exists there (§9), fix once where SITL-validated. COMMITTED+PUSHED cd4dc59 (corner-CBF + 15px
inset + N_z=0.02, validated noiseless 5/5 noisy 16/20, IC5/SEN deferred note) + 7891a9d (UBUNTU_HANDOFF.md
at repo root: git pull, merge env-gated LPF-before-CBF in PX4 controller.py, fresh SITL A/B, fix shared
SEN_FUNNEL §9, compare). Diagnostic artifacts deleted. Ubuntu picks up the PX4 work from there.
See [[feedback_shared_issue_fix_in_ubuntu]].

OPEN: PX4 — recommended moving LPF before CBF there too
(PX4's Fix B currently leaves attitude direction unfiltered = same gap); offered to draft env-gated A/B,
NOT blind-port (SITL test: chatter/W_U, fly-away, visibility). Sweep artifacts: sweep_cbf.{m,csv,log}.

**PENDING:** (1) [DONE — see MATLAB VALIDATED above].
(2) SEN_FUNNEL soft-clip "needs tuning" (FUNNEL_CBF_DESIGN §9: goes gentle at bound, regressed
in SITL; may need hard outlier-containment). (3) **Gain-sync NOT propagated** to
`run_simulation.m` / `Comparison/InitGains_Comparison.m` — this is a structural feature add to the
single-run only; comparison study is LOCKED. Open question whether to propagate.
