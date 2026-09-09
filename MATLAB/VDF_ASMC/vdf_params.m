function P = vdf_params()
%VDF_PARAMS  Baked VDF-ASMC controller parameters (single source of truth).
%   P = vdf_params() returns the complete, documented parameter struct for the
%   VDF-ASMC soft-precise-landing controller, matching control_formulation.tex and
%   the validated 25/25 baked config (combined-barrier surface + target-visibility
%   CBF). Symbol names follow the paper; the eq. references point into the .tex.
%
%   This is the ONLY place gains live. Every block reads from P; the simulation
%   driver and every harness use the one controller, so the implementations cannot
%   drift apart (the root cause of the earlier run_simulation/comparison divergence).

% ---- Plant / camera (Constants.m: m, J, g, f, res) ----------------------------
Constants;                              % populates m, J, g, f, res, zf into workspace
P.m   = m;                              % mass [kg]
P.J   = J;                              % inertia [kg m^2]
P.g   = g(:);                           % gravity vector (inertial), [0;0;9.81] NED-down
P.f   = f;                              % focal length [px]
P.res = res(:);                         % sensor resolution [r_h; r_w] [px]
P.zf  = zf;                             % above-target landing gap [m]
P.dt  = 0.01;                           % control step [s]

% half-FoV in tangent (feature) units  phi_max = R/(2f)   (tex eq. normalized-error)
P.phi_max     = P.res / (2*P.f);                 % [2x1] for r_bar_e (s-axis order)
P.fov_inset_px = 15;                             % CBF corner safety inset [px]
P.phi_max_cbf = (P.res/2 - P.fov_inset_px) / P.f;% CBF inset FoV-edge tangent (C_nP-axis order)

% ---- Image-feature funnel  (tex eq. position barrier; constrains r_bar_e) ------
% >>> PX4 PARITY PORT 2026-09-03 (Wave 1). PX4 is now the baseline; values below are its live
% >>> defaults. Prior MATLAB values + their rationale are kept as PRIOR for revert.
% >>> Units are directly comparable: PX4 _p_10 = center/focal == MATLAB P.phi_max = res/(2f).
P.p_r0   = [10.0; 10.0];                % p_{r0} initial half-width (FoV units). PORTED (PRIOR 1.2):
                                        % PX4 PLASMC_PR0_{X,Y}. 8x looser initial funnel -- effectively
                                        % unconstrained at t=0, convergence carried by the ASMC.
P.p_rinf = [0.8; 0.8];                  % p_{r,inf} terminal floor. PORTED (PRIOR 0.85, itself LOCKED
                                        % 1.0->0.85 2026-06-26 for ~15% precision, multi-init worst xy
                                        % 0.013->0.011). PX4 PLASMC_PRINF_{X,Y}. NB both <1 dip below
                                        % Standing Cond 1 (proof caveat); proof-clean alt = 1.0.
P.Xi_r   = diag([0.30, 0.30]);          % Xi_r funnel contraction rate.
                                        % ⚠ 2026-09-04 RETUNED, NOT ported as PX4's raw value. PX4's
                                        % PLASMC_XIR_{X,Y}=0.10 was tried and root-caused (isolation +
                                        % 16-point Xi_h x Xi_r grid, MATLAB/PX4_PARITY_PORT_SPEC.md) as
                                        % the DOMINANT cause of an IC5 Lissajous/Circular fly-away (0/4
                                        % clean at Xi_r in {0.10,0.15} regardless of Xi_h; worst_xy up to
                                        % 2.3m). Xi_r=0.30 is the ONLY grid column reaching 4/4 clean
                                        % (worst_xy=0.07, worst_v=0.12) -- this is unchanged from the
                                        % PRE-PORT MATLAB value (itself LOCKED 0.10->0.3 for faster
                                        % position contraction, 2026-06-20-era). PX4's 0.10 does not
                                        % transfer to MATLAB's plant; do not re-port it without a fresh
                                        % IC5-Lissajous/Circular check.

% ---- Optic-flow funnel  (tex eq. PPC on h_e; p_h(t)) ---------------------------
P.p_h0   = [15.0; 15.0; 10.0];          % p_{h0} initial half-width (code p_20; PX4 PLASMC_P20_{X,Y,Z}).
                                        % PORTED (PRIOR [25;25;4]): xy tighter, z 2.5x looser.
P.p_hinf = [2.5;  2.5;  1.5];           % p_{h,inf} terminal floor (PX4 PLASMC_P2INF_{X,Y,Z}). PORTED
                                        % xy 1.0->2.5; z 1.5 ALREADY MATCHED (PX4 comments it "vdf
                                        % p_hinf z"). PX4 rebaked xy 1.0->2.5 on 2026-08-28 after
                                        % tracing P2INF_xy=0.5/1.0 as the MECHANICAL TRIGGER of the
                                        % funnel-breach/containment/dh_d-leak chain (s_e_n was small and
                                        % still converging when p(t) hit its floor); 2.5 gave 5/5 clean.
                                        % PRIOR rationale (lateral 0.5->1.0 baked 2026-06-25): looser
                                        % funnel cut terminal y-chase-lag on the fast Lissajous axis,
                                        % paired with chi_r 2.0 -> Liss 1.4x xy 0.0758->0.0654, 45/45.
P.Xi_h   = diag([0.2, 0.2, 0.2]);       % Xi_h contraction rate (code gamma_2; PX4 PLASMC_XI2_{X,Y,Z}).
                                        % ⚠ 2026-09-04: NOT PORTED -- kept at the PRE-PORT MATLAB value.
                                        % PX4's 1.0 (5x this) paired with the fixed Xi_r=0.30 reaches
                                        % only 3/4 clean on the IC5 Lissajous/Circular grid (worst_xy=
                                        % 1.24m, a real fly-away). An intermediate 0.4 (first tried, also
                                        % paired with Xi_r=0.30) reached 4/4 on that SAME 4-cell IC5
                                        % probe but then FAILED the broader IC2/IC3/IC4 Circular cells
                                        % (noiseless, xy up to 1.23m) that the IC5-only probe never
                                        % covered -- i.e. it looked safe on a narrow test and wasn't.
                                        % 0.2 (unchanged) passes ALL of IC1-5 x Circular AND the IC5
                                        % Lissajous/Circular grid (both are the literal pre-port MATLAB
                                        % config, whose bit-exactness was independently confirmed by the
                                        % full-revert test). CONCLUSION: PX4's faster Xi_h does not
                                        % transfer to MATLAB's plant at all, not even partially; only
                                        % Xi_r needed retuning. See MATLAB/PX4_PARITY_PORT_SPEC.md.
                                        % NB in PX4 this is set in the combined-barrier rebake block, NOT
                                        % the pa("XI2",...) default, which it overrides.

% ---- Combined sliding surface  sigma = zeta_h + chi*zeta_aug  (tex eq. sliding) -
P.chi_r = [2.0; 2.0];                   % lateral surface gain (PD: zeta_h + chi_r*zeta_r).
                                        % REVERTED 2026-09-09 to MATLAB's own value (PX4's PLASMC_CHI_R=1.5
                                        % loosened the moving-traj terminal error: Linear worst xy 3.4->2.4 cm
                                        % at chi_r=2.0, full 5x5 realistic gate 25/25 SP; gain VALUES don't
                                        % port -- see feedback_gain_values_not_portable_either_direction).
                                        % Rationale for MATLAB's 2.0: 1.15->2.0 baked
                                        % 2026-06-25: drives terminal lateral barrier harder to kill the
                                        % per-axis terminal y-chase-lag (the standing lateral limit, proof
                                        % S6). With p_hinf 1.0 + per-axis: Lissajous 1.4x 0.0758->0.0654 at
                                        % ORIGINAL w2=0.85 (removes the w2 0.8 trim), 45/45 held, multi_init
                                        % worst xy improved 0.041->0.034. (Old "1.2 regresses Liss-IC3"
                                        % warning was scalar-theta/w2=0.85 era; stale under per-axis+p_hinf)
P.chi_z = 0.1;                          % descent surface gain (PI: zeta_h3 + chi_z*int zeta_h3).
                                        % ALREADY MATCHES PX4 -- no port needed. PX4 calls this
                                        % PLASMC_OMEGA_Z (controller.py:2850 states "chi_z = Omega_z"
                                        % verbatim); both are 0.1. NB PX4's OMEGA_X/Y are INACTIVE under
                                        % combined_barrier (the lateral surface uses chi_r*dzeta_r), so
                                        % there is no lateral Omega to port. Prior: 0.025->0.1 baked 2026-06-20: stronger descent integral drives h_ez->0 (loom regulated to h_rd -> constant area rate, no dPdt ramp) -> kills the terminal 1/z fly-away. Fly-aways -91% (35->3/300), SP 87->97%, full gate. (the integral GAIN matters, not the izeta clamp)

% ---- Leakage ASMC  (tex eq. adaptive control law + adaptive law) ---------------
P.Gamma   = diag([0.25, 0.25, 0.75]);   % Gamma linear sliding gain (PX4 PLASMC_GAMMA_{X,Y,Z}).
                                        % PORTED xy 0.4375/0.5->0.25 (symmetric); z 0.75 ALREADY MATCHED.
                                        % PX4 rationale: the reaching gain is the terminal-limit-cycle
                                        % FORCING amplitude; lower Gamma shrinks the cycle.
P.E       = diag([1.0, 1.0, 0.5]);      % E boundary-layer thickness (PX4 PLASMC_E_{X,Y,Z}).
                                        % PORTED xy 0.5->1.0; z 0.5 ALREADY MATCHED.
                                        % ⚠ THIS REVERTS a MATLAB LOCK. PRIOR (E_xy 1.0->0.5 LOCKED
                                        % 2026-06-26): 0.5 escapes the boundary layer so kappa's switching
                                        % is DELIVERED against sigma (56% engaged under stress vs 4% at
                                        % E_xy=1.0). PX4 keeps 1.0 because its X/Y are NOISE-pumped and
                                        % kappa switching hurts there -- a real-perception consideration
                                        % MATLAB's synthetic perception does not have. Prime revert
                                        % candidate if lateral stress performance regresses.
P.N       = diag([0.10, 0.10, 0.10]);   % N adaptation rate. ALREADY MATCHES PX4 (PLASMC_N=0.1 x3) --
                                        % no port needed. LOCKED 0.02->0.10: primes the kappa-ODE
                                        % (tau 1/(N P) ~33s->7s) so kappa adapts within the descent
P.Pleak   = diag([2.5, 2.5, 5.0]);      % P kappa leakage (PX4 PLASMC_P_{X,Y,Z}). PORTED
                                        % [0.5;0.5;1.5]->[2.5;2.5;5.0]. ⚠ REVERTS-AND-EXCEEDS a MATLAB
                                        % LOCK (which had gone [1.5;1.5;5.0]->[0.5;0.5;1.5] so that lower
                                        % leakage RAISES sustained kappa, k* = thG|s|/P). Porting back up
                                        % LOWERS sustained kappa ~5x on xy. Pairs with kappa0 below.
P.kappa0  = [0.5; 0.5; 0.25];           % kappa(0) (PX4 PLASMC_KAPPA0_{X,Y,Z}). PORTED [.05;.05;.05]->
                                        % [.5;.5;.25] (10x/5x). ⚠ REVERTS a MATLAB LOCK
                                        % ([.125;.125;.25]->.05, "lower start so kappa adapts UP under
                                        % stress; 7x SP 5/5 vs baked 3/5"). PX4 uses the high start as a
                                        % BOOTSTRAP: z braking authority from t=0 -> soft touchdown.
P.kappa_max = [30.0; 30.0; 3.0];        % NEW, PORTED FROM PX4 (PLASMC_KAPPA_MAX_{X,Y,Z}). MATLAB had no
                                        % cap. xy=30 came from real hardware runaway (kappa_xy pinned
                                        % 25-29 for 10-28 s); z=3.0 is load-bearing in bad reps and inert
                                        % in good ones (clean reps sit at kappa_z~1).
                                        % ⚠ REQUIRES A CODE CHANGE: +blocks/asmc.m must clamp the kappa
                                        % state to this. Setting it here alone has NO effect.
P.hd_kr   = 0.5;                        % h_d back-map convergence gain k_r (PX4 PLASMC_HD_KR).
                                        % PORTED 2026-09-03; MATLAB previously had NO such term
                                        % (h_d used the funnel-prescribed rate alone, matching the
                                        % manuscript). Adds -k_r*G_r^{-1}*zeta_r to s_dot_presc in
                                        % +blocks/position_funnel.m -> h_e becomes the scaled
                                        % (dzeta_r + k_r*zeta_r), i.e. exponential zeta_r convergence.
                                        % ⚠ UNSWEPT on PX4 (baked 2026-06-29 in a bundled re-bake).
                                        % ⚠ MANUSCRIPT: this falsifies the three "no back-mapped rate"
                                        % statements (lines 286, 290, Remark rem:normalization) and
                                        % changes eq. `h_d final` + eq. `h_e identity`. Set 0 to revert.
P.izeta2_max = 5.0;                     % anti-windup clamp on int(zeta_h3)
P.S_margin   = 0.05;                    % funnel-saturation guard (|zeta|<=3.66, G finite)
P.drop_sddot = true;                    % s_ddot-drop (validated combined-barrier default)

% ---- Descent reference  (tex h_d final: h_rd < 0) ------------------------------
P.h_rd = -0.38;                         % desired descent optic flow. RE-TUNED 2026-09-09.
                                        % PX4's -0.30 (ported 2026-09-03, LANDING_REF_RAD_OPT_FLOW)
                                        % slowed the descent ~40% -> multi-init mean t_f 10.3 s -> 16.7 s
                                        % with no accuracy gain; that was the entire moving-traj
                                        % "regression" vs the manuscript numbers. -0.42 (the old locked
                                        % value) is now too aggressive noiseless (1 FoV fail); -0.38 is
                                        % the sweet spot on the CURRENT stack (two-tier CBF + drift lead +
                                        % yaw rate law + per-axis theta): 5x5 NOISELESS 25/25 SP 0 FoV
                                        % t_f 8.8 s, REALISTIC 25/25 SP 0 FoV t_f 11.4 s, no 1/z fly-away.
                                        % Gain VALUES don't port (feedback_gain_values_not_portable_either_direction).

% ---- Virtual-compass yaw ASMC  (tex eq. yaw control law) -----------------------
P.Omega_a = 0.1;   % chi_alpha  (sigma_a = alpha_e + chi_a*int alpha_e). PORTED FROM PX4 2026-09-03
                   % (PLASMC_YAW_OMEGA=0.1; PRIOR 0.25) -- user rule: where MATLAB has no explicit
                   % reason to retain its own value, take PX4's. NB controller.py:531-537 frames the
                   % PX4 cut as margin against the PX4 inner-loop lag (K_R_YAW + rate loop + tau_ua
                   % LPF) that MATLAB does not have, so this may be over-damped here; it is the second
                   % revert candidate (after Gamma_a) if yaw convergence slows.
                   % PRIOR: re-baked 0.5->0.25 (2026-06-21 deep sweep, clean win): yaw ASMC was
                   % slightly over-gained -> gentler yaw cuts the yaw->image->lateral pumping;
                   % eliminates the S3 fails, +SP, edge held. Was paired with Gamma_a=0.25 (now 0.5),
                   % so the 06-21 pairing is fully superseded rather than half-broken.
P.Gamma_a = 0.5;   % gamma_alpha. PORTED FROM PX4 2026-09-03 (PLASMC_YAW_GAMMA=0.5; PRIOR 0.25).
                   % ⚠ REVERTS the 2026-06-21 re-bake 0.5->0.25 (which was a paired change with
                   % Omega_a and reported as a clean win: eliminated the S3 fails, +SP, edge held).
                   % NB Omega_a is deliberately NOT ported (see above), so this breaks that pairing --
                   % if the yaw->image->lateral pumping returns, revert this first.
P.n_a     = 1.0;   % eta_alpha
P.p_a     = 2.0;   % rho_alpha
P.kappa_a0 = 2.0;  % kappa_alpha(0)
P.E_a     = 3.0;   % eps_alpha boundary layer

% ---- Yaw-rate law  (PLASMC_YAW_RATE_LAW port; PX4 87cf020, baked ON @ 63aa258) -
% Opt-in ALTERNATIVE to the kappa_a ASMC above. Drops the sliding-mode switching
% term and drives u_a (the psi_d rate) as a PI on alpha_e that substitutes the
% MEASURED derivative w_z = V_w(3) (~= alpha_e_dot, tex eq. `alpha_e_dot`) for a
% finite difference:
%     d/dt w_rl = yrl_kp*alpha_e + yrl_wz_sign*w_z - yrl_ki*int(alpha_e)
%     u_a       = clip(w_rl, +-yaw_rate_max)     (anti-windup freezes yrl_ie)
% DEFAULT (2026-09-09): validated clean win on the cross-marker stack -- IC1-5
% noiseless + realistic seed=1 both 25/25 soft-precise, 75/75 noisy multi-seed,
% mean|e_a| 0.5 deg vs the ASMC's 2.5 (max 2 vs 21); on CircularYaw it holds
% |e_a| ~1 deg to 0.7 rad/s target spin where the ASMC lags 12-23 deg. Matches
% PX4's cross-marker default (63aa258). P.yaw_rate_law=0 restores the kappa_a
% leakage ASMC (kept as the documented alternative / fallback).
% PX4's PLASMC_YAW_RL_WZ_SCALE (2.5) is a PERCEPTION magnitude-deficit factor and
% does NOT port -- MATLAB's V_w(3) is the analytic pseudo-inverse recovery, scale 1.
% Sign derived for MATLAB's (non-inverted) plant: V_w(3) ~= +alpha_e_dot, closed
% loop  alpha_e'' + alpha_e' + yrl_kp*alpha_e ~ d(d_alpha)/dt  Hurwitz for yrl_kp>0.
P.yaw_rate_law = 1;
P.yrl_kp       = 0.3;   % k_p (PLASMC_YAW_RL_KP)
P.yrl_ki       = 0.0;   % k_i, STABILISING sign +k_i*int(e_a), needs 0<=k_i<k_p; default 0
                        % (rejects a constant d_alpha bias; PX4's -k_i was a dead-end sign)
P.yrl_wz_sign  = 1.0;   % V_w(3) already carries +w_z = +alpha_e_dot for this plant
P.yaw_rate_max = 2.0;   % rad/s clip on u_a (PX4 _psid_rate)

% ---- Target-visibility CBF  (tex eq. cbf qp) ----------------------------------
P.theta_cap = deg2rad(43.94);           % post-QP deliverable-tilt cap. PORTED FROM PX4 2026-09-03
                                        % (was 60 deg). PX4 derives it as arccos(g/A_CAP) =
                                        % arccos(9.81/13.610) = 43.94 deg (controller.py
                                        % THETA_CAP_DEG_DERIVED, baked 2026-08-23). The old 60 deg
                                        % assumed a 2x-hover-thrust margin, which the measured
                                        % airframe (1.389 g) does not support -- 60 deg demands
                                        % g/cos(60) = 2 g. PAIRED with T_max 60->28.7725 N in
                                        % Constants.m: porting either alone is incoherent, since
                                        % this cap is a function of A_CAP.
P.tau_ia    = 0.08;                     % upstream LPF time constant on commanded accel [s]
P.a_floor   = -50;                      % inertial-z accel floor (keep thrust direction realizable)
% ---- CBF_JOINT_QP (PX4 parity port 2026-09-03) --------------------------------
% The visibility QP now solves for the FULL I_a (lateral AND vertical), interleaved
% with a descent-rate relief and a TRUE-thrust deliverability sphere. See
% Common/cbf2_filter.m (jqp branch) and MATLAB/PX4_PARITY_PORT_SPEC.md §B2/B3.
P.jqp_on = true;                        % false -> legacy theta-QP (bit-identical to pre-port)
P.A_cap  = T_max / P.m;                 % |I_a| <= A_cap. 28.7725/2.114 = 13.610 m/s^2 = 1.389 g,
                                        % identical to PX4's A_CAP by construction (same T_max,
                                        % margin and mass). Derived, NOT hand-set -- it tracks
                                        % Constants.m T_max automatically.
P.k_az   = 5.0;                         % descent-rate relief gain (PX4 CBF_AZ_COST_GAIN).
                                        % ⚠ PICKED, NOT SWEPT on PX4 -- and chosen while the
                                        % sphere never bound (that bug was fixed 2026-09-03), so
                                        % its tuning regime no longer exists. Re-sweep. Set 0 to
                                        % disable the relief while keeping the joint solve.

% ---- Two-tier visibility conditioning (port of visibility_projection.py) ------
% docs/CBF_visibility spec (9-Sep-2026). DEFAULT: the two-tier projection below
% REPLACES the joint-QP above (cbf_two_tier=false restores cbf2_filter exactly).
% UNVALIDATED in MATLAB -- run the IC / 50-cell gate before trusting it.
P.cbf_two_tier     = true;
P.cbf_buffer_frac  = 0.15;   % b: FoV-edge buffer, phi = (res/2/f)*(1-b)  (CBF_BUFFER_FRAC)
P.cbf_vis_rho      = 2000;   % rho: per-axis visibility-slack penalty       (CBF_VIS_RHO)
P.cbf_gmin         = 0.2;    % g_min: Tier-2 descent-governor floor         (CBF_GMIN)
P.cbf_treact       = 1.5;    % T_react: Tier-2 reaction horizon [s]         (CBF_TREACT)
P.cbf_gz_lpf       = 0.7;    % one-pole LPF on g_z so a_z does not step
P.cbf_drift_tau    = 0.15;   % tau: moving-target lead horizon [s] (PX4 b71a950, flipped 0->0.15);
                             %   0 -> reactive only. d = V_h(1:2), condition_drift'd (below).
P.cbf_drift_max    = 0.5;    % radial clamp on the conditioned drift |d| (CBF_DRIFT_MAX)
P.cbf_drift_lpf_alpha = 0.12;% one-pole LPF coeff in condition_drift (CBF_DRIFT_LPF_ALPHA)
% Frame knobs -- VERIFY against MATLAB's I_R_C convention before a gate:
P.cbf_mount_deg    = 90;     % camera-mount yaw offset in P_map = Rz(mount)*Rz(-yaw)
P.cbf_Le_sign      = -1;     % L_e = cbf_Le_sign*(L_omega*M); spec sign is -1

% ---- Geometric SO(3) tracker  (tex eq. so3 torque) ----------------------------
P.kR     = diag([2.5, 2.5, 0.5]);  % PORTED FROM PX4 2026-09-03: PITCH 1.5->2.5 (PLASMC_KR_PITCH=2.5);
                                   % roll 2.5 and yaw 0.5 already matched. PX4 baked roll AND pitch to
                                   % 2.5 ("rp 1.5->2.5", lateral mid-descent limit-cycle fix: the inner
                                   % attitude lag was the binding limit, eR_pitch -22deg vs cmd 33deg);
                                   % MATLAB had baked roll only. PX4 keeps YAW at 0.5 -- K_R_YAW^ was
                                   % RULED OUT there (yaw rate loop is slow, ~287ms; stiffening
                                   % over-drives the lag). PRIOR rationale: roll 1.5->2.5 baked 2026-06-20: stiffer roll adds Y-attitude damping that kills the terminal lateral limit cycle (Liss-IC3) -> noiseless 25/25 + real 25/25 + full +/-40%; sharp optimum (2.0/3.0 worse, phase-damping)
P.kOmega = diag([0.3, 0.3, 0.2]);  % yaw-rate 0.1->0.2 baked 2026-06-20: ROOT cycle fix. kOmega_z=0.1 was under-damped -> yaw limit cycle (worst Circ-IC3=1.78) that PUMPED the lateral cycles via yaw-image coupling. 0.2 kills yaw cycle (-90%) AND lateral (Y 0.98->0.11) at full gate; monotonic (vs fragile kR)
P.kI_R   = diag([0,0,0]);  P.ie_R_max = 0.5;     % integral attitude term (off) + anti-windup

% ---- Adaptive CoG feedforward (Lee-style; baked-on) ---------------------------
P.gamma_cog = 0.005;  P.cog_c2 = 2.0;  P.cog_max = 0.02;  P.cog_leak = 0;

% ---- Per-axis regressor-norm theta (mirror PX4 PLASMC_THETA_PER_AXIS) ----------
% false -> switching gain uses the shared scalar ||Theta||_F (published law, parity).
% true  -> per-axis row-norm theta_k=sqrt(v_k^2+1) (tight bound; decouples z from the
% lateral zeta_r blow-up). theta_k==||Theta||_F recovers the scalar law exactly ->
% strict generalization. Enable via VDF_OVERRIDE.theta_per_axis=true. See
% Soft_Precise_Landing/Drafts/PER_AXIS_THETA_PROOF.md.
P.theta_per_axis = true;   % LOCKED default 2026-06-26: the current formulation (all harnesses pin it ON;
                           % strict generalization, recovers the scalar law when theta_k==||Theta||_F).

% ---- Estimation / timing -------------------------------------------------------
P.ZOH      = floor(100/30);              % image refresh decimation (=3)
P.fw       = 11;                         % Savitzky-Golay window (FILTER_WINDOW)
P.pinv_tol = 4;                          % pinv(L_s, tol) singular-value cutoff
P.dhd_cap  = 20;                         % hard cap on d/dt(h_d) (DH_D_CAP spike killer)
P.alpha_ia = P.tau_ia/(P.tau_ia + P.dt); % I_a_cd LPF coefficient

% ---- Tuning override hooks (globals; default OFF -> no behavior change) --------
% Lateral-precision levers for high-speed-target sweeps. Empty => baked values.
global GAMMA_XY_OVERRIDE CHI_R_OVERRIDE P2INF_XY_OVERRIDE THETA_CAP_OVERRIDE
if ~isempty(GAMMA_XY_OVERRIDE), P.Gamma(1,1)=GAMMA_XY_OVERRIDE(1); P.Gamma(2,2)=GAMMA_XY_OVERRIDE(end); end
if ~isempty(CHI_R_OVERRIDE),    P.chi_r = CHI_R_OVERRIDE(:); end
if ~isempty(P2INF_XY_OVERRIDE), P.p_hinf(1)=P2INF_XY_OVERRIDE(1); P.p_hinf(2)=P2INF_XY_OVERRIDE(end); end
if ~isempty(THETA_CAP_OVERRIDE), P.theta_cap = deg2rad(THETA_CAP_OVERRIDE(1)); end

% Generic field overlay (master override; default OFF). Lets a sweep harness set
% ANY combined-barrier field by name without a dedicated global, e.g.
%   global VDF_OVERRIDE; VDF_OVERRIDE.chi_r = [1.15;1.15];
% Applied LAST so it wins over the specific hooks above. Unknown names error out
% (a struct field that is not a P field is a typo, not a silent no-op).
global VDF_OVERRIDE
if ~isempty(VDF_OVERRIDE)
    fn = fieldnames(VDF_OVERRIDE);
    for i = 1:numel(fn)
        assert(isfield(P, fn{i}), 'vdf_params:VDF_OVERRIDE unknown field "%s"', fn{i});
        P.(fn{i}) = VDF_OVERRIDE.(fn{i});
    end
end
end
