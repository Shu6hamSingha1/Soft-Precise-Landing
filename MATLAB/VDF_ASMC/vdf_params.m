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
P.Xi_r   = diag([0.10, 0.10]);          % Xi_r funnel contraction rate. PORTED (PRIOR 0.3, which was
                                        % itself LOCKED 0.10->0.3 for faster position contraction) --
                                        % PX4 PLASMC_XIR_{X,Y}=0.10 REVERTS that bake. Slower
                                        % contraction; re-check the engR<0.65 / no-overtake margin that
                                        % motivated 0.3, since it pairs with p_rinf.

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
P.Xi_h   = diag([1.0, 1.0, 1.0]);       % Xi_h contraction rate (code gamma_2; PX4 PLASMC_XI2_{X,Y,Z}).
                                        % PORTED (PRIOR 0.2 x3) -- 5x faster funnel contraction. NB in
                                        % PX4 this is set in the combined-barrier rebake block, NOT the
                                        % pa("XI2",...) default, which it overrides.

% ---- Combined sliding surface  sigma = zeta_h + chi*zeta_aug  (tex eq. sliding) -
P.chi_r = [1.5; 1.5];                   % lateral surface gain (PD: zeta_h + chi_r*zeta_r).
                                        % PORTED FROM PX4 2026-09-03 (PLASMC_CHI_R_{X,Y}=1.5; PRIOR 2.0).
                                        % PRIOR rationale follows: 1.15->2.0 baked
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
P.h_rd = -0.30;                         % desired descent optic flow. PORTED FROM PX4 2026-09-03
                                        % (PRIOR -0.42; PX4 env is LANDING_REF_RAD_OPT_FLOW, set in
                                        % apps/landing_test.py:31, NOT controller.py). ⚠ -0.42 was the
                                        % LOCKED Table S1 value: a deep sweep flagged -0.38 as +SP on the
                                        % run_simulation seed-ensemble, but re-running EVERYTHING exposed
                                        % that it reintroduces the terminal 1/z fly-away (Linear 28 m
                                        % fly-away on comparison seed 1002; L1/L2/L4 fails in the combo)
                                        % -> reverted then. -0.30 is a LARGER step in that same direction,
                                        % so watch specifically for the terminal 1/z fly-away on Linear.

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
