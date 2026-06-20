% Shared helpers live in ../Common; plotters in ./plotters
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, 'plotters'));
clear mfile_dir;

%% Started working on 14/01/2026
% Last updated on 2026-04-12
%% Refer to "Adaptive Visual Control Strategies for Autonomous Landing of Quadrotor Platforms with visibility constraaints"
% Adaptive Gain Controller and IBVS
% Inertial Frame --> NED
% Camera/Visual Frame ---> NED
% Outer Loop --> s --> h --> a --> I_a_cd
% Inner Loop --> R_d (from I_a_cd + psi_d) --> geometric SO(3) --> tau, T
%
% PLASMC + geometric attitude control:
%   - Yaw ASMC on alpha error (e_a = V_s(4) - V_s_d(4)) produces yaw rate u_a
%   - u_a is integrated into desired heading psi_d (replaces compass)
%   - R_d constructed from I_a_cd (body-z) + psi_d (heading vector)
%   - Geometric SO(3) torque: tau = -kR*e_R - kOmega*e_Omega + w × Jw
%   - No Euler W-matrix, no channel coupling
% clc;
% close all;
clear;
% RNG_SEED_OVERRIDE (global, survives `clear`): fix the noise realization so a
% failing case is reproducible for A/B (e.g. SEN containment). Empty => shuffle.
global RNG_SEED_OVERRIDE;
if isempty(RNG_SEED_OVERRIDE); rng('shuffle'); else; rng(RNG_SEED_OVERRIDE); end

% header
fprintf('Adaptive - Geometric SO(3) Flight Controller\n\n' );

Constants;

InitVar;

init_robustness;    % samples wind / mass / inertia / CoG / pixel-noise model

% Disturbance ABLATION toggles (globals; default = all disturbances active).
% Set =1 to disable ONE source, isolating its effect on the noisy failure:
%   ABL_PIX pixel noise | ABL_OUT outliers | ABL_PARAM mass/inertia/CoG |
%   ABL_WIND wind | GE_OFF ground effect | DELAY_OFF actuator delay.
global ABL_PIX ABL_OUT ABL_PARAM ABL_WIND GE_OFF DELAY_OFF ABL_MASS ABL_INERTIA ABL_COG;
abl_pix   = ~isempty(ABL_PIX)   && ABL_PIX;
abl_out   = ~isempty(ABL_OUT)   && ABL_OUT;
abl_param = ~isempty(ABL_PARAM) && ABL_PARAM;
abl_wind  = ~isempty(ABL_WIND)  && ABL_WIND;
abl_mass  = ~isempty(ABL_MASS)    && ABL_MASS;       % isolate: nominal mass
abl_iner  = ~isempty(ABL_INERTIA) && ABL_INERTIA;    % isolate: nominal inertia
abl_cog   = ~isempty(ABL_COG)     && ABL_COG;        % isolate: zero CoG offset
if ~isempty(GE_OFF)    && GE_OFF;    GE    = 0; end
if ~isempty(DELAY_OFF) && DELAY_OFF; delay = 0; end

%% =========================================================================
%  PLASMC GAINS (from InitGains_Comparison.m — tuned for comparison study)
% =========================================================================
K_ctrl = struct();

K_ctrl.p_10     = K.p_10;                     % sensor-half, used for r_e normalization (Approach 2)

K_ctrl.rp = diag([9.0, 9.0]);                 % Combo D: x1.5 from 6.0 (deep-sweep precision winner, -25% maxXY)
K_ctrl.ri = diag([0.1, 0.1]);
K_ctrl.rd = diag([1.4375, 1.4375]);           % Combo D: x1.25 from 1.15 (D-damping pair for rp x1.5; recovers soft margin)

K_ctrl.gamma_2  = [0.2, 0.2, 0.2];            % prior 25/25 baseline
K_ctrl.p_20     = [25.0; 25.0; 4.0];  % vertical tightened (deep-sweep, -4.8% aggT)
K_ctrl.p_2inf   = [2.5;  2.5;  1.5 ];          % reverted from Combo A (z-tightening acted as speed knob under FW=11)

K_ctrl.Omega   = diag([0.05, 0.05, 0.025]);   % vertical bumped 4x (0.006->0.025) to close IC4 hover-fail after Approach 2
K_ctrl.Gamma   = diag([0.4375, 0.5,   0.75 ]); % lateral symmetry lock (IC=±2)
K_ctrl.P       = diag([1.5,   1.5,   5.0  ]);
K_ctrl.N       = diag([0.02,  0.02,  0.02 ]);   % N_z 0.05->0.02 (PX4 value): kappa_z adaptation too fast on noisy flow -> runaway on funnel breach (CONTROLLER_PARITY.md:61)
K_ctrl.kappa_0 = [0.125; 0.125; 0.25];
K_ctrl.E       = diag([1.0,   1.0,   0.5  ]);  % E_z 1.0->0.5 baked 2026-06-20: tighter descent boundary layer engages kappa switching on the Z limit cycle -> -8%, better h_ez. See vdf_params.

% ---- Middle-loop SMC authority hooks (globals; default = no change) -------
% Override the LATERAL (xy) diagonal entries only (z untouched) to test the
% "delivery" lever (the SMC isn't reaching: V_h_e stuck off-zero). Gamma=reaching
% rate, E=boundary layer (smaller=stiffer), P=kappa-leakage, kappa0=initial gain.
global GAMMA_XY_OVERRIDE E_XY_OVERRIDE P_XY_OVERRIDE KAPPA0_XY_OVERRIDE N_XY_OVERRIDE N_Z_OVERRIDE;
if ~isempty(GAMMA_XY_OVERRIDE);  K_ctrl.Gamma(1,1)=GAMMA_XY_OVERRIDE(1); K_ctrl.Gamma(2,2)=GAMMA_XY_OVERRIDE(end); end
if ~isempty(E_XY_OVERRIDE);      K_ctrl.E(1,1)=E_XY_OVERRIDE(1);         K_ctrl.E(2,2)=E_XY_OVERRIDE(end);         end
if ~isempty(P_XY_OVERRIDE);      K_ctrl.P(1,1)=P_XY_OVERRIDE(1);         K_ctrl.P(2,2)=P_XY_OVERRIDE(end);         end
if ~isempty(KAPPA0_XY_OVERRIDE); K_ctrl.kappa_0(1)=KAPPA0_XY_OVERRIDE(1);K_ctrl.kappa_0(2)=KAPPA0_XY_OVERRIDE(end);end
if ~isempty(N_XY_OVERRIDE);      K_ctrl.N(1,1)=N_XY_OVERRIDE(1);         K_ctrl.N(2,2)=N_XY_OVERRIDE(end);         end
if ~isempty(N_Z_OVERRIDE);       K_ctrl.N(3,3)=N_Z_OVERRIDE(1);          end
% Velocity-funnel initial width p_20 lateral override (z untouched). NB: p_s_0
% (SEN/position funnel) is FoV-locked, do NOT override it; p_20 is free.
global P20_XY_OVERRIDE P20_Z_OVERRIDE P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE GAMMA2_XY_OVERRIDE;
if ~isempty(P20_XY_OVERRIDE);    K_ctrl.p_20(1)=P20_XY_OVERRIDE(1);      K_ctrl.p_20(2)=P20_XY_OVERRIDE(end); end
if ~isempty(P20_Z_OVERRIDE);     K_ctrl.p_20(3)=P20_Z_OVERRIDE(1);       end  % descent flow funnel (combined-barrier coupling)
% Flow-funnel TERMINAL tightness (combined-barrier chase-lag / terminal-velocity lever)
if ~isempty(P2INF_XY_OVERRIDE);  K_ctrl.p_2inf(1)=P2INF_XY_OVERRIDE(1);  K_ctrl.p_2inf(2)=P2INF_XY_OVERRIDE(end); end
if ~isempty(P2INF_Z_OVERRIDE);   K_ctrl.p_2inf(3)=P2INF_Z_OVERRIDE(1);   end  % descent-axis terminal-velocity lever (per-axis vz margin)
if ~isempty(GAMMA2_XY_OVERRIDE); K_ctrl.gamma_2(1)=GAMMA2_XY_OVERRIDE(1);K_ctrl.gamma_2(2)=GAMMA2_XY_OVERRIDE(end); end

% Hard clamp on the adaptive gain kappa (anti-runaway backstop; PX4 KAPPA_MAX).
% Default off. KAPPA_MAX_Z clamps the vertical axis only; KAPPA_MAX clamps all 3.
global KAPPA_MAX KAPPA_MAX_Z;

% Geometric SO(3) attitude gains (tuned for X500 Gazebo inertia)
K_ctrl.kR     = diag([2.5, 1.5, 0.5]);  % roll 1.5->2.5 baked 2026-06-20: Y-attitude damping kills the Liss-IC3 terminal limit cycle -> noiseless 25/25 + full +/-40% (sharp optimum; see vdf_params)
K_ctrl.kOmega = diag([0.3, 0.3, 0.2]);  % yaw-rate 0.1->0.2 baked 2026-06-20: root fix for the yaw limit cycle that pumps the lateral cycles (see vdf_params)
% Geometric SO(3) tracking-gain hooks (globals; default = no change). 3-vectors
% [roll pitch yaw] -> diag. Test whether faster/slower attitude tracking changes
% the early roll establishment under the thrashing R_d command.
global KR_OVERRIDE KOMEGA_OVERRIDE;
if ~isempty(KR_OVERRIDE);     K_ctrl.kR     = diag(KR_OVERRIDE(:));     end
if ~isempty(KOMEGA_OVERRIDE); K_ctrl.kOmega = diag(KOMEGA_OVERRIDE(:)); end

% Integral attitude term (k_I·∫e_R) — rejects the CONSTANT body-frame torque from
% a CoG offset (r_cog × F), the pinned root of the IC5 noisy failures (the
% geometric law is P+rate only -> cannot null a constant torque). Default 0
% (off); KIR_OVERRIDE sets the [roll pitch yaw] gain. ie_R clamp = anti-windup.
K_ctrl.kI_R = diag([0, 0, 0]);
global KIR_OVERRIDE;
if ~isempty(KIR_OVERRIDE); K_ctrl.kI_R = diag(KIR_OVERRIDE(:)); end
ie_R_max = 0.5;   % anti-windup clamp on ∫e_R

% Thrust-scaled adaptive CoG feed-forward (default-OFF). The CoG offset injects a
% PURELY THRUST-PROPORTIONAL body torque: UAVDyn_robust gives tau_d = r_cog x f_b
% with f_b=[0;0;-T] -> tau_d = T*[-dy; dx; 0] (xy only, zero yaw). This is the
% pinned root of the IC5 noisy failures. A plain integral term (kI_R) fails because
% it ignores the thrust regressor (integrates e_R regardless of T -> lags, can
% destabilize the fast loop). Here a Lee-style geometric adaptive law estimates the
% per-thrust coefficient theta_hat ~ [-dy; dx] and cancels with tau_ff = -T*theta_hat.
% Lyapunov-grounded adaptation on the thrust-scaled composite attitude error
% (e_Omega + c2*e_R)_xy; the c2*e_R term makes the constant torque observable at
% steady state (e_Omega->0). theta_hat clamped (anti-windup); |r_cog|<=5mm -> bound 0.02.
% BAKED 2026-06-15: GAMMA_COG default 0->0.005. IC1-5 gate (validate_cogff.m) passed
% NO-REGRESSION on all 5 canonical ICs (IC1/centered stays perfect) and IMPROVES IC3/IC4/IC5
% (base SP 27/30 -> 29/30); fixes the IC5 noisy CoG-offset fly-aways (seed4 TL->land,
% seed6 78m->2.8m). Set GAMMA_COG=0 to disable. See [[feedback_cog_adaptive_feedforward]].
global GAMMA_COG COG_C2 COG_MAX;
if isempty(GAMMA_COG); gamma_cog = 0.005; else; gamma_cog = GAMMA_COG(1); end
if isempty(COG_C2);    cog_c2    = 2.0; else; cog_c2    = COG_C2(1);    end
if isempty(COG_MAX);   cog_max   = 0.02; else; cog_max  = COG_MAX(1);   end
% COG_LEAK = sigma-modification leakage rate [1/s]. Lets GAMMA_COG be larger (fast
% EARLY convergence — the non-SP IC5 seeds 4 & 6 fail because theta_hat converges
% too late, t~3.4s/2.1s) while the leakage bleeds off the steady-state bias that a
% plain high gamma accrues (which over-adapts -> lateral offset -> breaks precision).
global COG_LEAK;
if isempty(COG_LEAK);  cog_leak  = 0;   else; cog_leak = COG_LEAK(1);   end

% Yaw adaptive SMC — generates heading reference psi_d (no compass)
% e_a = V_s(4) - V_s_d(4) = alpha - alpha_d   (image-based, Eq. 19)
% psi_d(t) = psi_d(t-1) + u_a*dt              (integrated ASMC rate)
% R_d uses psi_d as heading vector; geometric controller tracks R_d
% Yaw ASMC now drives a heading reference (guidance), not a rate setpoint.
% Gains slowed to match ~0.8 rad/s natural slew for |e_a|=pi/2 errors.
K_ctrl.Omega_a   = 0.5;  % reverted 0.8 -> 0.5 to curb integral windup at high wz
K_ctrl.Gamma_a   = 0.5;
K_ctrl.n_a       = 1.0;   % raised 0.1 -> 1.0: faster kappa_a rise to tighten startup transient
K_ctrl.p_a       = 2;
K_ctrl.kappa_a_0 = 2.0;   % pre-seed above wz=1.5 so sat*kappa_a can provide DC feed-forward
K_ctrl.E_a       = 3.0;   % wide boundary layer to smooth sat*kappa_a at kappa_a_0=2.0

% Target-visibility CBF (camera-plane theta-QP) — replaces the cone clamp.
% Ported + validated from PX4 (docs/CBF_visibility.pdf, src/cbf_visibility.py).
K_ctrl.theta_cap = deg2rad(60);          % post-QP deliverable-tilt cap
% Probe hooks (globals, survive `clear`): relax visibility limits to test whether
% the IC5 fly-away is tilt-vs-visibility coupling (recovers) or a control
% instability (doesn't). Empty => defaults.
global THETA_CAP_OVERRIDE FOV_INSET_OVERRIDE;
if ~isempty(THETA_CAP_OVERRIDE); K_ctrl.theta_cap = deg2rad(THETA_CAP_OVERRIDE); end
fov_inset_px     = 15;                    % corner safety margin (px): slack for the
                                         % one-step linearization + attitude-slew lag
                                         % so the realized corner stays inside the true
                                         % res/2 edge; matches the old rho_fov_0 inset
if ~isempty(FOV_INSET_OVERRIDE); fov_inset_px = FOV_INSET_OVERRIDE; end
phi_max_cbf      = ([res(1); res(2)]/2 - fov_inset_px) / f;   % inset FoV-edge tangent
                                         % half-extent, C_nP-axis order (NB: NOT K.p_10,
                                         % which is axis-swapped [res(2);res(1)]/2f)

% Precision: PPC funnel on the normalized position error s_e_n (SEN_FUNNEL) —
% replaces the legacy outer PID. Decoupled from visibility (the CBF owns that).
% Back-mapped form (FUNNEL_CBF_DESIGN.md §9); reuses the outer rp/ri/rd gains.
K_ctrl.gamma_s     = diag([0.5, 0.5]);   % funnel contraction rate
K_ctrl.p_s_0       = [1.2; 1.2];         % initial normalized-error envelope
K_ctrl.p_s_inf     = [0.35; 0.35];       % terminal envelope floor (angular; ~0.07 m at Z=0.2)
K_ctrl.izeta_s_max = 5.0;                % anti-windup clamp on the zeta_s integral

% ---- COMBINED-BARRIER sliding surface (control_formulation.tex S-III) -------
% Gated mode (default OFF = the back-mapped SEN funnel above). Replaces the
% back-map with the position barrier zeta_r entering the sliding surface DIRECTLY:
% lateral sigma_k = zeta_h_k + chi_r*zeta_r_k (PD), descent sigma_3 = zeta_h_3 +
% chi_z*int(zeta_h_3) (PI, unchanged). h_d uses the setpoint rate V_sd (replacing
% V_ds_d). No back-map -> no G_s^-1->0 demand-starvation (the IC5 deficit).
global COMBINED_BARRIER CB_DROP_SDDOT CB_SDDOT_TAU CB_SDOT_FILT;
combined_barrier = ~isempty(COMBINED_BARRIER) && COMBINED_BARRIER(1) ~= 0;
% s_ddot feedforward (d/dt of the measured s_dot in h_d) handling in c_h:
%   CB_DROP_SDDOT : drop it entirely (kappa absorbs it as d_h) — blunt, loses target-accel FF.
%   CB_SDDOT_TAU>0: LPF it (time constant tau) — keeps the smooth target-accel feedforward,
%                   low-passes out the 1/z-inflated terminal spike. Preferred over dropping.
% s_ddot-drop is part of the validated combined-barrier design -> default-ON in
% combined mode (set CB_DROP_SDDOT=0 to force it off); always off in back-mapped mode.
cb_drop_sddot = combined_barrier && (isempty(CB_DROP_SDDOT) || CB_DROP_SDDOT(1) ~= 0);
cb_sddot_tau  = 0; if ~isempty(CB_SDDOT_TAU); cb_sddot_tau = CB_SDDOT_TAU(1); end
K_ctrl.p_r_0   = [1.2; 1.2];             % image-feature funnel initial half-width (r_bar_e = s_e/phi_max, FoV units)
K_ctrl.p_r_inf = [1.0; 1.0];             % terminal floor — FoV-consistent (proof Standing Condition 1: p_r_inf>=1)
K_ctrl.xi_r    = diag([0.10, 0.10]);     % funnel contraction rate Xi_r
K_ctrl.chi_r   = [1.15; 1.15];           % surface gain (manifold |zeta_r|~|zeta_h|/chi_r). Baked 0.85->1.15 (2026-06-20): clears Sinusoidal +40% precision edge -> full +/-40% guaranteed, keeps 25/25 (1.2 regresses Liss-IC3)
K_ctrl.chi_z   =  0.1;                    % descent surface gain. 0.025(=Omega_z)->0.1 baked 2026-06-20: stronger descent integral drives h_ez->0 -> kills the terminal 1/z fly-away (fly-aways -91%); decoupled from Omega(3,3). See vdf_params.
phi_max_xy     = [res(1); res(2)] / (2*f);   % half-FoV tangent (C_nP-axis order) for r_bar_e
global CHI_R_OVERRIDE PR0_OVERRIDE PRINF_OVERRIDE;
if ~isempty(CHI_R_OVERRIDE); K_ctrl.chi_r   = CHI_R_OVERRIDE(:); end
if ~isempty(PR0_OVERRIDE);   K_ctrl.p_r_0   = PR0_OVERRIDE(:);   end
if ~isempty(PRINF_OVERRIDE); K_ctrl.p_r_inf = PRINF_OVERRIDE(:); end  % position-funnel floor (precision lever)
% Combined-barrier uses a tighter lateral optic-flow-funnel floor p_2inf_xy=0.5
% (chase-lag terminal velocity; proof manifold |zeta_r|~|zeta_h|/chi_r). p_2inf is
% the SHARED flow funnel -> apply only in combined mode, only if not overridden.
if combined_barrier && isempty(P2INF_XY_OVERRIDE)
    K_ctrl.p_2inf(1) = 0.5; K_ctrl.p_2inf(2) = 0.5;
end

% ---- SEN_FUNNEL tuning hooks (globals; default = no change) -------------
% Set BEFORE calling the canonical script (survive the top-of-file `clear`).
% SEN_CONTAIN_MODE: 0 = legacy soft-clip (current); 1 = hard outlier-
%   containment — on breach |s_e_n|>=p_s, EXPAND the effective funnel to admit
%   the error (p_s_eff = |s_e_n|/(1-margin)) so the back-mapped barrier stays
%   valid and the inward demand scales with the actual error (~-1.61*p_s_eff
%   ∝ |s_e_n|) instead of collapsing to ∝ p_s_inf (FUNNEL_CBF_DESIGN.md §9).
global SEN_PS0_OVERRIDE SEN_PSINF_OVERRIDE SEN_GAMMAS_OVERRIDE ...
       SEN_IZETASMAX_OVERRIDE SEN_CONTAIN_MODE SEN_RP_OVERRIDE ...
       SEN_RI_OVERRIDE SEN_RD_OVERRIDE;
if ~isempty(SEN_PS0_OVERRIDE);       K_ctrl.p_s_0   = SEN_PS0_OVERRIDE(:);   end
if ~isempty(SEN_PSINF_OVERRIDE);     K_ctrl.p_s_inf = SEN_PSINF_OVERRIDE(:); end
if ~isempty(SEN_GAMMAS_OVERRIDE);    K_ctrl.gamma_s = diag(SEN_GAMMAS_OVERRIDE(:)); end
if ~isempty(SEN_IZETASMAX_OVERRIDE); K_ctrl.izeta_s_max = SEN_IZETASMAX_OVERRIDE; end
if ~isempty(SEN_RP_OVERRIDE);        K_ctrl.rp = diag(SEN_RP_OVERRIDE(:));   end
if ~isempty(SEN_RI_OVERRIDE);        K_ctrl.ri = diag(SEN_RI_OVERRIDE(:));   end
if ~isempty(SEN_RD_OVERRIDE);        K_ctrl.rd = diag(SEN_RD_OVERRIDE(:));   end
if isempty(SEN_CONTAIN_MODE);        sen_contain_mode = 0; ...
else;                                sen_contain_mode = SEN_CONTAIN_MODE; end
% SEN_RECOVER_ST: past-breach STRONG recovery (default-OFF). On breach, pin S_s at
% this small target ST via p_s_eff=|s_e_n|/ST, so the back-mapped P-demand becomes
% -rp*[g(ST)/(2*ST)]*|s_e_n| with g(S)=(1-S^2)log((1+S)/(1-S)). Since g(S)/S -> 2 as
% S->0, a small ST recovers ~FULL -rp linear restoring gain (vs the legacy
% containment's 0.95 pin = 0.19*rp), eliminating the anti-restoring collapse beyond
% breach (marginal P-gain flips +ve for |S|>0.648). [] = off. Takes precedence over
% SEN_CONTAIN_MODE. CAVEAT: outer-demand fix only -- whether RECOVERY improves
% depends on the inner loop delivering it (prior SEN_RECOVERY_K A/B said inner-bound).
global SEN_RECOVER_ST;
if isempty(SEN_RECOVER_ST); sen_recover_st = 0; else; sen_recover_st = SEN_RECOVER_ST(1); end

% Target trajectory (override hook; default Circular as in the canonical single-run).
global TRAJ_OVERRIDE;
if isempty(TRAJ_OVERRIDE); traj_type = "Circular"; else; traj_type = string(TRAJ_OVERRIDE); end

% Feature-space SETPOINT TRAJECTORY (waypoint-style reference governor). Reuses
% the funnel's exp-decay operator on the SETPOINT (not the envelope): the SEN
% error tracks a moving V_s_ref that walks from V_s(0) to V_s_d at rate
% SREF_GAMMA, so the initial demand is bounded/smooth (eta(0)=0) instead of
% barrier-amplified. [] = off (regulate to fixed V_s_d, current behavior).
global SREF_GAMMA;
sref_on = ~isempty(SREF_GAMMA);
if sref_on; gamma_ref = SREF_GAMMA(1); else; gamma_ref = 0; end

% Extra conditioning of V_dh_d (noisy desired-flow derivative feeding the c-term):
% DH_D_CAP = hard clip magnitude (PX4 DH_D_MAX-style spike killer, no lag);
% DH_D_TAU = first-order LPF time constant (smooths, adds lag). 0/[] = off.
% BAKED 2026-06-15: DH_D_CAP default 20 (was 0). The c-term differentiates the
% noisy desired flow; raw_dh_d spikes ~O(100) on funnel firing -> ±5 m/s^2
% command thrash that noise-weakens/flips the lateral cmd (IC5 seed-6 runaway).
% A hard cap (NOT an LPF -- lag hurt, SP 9->5) is the PX4 DH_D_MAX analogue.
% Validated no-regression on the canonical 5 ICs (noiseless+noisy n=5: SP/land
% byte-identical to base; cap only engages on pathological spikes). LPF dead-end.
global DH_D_CAP DH_D_TAU;
if isempty(DH_D_CAP); dhd_cap = 20; else; dhd_cap = DH_D_CAP(1); end
if isempty(DH_D_TAU); dhd_tau = 0; else; dhd_tau = DH_D_TAU(1); end
dh_d_prev = zeros(3,1);
sddot_filt = zeros(3,1);   % LPF state for the filtered-s_ddot combined-barrier variant

% C_SIMPLE = first-principles c-term (manuscript form, default-OFF).
% Derived from scratch: differentiating the PRIMITIVE optic flow h=v/z ONCE
% gives c_tilde_h = -psi_dot_b*(e3 x h) - (e3.h)*h, i.e. clean IMU yaw-rate
% frame rotation + loom only. The default c carries spurious second-derivative
% (s-double-dot) terms (w x (w x s), 2 w x h, dw x s) built from the NOISY
% optic-flow-recovered w and its derivative dw -> the c-term noise channel
% behind the IC5 seed-4/seed-6 runaways. C_SIMPLE=1 replaces that block with
% the manuscript c_tilde_h using the clean body yaw rate psi_dot_b.
global C_SIMPLE;
c_simple = ~isempty(C_SIMPLE) && C_SIMPLE(1) ~= 0;

%% =========================================================================
%  PRE-ALLOCATE ARRAYS
% =========================================================================
N_steps = numel(tRange);

U_DS      = zeros(4,  N_steps);
X_DS      = zeros(13, N_steps + 1);   X_DS(:,1) = x_c;
V_X_DS    = zeros(24, N_steps);
D_DS      = zeros(15, N_steps);       % [V_h_d(3); I_a_cd(3); e_R(3); tau(3); T(1); psi_d(1); u_a(1)]
P_DS      = zeros(2,  12, N_steps);

x_t       = zeros(7,  N_steps);
dx_t      = zeros(6,  N_steps);

V_h_d     = zeros(3,  N_steps);
V_h_e     = zeros(3,  N_steps);
I_a_cd    = zeros(3,  N_steps);

% Inner-loop logging (geometric SO(3))
eR_log    = zeros(3,  N_steps);
B_tau_cd_log = zeros(3, N_steps);
B_T_cd    = zeros(1,  N_steps);
psi_d_log = zeros(1,  N_steps);
u_a_log   = zeros(1,  N_steps);

% Delay buffer for u_2 = [tau; T]
u_2_buf   = zeros(4,  N_steps);

% Raw visual signal storage for Savitzky-Golay filter (ACTUAL mode)
V_s_raw   = zeros(4,  N_steps);
V_h_raw   = zeros(3,  N_steps);
V_w_raw   = zeros(3,  N_steps);
V_dw_raw  = zeros(3,  N_steps);

% V_w_a buffer for smooth derivative
raw_dw_a  = zeros(3,  N_steps + 3);

% _prev variables
V_2nP_i_prev = zeros(8, 1);
% --- SCALE-FREE moment loom estimator (USE_MOMENT_LOOM, default off) --------
% V_h(3) = vz/z from the apparent-area rate: -1/2 * d(ln M)/dt, M = 2nd-moment
% corner spread. Only the FRACTIONAL rate is used (M ~ (size/Z)^2 -> dlnM/dt =
% -2 Zdot/Z) -> independent of marker size, units, depth; fixed factor 1/2, no
% calibration. Rotation-immune (area-preserving curl/shear). Replaces the
% ill-conditioned pinv(L_s) loom (sigma_min~2 weak mode). [[project_moment_loom]]
M_prev = [];  raw_dlnM = zeros(1,4);  Vh3_mom = 0;
V_w_i_prev   = zeros(3, 1);
V_w_a_prev   = zeros(3, 1);

% Initial desired heading = initial UAV yaw (from identity quaternion -> 0)
yaw_init = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
                 1 - 2*(q_c(3)^2 + q_c(4)^2));
psi_d    = yaw_init;

% I_a_cd low-pass filter state (matches tau_w=0.08s ~2Hz cutoff); mimics
% PID cascade filter budget so R_d/e_R don't chatter on noisy outer output.
tau_ia      = 0.08;   % LPF tau on I_a_cd (noise absorption dominates phase lag)
alpha_ia    = tau_ia / (tau_ia + dt);
I_a_cd_filt = -g;

% Initialise image-block variables so they persist across ZOH steps
V_nP_i  = zeros(2, 4);
V_nP_a  = zeros(2, 4);
C_nP    = zeros(2, 4);
L_s     = zeros(8, 6);
V_s_i   = zeros(4, 1);
V_s_a   = zeros(4, 1);
V_h_i   = zeros(3, 1);
V_h_a   = zeros(3, 1);
V_w_i   = zeros(3, 1);
V_dw_i  = zeros(3, 1);
V_s     = zeros(4, 1);
V_h     = zeros(3, 1);
V_w     = zeros(3, 1);
V_dw    = zeros(3, 1);

% PLASMC-specific arrays
p_2         = zeros(3, N_steps);
dp_2        = zeros(3, N_steps);
S_2         = zeros(3, 3, N_steps);
G_2         = zeros(3, 3, N_steps);
zeta_2      = zeros(3, N_steps);
izeta_2     = zeros(3, N_steps);
sigma       = zeros(3, N_steps);
raw_dh_d    = zeros(3, N_steps + 3);

% Outer-loop SEN_FUNNEL: back-mapped PPC on normalized position error s_e_n
V_s_e        = zeros(2, N_steps);
V_s_e_n      = zeros(2, N_steps);
p_s          = zeros(2, N_steps);
dp_s         = zeros(2, N_steps);
S_s          = zeros(2, 2, N_steps);
G_s          = zeros(2, 2, N_steps);
zeta_s       = zeros(2, N_steps);
izeta_s      = zeros(2, N_steps);
raw_dzeta_s  = zeros(2, N_steps + 3);
sen_breach   = false(2, N_steps);

% Combined-barrier position-barrier logging (lateral)
r_bar_e      = zeros(2, N_steps);
p_r          = zeros(2, N_steps);
S_r          = zeros(2, N_steps);
zeta_r       = zeros(2, N_steps);
dzeta_r      = zeros(2, N_steps);
raw_ds_meas  = zeros(2, N_steps + 3);   % measured finite-diff centroid rate (h_d feedforward)
h_d_noS      = zeros(3, N_steps);        % h_d WITHOUT the s_dot term (transport+descent) for s_ddot-drop variant

% Target-visibility CBF logging + persistent state
theta_cur_log  = zeros(1, N_steps);
theta_cone_log = zeros(1, N_steps);
cbf_ok_log     = false(1, N_steps);
cbf_state = struct('delta_prev', [], 'ddelta_ref', zeros(2,1), ...
                   'decode_fail_n', 0, 'phase2_alpha', 0.0, ...
                   'cr_prev', [], 'd', zeros(2,1), 'Lw2_prev', []);
th_safe = [];
th_safe_prev_n = 0;   % previous CBF lean magnitude (for TILT_RATE_MAX limit)
ie_R = zeros(3,1);    % integral of attitude error (CoG-torque rejection)
thetahat_cog = zeros(2,1);   % adaptive per-thrust CoG-torque coeff [-dy; dx] (xy)
THETAHAT_COG = zeros(2, N_steps);   % log of CoG estimate (diagnostics)
kappa       = [K_ctrl.kappa_0, zeros(3, N_steps)];
kappa_a     = [K_ctrl.kappa_a_0, zeros(1, N_steps)];
e_a         = zeros(1, N_steps);
ie_a        = zeros(1, N_steps);

flag = false;
landed = false;

% FoV failure flag (Approach 2): any physical-corner pixel leaving the
% sensor box [-res(1)/2, res(1)/2] x [-res(2)/2, res(2)/2] strictly counts
% as controller failure — the camera has lost the target.
fov_fail   = false;
fov_fail_t = NaN;

%% Solving for Control Law and then the System Dynamics using this Control Law
% Finding System State Evolution from t=t_0 to t=t_end
for idx=1:N_steps
% *************************************************************************
% Compute rotation matrices and yaw
% *************************************************************************
    I_R_C = quat2rotm(q_c');
    E_cr = quat2eul(q_c', 'XYZ');

% Extract yaw from current rotation
    yaw = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
        1 - 2*(q_c(3)^2 + q_c(4)^2));

% Virtual camera rotation
    I_R_V = rotz(rad2deg(yaw));

% *************************************************************************
% Simulating moving target
% *************************************************************************
    % Trajectory selected via TRAJ_OVERRIDE (default "Circular"); options:
    % "Static" | "Linear" | "Sinusoidal" | "Lissajous" | "Circular"
    traj_t = traj_Gen((idx-1)*dt, traj_type);

    x_t(:,idx) = traj_t(:,1);
    dx_t(:,idx) = traj_t(1:end-1,2);

    I_R_T = quat2rotm(x_t(4:7,idx)');


    if mod(idx-1,ZOH) == 0
    % *************************************************************************
    % Simulating Image Feature Points in Image Plane
    % *************************************************************************
        I_nP3 = I_R_T * T_nP3 + x_t(1:3,idx);
        C_nP3 = transpose(I_R_C)*(I_nP3 - I_p_c);
        C_s_tc = transpose(I_R_C)*(x_t(1:3,idx) - I_p_c);
        C_nP = (f/(C_s_tc(3)+zf))*C_nP3(1:2,:);

        if NOISE && ~abl_pix
            % Depth-dependent pixel noise: sigma_px(z) = 0.3 + 0.5/(z+0.5) [px]
            z_dep = max(abs(C_s_tc(3)), 0.1);
            sigma_px = px_sigma0 + px_sigma1 / (z_dep + px_depth_offset);
            C_nP = C_nP + (sigma_px / f) * randn(size(C_nP));
        end
        if NOISE && ~abl_out
            % Outlier injection (detector glitch)
            if rand < outlier_prob
                col = randi(size(C_nP,2));
                C_nP(:,col) = C_nP(:,col) + (outlier_mag / f) * sign(randn(2,1));
            end
        end

        % FoV failure check — strict.  Any physical-corner pixel outside the
        % sensor box terminates the run with success=false.
        if any(abs(C_nP(1,:)) > res(1)/2) || any(abs(C_nP(2,:)) > res(2)/2)
            fov_fail   = true;
            fov_fail_t = tRange(idx);
            fprintf('  BREAK: FoV violation at idx=%d (t=%.2f), max|u|=%.1f, max|v|=%.1f\n', ...
                idx, tRange(idx), max(abs(C_nP(1,:))), max(abs(C_nP(2,:))));
            break;
        end

    % *************************************************************************
    % Computing Scale-Independent Target Image Parameters
    % *************************************************************************
        V_R_C = I_R_V' * I_R_C;
        rays = [C_nP; f*ones(1,size(C_nP, 2))];
        vr = V_R_C * rays;
        V_nP_i = f*vr(1:2,:)./vr(3,:);
        for j=1:size(C_nP,2)
            L_s(2*j-1:2*j,:) = [f, 0, -V_nP_i(1,j), -V_nP_i(1,j)*V_nP_i(2,j)/f, (f^2+V_nP_i(1,j)^2)/f, -V_nP_i(2,j);
                0, f, -V_nP_i(2,j), -(f^2+V_nP_i(2,j)^2)/f, V_nP_i(1,j)*V_nP_i(2,j)/f, V_nP_i(1,j)];
        end
        V_s_i = image_feature(V_nP_i/f);

        V_2nP_i = reshape(V_nP_i,[],1);

        if idx == 1
            dPdt = zeros(size(V_2nP_i));
        else
            dPdt = (V_2nP_i - V_2nP_i_prev)/dt/ZOH;
        end
        V_2nP_i_prev = V_2nP_i;

        V_v_i = pinv(L_s,4)*dPdt;
        V_h_i = V_v_i(1:3);
        V_w_i = V_v_i(4:6);

        if idx == 1
            V_dw_i = zeros(3,1);
        else
            V_dw_i = (V_w_i - V_w_i_prev)/dt/ZOH;
        end
        V_w_i_prev = V_w_i;

    % *************************************************************************
    % Calculating Scale-Independent Target Image Parameters Analytically
    % *************************************************************************
        V_nP3 = transpose(I_R_V)*(I_nP3 - I_p_c);
        V_s_tc = transpose(I_R_V)*(x_t(1:3,idx) - I_p_c);
        V_nP_a = (f/(V_s_tc(3)+zf))*V_nP3(1:2,:);
        V_s_a = image_feature(V_nP_a/f);
        V_h_a = transpose(I_R_V) * (dx_t(1:3,idx) - I_v_c) / (V_s_tc(3)+zf);
        % SCALE-FREE moment loom: M = 2nd-moment spread of the corners (any units; only
        % its FRACTIONAL rate is used -> scale/depth-invariant). V_h(3) = -1/2 d(ln M)/dt.
        cenM = mean(V_nP_i, 2);
        M_now = sum(sum((V_nP_i - cenM).^2));
        if isempty(M_prev) || M_prev <= 0 || M_now <= 0
            dlnM = 0;
        else
            dlnM = (log(M_now) - log(M_prev)) / (dt*ZOH);
        end
        M_prev = M_now;
        raw_dlnM = [raw_dlnM(2:end), dlnM];
        Vh3_mom = -0.5 * smooth4(raw_dlnM);
    end

    I_w_c = I_R_C * B_w_c;
    V_w_a = transpose(I_R_V) * (dx_t(4:6,idx) - [0;0;I_w_c(3)]);

    if idx == 1
        raw_dw_a(:,idx+3) = zeros(3,1);
    else
        raw_dw_a(:,idx+3) = (V_w_a - V_w_a_prev)/dt;
    end
    V_dw_a = smooth4(raw_dw_a(:,end-3:end));
    V_w_a_prev = V_w_a;

% *************************************************************************
% Select actual vs analytical image parameters (Savitzky-Golay filter)
% *************************************************************************
    if ACTUAL
        V_s_raw(:,idx) = V_s_i;
        V_h_raw(:,idx) = V_h_i;
        V_w_raw(:,idx) = V_w_i;
        V_dw_raw(:,idx) = V_dw_i;

        if idx < FILTER_WINDOW
            V_s  = mean(V_s_raw(:,  1:idx), 2);
            V_h  = mean(V_h_raw(:,  1:idx), 2);
            V_w  = mean(V_w_raw(:,  1:idx), 2);
            V_dw = mean(V_dw_raw(:, 1:idx), 2);
        else
            V_s_vec = sgolayfilt(V_s_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
            V_s = V_s_vec(:, end);
            V_h_vec = sgolayfilt(V_h_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
            V_h = V_h_vec(:, end);
            V_w_vec = sgolayfilt(V_w_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
            V_w = V_w_vec(:, end);
            V_dw_vec = sgolayfilt(V_dw_raw(:,idx-FILTER_WINDOW+1:idx),2,FILTER_WINDOW,[],2);
            V_dw = V_dw_vec(:, end);
        end

    else
        V_s = V_s_a;
        V_h = V_h_a;
        V_w = V_w_a;
        V_dw = V_dw_a;
    end
    % SCALE-FREE moment loom: replace the pinv-derived V_h(3) with the area-rate
    % divergence (decoupled, rotation-immune, no ill-conditioned inversion, no scale).
    % MOMENT_LOOM_GAIN default 1.0 (compensation hurts -> keep raw). [[project_moment_loom]]
    global USE_MOMENT_LOOM MOMENT_LOOM_GAIN;
    if ~isempty(USE_MOMENT_LOOM) && USE_MOMENT_LOOM(1) ~= 0
        klg = 1.0; if ~isempty(MOMENT_LOOM_GAIN); klg = MOMENT_LOOM_GAIN(1); end
        V_h(3) = klg * Vh3_mom;
    end

% *************************************************************************
% Early landing check (before controller can violate barrier at boundary)
% *************************************************************************
    alt_above = abs(I_p_c(3) - x_t(3,idx));
    xy_err   = norm(I_p_c(1:2) - x_t(1:2,idx));
    rel_vel  = norm(I_v_c - dx_t(1:3,idx));
    if alt_above <= zf
        precise = xy_err <= 0.08;
        soft    = rel_vel <= 0.2;
        fprintf('Landed at t = %.2f s  (alt=%.3fm, xy=%.3fm, v_rel=%.3fm/s, precise=%d, soft=%d)\n', ...
                tRange(idx), alt_above, xy_err, rel_vel, precise, soft);
        landed = true;
        break;
    end

% *************************************************************************
% Computing Desired Optical Flow (SEN_FUNNEL — back-mapped PPC on s_e_n)
% Precision-only position funnel on the normalized position error; visibility
% is enforced downstream by the target-visibility CBF on I_a_cd.
%   S_s = s_e_n/p_s -> zeta_s = log((1+S)/(1-S)) -> G_s
%   zeta_sd = -rp*zeta_s - ri*int(zeta_s) - rd*d(zeta_s)
%   V_ds_d  = G_s\zeta_sd + S_s*dp_s
% *************************************************************************
    if idx == 1; V_s_init = V_s(1:2); end             % initial measured feature
    if sref_on
        % moving setpoint: V_s_ref walks V_s(0) -> V_s_d (reuses p_s decay form)
        sref_decay  = exp(-gamma_ref * tRange(idx));
        V_s_ref     = V_s_d(1:2) + sref_decay * (V_s_init - V_s_d(1:2));
        dV_s_ref    = -gamma_ref * sref_decay * (V_s_init - V_s_d(1:2));  % feedforward
        V_s_e(:,idx)= V_s(1:2) - V_s_ref;             % small residual to track
    else
        V_s_e(:,idx)= V_s(1:2) - V_s_d(1:2);
        dV_s_ref    = [0;0];
    end
    V_s_e_n(:,idx) = V_s_e(:,idx) ./ K_ctrl.p_10;     % normalize by sensor half (logging / SEN funnel)

  if ~combined_barrier
    % ====================== BACK-MAPPED SEN FUNNEL (default) ======================
    p_s(:,idx)  = expm(-K_ctrl.gamma_s*tRange(idx)) * (K_ctrl.p_s_0 - K_ctrl.p_s_inf) + K_ctrl.p_s_inf;
    dp_s(:,idx) = -K_ctrl.gamma_s * expm(-K_ctrl.gamma_s*tRange(idx)) * (K_ctrl.p_s_0 - K_ctrl.p_s_inf);

    S_s_margin = 0.05;   % funnel saturation guard: keeps |zeta_s| bounded, G_s finite
    sen_breach(:,idx) = false;
    for j=1:2
        p_s_eff = p_s(j,idx);
        raw_ratio = V_s_e_n(j,idx) / p_s(j,idx);
        if abs(raw_ratio) >= 1.0
            sen_breach(j,idx) = true;
            if sen_recover_st > 0
                p_s_eff = abs(V_s_e_n(j,idx)) / sen_recover_st;
            elseif sen_contain_mode == 1
                p_s_eff = abs(V_s_e_n(j,idx)) / (1 - S_s_margin);
            end
        end
        S_s(j,j,idx) = V_s_e_n(j,idx) / p_s_eff;
        S_s(j,j,idx) = min(max(S_s(j,j,idx), -1+S_s_margin), 1-S_s_margin);
        zeta_s(j,idx) = log((1+S_s(j,j,idx))/(1-S_s(j,j,idx)));
        G_s(j,j,idx)  = (exp(zeta_s(j,idx)) + 1)^2/(2*exp(zeta_s(j,idx))*p_s_eff);
    end

    if idx == 1
        izeta_s(:,idx) = dt*zeta_s(:,idx);
        raw_dzeta_s(:,idx+3) = zeros(2,1);
    else
        izeta_s(:,idx) = izeta_s(:,idx-1) + dt*(zeta_s(:,idx-1) + zeta_s(:,idx))/2;
        raw_dzeta_s(:,idx+3) = (zeta_s(:,idx) - zeta_s(:,idx-1))/dt;
    end
    izeta_s(:,idx) = max(min(izeta_s(:,idx), K_ctrl.izeta_s_max), -K_ctrl.izeta_s_max);
    dzeta_s = smooth4(raw_dzeta_s(:,idx:idx+3));

    dzeta_sd  = -K_ctrl.rp*zeta_s(:,idx) - K_ctrl.ri*izeta_s(:,idx) - K_ctrl.rd*dzeta_s;
    V_ds_d_xy = G_s(:,:,idx)\dzeta_sd + S_s(:,:,idx)*dp_s(:,idx);
    if sref_on; V_ds_d_xy = V_ds_d_xy + dV_s_ref; end   % setpoint-trajectory feedforward
    V_ds_d    = [V_ds_d_xy; 0.0];

    V_h_d(:,idx) = V_ds_d + cross(V_w, V_s(1:3)) + (h_rd ...
        - dot(cross(V_w, V_s(1:3)), e3))*V_s(1:3);

  else
    % ============ COMBINED-BARRIER: position barrier zeta_r + h_d via V_sd ============
    % rotation feedforward (c_simple selects the corrected clean-IMU psi_dot_b form);
    % MEASURED finite-difference centroid rate s_dot (V_s_d=0 => s_e=centroid):
    % this is the h_d feedforward AND the r_bar_e rate (NOT a desired/PID rate) —
    % collapses h_e to the descent error (h.e3-h_rd)s; lateral via zeta_r in surface.
    % s_dot filter. Default = 2-pt diff + smooth4 (noisy under pixel noise). BETTER
    % (CB_SDOT_FILT=W>=3): causal Savitzky-Golay (quadratic least-squares) derivative
    % over the last W centroid samples -> strong noise rejection; the SAME fit yields a
    % clean s_ddot (2nd-deriv coeff) with NO double finite-difference.
    s_ddot_meas = [];
    if ~isempty(CB_SDOT_FILT) && CB_SDOT_FILT(1) >= 3
        W = round(CB_SDOT_FILT(1)); w0 = max(1, idx-W+1); seg = V_s_e(:, w0:idx); n = size(seg,2);
        if n >= 3
            T  = ((0:n-1) - (n-1))' * dt;          % sample times rel. to latest (=0)
            B  = [ones(n,1), T, T.^2];             % quadratic basis (poly order 2)
            Pd = (B'*B) \ B';                      % 3 x n least-squares projector
            s_dot_meas  = (Pd(2,:) * seg')';       % 1st-deriv at t=0  (2x1)
            s_ddot_meas = (2*Pd(3,:) * seg')';     % clean 2nd-deriv   (2x1)
        else
            s_dot_meas = (V_s_e(:,idx) - V_s_e(:,max(1,idx-1)))/dt;
        end
    else
        if idx == 1
            raw_ds_meas(:,idx+3) = [0;0];
        else
            raw_ds_meas(:,idx+3) = (V_s_e(:,idx) - V_s_e(:,idx-1))/dt;
        end
        s_dot_meas = smooth4(raw_ds_meas(:,idx:idx+3));
    end
    % transport feedforward; c_simple selects the corrected clean-IMU psi_dot_b form
    if c_simple
        phi_b   = atan2(I_R_C(3,2), I_R_C(3,3));
        theta_b = -asin(max(min(I_R_C(3,1), 1), -1));
        psi_dot_b = (B_w_c(2)*sin(phi_b) + B_w_c(3)*cos(phi_b)) / cos(theta_b);
        rot    = psi_dot_b * cross(e3, V_s(1:3));            % psi_dot_b (e3 x s)
        h_d_ff = h_rd * V_s(1:3);                            % rot . e3 = 0
    else
        rot    = cross(V_w, V_s(1:3));                       % w x s
        h_d_ff = (h_rd - dot(rot, e3)) * V_s(1:3);
    end
    % image-feature funnel + position barrier on r_bar_e = s_e_xy / phi_max (FoV units)
    r_bar_e(:,idx) = V_s_e(:,idx) ./ phi_max_xy;
    dr_bar_e       = s_dot_meas  ./ phi_max_xy;              % measured rate (rel deg 2)
    p_r(:,idx) = expm(-K_ctrl.xi_r*tRange(idx)) * (K_ctrl.p_r_0 - K_ctrl.p_r_inf) + K_ctrl.p_r_inf;
    dp_r       = -K_ctrl.xi_r * expm(-K_ctrl.xi_r*tRange(idx)) * (K_ctrl.p_r_0 - K_ctrl.p_r_inf);
    S_r_margin = 0.05;
    for j=1:2
        S_r(j,idx)    = min(max(r_bar_e(j,idx)/p_r(j,idx), -1+S_r_margin), 1-S_r_margin);
        zeta_r(j,idx) = log((1+S_r(j,idx))/(1-S_r(j,idx)));
        g_rj          = (exp(zeta_r(j,idx))+1)^2 / (2*exp(zeta_r(j,idx))*p_r(j,idx));
        dzeta_r(j,idx)= g_rj * (dr_bar_e(j) - S_r(j,idx)*dp_r(j));
    end
    % desired optic flow: h_d = s_dot(MEASURED) + transport + h_rd*s (blended surface)
    h_d_noS(:,idx) = rot + h_d_ff;                  % transport+descent only (no s_dot -> no s_ddot in dh_d)
    V_h_d(:,idx)   = [s_dot_meas; 0.0] + h_d_noS(:,idx);
  end

    V_h_e(:,idx) = V_h - V_h_d(:,idx);

% *************************************************************************
% Computing Outer Loop Control Inputs
% *************************************************************************
    p_2(:,idx) = expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * (K_ctrl.p_20 - K_ctrl.p_2inf) + K_ctrl.p_2inf;
    dp_2(:,idx) = -diag(K_ctrl.gamma_2) * expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * (K_ctrl.p_20 - K_ctrl.p_2inf);

    S_2_margin = 0.05;   % funnel saturation guard: keeps |zeta_2|<=3.66, G_2 finite
    for j=1:3
        S_2(j,j,idx) = V_h_e(j,idx) / p_2(j,idx);
        S_2(j,j,idx) = min(max(S_2(j,j,idx), -1+S_2_margin), 1-S_2_margin);
        zeta_2(j,idx) = log((1+S_2(j,j,idx))/(1-S_2(j,j,idx)));
        G_2(j, j,idx) = (exp(zeta_2(j,idx)) + 1)^2/(2*exp(zeta_2(j,idx))*p_2(j,idx));
    end

    if idx == 1
        izeta_2(:,idx) = dt*zeta_2(:,idx);
    else
        izeta_2(:,idx) = izeta_2(:,idx-1) + dt*(zeta_2(:,idx-1) + zeta_2(:,idx))/2;
    end
    % Anti-windup: clamp izeta_2 to prevent spike accumulation
    izeta_2_max = 5.0;   % 50/50 lock-in (synced with run_simulation.m)
    izeta_2(:,idx) = max(min(izeta_2(:,idx), izeta_2_max), -izeta_2_max);

    if ~combined_barrier
        sigma(:,idx) = zeta_2(:,idx) + K_ctrl.Omega*izeta_2(:,idx);
        chi_zeta_aug = K_ctrl.Omega*zeta_2(:,idx);   % chi*zeta_dot_aug (PI: d/dt int zeta = zeta)
    else
        % combined surface: lateral PD (zeta_h + chi_r*zeta_r), descent PI (zeta_h + chi_z*int zeta_h3)
        sigma(:,idx) = [ zeta_2(1,idx) + K_ctrl.chi_r(1)*zeta_r(1,idx);
                         zeta_2(2,idx) + K_ctrl.chi_r(2)*zeta_r(2,idx);
                         zeta_2(3,idx) + K_ctrl.chi_z   *izeta_2(3,idx) ];
        chi_zeta_aug = [ K_ctrl.chi_r(1)*dzeta_r(1,idx);   % chi*zeta_dot_aug = [chi_r.*dzeta_r; chi_z*zeta_h3]
                         K_ctrl.chi_r(2)*dzeta_r(2,idx);
                         K_ctrl.chi_z   *zeta_2(3,idx) ];
    end

% Computing Known System Dynamics (c)
    if idx == 1
        raw_dh_d(:,idx+3) = zeros(3,1);
    elseif combined_barrier && cb_drop_sddot
        raw_dh_d(:,idx+3) = (h_d_noS(:,idx) - h_d_noS(:,idx-1))/dt;   % drop s_ddot -> absorbed by kappa as d_h
    elseif combined_barrier && cb_sddot_tau > 0
        % FILTERED s_ddot: LPF the centroid-accel contribution -> keeps the smooth
        % target-accel feedforward, low-passes out the 1/z-inflated terminal spike.
        sddot_raw  = ((V_h_d(:,idx)-h_d_noS(:,idx)) - (V_h_d(:,idx-1)-h_d_noS(:,idx-1)))/dt;
        a_sdd      = cb_sddot_tau/(cb_sddot_tau+dt);
        sddot_filt = a_sdd*sddot_filt + (1-a_sdd)*sddot_raw;
        raw_dh_d(:,idx+3) = (h_d_noS(:,idx) - h_d_noS(:,idx-1))/dt + sddot_filt;
    elseif combined_barrier && ~isempty(s_ddot_meas)
        % KEEP s_ddot but use the CLEAN 2nd-deriv from the Savitzky-Golay fit (no double
        % finite-difference) -> removes the s_ddot noise at the source.
        raw_dh_d(:,idx+3) = [s_ddot_meas; 0.0] + (h_d_noS(:,idx) - h_d_noS(:,idx-1))/dt;
    else
        raw_dh_d(:,idx+3) = (V_h_d(:,idx) - V_h_d(:,idx-1))/dt;       % full s_ddot (double finite-diff)
    end
    V_dh_d = smooth4(raw_dh_d(:,idx:idx+3));
    if dhd_tau > 0                                   % optional LPF (smooths, lags)
        a_dhd = dhd_tau/(dhd_tau+dt);
        V_dh_d = a_dhd*dh_d_prev + (1-a_dhd)*V_dh_d;
    end
    if dhd_cap > 0                                   % optional hard cap (spike killer)
        V_dh_d = max(min(V_dh_d, dhd_cap), -dhd_cap);
    end
    dh_d_prev = V_dh_d;

    if c_simple
        % First-principles c_tilde_h (manuscript form): clean IMU yaw-rate
        % frame rotation + loom. psi_dot_b = ZYX yaw Euler rate from body rates
        % (matches the line-313 yaw extraction); roll/pitch from I_R_C.
        phi_b   = atan2(I_R_C(3,2), I_R_C(3,3));
        theta_b = -asin(max(min(I_R_C(3,1), 1), -1));
        psi_dot_b = (B_w_c(2)*sin(phi_b) + B_w_c(3)*cos(phi_b)) / cos(theta_b);
        c = -psi_dot_b*cross(e3, V_h) - dot(V_h, e3)*V_h - V_dh_d;
    else
        c = cross(V_dw, V_s(1:3)) + cross(V_w, cross(V_w, V_s(1:3))) ...
            + 2 * cross(V_w, V_h) - (dot(V_h + cross(V_w, V_s(1:3)), e3))*V_h - V_dh_d;
    end

    Theta = [- c + S_2(:,:,idx)*dp_2(:,idx) ...
        - G_2(:,:,idx)\chi_zeta_aug, eye(3)];
    Theta_norm = norm(Theta,'fro');

% Updating Control Parameter 'kappa' using RK-5 Method
    const_kappa = [K_ctrl.N; K_ctrl.P];
    u_kappa = [sigma(:,idx); Theta_norm];
    kappa(:,idx+1) = RK5(@(t, X) kappa_Solver(t, X, u_kappa, const_kappa, G_2(:,:,idx)), t0, kappa(:,idx), dt);

    if ~isempty(KAPPA_MAX);   kappa(:,idx+1)   = min(kappa(:,idx+1),   KAPPA_MAX(1));   end
    if ~isempty(KAPPA_MAX_Z); kappa(3,idx+1)   = min(kappa(3,idx+1),   KAPPA_MAX_Z(1)); end

    if any(isnan(kappa(:,idx+1)))
        break
    end

% Computing Outer Loop Control Output
    u_sw = -K_ctrl.Gamma*sigma(:,idx) - Theta_norm*diag(sat(K_ctrl.E\sigma(:,idx)))*G_2(:,:,idx)*kappa(:,idx+1);
    u_eq = G_2(:,:,idx)*(-c + S_2(:,:,idx)*dp_2(:,idx) ...
        - G_2(:,:,idx)\chi_zeta_aug);

    V_a_cd = - G_2(:,:,idx)\(u_sw + u_eq);

    I_a_cd(:,idx) = I_R_V * V_a_cd - g;   % raw SMC command (logged)

    if any(isnan(I_a_cd(:,idx)))
       break;
    end

% *************************************************************************
% LPF BEFORE the CBF (Option 1): condition pixel-noise spikes amplified through
% L_s^-1 / the c-term products at low altitude, on the DESIRED accel — upstream
% of the CBF projection. The QP re-imposes the hard FoV bound on whatever it is
% given, so filtering the input cleans noise WITHOUT smearing the bound (filtering
% the CBF OUTPUT would mix safe leans across moving per-step boxes). One filter
% serves both the lateral desired lean (theta_unsafe) and the vertical thrust.
% *************************************************************************
    I_a_cd_filt = alpha_ia * I_a_cd_filt + (1 - alpha_ia) * I_a_cd(:,idx);

% *************************************************************************
% Target-visibility CBF (camera-plane theta-QP) — replaces the cone clamp.
%   A QP over the body tilt that keeps the marker on the sensor: projects the
%   desired lean theta_d onto the FoV box |cr + L_w*M*(theta-theta_curr)+tau d|
%   <= phi_max, then applies the post-QP deliverability cap. Returns the safe
%   lean th_safe; R_d is built from it directly (Fix B) below. Operates on the
%   FILTERED desired accel so th_safe is smooth yet still exactly in-box.
% *************************************************************************
    R33           = max(min(I_R_C(3,3), 1), -1);
    theta_current = acos(R33);

    % z-upright guard BEFORE the CBF (matches the Python contract)
    if I_a_cd_filt(3) >= 0
        I_a_cd_filt(3) = -3.0;
    end

    refresh    = (mod(idx-1,ZOH) == 0);   % image-refresh step (C_nP updated)
    dt_img     = ZOH*dt;                  % effective image dt for drift/loom FD
    theta_seed = K_ctrl.theta_cap;        % Phase-2 fallback cone seed
    [I_a_cd_filt, theta_cone, cbf_ok, th_safe, cbf_state] = cbf2_filter( ...
        I_a_cd_filt, I_R_C, R33, yaw, C_nP, f, phi_max_cbf, ...
        K_ctrl.theta_cap, theta_seed, dt_img, refresh, B_w_c(1:2), cbf_state);

    % Inner-loop deliverable-tilt saturation (Property 1): the deliverable cap was
    % MOVED OUT of cbf2_filter (now pure visibility QP) to here, clipping the safe
    % lean before the attitude is built.
    if ~isempty(th_safe)
        tn_cap = norm(th_safe);
        if tn_cap > K_ctrl.theta_cap; th_safe = th_safe*(K_ctrl.theta_cap/tn_cap); end
    end

    I_a_cd_filt(3) = max(I_a_cd_filt(3), -50);

    % TILT-RATE LIMIT (TILT_RATE_MAX rad/s): cap how fast the CBF lean magnitude
    % can GROW, so the startup tilt-up is gradual -> the marker centroid shifts
    % slowly enough for the CBF to hold the edge-corner (IC5 startup ejection:
    % the pitch slammed to 0.20 rad in 0.4 s and swung a corner out of FoV before
    % the drone even translated). Shrinking is unrestricted (always visibility-
    % safe). Acts on th_safe (the attitude is built from it). [] = off.
    % Gated to the STARTUP window only (t < TILT_RATE_T, default 0.4s): a global
    % rate limit starves the ongoing lateral corrections -> breaches. The ejection
    % is purely the initial tilt-up, so ramp only that.
    global TILT_RATE_MAX TILT_RATE_T;
    if isempty(TILT_RATE_T); trate_T = 0.4; else; trate_T = TILT_RATE_T(1); end
    if ~isempty(TILT_RATE_MAX) && TILT_RATE_MAX > 0 && ~isempty(th_safe) ...
            && tRange(idx) < trate_T
        tn_now = norm(th_safe);
        if tn_now > th_safe_prev_n + TILT_RATE_MAX*dt && tn_now > 1e-9
            th_safe = th_safe * (th_safe_prev_n + TILT_RATE_MAX*dt) / tn_now;
        end
    end
    if ~isempty(th_safe); th_safe_prev_n = norm(th_safe); end

    theta_cur_log(idx)  = theta_current;
    theta_cone_log(idx) = theta_cone;
    cbf_ok_log(idx)     = cbf_ok;
    if norm(I_a_cd_filt) > 1e02
        break;
    end

% *************************************************************************
% Yaw adaptive SMC — drives alpha -> alpha_d, outputs rate u_a
% *************************************************************************
    % alpha has period pi (ellipse orientation symmetry) -> wrap e_a to [-pi/2, pi/2]
    e_raw    = V_s(4) - V_s_d(4);
    e_a(idx) = atan2(sin(2*e_raw), cos(2*e_raw)) / 2;
    if idx == 1
        ie_a(idx) = dt * e_a(idx);
    else
        ie_a(idx) = ie_a(idx-1) + dt*(e_a(idx-1) + e_a(idx))/2;
    end
    sigma_a = e_a(idx) + K_ctrl.Omega_a * ie_a(idx);

    const_kappa_a = [K_ctrl.n_a; K_ctrl.p_a];
    kappa_a(idx+1) = RK5(@(t,X) kappa_a_Solver(t, X, sigma_a, const_kappa_a), ...
                         t0, kappa_a(idx), dt);
    u_a = K_ctrl.Gamma_a*sigma_a + sat(sigma_a/K_ctrl.E_a)*kappa_a(idx+1) ...
          + K_ctrl.Omega_a*e_a(idx);

    if ~isfinite(u_a), break; end

    u_a_sat = u_a;                           % no cap; pass ASMC rate directly to heading integrator

    % Integrate ASMC rate into desired heading (replaces compass)
    psi_d = psi_d + u_a_sat * dt;
    psi_d = atan2(sin(psi_d), cos(psi_d));   % wrap to [-pi, pi]

% *************************************************************************
% Construct R_d from I_a_cd (roll/pitch) + psi_d (heading)
% *************************************************************************
    I_F   = m * I_a_cd_filt;             % filtered thrust force vector in NED
    f_mag = norm(I_F);
    T_cd  = f_mag;

    if f_mag < 1e-6
        R_d = eye(3);
    else
        if ~isempty(th_safe)
            % Fix B (direct-th_safe): build the desired body-z from the CBF's
            % unfiltered safe lean -> the CBF visibility bound lands EXACTLY on
            % the commanded attitude (no tau_ia LPF smear of the hard bound).
            %   rd3 = [-Rz(yaw)*th_safe; 1] / norm
            % CONSISTENT thrust: divide |I_a_filt(3)| by the ACTUAL tilt cosine
            % R33 (= I_R_C(3,3)) — exactly as PX4 divides B_T by cos(euler) of the
            % MEASURED attitude, NOT by the commanded rd3(3). This realizes a
            % vertical thrust component = m*|I_a_filt(3)| at all times, so there
            % is no thrust/attitude timing mismatch. (Using rd3(3), the commanded
            % cosine, inflated T_cd while the actual attitude lagged -> early
            % climb -> a touchdown limit cycle; PX4 never had this because it
            % divides by the measured tilt. Lands soft+precise; CBF bound exact.)
            a_xy_dir = [cos(yaw)*th_safe(1) - sin(yaw)*th_safe(2); ...
                        sin(yaw)*th_safe(1) + cos(yaw)*th_safe(2)];   % Rz(yaw)*th_safe
            rd3  = [-a_xy_dir; 1];
            rd3  = rd3 / norm(rd3);
            T_cd = m * abs(I_a_cd_filt(3)) / max(R33, 1e-3);   % actual tilt cos (PX4-faithful)
        else
            rd3 = -I_F / f_mag;          % Phase-2 fallback: force-vector path
        end
        a_h = [cos(psi_d); sin(psi_d); 0];
        rd2_raw = cross(rd3, a_h);
        n2 = norm(rd2_raw);
        if n2 < 1e-6, rd2_raw = [0;1;0]; n2 = 1; end
        rd2 = rd2_raw / n2;
        rd1 = cross(rd2, rd3);
        R_d = [rd1, rd2, rd3];
    end

% *************************************************************************
% Geometric SO(3) attitude controller
%   e_R     = 0.5 * vee(R_d' * R - R' * R_d)
%   e_Omega = B_w_c - R' * R_d * Omega_d   (Omega_d = 0)
%   tau     = -kR*e_R - kOmega*e_Omega + w x Jw
% *************************************************************************
    eR_mat = 0.5 * (R_d' * I_R_C - I_R_C' * R_d);
    e_R    = [eR_mat(3,2); eR_mat(1,3); eR_mat(2,1)];   % vee map
    e_Omega = B_w_c;                                    % Omega_d = 0

    ie_R = ie_R + e_R * dt;                             % integral of attitude error
    ie_R = max(min(ie_R, ie_R_max), -ie_R_max);        % anti-windup clamp

    % Thrust-scaled adaptive CoG feed-forward (tau_d = T*[-dy;dx;0] cancellation)
    if gamma_cog > 0
        e_comp = e_Omega(1:2) + cog_c2 * e_R(1:2);     % composite attitude error (xy)
        thetahat_cog = thetahat_cog ...
            + dt * (gamma_cog * T_cd * e_comp - cog_leak * thetahat_cog);  % sigma-mod
        thetahat_cog = max(min(thetahat_cog, cog_max), -cog_max);   % anti-windup
        tau_cog = [-T_cd * thetahat_cog; 0];           % tau_ff = -T*theta_hat
    else
        tau_cog = zeros(3,1);
    end
    THETAHAT_COG(:,idx) = thetahat_cog;
    B_tau_cd = -K_ctrl.kR*e_R - K_ctrl.kI_R*ie_R - K_ctrl.kOmega*e_Omega ...
        + cross(B_w_c, J*B_w_c) + tau_cog;

% Ground effect on thrust
    if GE
        z_ge = -max(abs(x_c(3)), r);
        T_cd = 1/(1-(r/(4*z_ge))^2) * T_cd;
    end

% Saturate torques and thrust
    B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max), tau_xy_max);
    B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max),  tau_z_max);
    T_cd          = max(min(T_cd, T_max), T_min);

    B_T_cd(idx) = T_cd;

% *************************************************************************
% Simulating Computational Delay
% *************************************************************************
    u_2_buf(:,idx) = [B_tau_cd; T_cd];
    if idx > delay
        u_2 = u_2_buf(:, idx - delay);
    else
        u_2 = [zeros(3,1); m*norm(g)];   % hover
    end

    if any(isnan(u_2)) || norm(u_2) > 1e4, break; end

    % Update wind state: mean + colored turbulence + velocity-drag
    if NOISE
        F_turb   = F_turb + (dt/wind_tau) * (-F_turb + wind_sigma*randn(3,1));
        v_rel    = wind_mean - x_c(8:10);
        F_wind_I = wind_mean*C_d_wind + F_turb + C_d_wind*v_rel;
        F_wind_eff = F_wind_I; if abl_wind; F_wind_eff = zeros(3,1); end   % ablate wind
        m_eff = m_p; J_eff = J_p; cog_eff = r_cog;                          % ablate parametric:
        if abl_param || abl_mass; m_eff   = m;           end                %  mass
        if abl_param || abl_iner; J_eff   = J;           end                %  inertia
        if abl_param || abl_cog;  cog_eff = zeros(3,1);  end                %  CoG offset
        x_c = RK5(@(t, x) UAVDyn_robust(t, x, u_2, m_eff, J_eff, F_wind_eff, cog_eff), t0, x_c, dt);
    else
        x_c = RK5(@(t, x) UAVDyn(t, x, u_2), t0, x_c, dt);
    end

    if any(isnan(x_c))
        break;
    end

% *************************************************************************
% Updating System States in Inertial frame
% *************************************************************************
    I_p_c = x_c(1:3);
    q_c = x_c(4:7);
    I_v_c = x_c(8:10);
    B_w_c = x_c(11:13);

    q_c = q_c / norm(q_c);

% *************************************************************************
% Logging states, control inputs and output (Datasets)
% *************************************************************************
    U_DS(:,idx) = u_2;
    X_DS(:,idx+1) = x_c;
    V_X_DS(:,idx) = [V_s_i(1:2); V_s_i(4); V_h_i; V_w_i; V_dw_i; V_s_a(1:2); V_s_a(4); V_h_a; V_w_a; V_dw_a];
    eR_log(:,idx) = e_R;
    B_tau_cd_log(:,idx) = B_tau_cd;
    psi_d_log(idx) = psi_d;
    u_a_log(idx)   = u_a;
    D_DS(:,idx) = [V_h_d(:,idx); I_a_cd(:,idx); e_R; B_tau_cd; T_cd; psi_d; u_a];
    P_DS(:,:,idx) = [V_nP_i, V_nP_a, C_nP];

% *************************************************************************
% Termination Condition
% *************************************************************************
    alt_above = abs(I_p_c(3) - x_t(3,idx));
    xy_err   = norm(I_p_c(1:2) - x_t(1:2,idx));
    rel_vel  = norm(I_v_c - dx_t(1:3,idx));
    if alt_above <= zf
        precise = xy_err <= 0.08;
        soft    = rel_vel <= 0.2;
        fprintf('Landed at t = %.2f s  (alt=%.3fm, xy=%.3fm, v_rel=%.3fm/s, precise=%d, soft=%d)\n', ...
                tRange(idx), alt_above, xy_err, rel_vel, precise, soft);
        landed = true;
        break;
    end

% *************************************************************************
% Updating to next iteration
% *************************************************************************
     t0 = t0+dt;
end

if ~landed
    idx = idx - 1;
end

save("temp1.mat");
data = load("temp1.mat");
% plotter_adaptive(data);   % Disabled: plotter reads removed fields (p_1, S_1, zeta_1) — Approach 2 refactor
% delete("temp.mat")
