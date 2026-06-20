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
P.p_r0   = [1.2; 1.2];                  % p_{r0}   initial half-width (FoV units)
P.p_rinf = [1.0; 1.0];                  % p_{r,inf} terminal floor (Standing Cond. 1: >=1)
P.Xi_r   = diag([0.10, 0.10]);          % Xi_r     funnel contraction rate

% ---- Optic-flow funnel  (tex eq. PPC on h_e; p_h(t)) ---------------------------
P.p_h0   = [25.0; 25.0; 4.0];           % p_{h0}    initial half-width  (code p_20)
P.p_hinf = [0.5;  0.5;  1.5];           % p_{h,inf} terminal floor (lateral 0.5 = chase-lag lever)
P.Xi_h   = diag([0.2, 0.2, 0.2]);       % Xi_h      contraction rate    (code gamma_2)

% ---- Combined sliding surface  sigma = zeta_h + chi*zeta_aug  (tex eq. sliding) -
P.chi_r = [1.15; 1.15];                 % lateral surface gain (PD: zeta_h + chi_r*zeta_r). Baked 0.85->1.15 (2026-06-20): drives terminal lateral barrier harder -> clears the Sinusoidal +40% precision edge for FULL +/-40% guaranteed, keeps 25/25; 1.2 regresses Liss-IC3 (upper edge)
P.chi_z = 0.1;                          % descent surface gain (PI: zeta_h3 + chi_z*int zeta_h3). 0.025->0.1 baked 2026-06-20: stronger descent integral drives h_ez->0 (loom regulated to h_rd -> constant area rate, no dPdt ramp) -> kills the terminal 1/z fly-away. Fly-aways -91% (35->3/300), SP 87->97%, full gate. (the integral GAIN matters, not the izeta clamp)

% ---- Leakage ASMC  (tex eq. adaptive control law + adaptive law) ---------------
P.Gamma   = diag([0.4375, 0.5, 0.75]);  % Gamma    linear sliding gain
P.E       = diag([1.0, 1.0, 1.0]);      % E        boundary-layer thickness (sat E^-1 sigma)
P.N       = diag([0.02, 0.02, 0.02]);   % N        adaptation rate
P.Pleak   = diag([1.5, 1.5, 5.0]);      % P        kappa leakage
P.kappa0  = [0.125; 0.125; 0.25];       % kappa(0) initial switching gain
P.izeta2_max = 5.0;                     % anti-windup clamp on int(zeta_h3)
P.S_margin   = 0.05;                    % funnel-saturation guard (|zeta|<=3.66, G finite)
P.drop_sddot = true;                    % s_ddot-drop (validated combined-barrier default)

% ---- Descent reference  (tex h_d final: h_rd < 0) ------------------------------
P.h_rd = -0.42;                         % desired descent optic flow (locked Table S1)

% ---- Virtual-compass yaw ASMC  (tex eq. yaw control law) -----------------------
P.Omega_a = 0.5;   % chi_alpha  (sigma_a = alpha_e + chi_a*int alpha_e)
P.Gamma_a = 0.5;   % gamma_alpha
P.n_a     = 1.0;   % eta_alpha
P.p_a     = 2.0;   % rho_alpha
P.kappa_a0 = 2.0;  % kappa_alpha(0)
P.E_a     = 3.0;   % eps_alpha boundary layer

% ---- Target-visibility CBF  (tex eq. cbf qp) ----------------------------------
P.theta_cap = deg2rad(60);              % post-QP deliverable-tilt cap
P.tau_ia    = 0.08;                     % upstream LPF time constant on commanded accel [s]
P.a_floor   = -50;                      % inertial-z accel floor (keep thrust direction realizable)

% ---- Geometric SO(3) tracker  (tex eq. so3 torque) ----------------------------
P.kR     = diag([2.5, 1.5, 0.5]);  % roll 1.5->2.5 baked 2026-06-20: stiffer roll adds Y-attitude damping that kills the terminal lateral limit cycle (Liss-IC3) -> noiseless 25/25 + real 25/25 + full +/-40%; sharp optimum (2.0/3.0 worse, phase-damping)
P.kOmega = diag([0.3, 0.3, 0.2]);  % yaw-rate 0.1->0.2 baked 2026-06-20: ROOT cycle fix. kOmega_z=0.1 was under-damped -> yaw limit cycle (worst Circ-IC3=1.78) that PUMPED the lateral cycles via yaw-image coupling. 0.2 kills yaw cycle (-90%) AND lateral (Y 0.98->0.11) at full gate; monotonic (vs fragile kR)
P.kI_R   = diag([0,0,0]);  P.ie_R_max = 0.5;     % integral attitude term (off) + anti-windup

% ---- Adaptive CoG feedforward (Lee-style; baked-on) ---------------------------
P.gamma_cog = 0.005;  P.cog_c2 = 2.0;  P.cog_max = 0.02;  P.cog_leak = 0;

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
