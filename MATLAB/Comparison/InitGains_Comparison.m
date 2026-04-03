%% =========================================================================
% InitGains_Comparison.m
%
% Gain initialisation for the comparative study of five landing controllers.
%
% COORDINATE CONVENTION: Inertial frame = NED  (I_)
%   x = North,  y = East,  z = Down
%   g = [0;0;9.81] m/s^2  (gravity points in +z = downward)
%   UAV initial altitude: I_pz_c = -5.0 m  (above ground → negative z)
%
% Controllers 1 (PLASMC):
%   Outputs I_a_cd → shared attitude-PID inner loop in main script
%
% Controllers 2-5 (Lin, Zhang, Chen, Cho):
%   Each controller produces u_2 = [tau; T] directly via its own
%   geometric SO(3) inner loop (kR, kOmega gains below).
%   The shared attitude-PID inner loop in the main script is BYPASSED.
% =========================================================================

%% =========================================================================
%  1.  K_PLASMC   (Proposed — from manuscript)
% =========================================================================
K_PLASMC = struct();

K_PLASMC.gamma_1  = [0.5, 0.5];
K_PLASMC.p_10     = K.p_10;
K_PLASMC.p_1inf   = [0.2; 0.2];

K_PLASMC.zp = diag([7.5, 7.5]);
K_PLASMC.zi = diag([0.1, 0.1]);
K_PLASMC.zd = diag([1.2, 1.2]);

K_PLASMC.gamma_2  = [0.5, 0.5, 0.5];
K_PLASMC.p_20     = [10.0; 10.0; 10.0];
K_PLASMC.p_2inf   = [1.0;  1.0;  1.0];

K_PLASMC.Omega   = diag([0.005, 0.005, 0.01 ]);
K_PLASMC.Gamma   = diag([0.2,   0.2,   0.1  ]);
K_PLASMC.P       = diag([2.0,   2.0,   5.0  ]);
K_PLASMC.N       = diag([0.05,  0.05,  0.05 ]);
K_PLASMC.kappa_0 = [0.1; 0.1; 0.1];
K_PLASMC.E       = diag([1.5,   1.5,   0.5  ]);

% Shared attitude-PID inner loop (PLASMC only)
K_PLASMC.ep = diag([5.0, 5.0, 1.0]);
K_PLASMC.ei = diag([0.1, 0.1, 0.5]);
K_PLASMC.ed = diag([0.1, 0.1, 0.3]);
K_PLASMC.wp = diag([5.0, 5.0, 5.0]);
K_PLASMC.wi = diag([0.01, 0.01, 0.1]);
K_PLASMC.wd = diag([0.1,  0.1,  0.2]);
K_PLASMC.ff = diag([0.1,  0.1,  0.1]);

%% =========================================================================
%  Shared geometric SO(3) inner-loop gains  (controllers 2-5)
%
%  kR     — attitude error gain  (Eq. 50, Lin 2022 / analogous in others)
%  kOmega — angular velocity error gain
%
%  Tuned to match the X500 inertia J = diag([0.0256, 0.0256, 0.0440]).
%  Bandwidth ~10 rad/s in roll/pitch, ~5 rad/s in yaw.
% =========================================================================
kR_shared     = diag([0.8,  0.8,  0.4 ]);
kOmega_shared = diag([0.15, 0.15, 0.08]);

%% =========================================================================
%  2.  K_Lin2022   (Lin et al., 2022, Sec. IV-B)
% =========================================================================
K_Lin2022 = struct();

% k1=0.1, k2=2.0: vhat_z(t=0)=0.88 m/s, F_z(t=0)=-19.3 N (upward, <0) so
% T=19.4 N < weight=20.6 N and UAV physically descends from the first step.
% k1=0.5/k2=8.0 caused F_z>0 (inverted-UAV demand) → xi_v diverged in 3 steps.
K_Lin2022.k1 = 0.1;
K_Lin2022.k2 = 2.0;

K_Lin2022.rho_inf_p     = 0.05;
K_Lin2022.rho_inf_v     = 0.05;
% l_p/l_v=0.05: slow decay gives the UAV time to build velocity before the
% performance constraint tightens (0.15 was too fast relative to actuator limits)
K_Lin2022.l_p           = 0.05;
K_Lin2022.l_v           = 0.05;
% Larger margin so xi_p/xi_v stay well within (-1,1) throughout
K_Lin2022.rho_p0_margin = 1.0;
K_Lin2022.rho_v0_margin = 1.0;

% r_pt_des = [0;0;0]: target the landing point exactly (NED).
% [0;0;-0.4] was causing the UAV to hover 0.4 m above target and never land.
K_Lin2022.r_pt_des = [0; 0; 0];
K_Lin2022.psi_des  = 0;

% Geometric inner-loop gains  (Eqs. 50-52, Lin 2022)
K_Lin2022.kR     = kR_shared;
K_Lin2022.kOmega = kOmega_shared;

%% =========================================================================
%  3.  K_Zhang2026   (Zhang & Wu, 2026, Tables II & III)
% =========================================================================
K_Zhang2026 = struct();

% z-axis Kc reduced: original Kc1z*Kc3z+Kc2z=4.04 nearly cancels gravity
% at re_z=-5 -> T=5.5N (almost free-fall). New z gains give Fc_z=-14N, T=15N<weight.
K_Zhang2026.Kc1 = diag([0.2, 0.2, 0.3]);
K_Zhang2026.Kc2 = diag([1.8, 1.8, 1.0]);
K_Zhang2026.Kc3 = diag([0.8, 0.8, 1.0]);

% lAF1/lAF2 halved, PNF_poly raised to 50 N^2 (actual noise force std ~3N,
% so noise power ~9-30 N^2): prevents omega_AF spiking to 5.8 rad/s on step 1
% which caused alternating-sign AEDO oscillations diverging in 7 steps.
K_Zhang2026.lAF1      = 2;
K_Zhang2026.lAF2      = 2;
K_Zhang2026.omega_AFm = 0.5;
K_Zhang2026.PNF_poly  = [0, 0, 50];
K_Zhang2026.xhat_AF0  = zeros(6,1);
K_Zhang2026.omega_AF0 = 0.5;

K_Zhang2026.kR     = kR_shared;
K_Zhang2026.kOmega = kOmega_shared;

%% =========================================================================
%  4.  K_Chen2025   (Chen et al., 2025, Sec. V)
% =========================================================================
K_Chen2025 = struct();

% kr negated: with kr=+8 f_out = -8*3*k1*e drove UAV AWAY from target
% (horizontal force was -4.8*e_x = -1.92N when e_x=+0.4 -> wrong sign).
% kr=-8 gives +4.8*e_x = +1.92N -> toward target. z unaffected (e_z~0 at t=0).
K_Chen2025.kr     = -8;
K_Chen2025.k1_obs = 0.2;
K_Chen2025.k2_obs = 20;
K_Chen2025.k3_obs = 2;
K_Chen2025.k4_obs = 0.4;
K_Chen2025.zstar0 = 10.0;
K_Chen2025.q_d    = [0; 0; 1];
K_Chen2025.psi_des = 0;

K_Chen2025.kR     = kR_shared;
K_Chen2025.kOmega = kOmega_shared;

%% =========================================================================
%  5.  K_Cho2022   (Cho et al., 2022, Tables 2 & 3)
% =========================================================================
K_Cho2022 = struct();

% lambda signs negated: Jacobian uses -f/z (forward-camera) but simulation
% has downward-looking camera -> vd_ibvs had wrong sign in all 3 axes.
% UAV saturated at v_sat=[-3,-3,-0.5] flying away from target indefinitely.
% Negating lambda flips vd sign -> UAV converges toward target.
K_Cho2022.lambda_IBVS = [-2.0; -2.0; -8.0; 0; 0; -0.2];
K_Cho2022.v_sat       = [3.0; 3.0; 0.5; 0.5];
K_Cho2022.k_sigmoid   = 0.002;
K_Cho2022.use_sq_comp = true;
K_Cho2022.Kv          = diag([2.0, 2.0, 2.0]);
K_Cho2022.psi_des     = 0;

K_Cho2022.kR     = kR_shared;
K_Cho2022.kOmega = kOmega_shared;

%% =========================================================================
%  Bouazza 2025  (unused in main comparison, kept for reference)
% =========================================================================
K_Bouazza = struct();
K_Bouazza.kR     = kR_shared;
K_Bouazza.kOmega = kOmega_shared;
K_Bouazza.S_ric  = blkdiag(0.05*eye(3), 0.05*eye(3));
K_Bouazza.D_ric  = 10 * eye(3);
K_Bouazza.Kp     = diag([1.5, 1.5, 2.0]);
K_Bouazza.Kd     = diag([0.8, 0.8, 1.2]);
K_Bouazza.P0_ric = 2 * eye(6);
K_Bouazza.psi_des = 0;

fprintf('Comparison gains initialised for 5 controllers.\n');
