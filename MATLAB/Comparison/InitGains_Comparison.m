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
% Controller 1 (PLASMC):
%   Outputs I_a_cd → yaw ASMC heading generator → geometric SO(3) inner loop
%   (kR, kOmega) inline in visualControl_comparison.m.
%
% Controllers 2-5 (Lin, Zhang, Chen, Cho):
%   Each controller produces u_2 = [tau; T] directly via its own
%   geometric SO(3) inner loop (kR_shared, kOmega_shared gains below).
% =========================================================================

%% =========================================================================
%  1.  K_PLASMC   (Proposed — from manuscript)
% =========================================================================
K_PLASMC = struct();

% Approach 2: visibility funnel (p_1, gamma_1, p_1inf) REMOVED. Physical-corner
% visibility is enforced downstream by the funnel-margin cone clamp on I_a_cd.
% p_10 retained — used as sensor-half for V_s_e_n normalization in outer PID.
K_PLASMC.p_10     = K.p_10;                     % sensor-half, r_e normalization (Approach 2)

K_PLASMC.rp = diag([9.0, 9.0]);                 % Combo D: x1.5 from 6.0 (deep-sweep precision winner, -25% maxXY)
K_PLASMC.ri = diag([0.1, 0.1]);
K_PLASMC.rd = diag([1.4375, 1.4375]);           % Combo D: x1.25 from 1.15 (D-damping pair for rp x1.5; recovers soft margin)

K_PLASMC.gamma_2  = [0.2, 0.2, 0.2];           % prior 25/25 baseline
K_PLASMC.p_20     = [25.0; 25.0; 4.0];  % vertical tightened (deep-sweep, -4.8% aggT)
K_PLASMC.p_2inf   = [2.5;  2.5;  1.5 ];        % reverted from Combo A (z-tightening acted as speed knob under FW=11)

K_PLASMC.Omega   = diag([0.05, 0.05, 0.025]);   % vertical bumped 4x (0.006->0.025) to close IC4 hover-fail after Approach 2
K_PLASMC.Gamma   = diag([0.4375, 0.5,   0.75 ]); % lateral symmetry lock (IC=±2)
K_PLASMC.P       = diag([1.5,   1.5,   5.0  ]);
K_PLASMC.N       = diag([0.02,  0.02,  0.05 ]);
K_PLASMC.kappa_0 = [0.125; 0.125; 0.25];
K_PLASMC.E       = diag([1.0,   1.0,   1.0  ]);  % z firmed 0.9->1.0 (paired with rd=1.15); 1.1 was saturated

% Geometric SO(3) attitude gains (SO(3) baseline — 2026-04-13)
K_PLASMC.kR     = diag([1.5, 1.5, 0.5]);  % reverted 2026-04-16: combo3 kR x1.25 broke Linear realistic Run 5 soft-landing (v_rel=0.237 m/s > 0.20 m/s on IC [2,2,-3])
K_PLASMC.kOmega = diag([0.3, 0.3, 0.1]);

% Yaw adaptive SMC — heading-rate generator (SO(3) baseline)
K_PLASMC.Omega_a   = 0.5;
K_PLASMC.Gamma_a   = 0.5;
K_PLASMC.n_a       = 1.0;
K_PLASMC.p_a       = 2;
K_PLASMC.kappa_a_0 = 2.0;
K_PLASMC.E_a       = 3.0;

% funnel-margin cone clamp (Approach 2) — synced with Multi_init/run_simulation.m
K_PLASMC.rho_fov_0   = [145; 105];              % was res/2=[160;120]; inset 15 px per side to widen safety margin against IC5 transient v-axis breach
K_PLASMC.rho_fov_inf = [40; 40];                % terminal pixel margin (px)
K_PLASMC.l_fov       = 0.1;                     % cone-decay rate
K_PLASMC.theta_cap   = deg2rad(60);             % soft cone ceiling

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
% k1 raised 0.1→0.3, k2 2.0→3.0: with k1=0.1 the prescribed velocity
% vhat_z≈0.023 m/s near z=0 → UAV stalls 0.18m above ground at t=40s.
% k1=0.3 gives vhat_z≈0.068 m/s, landing ~t=38s; rho_v0 re-computed from
% vhat_init so initial xi_v stays well within ±0.999.
% Best-effort retune within paper's framework. Paper values (k1=4.5,
% rho_inf=0.05) crash in 1-2 steps on our IC=[2,2,-5] harness because
% vhat saturates the PPC barrier instantly. This is the minimum
% deviation from the paper that keeps xi_v inside (-1,1):
%  - k1 lowered so vhat stays finite at step 1
%  - rho_inf vectorized per axis (paper's formulation allows this,
%    eq. 14 uses diag Q_p); wide xy absorbs wind, tight z preserves
%    descent authority
%  - l slowed so bound stays wide during catch-up
K_Lin2022.k1 = 0.6;
K_Lin2022.k2 = 4.0;

% z-bound widened to 0.15: Linear/Circ carry Lin heave (A_z=0.2, w_z=0.5)
% so target v_z oscillates ±0.1 m/s. Tighter bound barrier-saturated xi_v(3).
K_Lin2022.rho_inf_p     = [0.30; 0.30; 0.15];
K_Lin2022.rho_inf_v     = [0.30; 0.30; 0.15];
K_Lin2022.l_p           = [0.03; 0.03; 0.10];
K_Lin2022.l_v           = [0.03; 0.03; 0.10];
K_Lin2022.rho_p0_margin = 1.5;
K_Lin2022.rho_v0_margin = 1.5;

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

% Redesigned for zeta=0.7 (near-critical damping) to eliminate overshoot:
%   xy: pos_gain=Kc1*Kc3+Kc2=1.035 N/m, vel_gain=m*Kc1+Kc3=2.06 N/(m/s)
%       omega_n=0.70 rad/s, zeta=0.70 -> overshoot<5% (was ~50% with old gains)
%    z: pos_gain=2.10 N/m, vel_gain=2.92 N/(m/s), omega_n=1.0, zeta=0.70
%   T at t=0: Fc_z=-10.1N -> T=10.9N < weight=20.6N -> descends correctly.
% Best-effort retune. Paper Table III values (Kc1(3)=0.6, Kc2(3)=2.6)
% give vertical omega_n~1.6 rad/s which finishes descent before
% horizontal converges under our IC. Horizontal sped up to match a
% gentled vertical so both loops converge on same time scale.
% Inner-loop Kc4/Kc5 unused — shared geometric SO(3) replaces eq. 212.
% z slowed further: omega_n_z ~0.4 rad/s so peak descent v_z from 5m
% stays <1.5 m/s for soft landing. xy unchanged.
K_Zhang2026.Kc1 = diag([0.25, 0.25, 0.03]);
K_Zhang2026.Kc2 = diag([2.0,  2.0,  0.15]);
K_Zhang2026.Kc3 = diag([2.5,  2.5,  0.5 ]);

% AEDO: lAF1/2 and PNF tightened further (prev PNF=50 still caused
% omega_AF to drift high enough to produce 7 post-landing oscillations).
% lAF1=1,lAF2=0.5 slows observer; PNF=150 keeps bandwidth near omega_AFm.
% AEDO (paper omega_AFm=1 rad/s per p8, kept)
K_Zhang2026.lAF1      = 1;
K_Zhang2026.lAF2      = 0.5;
K_Zhang2026.omega_AFm = 1.0;
K_Zhang2026.PNF_poly  = [0, 0, 150];
K_Zhang2026.xhat_AF0  = zeros(6,1);
K_Zhang2026.omega_AF0 = 1.0;

K_Zhang2026.kR     = kR_shared;
K_Zhang2026.kOmega = kOmega_shared;

%% =========================================================================
%  4.  K_Chen2025   (Chen et al., 2025, Sec. V)
% =========================================================================
K_Chen2025 = struct();

% Paper gains: kr=8, k1=0.2, k2=20, k3=2, k4=0.4
% Force mapping corrected: I_F = -m*I_R_V*f - m*g (from ξ̈ = -Rψ f).
% Gains reduced from paper values: pixel noise (awgn SNR=50) amplified
% through finite-diff de (noise ~ 1/z^2). At alt<2m, de noise produces
% force > thrust capacity via r-s+eps ~ 3*de. No safe kr exists:
% kr=0.7 stalls 0.34m, kr=1.0 stalls 0.41m, kr=1.2 oscillates 0.5m,
% kr=1.5 reaches ground at 11m/s impact. Structural limitation.
% q_d(3) = zf/zstar0 so error is exactly zero at landing height zf=0.1m.
% Best-effort retune. Paper values (kr=8, k2=20) amplify pixel noise
% through finite-diff de -> v_xy >10 m/s, T saturates 60N instantly.
% Structural limitation documented in project_chen2025_limitation.md:
% no safe kr exists for noisy landing. These are the most benign
% gains that keep the simulation numerically stable.
K_Chen2025.kr     = 1.0;
K_Chen2025.k1_obs = 0.2;
K_Chen2025.k2_obs = 5;
K_Chen2025.k3_obs = 0.5;
K_Chen2025.k4_obs = 0.4;
K_Chen2025.zstar0 = 1.5;
K_Chen2025.q_d    = [0; 0; 0.0667]; % zf/zstar0 = 0.1/1.5
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
% lambda(6)=0: removes yaw-rate command that was causing a slow spiral.
% lambda_xy kept -0.8: feedforward frame fix (I_R_V'*I_v_t) already removes
% the main oscillation cause. -0.5 was too slow to close initial 2m offset
% vs 0.5 m/s moving target within 40s (effective gain 0.1 at z=5m).
% Best-effort retune. Paper Table 2 values (lambda_xy=2, lambda_z=5)
% are too aggressive for moving targets (Linear/Lissajous fail to land)
% but work for Static. lambda_xy=1.2 is the compromise that lands moving
% targets; Kv lateral 2->3.5 fights wind drift; k_sigmoid 0.002->0.02
% so adaptive altitude gain ad_z saturates to 1 near landing (was stuck
% at 0.5, halving descent); v_sat(3) 0.4->0.7 removes descent cap.
K_Cho2022.lambda_IBVS = [-1.2; -1.2; -2.0; 0; 0; 0];
% k_sigmoid=0.1 was tested and broke moving targets: feature centroid
% fluctuates on moving trajs -> ad_z jitters -> z-command jitters. Kept
% k_sigmoid=0.02 and v_sat(3)=0.7. Static stall at ~0.35m is accepted.
K_Cho2022.v_sat       = [1.0; 1.0; 0.7; 0.2];
K_Cho2022.k_sigmoid   = 0.02;
K_Cho2022.use_sq_comp = true;
K_Cho2022.Kv          = diag([3.5, 3.5, 2.0]);
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
