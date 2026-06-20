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
K_PLASMC.E       = diag([1.0,   1.0,   0.5  ]);  % E_z 1.0->0.5 baked 2026-06-20 (synced w/ vdf_params); descent boundary layer engages kappa on Z cycle. Vestigial for ctrl-1 (uses vdf_params).

% Geometric SO(3) attitude gains (SO(3) baseline — 2026-04-13)
K_PLASMC.kR     = diag([2.5, 1.5, 0.5]);  % roll 1.5->2.5 baked 2026-06-20 (synced w/ vdf_params); vestigial for ctrl-1 (uses vdf_params), kept for sync. NB: baselines 2-5 use kR_shared, NOT this.
K_PLASMC.kOmega = diag([0.3, 0.3, 0.2]);  % yaw-rate 0.1->0.2 baked 2026-06-20 (synced w/ vdf_params); root yaw-cycle fix. Vestigial for ctrl-1 (uses vdf_params); baselines use their own kOmega.

% Yaw adaptive SMC — heading-rate generator (SO(3) baseline)
K_PLASMC.Omega_a   = 0.5;
K_PLASMC.Gamma_a   = 0.5;
K_PLASMC.n_a       = 1.0;
K_PLASMC.p_a       = 2;
K_PLASMC.kappa_a_0 = 2.0;
K_PLASMC.E_a       = 3.0;

% VESTIGIAL for ctrl-1: the comparison's VDF-ASMC (ctrl-1) now runs on the verified
% VDF_ASMC blocks + vdf_params() (single source of truth), so these K_PLASMC fields
% are no longer the control source. Kept only for back-compat / logging. The
% funnel-margin cone clamp (rho_fov, l_fov) was REPLACED by the cbf2 visibility CBF;
% theta_cap MOVED OUT of the CBF into the inner-loop deliverable-tilt saturation
% (so3_tracker, vdf_params.theta_cap). Do not tune these expecting an effect on ctrl-1.
K_PLASMC.rho_fov_0   = [145; 105];              % (dead) superseded by cbf2
K_PLASMC.rho_fov_inf = [40; 40];                % (dead) superseded by cbf2
K_PLASMC.l_fov       = 0.1;                     % (dead) superseded by cbf2
K_PLASMC.theta_cap   = deg2rad(60);             % (dead) now vdf_params.theta_cap, inner-loop

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
K_Lin2022.k2 = 3.0;   % R1 4.0->3.0: lower force gain -> smaller tilt spike on funnel-boundary saturation (FoV-survival)

% z-bound widened to 0.15: Linear/Circ carry Lin heave (A_z=0.2, w_z=0.5)
% so target v_z oscillates ±0.1 m/s. Tighter bound barrier-saturated xi_v(3).
% R1: l_{p,v}_xy 0.03->0.02 slow the lateral funnel contraction so the bound
% stays wide longer -> e_p sits inside rho_p instead of pinning xi_p at the
% barrier (the Static soft-stall + moving force spikes were barrier saturation).
K_Lin2022.rho_inf_p     = [0.30; 0.30; 0.15];
K_Lin2022.rho_inf_v     = [0.30; 0.30; 0.15];
K_Lin2022.l_p           = [0.02; 0.02; 0.10];
K_Lin2022.l_v           = [0.02; 0.02; 0.10];
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
% R1 (FoV-survival): the xy position term killed Static at 0.5s. pos_gain =
% Kc1*Kc3+Kc2 = 2.625 -> a_lat@2m=5.25 m/s² -> 28° tilt -> corner crosses the
% ±120px (41.6° half-FoV) v-axis at 29° off-nadir. Drop Kc2_xy 2.0->1.0:
% pos_gain=0.2*2.5+1.0=1.5 -> a_lat@2m=3.0 -> ~17° tilt (within ~18° budget).
% vel_gain=m*Kc1+Kc3=2.7, omega_n=1.22, zeta=1.1 (slightly overdamped, OK).
K_Zhang2026.Kc1 = diag([0.20, 0.20, 0.03]);
K_Zhang2026.Kc2 = diag([1.0,  1.0,  0.15]);
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
%  4.  K_Lin2023   (Lin et al., 2023, Sec. III) -- replaces Chen 2025
% =========================================================================
% Robust circle-feature IBVS with a performance funnel (PPC) on IMAGE
% features + geometric SO(3) attitude. Same funnel-backstepping structure
% as K_Lin2022, but the OUTER loop runs on circle-moment image features
% s_t = [an*xg; an*yg; an]  (Eq. 7) instead of metric position. That swap
% IS the IBVS (Lin2023) vs PBVS (Lin2022) distinction; it preserves the
% deliberate 2 PBVS + 2 IBVS baseline split.
%   k1 (paper Kt): feature-funnel -> virtual-velocity gain
%   k2 (paper Kv): velocity-funnel -> force gain
% SMOKE-TEST starting values (NOT yet tuned). Retune best-effort within the
% paper's framework on IC2 under the Table-II noise model, exactly as the
% other baselines were (published gains expected to need adjustment).
K_Lin2023 = struct();
K_Lin2023.k1 = [0.4; 0.4; 0.40];             % feature -> virtual velocity (Kt), per-axis
                                             % (lateral raised for centring; depth=0.40 is the
                                             %  FoV-safe sweet spot: 0.60 reintroduces FoV loss)
% R1: moving targets blew up via velocity-funnel barrier (e_v grows faster
% than rho_v contracts on a fast target -> xi_v->1 -> eps_v->inf -> T sat 60N
% -> break). Lower k2 4.0->2.5 (force gain) + widen rho_inf_v + slow l_v_xy so
% the velocity barrier stays clear of the moving-target tracking error.
K_Lin2023.k2 = 2.5;                          % velocity -> force           (Kv)
K_Lin2023.rho_inf_t = [0.10; 0.10; 0.03];    % feature funnel steady-state (tight depth floor -> an->1 -> z->0.2m touchdown)
K_Lin2023.rho_inf_v = [0.50; 0.50; 0.20];    % velocity funnel steady-state (widened: avoid barrier blow-up)
K_Lin2023.l_t       = [0.05; 0.05; 0.10];    % feature funnel decay rate (depth contraction tuned for touchdown)
K_Lin2023.l_v       = [0.02; 0.02; 0.08];    % velocity funnel decay rate (slowed: bound wider longer)
K_Lin2023.rho_t0_margin = [1.5; 1.5; 5.0];   % rho_t(0)=|e_t(0)|+margin, per-axis
                                             % (large depth margin: literal e_t(3)~12 needs
                                             %  funnel room so xi_t(3) doesn't start near 1)
K_Lin2023.rho_v0_margin = 1.5;               % rho_v(0) = |e_v(0)| + margin
K_Lin2023.psi_des = 0;

K_Lin2023.kR     = kR_shared;
K_Lin2023.kOmega = kOmega_shared;

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
% R1 (FoV-survival): Cho lost FoV in <2s on EVERY trajectory. At t=0 v=0 and
% v_des jumps to v_sat -> instant Kv_xy*v_sat_xy=3.5 m/s² lateral accel -> 20°
% tilt + 29° off-nadir -> corner out. Cap the initial lateral demand: v_sat_xy
% 1.0->0.5 and Kv_xy 3.5->1.8 -> a_lat_max=0.9 m/s² -> ~5° tilt (well inside
% budget). lambda_xy -1.2->-0.8 gentles the IBVS velocity / terminal jitter.
K_Cho2022.lambda_IBVS = [-0.8; -0.8; -2.0; 0; 0; 0];
% k_sigmoid=0.1 was tested and broke moving targets: feature centroid
% fluctuates on moving trajs -> ad_z jitters -> z-command jitters. Kept
% k_sigmoid=0.02 and v_sat(3)=0.7. Static stall at ~0.35m is accepted.
K_Cho2022.v_sat       = [0.5; 0.5; 0.7; 0.2];
K_Cho2022.k_sigmoid   = 0.02;
K_Cho2022.use_sq_comp = true;
K_Cho2022.Kv          = diag([1.8, 1.8, 2.0]);
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
