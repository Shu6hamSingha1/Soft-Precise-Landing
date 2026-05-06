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
rng('shuffle');

% header
fprintf('Adaptive - Geometric SO(3) Flight Controller\n\n' );

Constants;

InitVar;

init_robustness;    % samples wind / mass / inertia / CoG / pixel-noise model

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
K_ctrl.N       = diag([0.02,  0.02,  0.05 ]);
K_ctrl.kappa_0 = [0.125; 0.125; 0.25];
K_ctrl.E       = diag([1.0,   1.0,   1.0  ]);  % z firmed 0.9->1.0 (paired with rd=1.15); 1.1 was saturated

% Geometric SO(3) attitude gains (tuned for X500 Gazebo inertia)
K_ctrl.kR     = diag([1.5, 1.5, 0.5]);  % reverted 2026-04-16: combo3 kR x1.25 failed Linear realistic IC [2,2,-3] soft landing
K_ctrl.kOmega = diag([0.3, 0.3, 0.1]);

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

% funnel-margin cone clamp (Approach 2)
% Shrinking box funnel on physical pixel corners -> state-dependent cone angle.
K_ctrl.rho_fov_0   = [145; 105];         % was res/2=[160;120]; inset 15 px per side to widen safety margin against IC5 transient v-axis breach
K_ctrl.rho_fov_inf = [40; 40];           % terminal pixel margin (px) — widened to keep cone authority
K_ctrl.l_fov       = 0.1;                % reverted from Combo A (faster cone decay was a speed knob, not precision)
K_ctrl.theta_cap   = deg2rad(60);        % soft cone ceiling

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

% Outer-loop PID on normalized raw error (Approach 2 — replaces PPC/zeta_1 chain)
V_s_e        = zeros(2, N_steps);
V_s_e_n      = zeros(2, N_steps);
iV_s_e_n     = zeros(2, N_steps);
raw_dV_s_e_n = zeros(2, N_steps + 3);

% funnel-margin cone clamp logging (Approach 2)
rho_fov_log    = zeros(2, N_steps);
d_min_log      = zeros(1, N_steps);
theta_cur_log  = zeros(1, N_steps);
theta_cone_log = zeros(1, N_steps);
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
    % traj_t = traj_Gen((idx-1)*dt, "Static");
    % traj_t = traj_Gen((idx-1)*dt, "Linear");
    % traj_t = traj_Gen((idx-1)*dt, "Sinusoidal");
    % traj_t = traj_Gen((idx-1)*dt, "Lissajous");
    traj_t = traj_Gen((idx-1)*dt, "Circular");

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

        if NOISE
            % Depth-dependent pixel noise: sigma_px(z) = 0.3 + 0.5/(z+0.5) [px]
            z_dep = max(abs(C_s_tc(3)), 0.1);
            sigma_px = px_sigma0 + px_sigma1 / (z_dep + px_depth_offset);
            C_nP = C_nP + (sigma_px / f) * randn(size(C_nP));
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
% Computing Desired Optical Flow (Approach 2 — raw-error PID on normalized r_e)
% No virtual-feature visibility funnel: physical-corner visibility is
% enforced downstream by the funnel-margin cone clamp on I_a_cd.
% *************************************************************************
    V_s_e(:,idx)   = V_s(1:2) - V_s_d(1:2);
    V_s_e_n(:,idx) = V_s_e(:,idx) ./ K_ctrl.p_10;     % normalize by sensor half

    if idx == 1
        iV_s_e_n(:,idx) = dt * V_s_e_n(:,idx);
        raw_dV_s_e_n(:,idx+3) = zeros(2,1);
    else
        iV_s_e_n(:,idx) = iV_s_e_n(:,idx-1) + dt*(V_s_e_n(:,idx-1) + V_s_e_n(:,idx))/2;
        raw_dV_s_e_n(:,idx+3) = (V_s_e_n(:,idx) - V_s_e_n(:,idx-1))/dt;
    end
    dV_s_e_n = smooth4(raw_dV_s_e_n(:,idx:idx+3));

    V_ds_d_xy = -K_ctrl.rp*V_s_e_n(:,idx) - K_ctrl.ri*iV_s_e_n(:,idx) - K_ctrl.rd*dV_s_e_n;
    V_ds_d    = [V_ds_d_xy; 0.0];

    V_h_d(:,idx) = V_ds_d + cross(V_w, V_s(1:3)) + (h_rd ...
        - dot(cross(V_w, V_s(1:3)), e3))*V_s(1:3);

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

    sigma(:,idx) = zeta_2(:,idx) + K_ctrl.Omega*izeta_2(:,idx);

% Computing Known System Dynamics (c)
    if idx == 1
        raw_dh_d(:,idx+3) = zeros(3,1);
    else
        raw_dh_d(:,idx+3) = (V_h_d(:,idx) - V_h_d(:,idx-1))/dt;
    end
    V_dh_d = smooth4(raw_dh_d(:,idx:idx+3));

    c = cross(V_dw, V_s(1:3)) + cross(V_w, cross(V_w, V_s(1:3))) ...
        + 2 * cross(V_w, V_h) - (dot(V_h + cross(V_w, V_s(1:3)), e3))*V_h - V_dh_d;

    Theta = [- c + S_2(:,:,idx)*dp_2(:,idx) ...
        - G_2(:,:,idx)\(K_ctrl.Omega*zeta_2(:,idx)), eye(3)];
    Theta_norm = norm(Theta,'fro');

% Updating Control Parameter 'kappa' using RK-5 Method
    const_kappa = [K_ctrl.N; K_ctrl.P];
    u_kappa = [sigma(:,idx); Theta_norm];
    kappa(:,idx+1) = RK5(@(t, X) kappa_Solver(t, X, u_kappa, const_kappa, G_2(:,:,idx)), t0, kappa(:,idx), dt);

    if any(isnan(kappa(:,idx+1)))
        break
    end

% Computing Outer Loop Control Output
    u_sw = -K_ctrl.Gamma*sigma(:,idx) - Theta_norm*diag(sat(K_ctrl.E\sigma(:,idx)))*G_2(:,:,idx)*kappa(:,idx+1);
    u_eq = G_2(:,:,idx)*(-c + S_2(:,:,idx)*dp_2(:,idx) ...
        - G_2(:,:,idx)\(K_ctrl.Omega*zeta_2(:,idx)));

    V_a_cd = - G_2(:,:,idx)\(u_sw + u_eq);

    I_a_cd(:,idx) = I_R_V * V_a_cd - g;

    if any(isnan(I_a_cd(:,idx)))
       break;
    end

% *************************************************************************
% funnel-margin cone clamp (Approach 2)
%   theta_current = acos(I_R_C(3,3))           (body-z vs inertial-z tilt)
%   rho_fov(t)    = shrinking box half-widths centered on image center
%   d_min         = min axis-wise margin of any physical corner to box boundary
%   theta_cone    = min(theta_current + atan(d_min/f), theta_cap)
% *************************************************************************
    R33           = max(min(I_R_C(3,3), 1), -1);
    theta_current = acos(R33);

    rho_fov_curr = (K_ctrl.rho_fov_0 - K_ctrl.rho_fov_inf) * ...
                   exp(-K_ctrl.l_fov * tRange(idx)) + K_ctrl.rho_fov_inf;

    % Axis-wise corner margins (C_nP is 2x4 in px: row 1=u, row 2=v)
    d_corner_x = rho_fov_curr(1) - abs(C_nP(1,:));
    d_corner_y = rho_fov_curr(2) - abs(C_nP(2,:));
    d_min_fov  = max(min([d_corner_x, d_corner_y]), 0);   % clip outside to zero margin

    theta_cone = min(theta_current + atan(d_min_fov / f), K_ctrl.theta_cap);

    if I_a_cd(3,idx) >= 0
        I_a_cd(3,idx) = -3.0;
    end
    a_xy_limit = abs(I_a_cd(3,idx)) * tan(theta_cone);
    a_xy_norm  = norm(I_a_cd(1:2,idx));
    if a_xy_norm > a_xy_limit
        I_a_cd(1:2,idx) = a_xy_limit * I_a_cd(1:2,idx) / a_xy_norm;
    end
    I_a_cd(3,idx) = max(I_a_cd(3,idx), -50);

    rho_fov_log(:,idx)  = rho_fov_curr;
    d_min_log(idx)      = d_min_fov;
    theta_cur_log(idx)  = theta_current;
    theta_cone_log(idx) = theta_cone;
    if norm(I_a_cd(:,idx)) > 1e02
        break;
    end

    % Low-pass filter I_a_cd before R_d construction (absorbs pixel-noise
    % spikes amplified through L_s^-1 at low altitude; raw I_a_cd logged)
    I_a_cd_filt = alpha_ia * I_a_cd_filt + (1 - alpha_ia) * I_a_cd(:,idx);

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
        rd3 = -I_F / f_mag;              % desired body-z (opposes force, NED)
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

    B_tau_cd = -K_ctrl.kR*e_R - K_ctrl.kOmega*e_Omega + cross(B_w_c, J*B_w_c);

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
        x_c = RK5(@(t, x) UAVDyn_robust(t, x, u_2, m_p, J_p, F_wind_I, r_cog), t0, x_c, dt);
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
