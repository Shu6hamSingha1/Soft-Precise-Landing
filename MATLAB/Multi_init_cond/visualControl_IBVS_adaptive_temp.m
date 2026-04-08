%% Started working on 14/01/2026
% Last updated on 14/01/2026
%% Refer to "Adaptive Visual Control Strategies for Autonomous Landing of Quadrotor Platforms with visibility constraaints"
% Adaptive Gain Controller and IBVS
% Inertial Frame --> NED
% Camera/Visual Frame ---> NED
% Outer Loop --> s --> h --> a --> E
% Inner Loop --> dE --> w --> dw --> T
% clc;
% close all;
if exist('K', 'var') == 1
    clearvars -except K;
elseif isfile("bestParam.mat") == 1
    clear;
    load("bestParam.mat");
end
rng('shuffle');

% header
fprintf('Adaptive - PID Flight Controller\n\n' );

Constants;

InitVar;

%% =========================================================================
%  PLASMC GAINS (from InitGains_Comparison.m — tuned for comparison study)
% =========================================================================
K_ctrl = struct();

K_ctrl.gamma_1  = [0.2, 0.2];
K_ctrl.p_10     = K.p_10;
K_ctrl.p_1inf   = [0.2; 0.2];

K_ctrl.zp = diag([4.0, 4.0]);
K_ctrl.zi = diag([0.1, 0.1]);
K_ctrl.zd = diag([1.3, 1.3]);

K_ctrl.gamma_2  = [0.2, 0.2, 0.2];
K_ctrl.p_20     = [12.0; 12.0; 5.0];
K_ctrl.p_2inf   = [1.5;  1.5;  2.0];

K_ctrl.Omega   = diag([0.005, 0.005, 0.01 ]);
K_ctrl.Gamma   = diag([0.25,  0.25,  0.5  ]);
K_ctrl.P       = diag([1.5,   1.5,   5.0  ]);
K_ctrl.N       = diag([0.02,  0.02,  0.05 ]);
K_ctrl.kappa_0 = [0.1; 0.1; 0.2];
K_ctrl.E       = diag([2.5,   2.5,   0.5  ]);

% Attitude PID inner loop — roll/pitch only
K_ctrl.ep = diag([5.0, 5.0]);
K_ctrl.ei = diag([0.1, 0.1]);
K_ctrl.ed = diag([0.1, 0.1]);
K_ctrl.wp = diag([5.0, 5.0, 5.0]);
K_ctrl.wi = diag([0.01, 0.01, 0.1]);
K_ctrl.wd = diag([0.1,  0.1,  0.2]);
K_ctrl.ff = diag([0.1,  0.1,  0.1]);

% Yaw adaptive SMC (replaces PID yaw channel)
K_ctrl.Omega_a   = 1.5;
K_ctrl.Gamma_a   = 0.3;
K_ctrl.n_a       = 0.05;
K_ctrl.p_a       = 2;
K_ctrl.kappa_a_0 = 0.1;
K_ctrl.E_a       = 2.5;

%% =========================================================================
%  PRE-ALLOCATE ARRAYS
% =========================================================================
N_steps = numel(tRange);

U_DS      = zeros(4,  N_steps);
X_DS      = zeros(13, N_steps + 1);   X_DS(:,1) = x_c;
V_X_DS    = zeros(24, N_steps);
D_DS      = zeros(17, N_steps);
P_DS      = zeros(2,  12, N_steps);

x_t       = zeros(7,  N_steps);
dx_t      = zeros(6,  N_steps);

V_h_d     = zeros(3,  N_steps);
V_h_e     = zeros(3,  N_steps);
I_a_cd    = zeros(3,  N_steps);

% Inner-loop logging
E2_e      = zeros(2,  N_steps);
iE2_e     = zeros(2,  N_steps);
raw_dE2_e = zeros(2,  N_steps + 3);
w_e       = zeros(3,  N_steps);
iw_e      = zeros(3,  N_steps);
B_w_cd    = zeros(3,  N_steps);
B_w_cf    = zeros(3,  N_steps);
B_T_cd    = zeros(1,  N_steps);

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
B_dw_cf      = zeros(3, 1);

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
p_1         = zeros(2, N_steps);
dp_1        = zeros(2, N_steps);
p_2         = zeros(3, N_steps);
dp_2        = zeros(3, N_steps);
S_1         = zeros(2, 2, N_steps);
G_1         = zeros(2, 2, N_steps);
zeta_1      = zeros(2, N_steps);
izeta_1     = zeros(2, N_steps);
raw_dzeta_1 = zeros(2, N_steps + 3);
S_2         = zeros(3, 3, N_steps);
G_2         = zeros(3, 3, N_steps);
zeta_2      = zeros(3, N_steps);
izeta_2     = zeros(3, N_steps);
sigma       = zeros(3, N_steps);
raw_dh_d    = zeros(3, N_steps + 3);
V_s_e       = zeros(2, N_steps);
kappa       = [K_ctrl.kappa_0, zeros(3, N_steps)];
kappa_a     = [K_ctrl.kappa_a_0, zeros(1, N_steps)];
e_a         = zeros(1, N_steps);
ie_a        = zeros(1, N_steps);

flag = false;
landed = false;

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
    traj_t = traj_Gen((idx-1)*dt, "Static");
    % traj_t = traj_Gen((idx-1)*dt, "Linear");
    % traj_t = traj_Gen((idx-1)*dt, "Sinusoidal");
    % traj_t = traj_Gen((idx-1)*dt, "Lissajous");
    % traj_t = traj_Gen((idx-1)*dt, "Circular");

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
            C_nP = awgn(C_nP, 50, 'measured');
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
    if alt_above <= 0.20 && xy_err <= 0.3
        fprintf('Landed at t = %.2f s  (alt=%.3fm, xy=%.3fm)\n', ...
                tRange(idx), alt_above, xy_err);
        landed = true;
        break;
    end

% *************************************************************************
% Computing Desired Optical Flow with Visibility Constraints
% *************************************************************************
    V_s_e(:,idx) = V_s(1:2) - V_s_d(1:2);

    p_1(:,idx) = expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * (K_ctrl.p_10 - K_ctrl.p_1inf) + K_ctrl.p_1inf;
    dp_1(:,idx) = -diag(K_ctrl.gamma_1) * expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * (K_ctrl.p_10 - K_ctrl.p_1inf);

    for j=1:2
        S_1(j,j,idx) = V_s_e(j,idx)/p_1(j,idx);
        S_1(j,j,idx) = min(max(S_1(j,j,idx), -1+eps), 1-eps);
        zeta_1(j,idx) = log((1+S_1(j,j,idx))/(1-S_1(j,j,idx)));
        G_1(j, j,idx) = (exp(zeta_1(j,idx)) + 1)^2/(2*exp(zeta_1(j,idx))*p_1(j,idx));
    end

    if idx == 1
        izeta_1(:,idx) = dt*zeta_1(:,idx);
        raw_dzeta_1(:,idx+3) = zeros(2,1);
    else
        izeta_1(:,idx) = izeta_1(:,idx-1) + dt*(zeta_1(:,idx-1) + zeta_1(:,idx))/2;
        raw_dzeta_1(:,idx+3) = (zeta_1(:,idx) - zeta_1(:,idx-1))/dt;
    end
    dzeta_1 = smooth4(raw_dzeta_1(:,idx:idx+3));
    dzeta_1d = -K_ctrl.zp*zeta_1(:,idx) - K_ctrl.zi*izeta_1(:,idx) - K_ctrl.zd*dzeta_1;

    V_ds_d = [G_1(:, :, idx)\dzeta_1d + S_1(:,:,idx)*dp_1(:,idx);0.0];

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
    izeta_2_max = 5.0;
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

    if norm(I_a_cd(:,idx)) > 1e02 || any(isnan(I_a_cd(:,idx)))
       break;
    end

    % Cone clamp: keep acceleration vector within max attitude angle
    att_cone = deg2rad(35);
    if I_a_cd(3,idx) >= 0
        I_a_cd(3,idx) = -1.0;
    end
    a_xy_limit = abs(I_a_cd(3,idx)) * tan(att_cone);
    a_xy_norm  = norm(I_a_cd(1:2,idx));
    if a_xy_norm > a_xy_limit
        I_a_cd(1:2,idx) = a_xy_limit * I_a_cd(1:2,idx) / a_xy_norm;
    end

% *************************************************************************
% Attitude control: 2-DOF PID (roll/pitch) + Yaw ASMC
% *************************************************************************
    % Desired roll/pitch from acceleration command (use -V_s(4) for heading)
    if abs(cos(-V_s(4))*I_a_cd(1,idx) + sin(-V_s(4))*I_a_cd(2,idx)) < 1e-4
        theta_cd = 0;
    else
        theta_cd = atan2(-cos(-V_s(4))*I_a_cd(1,idx) - sin(-V_s(4))*I_a_cd(2,idx), ...
                         -I_a_cd(3,idx));
    end
    if abs(sin(-V_s(4))*I_a_cd(1,idx) - cos(-V_s(4))*I_a_cd(2,idx)) < 1e-4
        phi_cd = 0;
    else
        phi_cd = atan2(-sin(-V_s(4))*I_a_cd(1,idx) + cos(-V_s(4))*I_a_cd(2,idx), ...
                       -I_a_cd(3,idx)/cos(E_cr(2)));
    end
    E2_crd = [phi_cd; theta_cd];

    % Roll/pitch PID (2-DOF)
    E2_e(:,idx) = E_cr(1:2)' - E2_crd;

    if idx == 1
        iE2_e(:,idx)       = dt * E2_e(:,idx) / 2;
        raw_dE2_e(:,idx+3) = zeros(2,1);
        B_w_cf(:,idx)      = B_w_c;
        B_dw_cf            = zeros(3,1);
    else
        iE2_e(:,idx)       = iE2_e(:,idx-1) + dt*(E2_e(:,idx-1)+E2_e(:,idx))/2;
        raw_dE2_e(:,idx+3) = (E2_e(:,idx) - E2_e(:,idx-1)) / dt;
        B_w_cf(:,idx)      = alpha_w*B_w_cf(:,idx-1) + (1-alpha_w)*B_w_c;
        B_dw_cf            = alpha_dw*B_dw_cf + ...
                             (1-alpha_dw)*(B_w_cf(:,idx)-B_w_cf(:,idx-1))/dt;
    end

    dE2_cd = -K_ctrl.ep*E2_e(:,idx) - K_ctrl.ei*iE2_e(:,idx) ...
             -K_ctrl.ed*raw_dE2_e(:,idx+3);

    % Yaw adaptive SMC
    e_a(idx) = V_s(4) - V_s_d(4);
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

    dE_cd = [dE2_cd; u_a];
    if norm(dE_cd) > 1e02, break; end

    W = [1 , 0, -sin(E_cr(2)); ...
        0, cos(E_cr(1)), sin(E_cr(1))*cos(E_cr(2)); ...
        0, -sin(E_cr(1)), cos(E_cr(1))*cos(E_cr(2))];

    B_w_cd(:,idx) = W*dE_cd;

% Computing desired Thrust force of Camera
    B_T_cd(idx) = - m*(I_a_cd(3,idx))/(cos(E_cr(1))*cos(E_cr(2)));

% Adding ground effect to Vertical Control input
    if GE
        z_ge = -max(abs(x_c(3)), r);
        B_T_cd(idx) = 1/(1-(r/(4*z_ge))^2)*B_T_cd(idx);
    end

% *************************************************************************
% Simulating Computational Delay and Input Saturation
% *************************************************************************
    if idx > delay
        u_1 = [B_w_cd(:,idx - delay); B_T_cd(idx - delay)];
    else
        u_1 = [zeros(3,1);m*dot(g, I_R_C(:,3));];
    end

    u_1(4) = max(min(u_1(4), T_max), T_min);
    u_1(1:3) = max(min(u_1(1:3), w_max), -w_max);

% *************************************************************************
% Attitude Rate Control with Input Saturation
% *************************************************************************
    w_e(:,idx) = B_w_c - u_1(1:3);

    if idx == 1
        iw_e(:,idx) = dt*w_e(:,idx)/2;
    else
        iw_e(:,idx) = iw_e(:,idx-1) + (w_e(:,idx-1) + w_e(:,idx))*dt/2;
    end

    B_dw_cd = -K_ctrl.wp * w_e(:,idx) - K_ctrl.wi * iw_e(:,idx) - K_ctrl.wd * B_dw_cf + K_ctrl.ff * u_1(1:3);

    B_tau_cd = J * B_dw_cd + cross(B_w_c, J * B_w_c);

    B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max), tau_xy_max);
    B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max), tau_z_max);

    anti_windup_factor = abs(B_tau_cd) >= [tau_xy_max; tau_xy_max; tau_z_max];
    iw_e(:,idx) = iw_e(:,idx) .* (~anti_windup_factor);

% *************************************************************************
% Simulating UAV Flight Dynamics with System Delay
% *************************************************************************
    u_2 = [B_tau_cd; u_1(4)];

    x_c = RK5(@(t, x) UAVDyn(t, x, u_2), t0, x_c, dt);

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
    D_DS(:,idx) = [V_h_d(:,idx); I_a_cd(:,idx); E2_crd; dE_cd; B_w_cd(:,idx); B_dw_cd];
    P_DS(:,:,idx) = [V_nP_i, V_nP_a, C_nP];

% *************************************************************************
% Termination Condition
% *************************************************************************
    alt_above = abs(I_p_c(3) - x_t(3,idx));
    xy_err   = norm(I_p_c(1:2) - x_t(1:2,idx));
    if alt_above <= 0.20 && xy_err <= 0.3
        fprintf('Landed at t = %.2f s  (alt=%.3fm, xy=%.3fm)\n', ...
                tRange(idx), alt_above, xy_err);
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

save("temp.mat");
data = load("temp.mat");
plotter_adaptive(data);
delete("temp.mat")
