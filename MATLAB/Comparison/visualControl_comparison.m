%% =========================================================================
% visualControl_comparison.m
%
% Comparative Study: Five Outer-Loop Landing Controllers
%   1. PLASMC     - Proposed (Singhal et al.)
%   2. Lin 2022   - Lin et al., IEEE TII 2022
%   3. Zhang 2026 - Zhang & Wu, IEEE TIE 2026
%   4. Chen 2025  - Chen et al., IEEE TCST 2025
%   5. Cho 2022   - Cho et al., Aerosp. Sci. Technol. 2022
%
% Based on: visualControl_IBVS_adaptive_temp.m
%
% Changes from previous comparison script (to match _temp.m):
%   1. yaw / I_R_V computed before trajectory (not after)
%   2. Trajectory type: "Circular"
%   3. Entire image-feature block inside ZOH gate (only update every ZOH steps)
%   4. Depth: f/(C_s_tc(3)+zf) and f/(V_s_tc(3)+zf) (not f/z)
%   5. Noise: awgn(C_nP, 50, 'measured') (not randi+round)
%   6. dPdt, V_w_i, V_dw_i computed via _prev variables (not buffer+smooth4)
%   7. V_w_a uses scalar _prev variable + raw_dw_a buffer
%   8. ACTUAL mode: Savitzky-Golay filter (not mean/smooth4)
%   9. V_s, V_h, V_w, V_dw are column vectors (not indexed arrays)
%  10. S_1 / S_2 saturation: min(max()) clamp (no flag-break)
%  11. izeta_1 / izeta_2 first-step init: dt (not dt/2)
%  12. Termination distance: 0.1 m (not 0.18 m)
%  13. V_X_DS / P_DS logging matches _temp.m
% =========================================================================
% clc;
% close all;
if exist('K', 'var') == 1
    clearvars -except K;
elseif isfile("bestParam.mat") == 1
    clear; load("bestParam.mat");
end
rng('shuffle');

%% =========================================================================
%  SELECT CONTROLLER
%   1 = PLASMC (proposed)
%   2 = Lin 2022
%   3 = Zhang 2026
%   4 = Chen 2025
%   5 = Cho 2022
% =========================================================================
% CTRL_SEL = 1;   % <-- change here

ctrl_names = {'PLASMC (Proposed)', 'Lin 2022', ...
              'Zhang 2026', 'Chen 2025', 'Cho 2022'};
fprintf('%s\n\n', ctrl_names{CTRL_SEL});

%% =========================================================================
%  SHARED INITIALISATION
% =========================================================================
Constants;
InitVar;
InitGains_Comparison;

switch CTRL_SEL
    case 1,  K_ctrl = K_PLASMC;
    case 2,  K_ctrl = K_Lin2022;
    case 3,  K_ctrl = K_Zhang2026;
    case 4,  K_ctrl = K_Chen2025;
    case 5,  K_ctrl = K_Cho2022;
end

N_steps = numel(tRange);

%% =========================================================================
%  PRE-ALLOCATE SHARED LOGGING / BUFFER ARRAYS
% =========================================================================
U_DS      = zeros(4,  N_steps);
X_DS      = zeros(13, N_steps + 1);   X_DS(:,1) = x_c;
V_X_DS    = zeros(24, N_steps);
D_DS      = zeros(17, N_steps);
P_DS      = zeros(2,  12, N_steps);   % [V_nP_i(2x4), V_nP_a(2x4), C_nP(2x4)]

x_t       = zeros(7,  N_steps);
dx_t      = zeros(6,  N_steps);

V_h_d     = zeros(3,  N_steps);
V_h_e     = zeros(3,  N_steps);
I_a_cd    = zeros(3,  N_steps);

% Inner-loop logging
E_e       = zeros(3,  N_steps);
iE_e      = zeros(3,  N_steps);
raw_dE_e  = zeros(3,  N_steps + 3);
dE_e      = zeros(3,  N_steps);
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

% V_w_a buffer for smooth derivative (same pattern as _temp.m)
raw_dw_a  = zeros(3,  N_steps + 3);

% _prev variables (replace buffer+smooth4 in _temp.m)
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

%% =========================================================================
%  PRE-ALLOCATE PLASMC-SPECIFIC ARRAYS  (only if CTRL_SEL == 1)
% =========================================================================
if CTRL_SEL == 1
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
end

%% =========================================================================
%  PRE-ALLOCATE OTHER CONTROLLER STATES
% =========================================================================
if CTRL_SEL == 2
    rho_p0_lin = [];
    rho_v0_lin = [];
end

if CTRL_SEL == 3
    xhat_AF     = K_ctrl.xhat_AF0;
    omega_AF    = K_ctrl.omega_AF0;
    F_c_prev    = [0; 0; -m * 9.81];
    I_vm_c_prev = x_c(8:10);   % previous measured velocity for AEDO finite difference
end

if CTRL_SEL == 4
    e_hat_chen    = zeros(3,1);
    vvs_hat_chen  = zeros(3,1);
    v0_chen       = zeros(3,1);
    zstar_hat     = K_ctrl.zstar0;
    izeta_obs     = zeros(3,1);
    V_s_prev_chen = zeros(3,1);   % previous V_s for trapezoidal integral
end

%% =========================================================================
%  MAIN SIMULATION LOOP
% =========================================================================
% Safe default u_2 before loop (hover thrust, zero torques)
u_2 = [zeros(3,1); m*norm(g)];

for idx = 1:N_steps

% *************************************************************************
% Compute rotation matrices and yaw  (moved before trajectory in _temp.m)
% *************************************************************************
    I_R_C = quat2rotm(q_c');
    E_cr  = quat2eul(q_c', 'XYZ');

    yaw = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
                1 - 2*(q_c(3)^2 + q_c(4)^2));

    I_R_V = rotz(rad2deg(yaw));

% *************************************************************************
% Target trajectory  (Circular in _temp.m)
% *************************************************************************
    traj_t      = traj_Gen((idx-1)*dt, "Linear");
    x_t(:,idx)  = traj_t(:,1);
    dx_t(:,idx) = traj_t(1:end-1, 2);
    I_R_T       = quat2rotm(x_t(4:7,idx)');

% *************************************************************************
% Image feature block — gated by ZOH  (entire block inside if in _temp.m)
% *************************************************************************
    if mod(idx-1, ZOH) == 0

        % Feature points in inertial frame
        I_nP3 = I_R_T * T_nP3 + x_t(1:3,idx);

        % Feature points in camera frame
        C_nP3  = transpose(I_R_C) * (I_nP3 - I_p_c);

        % Target position in camera frame
        C_s_tc = transpose(I_R_C) * (x_t(1:3,idx) - I_p_c);

        % Project to image plane  (depth includes zf offset)
        C_nP = (f / (C_s_tc(3) + zf)) * C_nP3(1:2,:);

        % Noise  (awgn in _temp.m, not randi+round)
        if NOISE
            C_nP = awgn(C_nP, 50, 'measured');
        end

        % Virtual image plane transform
        V_R_C  = I_R_V' * I_R_C;
        rays   = [C_nP; f * ones(1, size(C_nP,2))];
        vr     = V_R_C * rays;
        V_nP_i = f * vr(1:2,:) ./ vr(3,:);

        % Image Jacobian
        for j = 1:size(C_nP,2)
            L_s(2*j-1:2*j,:) = [ f,  0, -V_nP_i(1,j), ...
                -V_nP_i(1,j)*V_nP_i(2,j)/f, (f^2+V_nP_i(1,j)^2)/f, -V_nP_i(2,j);
                 0,  f, -V_nP_i(2,j), ...
                -(f^2+V_nP_i(2,j)^2)/f, V_nP_i(1,j)*V_nP_i(2,j)/f,  V_nP_i(1,j)];
        end

        V_s_i = image_feature(V_nP_i / f);   % [xhat; yhat; 1; alpha]

        % Optical flow via image Jacobian  (_prev variables, not buffer)
        V_2nP_i = reshape(V_nP_i, [], 1);
        if idx == 1
            dPdt = zeros(size(V_2nP_i));
        else
            dPdt = (V_2nP_i - V_2nP_i_prev) / dt / ZOH;
        end
        V_2nP_i_prev = V_2nP_i;

        V_v_i = pinv(L_s, 4) * dPdt;
        V_h_i = V_v_i(1:3);
        V_w_i = V_v_i(4:6);

        % Angular rate derivative  (_prev variable)
        if idx == 1
            V_dw_i = zeros(3,1);
        else
            V_dw_i = (V_w_i - V_w_i_prev) / dt / ZOH;
        end
        V_w_i_prev = V_w_i;

        % Analytical image parameters
        V_nP3  = transpose(I_R_V) * (I_nP3 - I_p_c);
        V_s_tc = transpose(I_R_V) * (x_t(1:3,idx) - I_p_c);
        V_nP_a = (f / (V_s_tc(3) + zf)) * V_nP3(1:2,:);
        V_s_a  = image_feature(V_nP_a / f);
        V_h_a  = transpose(I_R_V) * (dx_t(1:3,idx) - I_v_c) / (V_s_tc(3) + zf);

    end   % ZOH gate

    % V_w_a computed every step (angular rate of virtual plane)
    I_w_c = I_R_C * B_w_c;
    V_w_a = transpose(I_R_V) * (dx_t(4:6,idx) - [0;0;I_w_c(3)]);

    % Smooth derivative of V_w_a  (buffer pattern, same as _temp.m)
    if idx == 1
        raw_dw_a(:,idx+3) = zeros(3,1);
    else
        raw_dw_a(:,idx+3) = (V_w_a - V_w_a_prev) / dt;
    end
    V_dw_a    = smooth4(raw_dw_a(:,end-3:end));
    V_w_a_prev = V_w_a;

% *************************************************************************
% Select actual vs analytical image parameters
% Savitzky-Golay filter in ACTUAL mode  (replaces mean/smooth4 in _temp.m)
% *************************************************************************
    if ACTUAL
        V_s_raw(:,idx) = V_s_i;
        V_h_raw(:,idx) = V_h_i;
        V_w_raw(:,idx) = V_w_i;
        V_dw_raw(:,idx)= V_dw_i;

        if idx < FILTER_WINDOW
            V_s  = mean(V_s_raw(:,  1:idx), 2);
            V_h  = mean(V_h_raw(:,  1:idx), 2);
            V_w  = mean(V_w_raw(:,  1:idx), 2);
            V_dw = mean(V_dw_raw(:, 1:idx), 2);
        else
            V_s_vec  = sgolayfilt(V_s_raw(:,  idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_s      = V_s_vec(:,  floor(FILTER_WINDOW/2 + 1));
            V_h_vec  = sgolayfilt(V_h_raw(:,  idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_h      = V_h_vec(:,  floor(FILTER_WINDOW/2 + 1));
            V_w_vec  = sgolayfilt(V_w_raw(:,  idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_w      = V_w_vec(:,  floor(FILTER_WINDOW/2 + 1));
            V_dw_vec = sgolayfilt(V_dw_raw(:, idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_dw     = V_dw_vec(:, floor(FILTER_WINDOW/2 + 1));
        end
    else
        V_s  = V_s_a;
        V_h  = V_h_a;
        V_w  = V_w_a;
        V_dw = V_dw_a;
    end

% *************************************************************************
% OUTER LOOP: Controller-specific I_a_cd
% *************************************************************************
    switch CTRL_SEL

        %------------------------------------------------------------------
        case 1   % PLASMC — inline, matching _temp.m exactly
        %------------------------------------------------------------------

        % Visibility constraint performance function
        p_1(:,idx)  = expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * ...
                      (K_ctrl.p_10 - K_ctrl.p_1inf) + K_ctrl.p_1inf;
        dp_1(:,idx) = -diag(K_ctrl.gamma_1) * ...
                       expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * ...
                       (K_ctrl.p_10 - K_ctrl.p_1inf);

        % Error and transformation S_1, zeta_1
        V_s_e(:,idx) = V_s(1:2) - V_s_d(1:2);   % V_s is a column vector now
        for j = 1:2
            S_1(j,j,idx) = V_s_e(j,idx) / p_1(j,idx);
            % Clamp to open interval (-1,1)  (min/max, no flag-break)
            S_1(j,j,idx) = min(max(S_1(j,j,idx), -1+eps), 1-eps);
            zeta_1(j,idx) = log((1 + S_1(j,j,idx)) / (1 - S_1(j,j,idx)));
            G_1(j,j,idx)  = (exp(zeta_1(j,idx)) + 1)^2 / ...
                             (2 * exp(zeta_1(j,idx)) * p_1(j,idx));
        end

        % Integral and derivative of zeta_1  (init: dt not dt/2)
        if idx == 1
            izeta_1(:,idx)       = dt * zeta_1(:,idx);
            raw_dzeta_1(:,idx+3) = zeros(2,1);
        else
            izeta_1(:,idx)       = izeta_1(:,idx-1) + ...
                                   dt*(zeta_1(:,idx-1) + zeta_1(:,idx))/2;
            raw_dzeta_1(:,idx+3) = (zeta_1(:,idx) - zeta_1(:,idx-1)) / dt;
        end
        dzeta_1  = smooth4(raw_dzeta_1(:,idx:idx+3));
        dzeta_1d = -K_ctrl.zp*zeta_1(:,idx) - K_ctrl.zi*izeta_1(:,idx) ...
                   -K_ctrl.zd*dzeta_1;

        % Desired optical flow h_d  (V_s is column vector, not indexed)
        V_ds_d       = [G_1(:,:,idx)\dzeta_1d + S_1(:,:,idx)*dp_1(:,idx); 0.0];
        V_h_d(:,idx) = V_ds_d + cross(V_w, V_s(1:3)) + ...
                       (h_rd - dot(cross(V_w, V_s(1:3)), e3)) * V_s(1:3);

        % Optical flow error
        V_h_e(:,idx) = V_h - V_h_d(:,idx);

        % Optical flow performance function
        p_2(:,idx)  = expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * ...
                      (K_ctrl.p_20 - K_ctrl.p_2inf) + K_ctrl.p_2inf;
        dp_2(:,idx) = -diag(K_ctrl.gamma_2) * ...
                       expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * ...
                       (K_ctrl.p_20 - K_ctrl.p_2inf);

        % Transformation S_2, zeta_2  (clamp, no flag-break)
        for j = 1:3
            S_2(j,j,idx) = V_h_e(j,idx) / p_2(j,idx);
            S_2(j,j,idx) = min(max(S_2(j,j,idx), -1+eps), 1-eps);
            zeta_2(j,idx) = log((1 + S_2(j,j,idx)) / (1 - S_2(j,j,idx)));
            G_2(j,j,idx)  = (exp(zeta_2(j,idx)) + 1)^2 / ...
                             (2 * exp(zeta_2(j,idx)) * p_2(j,idx));
        end

        % Integral of zeta_2  (init: dt not dt/2)
        if idx == 1
            izeta_2(:,idx) = dt * zeta_2(:,idx);
        else
            izeta_2(:,idx) = izeta_2(:,idx-1) + ...
                             dt*(zeta_2(:,idx-1) + zeta_2(:,idx))/2;
        end
        sigma(:,idx) = zeta_2(:,idx) + K_ctrl.Omega * izeta_2(:,idx);

        % Known dynamics c
        if idx == 1
            raw_dh_d(:,idx+3) = zeros(3,1);
        else
            raw_dh_d(:,idx+3) = (V_h_d(:,idx) - V_h_d(:,idx-1)) / dt;
        end
        V_dh_d = smooth4(raw_dh_d(:,idx:idx+3));

        c_dyn = cross(V_dw, V_s(1:3)) ...
              + cross(V_w, cross(V_w, V_s(1:3))) ...
              + 2*cross(V_w, V_h) ...
              - (dot(V_h + cross(V_w, V_s(1:3)), e3)) * V_h ...
              - V_dh_d;

        Theta = [-c_dyn + S_2(:,:,idx)*dp_2(:,idx) ...
                           - G_2(:,:,idx)\(K_ctrl.Omega*zeta_2(:,idx)), eye(3)];
        Theta_norm = norm(Theta, 'fro');

        % Update kappa
        const_kappa = [K_ctrl.N; K_ctrl.P];
        u_kappa     = [sigma(:,idx); Theta_norm];
        kappa(:,idx+1) = RK5(@(t,X) kappa_Solver(t, X, u_kappa, ...
                              const_kappa, G_2(:,:,idx)), t0, kappa(:,idx), dt);
        if any(isnan(kappa(:,idx+1))), break; end

        % Control law
        u_sw = -K_ctrl.Gamma * sigma(:,idx) ...
               - Theta_norm * diag(sat(K_ctrl.E \ sigma(:,idx))) ...
                 * G_2(:,:,idx) * kappa(:,idx+1);
        u_eq = G_2(:,:,idx) * (-c_dyn + S_2(:,:,idx)*dp_2(:,idx) ...
               - G_2(:,:,idx)\(K_ctrl.Omega*zeta_2(:,idx)));

        V_a_cd        = -G_2(:,:,idx) \ (u_sw + u_eq);
        I_a_cd(:,idx) = I_R_V * V_a_cd - g;

        %------------------------------------------------------------------
        case 2   % Lin 2022 — outer loop + geometric SO(3) inner loop
        %------------------------------------------------------------------
        if isempty(rho_p0_lin)
            e_p_init   = (I_p_c - x_t(1:3,idx)) - K_ctrl.r_pt_des;
            rho_p0_lin = abs(e_p_init) + K_ctrl.rho_p0_margin;
            % Compute initial virtual velocity so rho_v0 covers e_v at t=0
            xi_p_init  = max(min(e_p_init ./ rho_p0_lin, 0.999), -0.999);
            eps_p_init = 0.5 * log((1 + xi_p_init) ./ (1 - xi_p_init));
            q_p_init   = 1 ./ ((1 + xi_p_init) .* (1 - xi_p_init));
            vhat_init  = -K_ctrl.k1 * (q_p_init .* eps_p_init);
            rho_v0_lin = abs(I_v_c - vhat_init) + K_ctrl.rho_v0_margin;
        end
        rho_p     = (rho_p0_lin - K_ctrl.rho_inf_p) .* ...
                     exp(-K_ctrl.l_p * tRange(idx)) + K_ctrl.rho_inf_p;
        rho_p_dot = -(rho_p0_lin - K_ctrl.rho_inf_p) .* K_ctrl.l_p .* ...
                     exp(-K_ctrl.l_p * tRange(idx));
        rho_v     = (rho_v0_lin - K_ctrl.rho_inf_v) .* ...
                     exp(-K_ctrl.l_v * tRange(idx)) + K_ctrl.rho_inf_v;
        rho_v_dot = -(rho_v0_lin - K_ctrl.rho_inf_v) .* K_ctrl.l_v .* ...
                     exp(-K_ctrl.l_v * tRange(idx));

        [u_2, I_a_cd(:,idx), ~] = ...
            ctrl_Lin2022(I_p_c, I_v_c, x_t(1:3,idx), ...
                         rho_p, rho_p_dot, rho_v, rho_v_dot, ...
                         K_ctrl.r_pt_des, K_ctrl.psi_des, ...
                         I_R_C, B_w_c, K_ctrl, m, J, g, e3, ...
                         tau_xy_max, tau_z_max, T_max, T_min);

        %------------------------------------------------------------------
        case 3   % Zhang 2026 — AEDO backstepping + geometric inner loop
        %------------------------------------------------------------------
        zpq         = abs(I_p_c(3) - x_t(3,idx));
        P_NF_in     = polyval(K_ctrl.PNF_poly, zpq);
        I_vm_c_curr = x_c(8:10) + 0.015*randn(3,1);

        [u_2, xhat_AF, omega_AF, I_a_cd(:,idx)] = ...
            ctrl_Zhang2026(I_p_c, I_v_c, x_t(1:3,idx), dx_t(1:3,idx), ...
                           I_vm_c_prev, F_c_prev, ...
                           xhat_AF, P_NF_in, omega_AF, dt, ...
                           I_R_C, B_w_c, K_ctrl, m, J, g, e3, ...
                           tau_xy_max, tau_z_max, T_max, T_min);
        % Update AEDO variables: I_vm_c_prev stores last measured velocity
        % for finite-difference acceleration estimate in next step
        I_vm_c_prev = I_vm_c_curr;
        F_c_prev    = m * (I_a_cd(:,idx) + g);

        %------------------------------------------------------------------
        case 4   % Chen 2025 — IBVS observer + geometric inner loop
        %------------------------------------------------------------------
        psi_dot_curr = B_w_c(3);
        V_s_chen = [V_s(1:2); V_s_tc(3) / K_ctrl.zstar0];
        if idx == 1
            izeta_obs     = dt * (V_s_chen - K_ctrl.q_d) / 2;
            V_s_prev_chen = V_s_chen;
            e_hat_chen    = V_s_chen;   % sync observer to first measurement so
                                        % zeta_e=0 at t=0; prevents k2*(e-0) blowup
        else
            izeta_obs     = izeta_obs + ...
                dt*(V_s_prev_chen + V_s_chen - 2*K_ctrl.q_d)/2;
            V_s_prev_chen = V_s_chen;
        end
        [u_2, e_hat_chen, vvs_hat_chen, v0_chen, zstar_hat, I_a_cd(:,idx)] = ...
            ctrl_Chen2025(V_s_chen, K_ctrl.q_d, ...
                          e_hat_chen, vvs_hat_chen, v0_chen, zstar_hat, ...
                          izeta_obs, psi_dot_curr, dt, ...
                          I_R_C, I_R_V, B_w_c, K_ctrl, m, J, g, e3, ...
                          tau_xy_max, tau_z_max, T_max, T_min, K_ctrl.psi_des);

        %------------------------------------------------------------------
        case 5   % Cho 2022 — FF-IBVS + geometric inner loop
        %------------------------------------------------------------------
        [u_2, I_a_cd(:,idx), ~, ~] = ...
            ctrl_Cho2022(V_nP_i, V_nP_d, C_s_tc, dx_t(1:3,idx), ...
                         E_cr', f, K_ctrl.lambda_IBVS, K_ctrl.k_sigmoid, ...
                         K_ctrl.use_sq_comp, I_v_c, ...
                         I_R_C, I_R_V, B_w_c, K_ctrl, m, J, g, e3, ...
                         tau_xy_max, tau_z_max, T_max, T_min, K_ctrl.psi_des);

    end   % switch CTRL_SEL

% *************************************************************************
% SAFETY CHECK  (on I_a_cd for all; on u_2 for cases 2-5)
% *************************************************************************
    if norm(I_a_cd(:,idx)) > 1e2 || any(isnan(I_a_cd(:,idx))), break; end
    if CTRL_SEL > 1 && (any(isnan(u_2)) || norm(u_2) > 1e4), break; end

% *************************************************************************
% SHARED: Attitude PID inner loop — PLASMC (case 1) only
% *************************************************************************
    if CTRL_SEL == 1

    if abs(cos(yaw)*I_a_cd(1,idx) + sin(yaw)*I_a_cd(2,idx)) < 1e-4
        theta_cd = 0;
    else
        theta_cd = atan2(-cos(yaw)*I_a_cd(1,idx) - sin(yaw)*I_a_cd(2,idx), ...
                         -I_a_cd(3,idx));
    end
    if abs(sin(yaw)*I_a_cd(1,idx) - cos(yaw)*I_a_cd(2,idx)) < 1e-4
        phi_cd = 0;
    else
        phi_cd = atan2(-sin(yaw)*I_a_cd(1,idx) + cos(yaw)*I_a_cd(2,idx), ...
                       -I_a_cd(3,idx)/cos(E_cr(2)));
    end
    E_crd = [phi_cd; theta_cd];

    E_e(:,idx) = [E_cr(1:2)'; -V_s(4)] - [E_crd; -V_s_d(4)];

    if idx == 1
        iE_e(:,idx)       = dt * E_e(:,idx) / 2;
        raw_dE_e(:,idx+3) = zeros(3,1);
        B_w_cf(:,idx)     = B_w_c;
        B_dw_cf           = zeros(3,1);
    else
        iE_e(:,idx)       = iE_e(:,idx-1) + dt*(E_e(:,idx-1)+E_e(:,idx))/2;
        raw_dE_e(:,idx+3) = (E_e(:,idx) - E_e(:,idx-1)) / dt;
        B_w_cf(:,idx)     = alpha_w*B_w_cf(:,idx-1) + (1-alpha_w)*B_w_c;
        B_dw_cf           = alpha_dw*B_dw_cf + ...
                            (1-alpha_dw)*(B_w_cf(:,idx)-B_w_cf(:,idx-1))/dt;
    end
    dE_e(:,idx) = smooth4(raw_dE_e(:,end-3:end));

    dE_cd = -K_ctrl.ep*E_e(:,idx) - K_ctrl.ei*iE_e(:,idx) ...
            -K_ctrl.ed*raw_dE_e(:,idx+3);
    if norm(dE_cd) > 1e2, break; end

    W = [1,  0,              -sin(E_cr(2));
         0,  cos(E_cr(1)),    sin(E_cr(1))*cos(E_cr(2));
         0, -sin(E_cr(1)),    cos(E_cr(1))*cos(E_cr(2))];
    B_w_cd(:,idx) = W * dE_cd;

    B_T_cd(idx) = -m * I_a_cd(3,idx) / (cos(E_cr(1))*cos(E_cr(2)));
    if GE
        B_T_cd(idx) = 1/(1-(r/(4*x_c(3)))^2) * B_T_cd(idx);
    end

    if idx > delay
        u_1 = [B_w_cd(:,idx-delay); B_T_cd(idx-delay)];
    else
        u_1 = [zeros(3,1); m*dot(g, I_R_C(:,3))];
    end
    u_1(4)   = max(min(u_1(4),   T_max),  T_min);
    u_1(1:3) = max(min(u_1(1:3), w_max), -w_max);

    w_e(:,idx) = B_w_c - u_1(1:3);
    if idx == 1
        iw_e(:,idx) = dt * w_e(:,idx) / 2;
    else
        iw_e(:,idx) = iw_e(:,idx-1) + (w_e(:,idx-1)+w_e(:,idx))*dt/2;
    end

    B_dw_cd  = -K_ctrl.wp*w_e(:,idx) - K_ctrl.wi*iw_e(:,idx) ...
               -K_ctrl.wd*B_dw_cf   + K_ctrl.ff*u_1(1:3);
    B_tau_cd = J*B_dw_cd + cross(B_w_c, J*B_w_c);
    B_tau_cd(1:2) = min(max(B_tau_cd(1:2), -tau_xy_max),  tau_xy_max);
    B_tau_cd(3)   = min(max(B_tau_cd(3),   -tau_z_max),   tau_z_max);
    anti_windup_factor = abs(B_tau_cd) >= [tau_xy_max; tau_xy_max; tau_z_max];
    iw_e(:,idx)        = iw_e(:,idx) .* (~anti_windup_factor);

    u_2 = [B_tau_cd; u_1(4)];

    end   % CTRL_SEL == 1 (attitude PID)

% *************************************************************************
% UAV dynamics integration  (all controllers: u_2 = [tau; T])
% *************************************************************************
    x_c = RK5(@(t,x) UAVDyn(t,x,u_2), t0, x_c, dt);
    if any(isnan(x_c)), break; end

    I_p_c = x_c(1:3);   q_c   = x_c(4:7);
    I_v_c = x_c(8:10);  B_w_c = x_c(11:13);
    q_c   = q_c / norm(q_c);

% *************************************************************************
% Logging  (matches _temp.m: V_X_DS uses V_dw_i not column-indexed dw_a)
% *************************************************************************
    U_DS(:,idx)   = u_2;
    X_DS(:,idx+1) = x_c;
    V_X_DS(:,idx) = [V_s_i(1:2); V_s_i(4); V_h_i; V_w_i; V_dw_i; ...
                     V_s_a(1:2); V_s_a(4); V_h_a; V_w_a; V_dw_a];
    P_DS(:,:,idx) = [V_nP_i, V_nP_a, C_nP];
    % D_DS layout (matches plotter_adaptive):
    %   rows 1-3:  V_h_d   (desired optical flow)
    %   rows 4-6:  I_a_cd  (desired acceleration)
    %   rows 7-8:  E_crd   (desired roll, pitch)
    %   rows 9-11: dE_cd   (desired Euler rate)
    %   rows 12-14: B_w_cd (desired body rate)
    %   rows 15-17: B_dw_cd (desired body angular accel)
    if CTRL_SEL == 1
        % All fields populated by the shared attitude-PID inner loop
        D_DS(:,idx) = [V_h_d(:,idx); I_a_cd(:,idx); E_crd; ...
                       dE_cd; B_w_cd(:,idx); B_dw_cd];
    else
        % Cases 2-5: inner loop runs inside controller function.
        % E_crd, dE_cd, B_w_cd, B_dw_cd extracted from u_2 for logging.
        B_tau_log  = u_2(1:3);                   % torque command
        T_log      = u_2(4);                     % thrust command
        % Back-compute desired Euler angles from I_a_cd for logging
        yaw_log = atan2(2*(q_c(1)*q_c(4)+q_c(2)*q_c(3)), ...
                        1-2*(q_c(3)^2+q_c(4)^2));
        if abs(cos(yaw_log)*I_a_cd(1,idx)+sin(yaw_log)*I_a_cd(2,idx)) < 1e-4
            theta_log = 0;
        else
            theta_log = atan2(-cos(yaw_log)*I_a_cd(1,idx) ...
                              -sin(yaw_log)*I_a_cd(2,idx), -I_a_cd(3,idx));
        end
        if abs(sin(yaw_log)*I_a_cd(1,idx)-cos(yaw_log)*I_a_cd(2,idx)) < 1e-4
            phi_log = 0;
        else
            E_cr_log = quat2eul(q_c','XYZ');
            phi_log = atan2(-sin(yaw_log)*I_a_cd(1,idx) ...
                            +cos(yaw_log)*I_a_cd(2,idx), ...
                            -I_a_cd(3,idx)/cos(E_cr_log(2)));
        end
        E_crd_log = [phi_log; theta_log];
        D_DS(:,idx) = [zeros(3,1); I_a_cd(:,idx); E_crd_log; ...
                       zeros(3,1); B_tau_log; zeros(3,1)];
    end

% *************************************************************************
% Termination  (0.1 m in _temp.m, not 0.18 m)
% *************************************************************************
    if norm(I_p_c - x_t(1:3,idx)) <= 0.1
        fprintf('Landed at t = %.2f s\n', tRange(idx));
        break;
    end

    t0 = t0 + dt;

end   % main loop

idx = idx - 1;

%% =========================================================================
%  SAVE AND PLOT
% =========================================================================
% Patch PLASMC-specific fields so comp_result.mat is always loadable
% by run_comparison (which calls plotter_comparison, not plotter_adaptive).
if ~exist('p_1',    'var') || isempty(p_1),    p_1    = zeros(2, idx); end %#ok<NODEF>
if ~exist('p_2',    'var') || isempty(p_2),    p_2    = zeros(3, idx); end
if ~exist('S_1',    'var') || isempty(S_1),    S_1    = zeros(2,2,idx); end
if ~exist('zeta_1', 'var') || isempty(zeta_1), zeta_1 = zeros(2, idx); end

save("comp_result.mat");
fprintf('\nDone: %s  (%d steps)\n', ctrl_names{CTRL_SEL}, idx);
