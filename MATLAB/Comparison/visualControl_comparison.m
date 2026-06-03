% Shared helpers live in ../Common (collapsed from per-folder duplicates)
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Common'));

%% =========================================================================
% visualControl_comparison.m
%
% Comparative Study: Five Outer-Loop Landing Controllers
%   1. PLASMC     - Proposed (Singhal et al.)
%   2. Lin 2022   - Lin et al., IEEE TII 2022
%   3. Zhang 2026 - Zhang & Wu, IEEE TIE 2026
%   4. Lin 2023  - Lin et al., IEEE T-ASE 2023 (replaced Chen 2025)
%   5. Cho 2022   - Cho et al., Aerosp. Sci. Technol. 2022
%
% Based on: visualControl_IBVS_adaptive.m
%
% Changes from previous comparison script (to match _temp.m):
%   1. yaw / I_R_V computed before trajectory (not after)
%   2. Trajectory type: "Circular"
%   3. Entire image-feature block inside ZOH gate (only update every ZOH steps)
%   4. Depth: f/(C_s_tc(3)+zf) and f/(V_s_tc(3)+zf) (not f/z)
%   5. Noise: depth-dep sigma_px + outliers (shared init_robustness.m)
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
if exist('MC_SEED','var') == 1
    clearvars -except MC_SEED SPEED_MULT IC_OVERRIDE MS_STATE CTRL_SEL TRAJ_TYPE c ctrl_list ctrl_names trajType all_results;
else
    clearvars -except SPEED_MULT IC_OVERRIDE MS_STATE CTRL_SEL TRAJ_TYPE c ctrl_list ctrl_names trajType all_results;
end
if ~exist('SPEED_MULT','var') || isempty(SPEED_MULT)
    SPEED_MULT = 1.0;
end
if exist('MC_SEED','var') == 1
    rng(MC_SEED);
else
    rng('shuffle');
end

%% =========================================================================
%  SELECT CONTROLLER
%   1 = PLASMC (proposed)
%   2 = Lin 2022
%   3 = Zhang 2026
%   4 = Lin 2023
%   5 = Cho 2022
% =========================================================================
% CTRL_SEL = 1;   % <-- change here

ctrl_names = {'PLASMC (Proposed)', 'Lin 2022', ...
              'Zhang 2026', 'Lin 2023', 'Cho 2022'};
fprintf('%s\n\n', ctrl_names{CTRL_SEL});

%% =========================================================================
%  SHARED INITIALISATION
% =========================================================================
Constants;
InitVar;
% Allow caller-side IC override (e.g., multi-speed sweep at IC_1=[0,0,-5]).
% When IC_OVERRIDE is set, replace the position component of x_c/X_DS.
if exist('IC_OVERRIDE','var') && ~isempty(IC_OVERRIDE)
    x_c(1:3)    = IC_OVERRIDE(:);
    X_DS(1:3,1) = IC_OVERRIDE(:);
end
init_robustness;    % samples wind / mass / inertia / CoG / pixel-noise model
InitGains_Comparison;

switch CTRL_SEL
    case 1,  K_ctrl = K_PLASMC;
    case 2,  K_ctrl = K_Lin2022;
    case 3,  K_ctrl = K_Zhang2026;
    case 4,  K_ctrl = K_Lin2023;
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

% Inner-loop logging (geometric SO(3) for case 1)
eR_log       = zeros(3, N_steps);
B_tau_cd_log = zeros(3, N_steps);
B_T_cd       = zeros(1, N_steps);
psi_d_log    = zeros(1, N_steps);
u_a_log      = zeros(1, N_steps);

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

% Initial desired heading = current UAV yaw (for SO(3) PLASMC heading generator)
yaw_init = atan2(2*(q_c(1)*q_c(4) + q_c(2)*q_c(3)), ...
                 1 - 2*(q_c(3)^2 + q_c(4)^2));
psi_d    = yaw_init;

% I_a_cd LPF state (mimics PID cascade filter budget under noise)
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

%% =========================================================================
%  PRE-ALLOCATE u_2 BUFFER FOR DELAY (all controllers)
% =========================================================================
u_2_buf   = zeros(4, N_steps);
u_2_hover = [zeros(3,1); m*norm(g)];

%% =========================================================================
%  PRE-ALLOCATE PLASMC-SPECIFIC ARRAYS  (only if CTRL_SEL == 1)
% =========================================================================
if CTRL_SEL == 1
    % Approach 2: no visibility funnel (p_1/S_1/G_1/zeta_1/izeta_1 removed).
    p_2         = zeros(3, N_steps);
    dp_2        = zeros(3, N_steps);
    S_2         = zeros(3, 3, N_steps);
    G_2         = zeros(3, 3, N_steps);
    zeta_2      = zeros(3, N_steps);
    izeta_2     = zeros(3, N_steps);
    sigma       = zeros(3, N_steps);
    raw_dh_d    = zeros(3, N_steps + 3);

    % Outer-loop PID on normalized raw error (Approach 2)
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
    % Lin 2023 funnel states (adaptive rho(0)=|e(0)|+margin, set at idx==1)
    rho_t0_lin = [];
    rho_v0_lin = [];
end

%% =========================================================================
%  MAIN SIMULATION LOOP
% =========================================================================
% Safe default u_2 before loop (hover thrust, zero torques)
u_2 = [zeros(3,1); m*norm(g)];

landed  = false;
precise = false;
soft    = false;

% FoV failure flag — strict.  Any physical-corner pixel leaving the sensor
% box [-res(1)/2, res(1)/2] x [-res(2)/2, res(2)/2] counts as controller
% failure (target lost from camera view).  Applies to all 5 controllers
% since they share the C_nP projection.
fov_fail   = false;
fov_fail_t = NaN;

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
% Target trajectory  (set TRAJ_TYPE in workspace, defaults to "Circular")
% *************************************************************************
    if ~exist('TRAJ_TYPE', 'var')
        TRAJ_TYPE = "Static";
    end
    traj_t      = traj_Gen((idx-1)*dt, TRAJ_TYPE, SPEED_MULT);
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

        % Depth-dependent pixel noise (shared with run_simulation.m)
        if NOISE
            z_dep = max(abs(C_s_tc(3)), 0.1);
            sigma_px = px_sigma0 + px_sigma1 / (z_dep + px_depth_offset);
            C_nP = C_nP + (sigma_px / f) * randn(size(C_nP));
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
            V_s      = V_s_vec(:,  end);
            V_h_vec  = sgolayfilt(V_h_raw(:,  idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_h      = V_h_vec(:,  end);
            V_w_vec  = sgolayfilt(V_w_raw(:,  idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_w      = V_w_vec(:,  end);
            V_dw_vec = sgolayfilt(V_dw_raw(:, idx-FILTER_WINDOW+1:idx), 2, FILTER_WINDOW, [], 2);
            V_dw     = V_dw_vec(:, end);
        end
    else
        V_s  = V_s_a;
        V_h  = V_h_a;
        V_w  = V_w_a;
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
% OUTER LOOP: Controller-specific I_a_cd
% *************************************************************************
    switch CTRL_SEL

        %------------------------------------------------------------------
        case 1   % PLASMC (Approach 2) — raw-error PID + funnel-margin cone clamp
        %------------------------------------------------------------------

        % Desired Optical Flow (Approach 2 — raw-error PID on normalized r_e)
        % No virtual-feature visibility funnel: physical-corner visibility is
        % enforced downstream by the funnel-margin cone clamp on I_a_cd.
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

        % Optical flow performance function
        p_2(:,idx)  = expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * ...
                      (K_ctrl.p_20 - K_ctrl.p_2inf) + K_ctrl.p_2inf;
        dp_2(:,idx) = -diag(K_ctrl.gamma_2) * ...
                       expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * ...
                       (K_ctrl.p_20 - K_ctrl.p_2inf);

        % Transformation S_2, zeta_2
        S_2_margin = 0.05;   % funnel saturation guard: keeps |zeta_2|<=3.66, G_2 finite
        for j = 1:3
            S_2(j,j,idx) = V_h_e(j,idx) / p_2(j,idx);
            S_2(j,j,idx) = min(max(S_2(j,j,idx), -1+S_2_margin), 1-S_2_margin);
            zeta_2(j,idx) = log((1 + S_2(j,j,idx)) / (1 - S_2(j,j,idx)));
            G_2(j,j,idx)  = (exp(zeta_2(j,idx)) + 1)^2 / ...
                             (2 * exp(zeta_2(j,idx)) * p_2(j,idx));
        end

        % Integral of zeta_2
        if idx == 1
            izeta_2(:,idx) = dt * zeta_2(:,idx);
        else
            izeta_2(:,idx) = izeta_2(:,idx-1) + ...
                             dt*(zeta_2(:,idx-1) + zeta_2(:,idx))/2;
        end
        izeta_2_max = 5.0;
        izeta_2(:,idx) = max(min(izeta_2(:,idx), izeta_2_max), -izeta_2_max);
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

        if norm(I_a_cd(:,idx)) > 1e02 || any(isnan(I_a_cd(:,idx)))
            fprintf('  BREAK: I_a_cd norm=%.2f or NaN at idx=%d (t=%.2f)\n', ...
                    norm(I_a_cd(:,idx)), idx, tRange(idx));
            break;
        end

        % funnel-margin cone clamp (Approach 2)
        %   theta_current = acos(I_R_C(3,3))
        %   rho_fov(t)    = shrinking box half-widths centered on image center
        %   d_min         = min axis-wise margin of any physical corner to box
        %   theta_cone    = min(theta_current + atan(d_min/f), theta_cap)
        R33           = max(min(I_R_C(3,3), 1), -1);
        theta_current = acos(R33);

        rho_fov_curr = (K_ctrl.rho_fov_0 - K_ctrl.rho_fov_inf) * ...
                       exp(-K_ctrl.l_fov * tRange(idx)) + K_ctrl.rho_fov_inf;

        d_corner_x = rho_fov_curr(1) - abs(C_nP(1,:));
        d_corner_y = rho_fov_curr(2) - abs(C_nP(2,:));
        d_min_fov  = max(min([d_corner_x, d_corner_y]), 0);

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

        % LPF I_a_cd before R_d construction (noise absorber)
        I_a_cd_filt = alpha_ia * I_a_cd_filt + (1 - alpha_ia) * I_a_cd(:,idx);

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

        % PBVS measurement noise on 3D position/velocity
        if NOISE
            I_p_cm = I_p_c + sigma_pos * randn(3,1);
            I_v_cm = I_v_c + sigma_vel * randn(3,1);
        else
            I_p_cm = I_p_c;
            I_v_cm = I_v_c;
        end

        [u_2, I_a_cd(:,idx), ~] = ...
            ctrl_Lin2022(I_p_cm, I_v_cm, x_t(1:3,idx), ...
                         rho_p, rho_v, ...
                         K_ctrl.r_pt_des, K_ctrl.psi_des, ...
                         I_R_C, B_w_c, K_ctrl, m, J, g, ...
                         tau_xy_max, tau_z_max, T_max, T_min);

        %------------------------------------------------------------------
        case 3   % Zhang 2026 — AEDO backstepping + geometric inner loop
        %------------------------------------------------------------------
        % PBVS measurement noise on 3D position/velocity
        if NOISE
            I_p_cm = I_p_c + sigma_pos * randn(3,1);
            I_v_cm = I_v_c + sigma_vel * randn(3,1);
        else
            I_p_cm = I_p_c;
            I_v_cm = I_v_c;
        end

        zpq         = abs(I_p_cm(3) - x_t(3,idx));
        P_NF_in     = polyval(K_ctrl.PNF_poly, zpq);
        I_vm_c_curr = x_c(8:10) + sigma_vel*randn(3,1);

        [u_2, xhat_AF, omega_AF, I_a_cd(:,idx)] = ...
            ctrl_Zhang2026(I_p_cm, I_v_cm, x_t(1:3,idx), dx_t(1:3,idx), ...
                           I_vm_c_prev, F_c_prev, ...
                           xhat_AF, P_NF_in, omega_AF, dt, ...
                           I_R_C, B_w_c, K_ctrl, m, J, g, ...
                           tau_xy_max, tau_z_max, T_max, T_min);
        % Update AEDO variables: I_vm_c_prev stores last measured velocity
        % for finite-difference acceleration estimate in next step
        I_vm_c_prev = I_vm_c_curr;
        F_c_prev    = m * (I_a_cd(:,idx) + g);

        %------------------------------------------------------------------
        case 4   % Lin 2023 — robust circle-feature IBVS + funnel + SO(3)
        %------------------------------------------------------------------
        % Circle-moment image features  (Eq. 7): s_t = [an*xg; an*yg; an]
        %   an = sqrt(a*/a) from the simulated image-point polygon areas;
        %   (xg,yg) = image centroid. Built from V_nP_i (same noisy IBVS
        %   measurement the other IBVS baselines consume), NOT ground-truth
        %   depth -> keeps the controller image-based / scale-free.
        % Centroid in NORMALISED image coords (px/f ~ O(0.1)); the area
        % ratio an is scale-invariant so it is taken on the raw pixel polys.
        % Lin's literal feature an = sqrt(a*/a) (an>1 when high, ->1 at
        % touchdown). The image-dynamics inversion (Eq. 8) is handled by the
        % +k1 virtual-velocity sign in ctrl_Lin2023 (no reciprocal needed).
        xg     = mean(V_nP_i(1,:)) / f;
        yg     = mean(V_nP_i(2,:)) / f;
        a_img  = polyarea(V_nP_i(1,:), V_nP_i(2,:));
        a_des  = polyarea(V_nP_d(1,:), V_nP_d(2,:));
        an     = sqrt(max(a_des,1e-9) / max(a_img,1e-9));
        s_t_lin   = [an*xg; an*yg; an];
        s_t_d_lin = [mean(V_nP_d(1,:))/f; mean(V_nP_d(2,:))/f; 1];   % an_d = 1

        % Adaptive funnel rho(0) = |e(0)| + margin  (mirror Lin 2022)
        if isempty(rho_t0_lin)
            e_t_init   = s_t_lin - s_t_d_lin;
            rho_t0_lin = abs(e_t_init) + K_ctrl.rho_t0_margin;
            xi_t_init  = max(min(e_t_init ./ rho_t0_lin, 0.999), -0.999);
            eps_t_init = 0.5 * log((1 + xi_t_init) ./ (1 - xi_t_init));
            q_t_init   = 1 ./ ((1 + xi_t_init) .* (1 - xi_t_init));
            vhat_V0    = K_ctrl.k1 .* (q_t_init .* eps_t_init);   % +k1, per-axis (match ctrl)
            vhat_I0    = I_R_V * vhat_V0;
            rho_v0_lin = abs(I_v_c - vhat_I0) + K_ctrl.rho_v0_margin;
        end
        rho_t = (rho_t0_lin - K_ctrl.rho_inf_t) .* ...
                 exp(-K_ctrl.l_t * tRange(idx)) + K_ctrl.rho_inf_t;
        rho_v = (rho_v0_lin - K_ctrl.rho_inf_v) .* ...
                 exp(-K_ctrl.l_v * tRange(idx)) + K_ctrl.rho_inf_v;

        % Own-velocity feedback (Lin 2023 uses it); velocity noise as for PBVS
        if NOISE
            I_v_cm = I_v_c + sigma_vel * randn(3,1);
        else
            I_v_cm = I_v_c;
        end

        [u_2, I_a_cd(:,idx), ~] = ...
            ctrl_Lin2023(s_t_lin, s_t_d_lin, I_v_cm, rho_t, rho_v, ...
                         K_ctrl.psi_des, I_R_C, I_R_V, B_w_c, K_ctrl, m, J, g, ...
                         tau_xy_max, tau_z_max, T_max, T_min);

        %------------------------------------------------------------------
        case 5   % Cho 2022 — FF-IBVS + geometric inner loop
        %------------------------------------------------------------------
        [u_2, I_a_cd(:,idx), ~, ~] = ...
            ctrl_Cho2022(V_nP_i, V_nP_d, C_s_tc, dx_t(1:3,idx), ...
                         f, K_ctrl.lambda_IBVS, K_ctrl.k_sigmoid, ...
                         K_ctrl.use_sq_comp, I_v_c, ...
                         I_R_C, I_R_V, B_w_c, K_ctrl, m, J, g, ...
                         tau_xy_max, tau_z_max, T_max, T_min, K_ctrl.psi_des);

    end   % switch CTRL_SEL

% *************************************************************************
% CASE 1 (PLASMC): Yaw ASMC heading generator + geometric SO(3) torque
% *************************************************************************
    if CTRL_SEL == 1
        % alpha has period pi -> wrap e_a to [-pi/2, pi/2]
        e_raw    = V_s(4) - V_s_d(4);
        e_a(idx) = atan2(sin(2*e_raw), cos(2*e_raw)) / 2;
        if idx == 1
            ie_a(idx) = dt * e_a(idx);
        else
            ie_a(idx) = ie_a(idx-1) + dt*(e_a(idx-1) + e_a(idx))/2;
        end
        sigma_a = e_a(idx) + K_ctrl.Omega_a * ie_a(idx);

        const_kappa_a  = [K_ctrl.n_a; K_ctrl.p_a];
        kappa_a(idx+1) = RK5(@(t,X) kappa_a_Solver(t, X, sigma_a, const_kappa_a), ...
                             t0, kappa_a(idx), dt);
        u_a = K_ctrl.Gamma_a*sigma_a + sat(sigma_a/K_ctrl.E_a)*kappa_a(idx+1) ...
              + K_ctrl.Omega_a*e_a(idx);
        if ~isfinite(u_a), break; end

        % Integrate ASMC rate into desired heading (replaces compass)
        psi_d = psi_d + u_a * dt;
        psi_d = atan2(sin(psi_d), cos(psi_d));

        % Construct R_d from filtered I_a_cd + psi_d
        I_F   = m * I_a_cd_filt;
        f_mag = norm(I_F);
        T_cd  = f_mag;
        if f_mag < 1e-6
            R_d = eye(3);
        else
            rd3 = -I_F / f_mag;
            a_h = [cos(psi_d); sin(psi_d); 0];
            rd2_raw = cross(rd3, a_h);
            n2 = norm(rd2_raw);
            if n2 < 1e-6, rd2_raw = [0;1;0]; n2 = 1; end
            rd2 = rd2_raw / n2;
            rd1 = cross(rd2, rd3);
            R_d = [rd1, rd2, rd3];
        end

        % Geometric SO(3) torque
        eR_mat  = 0.5 * (R_d' * I_R_C - I_R_C' * R_d);
        e_R     = [eR_mat(3,2); eR_mat(1,3); eR_mat(2,1)];
        e_Omega = B_w_c;
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

        B_T_cd(idx)         = T_cd;
        eR_log(:,idx)       = e_R;
        B_tau_cd_log(:,idx) = B_tau_cd;
        psi_d_log(idx)      = psi_d;
        u_a_log(idx)        = u_a;

        u_2 = [B_tau_cd; T_cd];
    end

% *************************************************************************
% GROUND EFFECT + COMPUTATIONAL DELAY  (controllers 2-5)
% *************************************************************************
    if CTRL_SEL > 1
        if GE
            z_ge = -max(abs(x_c(3)), r);
            u_2(4) = 1/(1-(r/(4*z_ge))^2) * u_2(4);
        end
    end

    % Shared delay buffer for all controllers
    u_2_buf(:,idx) = u_2;
    if idx > delay
        u_2 = u_2_buf(:, idx - delay);
    else
        u_2 = u_2_hover;
    end

% *************************************************************************
% SAFETY CHECK
% *************************************************************************
    if norm(I_a_cd(:,idx)) > 1e2 || any(isnan(I_a_cd(:,idx))), break; end
    if any(isnan(u_2)) || norm(u_2) > 1e4, break; end

% *************************************************************************
% UAV dynamics integration  (all controllers: u_2 = [tau; T])
% *************************************************************************
    if NOISE
        F_turb   = F_turb + (dt/wind_tau) * (-F_turb + wind_sigma*randn(3,1));
        v_rel    = wind_mean - x_c(8:10);
        F_wind_I = wind_mean*C_d_wind + F_turb + C_d_wind*v_rel;
        x_c = RK5(@(t,x) UAVDyn_robust(t, x, u_2, m_p, J_p, F_wind_I, r_cog), t0, x_c, dt);
    else
        x_c = RK5(@(t,x) UAVDyn(t,x,u_2), t0, x_c, dt);
    end
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
        % SO(3) PLASMC: back-compute E_crd from I_a_cd (yaw from psi_d)
        if abs(cos(psi_d)*I_a_cd(1,idx)+sin(psi_d)*I_a_cd(2,idx)) < 1e-4
            theta_log1 = 0;
        else
            theta_log1 = atan2(-cos(psi_d)*I_a_cd(1,idx) ...
                               -sin(psi_d)*I_a_cd(2,idx), -I_a_cd(3,idx));
        end
        if abs(sin(psi_d)*I_a_cd(1,idx)-cos(psi_d)*I_a_cd(2,idx)) < 1e-4
            phi_log1 = 0;
        else
            phi_log1 = atan2(-sin(psi_d)*I_a_cd(1,idx) ...
                             +cos(psi_d)*I_a_cd(2,idx), ...
                             -I_a_cd(3,idx)/cos(E_cr(2)));
        end
        E_crd_log1 = [phi_log1; theta_log1];
        D_DS(:,idx) = [V_h_d(:,idx); I_a_cd(:,idx); E_crd_log1; ...
                       e_R; B_tau_cd; zeros(3,1)];
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

    t0 = t0 + dt;

end   % main loop

% Safety/NaN breaks exit before logging completes at idx — last valid data
% is at idx-1. Landing break exits after logging — data at idx is complete.
if ~landed
    idx = idx - 1;
end

% Strict FoV-failure policy: physical-corner FoV violation ⇒ run is failed
% even if the landing termination condition would otherwise have fired.
if fov_fail
    landed  = false;
    precise = false;
    soft    = false;
end

%% =========================================================================
%  SAVE AND PLOT
% =========================================================================
% Patch PLASMC-specific fields so comp_result.mat is always loadable
% by run_comparison (which calls plotter_comparison, not plotter_adaptive).
if ~exist('p_1',    'var') || isempty(p_1),    p_1    = zeros(2, idx); end %#ok<NODEF>
if ~exist('p_2',    'var') || isempty(p_2),    p_2    = zeros(3, idx); end
if ~exist('S_1',    'var') || isempty(S_1),    S_1    = zeros(2,2,idx); end
if ~exist('zeta_1', 'var') || isempty(zeta_1), zeta_1 = zeros(2, idx); end

save(fullfile(fileparts(mfilename('fullpath')), '..', 'Datasets', 'Comparison', 'comp_result.mat'));
fprintf('\nDone: %s  (%d steps)\n', ctrl_names{CTRL_SEL}, idx);
