%% ************************************************************************
% ctrl_Cho2022.m
%
% Full controller: Feed-Forward IBVS + Attitude PD Inner Loop
% Cho et al., "Autonomous Ship Deck Landing of a Quadrotor UAV Using
% Feed-Forward Image-Based Visual Servoing"
% Aerosp. Sci. Technol., Vol. 130, 2022
% DOI: 10.1016/j.ast.2022.107869
%
% PIPELINE:
%   Outer loop:
%     Image feature error + Jacobian → IBVS velocity command + FF
%     Velocity command → desired force vector
%   Inner loop:
%     Force vector → Rd, T
%     eR, eΩ → τ (geometric PD)
%
% COORDINATE CONVENTION:
%   Inertial frame: NED  (x=North, y=East, z=Down)
%   Virtual frame:  NED, yaw-decoupled
%   g_vec = [0;0;9.81]
%
% INPUTS:
%   V_nP_i, V_nP_d - image feature points (current, desired) [2xN]
%   C_s_tc         - target pos in camera frame               [3x1]
%   I_v_t          - target velocity in NED                   [m/s]
%   E_cr           - Euler angles [roll; pitch; yaw]
%   f_cam          - focal length                             [pixels]
%   lambda_IBVS    - IBVS gains
%   k_sigmoid, use_sq_comp - adaptive gain params
%   I_v_c          - UAV velocity in NED                      [m/s]
%   R_c            - body→inertial rotation                   [3x3]
%   I_R_V          - virtual→inertial rotation                [3x3]
%   B_w_c          - body angular velocity                    [rad/s]
%   K              - gains struct (Kv, kR, kOmega, v_sat)
%   m, J           - mass, inertia
%   g_vec, e3      - gravity, vertical unit
%   tau_xy_max, tau_z_max, T_max, T_min
%   psi_des        - desired yaw                              [rad]
%
% OUTPUTS:
%   u_2     - [4x1] = [tau_x; tau_y; tau_z; T]
%   I_a_cd  - [3x1] desired acceleration (for logging)
%   V_vd_ff - [6x1] desired camera velocity
%   e_feat  - [2Nx1] feature error
% *************************************************************************
function [u_2, I_a_cd, V_vd_ff, e_feat] = ...
          ctrl_Cho2022(V_nP_i, V_nP_d, C_s_tc, I_v_t, E_cr, ...
                       f_cam, lambda_IBVS, k_sigmoid, use_sq_comp, ...
                       I_v_c, R_c, I_R_V, B_w_c, K, m, J, g_vec, e3, ...
                       tau_xy_max, tau_z_max, T_max, T_min, psi_des)

    N = size(V_nP_i, 2);

    %% ---------------------------------------------------------------
    % OUTER LOOP: FF-IBVS velocity command
    % ---------------------------------------------------------------
    if use_sq_comp && N == 4
        V_nP_i = square_compensate(V_nP_i);
    end

    e_feat = reshape(V_nP_d - V_nP_i, [], 1);

    z_depth = C_s_tc(3);
    if abs(z_depth) < 0.01, z_depth = 0.01; end

    Ls = zeros(2*N, 6);
    for i = 1:N
        xp = V_nP_i(1,i);  yp = V_nP_i(2,i);
        Ls(2*i-1,:) = [-f_cam/z_depth, 0, xp/z_depth, ...
                        xp*yp/f_cam, -(f_cam^2+xp^2)/f_cam, yp];
        Ls(2*i,  :) = [0, -f_cam/z_depth, yp/z_depth, ...
                        (f_cam^2+yp^2)/f_cam, -xp*yp/f_cam, -xp];
    end

    Ls_pinv = pinv(Ls);
    if isscalar(lambda_IBVS)
        lam = lambda_IBVS * eye(6);
    else
        lam = diag(lambda_IBVS);
    end
    vd_ibvs = -lam * Ls_pinv * e_feat;

    % Adaptive altitude gain
    c    = norm(mean(V_nP_i, 2));
    ad_z = 1 - 1/(1 + exp(-k_sigmoid * c));

    vd_ibvs(3) = ad_z * vd_ibvs(3);
    vd_ibvs(1:3) = max(min(vd_ibvs(1:3),  K.v_sat(1:3)), -K.v_sat(1:3));
    vd_ibvs(6)   = max(min(vd_ibvs(6),    K.v_sat(4)),   -K.v_sat(4));

    V_v_t   = I_R_V' * I_v_t;                     % inertial → virtual frame
    V_vd_ff = vd_ibvs + [V_v_t; 0; 0; 0];

    % Desired velocity in NED
    I_v_des = I_R_V * V_vd_ff(1:3);

    % Desired force = proportional velocity tracking + gravity compensation
    I_F    = m * K.Kv * (I_v_des - I_v_c) - m * g_vec;
    I_a_cd = I_F / m;   % for logging

    %% ---------------------------------------------------------------
    % INNER LOOP: geometric attitude controller
    % ---------------------------------------------------------------
    f_mag = norm(I_F);
    T     = f_mag;

    if f_mag < 1e-6
        Rd = eye(3);
    else
        rd3 = -I_F / f_mag;
        a   = [cos(psi_des); sin(psi_des); 0];
        rd2_raw = cross(rd3, a);
        n2 = norm(rd2_raw);
        if n2 < 1e-6, rd2_raw = [0;1;0]; n2 = 1; end
        rd2 = rd2_raw / n2;
        rd1 = cross(rd2, rd3);
        Rd  = [rd1, rd2, rd3];
    end

    eR_mat = 0.5 * (Rd'*R_c - R_c'*Rd);
    eR     = vee_map(eR_mat);
    eOmega = B_w_c;

    tau_cd = -K.kR*eR - K.kOmega*eOmega + cross(B_w_c, J*B_w_c);
    tau_cd(1:2) = min(max(tau_cd(1:2), -tau_xy_max), tau_xy_max);
    tau_cd(3)   = min(max(tau_cd(3),   -tau_z_max),  tau_z_max);
    T           = max(min(T, T_max), T_min);

    u_2 = [tau_cd; T];

end

function nP_sq = square_compensate(nP)
    cx = mean(nP(1,:));  cy = mean(nP(2,:));
    d14 = norm(nP(:,1)-nP(:,4));  d12 = norm(nP(:,1)-nP(:,2));
    half = (d14+d12)/4;
    nP_sq = [cx-half, cx+half, cx+half, cx-half;
             cy+half, cy+half, cy-half, cy-half];
end

function v = vee_map(S)
    v = [S(3,2); S(1,3); S(2,1)];
end
