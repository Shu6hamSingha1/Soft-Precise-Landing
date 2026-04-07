%% ************************************************************************
% ctrl_Chen2025.m
%
% Full controller: IBVS Robust Observer + Attitude PD Inner Loop
% Chen et al., "Image-Based Visual Servoing of Micro Aerial Vehicles
% With Robust Observation and Output Feedback"
% IEEE Trans. Control Syst. Technol., Vol. 33, No. 6, 2025
% DOI: 10.1109/TCST.2025.3570243
%
% PIPELINE:
%   Outer loop (Eqs. 15-33):
%     Image moment error  e = q - qd                       (15)
%     Auxiliary variables:
%       zeta  = e - e_hat                                   (22)
%       eps   = de + k1*e                                   (23)
%       s     = vvs_hat - k1*e + de                         (24)
%       r     = -dzeta - k1*zeta + 3*de                     (25)
%     Controller: f = -kr*(r - s + eps) - zhat*M1*e1
%                     + (zhat/k1)*M2*e1                     (33)
%     Observer update: e_hat, vvs_hat, v0, zstar_hat     (26, 32)
%   Inner loop:
%     f_out -> I_F = I_R_V * f_out - m*g
%     I_F  -> T, Rd -> eR, eOmega -> tau
%
% INPUTS:
%   V_s, V_s_d   - image moments q, qd                    [3x1]
%   e_hat, vvs_hat, v0 - observer states
%   zstar_hat    - depth estimate (1/z*)
%   de           - edot (finite difference of e)            [3x1]
%   dzeta        - zetadot (finite difference of zeta)      [3x1]
%   psi_dot      - yaw rate                                [rad/s]
%   dt           - time step                               [s]
%   R_c          - rotation matrix body->inertial          [3x3]
%   I_R_V        - rotation virtual->inertial              [3x3]
%   B_w_c        - body angular velocity                   [rad/s]
%   K            - gains struct
%   m, J         - mass, inertia
%   g_vec, e3    - gravity, vertical unit
%   tau_xy_max, tau_z_max, T_max, T_min
%   psi_des      - desired yaw                             [rad]
%
% OUTPUTS:
%   u_2          - [4x1] = [tau_x; tau_y; tau_z; T]
%   e_hat_new, vvs_hat_new, v0_new, zstar_new
%   I_a_cd       - [3x1] desired acceleration (for logging)
% *************************************************************************
function [u_2, e_hat_new, vvs_hat_new, v0_new, zstar_new, I_a_cd] = ...
          ctrl_Chen2025(V_s, V_s_d, ...
                        e_hat, vvs_hat, v0, zstar_hat, ...
                        de, dzeta, psi_dot, dt, ...
                        R_c, I_R_V, B_w_c, K, m, J, g_vec, e3, ...
                        tau_xy_max, tau_z_max, T_max, T_min, psi_des)

    %% ---------------------------------------------------------------
    % OUTER LOOP: IBVS with robust observer  (Eqs. 15-33)
    % ---------------------------------------------------------------
    e     = V_s - V_s_d;              % Eq. 15
    e3_   = [0;0;1];
    Spsi  = skew_sym(psi_dot * e3_);
    M1    = 2 * Spsi;                 % Eq. 18
    M2    = Spsi * Spsi;              % Eq. 18
    k1    = K.k1_obs;
    k2    = K.k2_obs;

    e1    = k1 * e;                   % e1 = k1*e
    zeta  = e - e_hat;                % Eq. 22

    % Auxiliary variables  (Eqs. 23-25)
    % NOTE: s and r use edot (de), NOT epsilon
    vvs_cur = v0 - k2 * zeta;         % Eq. 24 (algebraic): recompute with current v0, ζ
    epsilon = de + e1;                 % Eq. 23: eps = edot + k1*e
    s       = vvs_cur - e1 + de;       % Eq. 24: s = vvs - e1 + edot
    r       = -dzeta - k1*zeta + 3*de; % Eq. 25: r = -zetadot - zeta1 + 3*edot

    % IBVS force (virtual frame)  (Eq. 33)
    f_out = -K.kr * (r - s + epsilon) ...
            - zstar_hat * (M1 * e1) ...
            + (zstar_hat / k1) * (M2 * e1);

    % Observer update  (Eq. 26)
    e_hat_dot   = -Spsi*e - vvs_cur - epsilon;
    e_hat_new   = e_hat + dt * e_hat_dot;
    vvs_hat_new = vvs_cur;            % algebraic output for next step
    v0_dot      = -Spsi*(vvs_cur + k1*zeta + epsilon) - k1*k2*zeta;
    v0_new      = v0 + dt * v0_dot;

    % Depth update  (Eq. 32)
    Mk        = M1*e1 - (M2/k1)*e1;
    dzstar    = K.k3_obs * (epsilon' + s' + r') * Mk;
    zstar_new = zstar_hat + dt * dzstar;

    % Inertial force: ξ̈ = -Rψ f (Eq. 2,17), so I_F = mξ̈ - mg = -m Rψ f - mg
    I_F    = -m * I_R_V * f_out - m * g_vec;
    I_a_cd = I_F / m;

    %% ---------------------------------------------------------------
    % INNER LOOP: attitude PD + feedforward
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

function S = skew_sym(v)
    S = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0  ];
end

function v = vee_map(S)
    v = [S(3,2); S(1,3); S(2,1)];
end
