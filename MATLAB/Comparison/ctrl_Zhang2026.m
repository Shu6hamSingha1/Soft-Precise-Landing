%% ************************************************************************
% ctrl_Zhang2026.m
%
% Full controller: AEDO Backstepping + Geometric Inner Loop
% Zhang & Wu, "Adaptive Extended Disturbance Observer-Based Control for
% Quadrotor Landing on Fast Platform"
% IEEE Trans. Ind. Electron., Vol. 73, No. 4, Apr. 2026
% DOI: 10.1109/TIE.2025.3629369
%
% PIPELINE (matches paper):
%   Outer loop  (Eqs. 39-41):
%     AEDO disturbance estimator + backstepping → Fc (force vector)
%   Inner loop  (Eqs. 42-47):
%     Fc → Rd (desired attitude), T (thrust)
%     eR, eΩ → τ (geometric SO(3) torque)
%
% COORDINATE CONVENTION:
%   Inertial frame: NED  (x=North, y=East, z=Down)
%   Body frame:     FRD
%   g_vec = [0;0;9.81]
%
% INPUTS:
%   I_p_c, I_v_c  - UAV position, velocity in NED          [m], [m/s]
%   I_p_t, I_v_t  - Target position, velocity in NED
%   I_vm_c        - Measured (noisy) UAV velocity           [m/s]
%   F_c_prev      - Previous control force                  [N]
%   xhat_AF       - AEDO state [6x1]
%   P_NF_in       - Noise power (scalar)
%   omega_AF      - Current AEDO bandwidth                  [rad/s]
%   dt            - Time step                               [s]
%   R_c           - Current rotation matrix body→inertial   [3x3]
%   B_w_c         - Body angular velocity                   [rad/s]
%   K             - gains struct
%   m, J          - mass, inertia
%   g_vec, e3     - gravity vector, vertical unit vector
%   tau_xy_max, tau_z_max, T_max, T_min - saturation limits
%
% OUTPUTS:
%   u_2           - [4x1] = [tau_x; tau_y; tau_z; T]
%   xhat_AF_new   - updated AEDO state
%   omega_AF_new  - updated bandwidth
%   I_a_cd        - [3x1] desired acceleration (for logging)
% *************************************************************************
function [u_2, xhat_AF_new, omega_AF_new, I_a_cd] = ...
          ctrl_Zhang2026(I_p_c, I_v_c, I_p_t, I_v_t, ...
                         I_vm_c, F_c_prev, ...
                         xhat_AF, P_NF_in, omega_AF, dt, ...
                         R_c, B_w_c, K, m, J, g_vec, e3, ...
                         tau_xy_max, tau_z_max, T_max, T_min)

    I3 = eye(3);  O3 = zeros(3);
    l1 = K.lAF1;  l2 = K.lAF2;  w = omega_AF;

    %% ---------------------------------------------------------------
    % AEDO: Adaptive Extended Disturbance Observer  (Eqs. 11-16)
    % ---------------------------------------------------------------
    L_AF     = [l1*w*I3; l2*w^2*I3];
    Fdm      = m * ((I_v_c - I_vm_c) / dt) + F_c_prev;
    AF       = [O3, I3; O3, O3];
    CF       = [I3, O3];
    innov    = Fdm - CF * xhat_AF;
    xhat_AF_new = xhat_AF + dt * (AF * xhat_AF + L_AF * innov);
    Fd_hat   = xhat_AF_new(1:3);

    % Adaptive bandwidth
    PF  = norm(innov)^2;
    PNF = max(P_NF_in, 1e-6);
    KF  = max(1, w^2);
    omega_AF_new = max((KF * PF/PNF)^(1/4), K.omega_AFm);

    %% ---------------------------------------------------------------
    % OUTER LOOP: backstepping position controller  (Eq. 39)
    % ---------------------------------------------------------------
    re     = I_p_c - I_p_t;
    re_dot = I_v_c - I_v_t;

    % Net force vector (NED, gravity compensated)
    Fc = -(K.Kc1*K.Kc3 + K.Kc2)*re ...
         -(m*K.Kc1 + K.Kc3)*re_dot ...
         - m*g_vec ...
         - Fd_hat;

    I_a_cd = Fc / m;   % for logging

    %% ---------------------------------------------------------------
    % INNER LOOP: geometric attitude controller  (Eqs. 42-47)
    % ---------------------------------------------------------------
    f_mag = norm(Fc);
    T     = f_mag;

    % Desired rotation: body-z aligned with -Fc (thrust opposes gravity)
    if f_mag < 1e-6
        Rd = eye(3);
    else
        rd3 = -Fc / f_mag;
        a   = [1; 0; 0];   % heading reference (North)
        rd2_raw = cross(rd3, a);
        n2 = norm(rd2_raw);
        if n2 < 1e-6, rd2_raw = [0;1;0]; n2 = 1; end
        rd2 = rd2_raw / n2;
        rd1 = cross(rd2, rd3);
        Rd  = [rd1, rd2, rd3];
    end

    % Attitude and angular velocity errors  (Eqs. 44-45)
    eR_mat = 0.5 * (Rd'*R_c - R_c'*Rd);
    eR     = vee_map(eR_mat);
    eOmega = B_w_c;   % Ω_d = 0

    % Torque  (Eq. 46)
    tau_cd = -K.kR*eR - K.kOmega*eOmega + cross(B_w_c, J*B_w_c);

    % Saturation
    tau_cd(1:2) = min(max(tau_cd(1:2), -tau_xy_max), tau_xy_max);
    tau_cd(3)   = min(max(tau_cd(3),   -tau_z_max),  tau_z_max);
    T           = max(min(T, T_max), T_min);

    u_2 = [tau_cd; T];

end

function v = vee_map(S)
    v = [S(3,2); S(1,3); S(2,1)];
end
