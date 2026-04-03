%% ************************************************************************
% ctrl_Lin2022.m
%
% Full controller: Low-Complexity PBVS + Geometric Inner Loop
% Lin et al., "Low-Complexity Control for Vision-Based Landing of
% Quadrotor UAV on Unknown Moving Platform"
% IEEE Trans. Ind. Informat., Vol. 18, No. 8, Aug. 2022
% DOI: 10.1109/TII.2021.3129486
%
% PIPELINE (matches paper exactly):
%   Outer loop  (Eqs. 9-18):
%     e_p → ξ_p → ε_p → v̂  (prescribed-performance position)
%     e_v → ξ_v → ε_v → F   (prescribed-performance velocity → force vector)
%   Inner loop  (Eqs. 43-52):
%     F  → Rd, fd  (desired attitude + thrust)
%     eR = 0.5*(Rd'R - R'Rd)^∨  (attitude error on SO(3))
%     eΩ = Ω - Rd'*Ω_d          (angular velocity error)
%     τ  = -kR*eR - kΩ*eΩ + Ω×JΩ
%     T  = -F · R*e3
%
% COORDINATE CONVENTION:
%   Inertial frame: NED  (x=North, y=East, z=Down)
%   Body frame:     FRD
%   g_vec = [0;0;9.81],  e3 = [0;0;1] (points down, = body thrust axis)
%   NED gravity adaptation:
%     Paper (ENU): F = -k2*Qv*εv + m*g*i3  (i3 up)
%     Here (NED):  F = -k2*Qv*εv - m*g_vec (g_vec down) → same direction
%
% INPUTS:
%   I_p_c, I_v_c  - UAV position, velocity in NED          [m], [m/s]
%   I_p_t         - Target position in NED                 [m]
%   rho_p, rho_p_dot, rho_v, rho_v_dot - perf. functions
%   r_pt_des      - Desired relative position (NED)        [m]
%   psi_des       - Desired yaw                            [rad]
%   R_c           - Current rotation matrix body→inertial  [3x3]
%   B_w_c         - Body angular velocity                  [rad/s]
%   K             - gains struct (k1,k2,kR,kOmega)
%   m, J          - mass [kg], inertia [3x3]
%   g_vec         - [0;0;9.81]
%   e3            - [0;0;1]
%   tau_xy_max, tau_z_max, T_max, T_min - saturation limits
%
% OUTPUTS:
%   u_2           - [4x1] = [tau_x; tau_y; tau_z; T]  (input to UAVDyn)
%   I_a_cd        - [3x1] desired acceleration (for logging)
%   Rd            - [3x3] desired rotation matrix
% *************************************************************************
function [u_2, I_a_cd, Rd] = ...
          ctrl_Lin2022(I_p_c, I_v_c, I_p_t, ...
                       rho_p, rho_p_dot, rho_v, rho_v_dot, ...
                       r_pt_des, psi_des, ...
                       R_c, B_w_c, K, m, J, g_vec, e3, ...
                       tau_xy_max, tau_z_max, T_max, T_min)

    %% ---------------------------------------------------------------
    % OUTER LOOP: prescribed-performance position & velocity control
    % ---------------------------------------------------------------

    % Position error  (Eq. 13)
    r_pt = I_p_c - I_p_t;
    e_p  = r_pt - r_pt_des;

    % Transformed position error  (Eq. 12)
    xi_p  = clamp_vec(e_p ./ rho_p, -0.999, 0.999);
    eps_p = 0.5 * log((1 + xi_p) ./ (1 - xi_p));
    q_p   = 1 ./ ((1 + xi_p) .* (1 - xi_p));
    Q_p   = diag(q_p);

    % Virtual velocity  (Eq. 15)
    vhat = -K.k1 * (Q_p' * eps_p);

    % Velocity error and transform  (Eq. 16)
    e_v   = I_v_c - vhat;
    xi_v  = clamp_vec(e_v ./ rho_v, -0.999, 0.999);
    eps_v = 0.5 * log((1 + xi_v) ./ (1 - xi_v));
    q_v   = 1 ./ ((1 + xi_v) .* (1 - xi_v));
    Q_v   = diag(q_v);

    % Thrust force vector  (Eq. 18, NED: replace +mg*i3_ENU with -m*g_vec)
    F_vec = -K.k2 * (Q_v' * eps_v) - m * g_vec;   % [3x1]
    f_mag = norm(F_vec);

    % Desired acceleration for logging
    I_a_cd = F_vec / m;

    %% ---------------------------------------------------------------
    % INNER LOOP: geometric SO(3) attitude controller  (Eqs. 43-52)
    % ---------------------------------------------------------------

    % --- Desired thrust scalar  (Eq. 43) ---
    % T = ||F_vec||  (magnitude, always positive)
    T = f_mag;

    % --- Desired rotation matrix Rd  (Eq. 44) ---
    % rd3 = F / ||F||  (desired body-z = thrust direction)
    % In NED/FRD: body-z points down, thrust pushes up (F points up, i.e. -z)
    % rd3 = -F_vec / ||F_vec||  (body-z toward ground, thrust force toward sky)
    if f_mag < 1e-6
        Rd = eye(3);
    else
        rd3 = -F_vec / f_mag;           % body-z axis (FRD: down = -thrust dir)
        a   = [cos(psi_des); sin(psi_des); 0];
        rd2_raw = cross(rd3, a);
        n2 = norm(rd2_raw);
        if n2 < 1e-6
            rd2_raw = [0; 1; 0];  n2 = 1;
        end
        rd2 = rd2_raw / n2;
        rd1 = cross(rd2, rd3);
        Rd  = [rd1, rd2, rd3];
    end

    % Assume zero desired angular velocity  (Ω_d = 0, Eq. 47)
    Omega_d = zeros(3,1);

    % --- Attitude error on SO(3)  (Eq. 48) ---
    % eR = 0.5 * vee(Rd'*R - R'*Rd)
    eR_mat = 0.5 * (Rd' * R_c - R_c' * Rd);
    eR     = vee_map(eR_mat);

    % --- Angular velocity error  (Eq. 49) ---
    eOmega = B_w_c - R_c' * Rd * Omega_d;

    % --- Control torque  (Eq. 50) ---
    tau_cd = -K.kR * eR - K.kOmega * eOmega + cross(B_w_c, J * B_w_c);

    % --- Saturation ---
    tau_cd(1:2) = min(max(tau_cd(1:2), -tau_xy_max),  tau_xy_max);
    tau_cd(3)   = min(max(tau_cd(3),   -tau_z_max),    tau_z_max);
    T           = max(min(T, T_max), T_min);

    u_2 = [tau_cd; T];

end

% vee map: skew-symmetric → vector
function v = vee_map(S)
    v = [S(3,2); S(1,3); S(2,1)];
end

function y = clamp_vec(x, lo, hi)
    y = max(lo, min(hi, x));
end
