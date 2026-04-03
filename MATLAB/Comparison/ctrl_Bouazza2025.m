%% ************************************************************************
% ctrl_Bouazza2025.m
%
% State Estimator + Controller: Cascade Observer (Bouazza et al., 2025)
% "Vision-Aided Relative State Estimation for Approach and Landing on a
%  Moving Platform with Inertial Measurements"
% arXiv:2512.19245v1, December 2025
%
% COORDINATE CONVENTIONS:
%   Inertial frame : NED  (I_)   x=North, y=East, z=Down
%   Body frame     : FRD  (B_)
%   g = [0;0;9.81] m/s^2 (gravity in +z = downward)
%   e3 = [0;0;1]  (points downward in NED — towards ground)
%
% BOUAZZA 2025 FRAME NOTES:
%   The paper uses e3 = (0,0,1)^T as the vertical direction.  In NED this
%   also points downward (toward ground), so e3 is consistent.
%   The normal vector η = R^T e3 ∈ S^2 in Eq.(11) points from the target
%   plane toward the UAV when the target is below (z < 0 in NED means
%   above ground, so the landing pad has z > 0, UAV has smaller z).
%
% OUTPUT CONVENTION:
%   I_a_cd = Kp*(I_xi_des - I_xi_hat) + Kd*(0 - I_v_hat) - g_vec
%   At hover: errors = 0 → I_a_cd = -g_vec = [0;0;-9.81] ✓
%
% INPUTS:
%   R_hat    - [3x3] Relative attitude estimate (SO(3))
%   xi_hat   - [3x1] Relative position estimate in body frame B    [m]
%   v_hat    - [3x1] Relative velocity estimate in body frame B    [m/s]
%   P_ric    - [6x6] Riccati covariance (positive definite)
%   omega_B  - [3x1] UAV angular velocity in B                     [rad/s]
%   omega_T  - [3x1] Target angular velocity in T                  [rad/s]
%   a_B      - [3x1] UAV specific acceleration in B                [m/s^2]
%   a_T      - [3x1] Target specific acceleration in T             [m/s^2]
%   eta_meas - [3x1] Target-plane normal measured in B             [unit]
%   yxi_meas - [3x1] Bearing unit vector to target in B            [unit]
%   dt       - [1x1] Time step                                     [s]
%   K        - struct  Fields: kR, S_ric, D_ric, Kp, Kd
%   I_R_B    - [3x3] Rotation: body → inertial (NED)
%   xi_des   - [3x1] Desired relative position in B frame          [m]
%   m        - [1x1] UAV mass                                      [kg]
%   g_vec    - [3x1] Gravity vector [0;0;9.81] in NED              [m/s^2]
%
% OUTPUTS:
%   I_a_cd     - [3x1] Net commanded acceleration in NED           [m/s^2]
%   R_hat_new  - [3x3] Updated relative attitude estimate
%   xi_hat_new - [3x1] Updated relative position estimate
%   v_hat_new  - [3x1] Updated relative velocity estimate
%   P_new      - [6x6] Updated Riccati covariance
%
% Reference: Bouazza et al., arXiv:2512.19245, 2025.
% *************************************************************************
function [I_a_cd, R_hat_new, xi_hat_new, v_hat_new, P_new] = ...
          ctrl_Bouazza2025(R_hat, xi_hat, v_hat, P_ric, ...
                           omega_B, omega_T, a_B, a_T, ...
                           eta_meas, yxi_meas, dt, K, ...
                           I_R_B, xi_des, m, g_vec)

    % e3 = [0;0;1] points downward in NED — consistent with Bouazza 2025
    e3 = [0; 0; 1];

    %% ---------------------------------------------------------------
    % PART 1: RELATIVE ATTITUDE OBSERVER  (Sec. 4.1, Eqs. 12, 14)
    % ---------------------------------------------------------------
    % Innovation: sigma_R = 2*kR*(R_hat*eta) x e3
    sigma_R = 2 * K.kR * cross(R_hat * eta_meas, e3);

    % Attitude dynamics  (Eq. 12)
    R_hat_dot = -skew_3x3(omega_T) * R_hat ...
              +  R_hat * skew_3x3(omega_B) ...
              +  skew_3x3(sigma_R) * R_hat;

    % Euler integration + re-orthonormalise onto SO(3)
    R_hat_raw = R_hat + dt * R_hat_dot;
    [U, ~, Vs] = svd(R_hat_raw);
    R_hat_new  = U * Vs';

    %% ---------------------------------------------------------------
    % PART 2: RICCATI OBSERVER  (Sec. 4.2, Eqs. 16-22)
    % ---------------------------------------------------------------
    A_ric = [-skew_3x3(omega_B), eye(3);
              zeros(3),         -skew_3x3(omega_B)];

    pi_yxi = eye(3) - yxi_meas * yxi_meas';
    C_ric  = [pi_yxi, zeros(3)];

    y_meas = -pi_yxi * xi_hat;

    % Riccati equation  (Eq. 22)
    P_dot = A_ric * P_ric + P_ric * A_ric' ...
          - P_ric * C_ric' * K.D_ric * C_ric * P_ric + K.S_ric;
    P_new = P_ric + dt * P_dot;

    % Keep P positive-definite
    P_new = (P_new + P_new') / 2;
    [V_eig, D_eig] = eig(P_new);
    D_eig = diag(max(diag(D_eig), 1e-6));
    P_new = V_eig * D_eig * V_eig';

    % Gain and innovations
    K_ric   = P_new * C_ric' * K.D_ric;
    sigma_xi = K_ric(1:3,:) * y_meas;
    sigma_v  = K_ric(4:6,:) * y_meas;

    % Observer propagation  (Eq. 16)
    xi_hat_dot = -skew_3x3(omega_B) * xi_hat + v_hat + sigma_xi;
    v_hat_dot  = -skew_3x3(omega_B) * v_hat  + a_B - R_hat_new' * a_T + sigma_v;

    xi_hat_new = xi_hat + dt * xi_hat_dot;
    v_hat_new  = v_hat  + dt * v_hat_dot;

    %% ---------------------------------------------------------------
    % PART 3: PD POSITION CONTROLLER
    % ---------------------------------------------------------------
    % Relative position/velocity in NED inertial frame
    I_xi_hat = I_R_B * xi_hat_new;
    I_v_hat  = I_R_B * v_hat_new;
    I_xi_des = I_R_B * xi_des;

    % PD law with NED gravity compensation
    %   I_a_cd = Kp*pos_err + Kd*vel_err - g_vec
    %   At hover: I_a_cd = -g_vec = [0;0;-9.81] ✓
    I_a_cd = K.Kp * (I_xi_des - I_xi_hat) ...
           + K.Kd * (zeros(3,1) - I_v_hat) ...
           - g_vec;

end

function S = skew_3x3(v)
    S = [  0    -v(3)  v(2);
          v(3)   0    -v(1);
         -v(2)  v(1)   0  ];
end
