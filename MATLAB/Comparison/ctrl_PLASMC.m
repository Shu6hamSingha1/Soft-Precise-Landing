%% ************************************************************************
% ctrl_PLASMC.m
%
% Outer-Loop Controller: PLASMC (Proposed - Singhal et al.)
% "Performance-Constrained Adaptive Sliding Mode Control for Guaranteed
%  Soft Precise Landing Using Optic Flow"
%
% This function computes the outer-loop desired acceleration I_a_cd using
% the Performance-constrained Leakage-type Adaptive Sliding Mode Control
% (PLASMC) strategy described in the manuscript.
%
% PIPELINE (matches visualControl_IBVS_adaptive.m exactly):
%   s --> zeta_1 --> h_d --> h_e --> zeta_2 --> sigma --> u_sw + u_eq
%   --> V_a_cd --> I_a_cd
%
% INPUTS:
%   V_s        - [3x1] Current image feature vector [xhat; yhat; 1]
%   V_h        - [3x1] Current optical flow vector in virtual frame
%   V_w        - [3x1] Current angular rate of virtual plane
%   V_dw       - [3x1] Derivative of V_w
%   V_s_d      - [4x1] Desired image features [xhat_d; yhat_d; 1; alpha_d]
%   h_rd       - [1x1] Desired radial optical flow (constant)
%   p_1        - [2x1] Performance function for visibility constraint
%   dp_1       - [2x1] Time derivative of p_1
%   p_2        - [3x1] Performance function for optical flow
%   dp_2       - [3x1] Time derivative of p_2
%   zeta_1_prev - [2x1] Previous zeta_1 (for PID integral/derivative)
%   izeta_1    - [2x1] Integral of zeta_1
%   dzeta_1    - [2x1] Smoothed derivative of zeta_1
%   zeta_2_prev - [3x1] Previous zeta_2
%   izeta_2    - [3x1] Integral of zeta_2
%   kappa      - [3x1] Current adaptive gain vector
%   V_dh_d     - [3x1] Smoothed time derivative of h_d
%   I_R_V      - [3x3] Rotation from virtual frame to inertial frame
%   K          - struct  All controller gains
%   e3         - [3x1] Unit vector [0;0;1]
%   g          - [3x1] Gravity vector [0;0;9.81]
%
% OUTPUTS:
%   I_a_cd     - [3x1] Desired acceleration in inertial frame
%   V_h_d      - [3x1] Desired optical flow
%   V_h_e      - [3x1] Optical flow error
%   zeta_1     - [2x1] Updated transformed position error
%   G_1        - [2x2] Performance Jacobian for visibility loop
%   S_1        - [2x2] Normalised position error (diagonal)
%   zeta_2     - [3x1] Updated transformed optical flow error
%   G_2        - [3x3] Performance Jacobian for optical flow loop
%   S_2        - [3x3] Normalised optical flow error (diagonal)
%   sigma      - [3x1] Sliding surface
%   Theta      - [3x6] Uncertainty coefficient matrix
%   V_a_cd     - [3x1] Desired acceleration in virtual frame
%   flag       - bool  Constraint violation flag
%
% Reference: Singhal, Sundaram, Keshavan, IEEE Trans. Aerosp. Electron. Syst.
% *************************************************************************
function [I_a_cd, V_h_d, V_h_e, zeta_1, G_1, S_1, ...
          zeta_2, G_2, S_2, sigma, Theta, V_a_cd, flag] = ...
          ctrl_PLASMC(V_s, V_h, V_w, V_dw, V_s_d, h_rd, ...
                      p_1, dp_1, p_2, dp_2, ...
                      zeta_1_prev, izeta_1, dzeta_1, ...
                      zeta_2_prev, izeta_2, ...
                      kappa, V_dh_d, ...
                      I_R_V, K, e3, g)

    flag = false;

    % Performance functions
    p_1(:,idx)  = expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * ...
                  (K_ctrl.p_10 - K_ctrl.p_1inf) + K_ctrl.p_1inf;
    dp_1(:,idx) = -diag(K_ctrl.gamma_1) * ...
                   expm(-diag(K_ctrl.gamma_1)*tRange(idx)) * ...
                   (K_ctrl.p_10 - K_ctrl.p_1inf);

    p_2(:,idx)  = expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * ...
                  (K_ctrl.p_20 - K_ctrl.p_2inf) + K_ctrl.p_2inf;
    dp_2(:,idx) = -diag(K_ctrl.gamma_2) * ...
                   expm(-diag(K_ctrl.gamma_2)*tRange(idx)) * ...
                   (K_ctrl.p_20 - K_ctrl.p_2inf);

    %% ------------------------------------------------------------------
    % STAGE 1: Visibility constraint loop  (Sec. III-A)
    %   Error: r_e = V_rhat - r_hat_d  -->  zeta_1
    % ------------------------------------------------------------------
    V_s_e = V_s(1:2) - V_s_d(1:2);   % [2x1] normalised position error

    % Initialise diagonal matrices
    S_1 = diag(zeros(2,1));
    G_1 = diag(zeros(2,1));
    zeta_1 = zeta_1_prev;

    for j = 1:2
        % Normalised error (Eq. 25 / S_1 = r_e / p_1)
        S_1(j,j) = V_s_e(j) / p_1(j);

        % Clamp: if |S_1| >= 1, saturate to previous value (Sec. III-A)
        if abs(S_1(j,j)) >= 1
            S_1(j,j) = abs(zeta_1_prev(j)) * sign(V_s_e(j));   % keep magnitude
            flag = true;
            break;
        end

        % Unconstrained error  (Eq. 27)
        zeta_1(j) = log((1 + S_1(j,j)) / (1 - S_1(j,j)));

        % Performance Jacobian  (Eq. 28, g_1k)
        G_1(j,j) = (exp(zeta_1(j)) + 1)^2 / (2 * exp(zeta_1(j)) * p_1(j));
    end

    if flag
        % Return safe defaults on constraint violation
        I_a_cd = zeros(3,1);
        V_h_d  = zeros(3,1);
        V_h_e  = zeros(3,1);
        zeta_2 = zeta_2_prev;
        G_2    = eye(3);
        S_2    = diag(zeros(3,1));
        sigma  = zeros(3,1);
        Theta  = zeros(3,6);
        V_a_cd = zeros(3,1);
        return;
    end

    %% ------------------------------------------------------------------
    % STAGE 2: Desired optical flow h_d  (Eq. 32)
    %   PID law for zeta_1 drives s_e --> 0
    % ------------------------------------------------------------------
    % PID on unconstrained position error
    dzeta_1d = - K.zp * zeta_1 - K.zi * izeta_1 - K.zd * dzeta_1;

    % s_dot_e = G1_inv * dzeta_1d + S1 * p1_dot   (Eq. 29)
    V_ds_d = [G_1 \ dzeta_1d + S_1 * dp_1; 0.0];

    % h_d = s_dot_e + cross(V_w, s) + (h_rd - dot(cross(V_w,s), e3)) * s
    %   (Eq. 32 / 17)
    V_h_d = V_ds_d ...
          + cross(V_w, V_s(1:3)) ...
          + (h_rd - dot(cross(V_w, V_s(1:3)), e3)) * V_s(1:3);

    %% ------------------------------------------------------------------
    % STAGE 3: Optical flow error and performance loop  (Sec. III-B)
    % ------------------------------------------------------------------
    V_h_e = V_h - V_h_d;   % [3x1]

    % Initialise
    S_2    = diag(zeros(3,1));
    G_2    = diag(zeros(3,1));
    zeta_2 = zeta_2_prev;

    for j = 1:3
        S_2(j,j) = V_h_e(j) / p_2(j);

        % Saturate on violation
        if abs(S_2(j,j)) >= 1
            S_2(j,j) = abs(zeta_2_prev(j)) * sign(V_h_e(j));
            V_h(j)   = S_2(j,j) * p_2(j) + V_h_d(j);
            flag = true;
            break;
        end

        % Unconstrained error  (Eq. 36)
        zeta_2(j) = log((p_2(j) + V_h_e(j)) / (p_2(j) - V_h_e(j)));

        % Performance Jacobian  (Eq. 37, g_2k)
        G_2(j,j) = (exp(zeta_2(j)) + 1)^2 / (2 * exp(zeta_2(j)) * p_2(j));
    end

    if flag
        I_a_cd = zeros(3,1);
        V_a_cd = zeros(3,1);
        sigma  = zeros(3,1);
        Theta  = zeros(3,6);
        return;
    end

    %% ------------------------------------------------------------------
    % STAGE 4: Sliding surface sigma  (Eq. 39)
    % ------------------------------------------------------------------
    sigma = zeta_2 + K.Omega * izeta_2;   % [3x1]

    %% ------------------------------------------------------------------
    % STAGE 5: Known system dynamics c  (Sec. III-C / Eq. 41 / c in code)
    %   c = cross(dw,s) + cross(w,cross(w,s)) + 2*cross(w,h)
    %     - (dot(h + cross(w,s), e3)) * h - dh_d
    % ------------------------------------------------------------------
    c = cross(V_dw,  V_s(1:3)) ...
      + cross(V_w,   cross(V_w, V_s(1:3))) ...
      + 2 * cross(V_w, V_h) ...
      - (dot(V_h + cross(V_w, V_s(1:3)), e3)) * V_h ...
      - V_dh_d;

    %% ------------------------------------------------------------------
    % STAGE 6: Uncertainty coefficient matrix Theta  (Eq. 41 / bottom)
    %   Theta = [-c + S2*dp2 - G2_inv*Omega*zeta2,  I_3]
    % ------------------------------------------------------------------
    Theta = [- c + S_2 * dp_2 - G_2 \ (K.Omega * zeta_2), eye(3)];
    Theta_norm = norm(Theta, 'fro');

    %% ------------------------------------------------------------------
    % STAGE 7: ASMC control law  (Eq. 42)
    %   u_sw = -Gamma*sigma - ||Theta|| * sat(E\sigma) * G2 * kappa
    %   u_eq = G2 * (-c + S2*dp2 - G2_inv*Omega*zeta2)
    %   V_a_cd = -G2_inv * (u_sw + u_eq)
    % ------------------------------------------------------------------
    u_sw = - K.Gamma * sigma ...
           - Theta_norm * diag(sat(K.E \ sigma)) * G_2 * kappa;

    u_eq = G_2 * (- c + S_2 * dp_2 - G_2 \ (K.Omega * zeta_2));

    V_a_cd = - G_2 \ (u_sw + u_eq);

    %% ------------------------------------------------------------------
    % STAGE 8: Transform to inertial frame  (matches main loop line)
    %   I_a_cd = I_R_V * V_a_cd - g
    % ------------------------------------------------------------------
    I_a_cd = I_R_V * V_a_cd - g;

end
