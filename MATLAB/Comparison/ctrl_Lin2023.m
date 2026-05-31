%% ************************************************************************
% ctrl_Lin2023.m
%
% Baseline controller: Robust circle-feature IBVS + performance funnel
%                      + geometric SO(3) attitude inner loop.
% Lin, Wang, Miao, Wang & Fierro,
%   "Robust Image-Based Landing Control of a Quadrotor on an Unpredictable
%    Moving Vehicle Using Circle Features"
%   IEEE Trans. Autom. Sci. Eng., Vol. 20, No. 2, pp. 1429-1440, 2023.
%   DOI: 10.1109/TASE.2022.3180506
%
% Replaces ctrl_Chen2025.m as the second IBVS baseline (Controller 4).
% Preserves the 2 PBVS (Lin2022, Zhang2026) + 2 IBVS (this, Cho2022) split.
%
% STRUCTURE = ctrl_Lin2022 with the OUTER loop on circle-moment IMAGE
% features instead of metric position. This is exactly the IBVS (Lin2023)
% vs PBVS (Lin2022) distinction. The log-barrier BLF backstepping and the
% NED force/attitude convention are reused verbatim from ctrl_Lin2022
% (proven sign-correct + stable in this harness). Lin2023's circle-moment
% features s_t are formed in the harness (see case 4) from the simulated
% image points; this function consumes them.
%
% PIPELINE (paper equation numbers in brackets)
%   feature error    e_t = s_t - s_t_d                                 (11)
%   transform        xi_t = e_t./rho_t,  eps_t = 0.5 ln((1+xi)/(1-xi)) (13)
%   virtual velocity vhat_V = -k1 (Q_t' eps_t),  vhat_I = I_R_V vhat_V (15)
%   velocity error   e_v = I_v_c - vhat_I                              (16)
%   transform        xi_v = e_v./rho_v,  eps_v = 0.5 ln((1+xi)/(1-xi)) (16)
%   force (NED)      F = -k2 (Q_v' eps_v) - m g_vec                    (19)
%   desired attitude rd3 = -F/||F||, rd2 = (rd3 x b)/||.||, rd1=rd2 x rd3 (20)
%   attitude error   eR = 0.5 (Rd' R - R' Rd)^vee                      (22)
%   torque           tau = -kR eR - kOmega eOmega + Omega x J Omega    (24)
%
% INPUTS
%   s_t        - circle-moment features [an*xg; an*yg; an]   [3x1]      (7)
%   s_t_d      - desired features (an_d = 1)                 [3x1]
%   I_v_c      - UAV velocity in NED (own-velocity feedback) [3x1]
%   rho_t      - feature funnel value rho_t(t)               [3x1]     (10)
%   rho_v      - velocity funnel value rho_v(t)              [3x1]     (10)
%   psi_des    - desired (fixed) yaw psi*                    [rad]
%   R_c        - rotation body->inertial                     [3x3]
%   I_R_V      - rotation virtual->inertial (yaw-only)        [3x3]
%   B_w_c      - body angular velocity                        [rad/s]
%   K          - gains struct (k1, k2, kR, kOmega)
%   m, J       - mass, inertia
%   g_vec      - gravity [0;0;9.81] (NED, points down)
%   tau_xy_max, tau_z_max, T_max, T_min
%
% OUTPUTS
%   u_2        - [4x1] = [tau_x; tau_y; tau_z; T]
%   I_a_cd     - [3x1] commanded inertial acceleration (for logging)
%   Rd         - [3x3] desired rotation matrix
% *************************************************************************
function [u_2, I_a_cd, Rd] = ...
          ctrl_Lin2023(s_t, s_t_d, I_v_c, rho_t, rho_v, ...
                       psi_des, R_c, I_R_V, B_w_c, K, m, J, g_vec, ...
                       tau_xy_max, tau_z_max, T_max, T_min)

    %% ---------------------------------------------------------------
    % OUTER LOOP: robust circle-feature IBVS funnel backstepping
    % ---------------------------------------------------------------
    % Feature error + log-barrier transform  (Eqs. 11, 13)
    e_t   = s_t - s_t_d;
    xi_t  = clamp_vec(e_t ./ rho_t, -0.999, 0.999);
    eps_t = 0.5 * log((1 + xi_t) ./ (1 - xi_t));
    q_t   = 1 ./ ((1 + xi_t) .* (1 - xi_t));

    % Virtual velocity (V-frame), rotated to inertial  (Eq. 15)
    % POSITIVE sign (opposite of ctrl_Lin2022's position-based -k1): the
    % image-feature dynamics (Eq. 8) ds_t = -(1/Z*)(v - v_t) carry an extra
    % inversion vs position dynamics (r_dot = v), so the stabilising virtual
    % velocity is vhat = +Kt e_t/(1-xi^2). Verified to give vhat_x<0 toward
    % target (UAV +X -> image xg<0 -> e_t1<0 -> vhat_x<0) and vhat_z>0 (descend).
    vhat_V = K.k1 .* (q_t .* eps_t);   % K.k1 is [3x1] (per-axis): literal
    vhat_I = I_R_V * vhat_V;            % feature an=sqrt(a*/a) makes the depth
                                        % error large, so depth gets a gentler
                                        % gain than the lateral axes.

    % Velocity error + transform (inertial)  (Eq. 16)
    e_v   = I_v_c - vhat_I;
    xi_v  = clamp_vec(e_v ./ rho_v, -0.999, 0.999);
    eps_v = 0.5 * log((1 + xi_v) ./ (1 - xi_v));
    q_v   = 1 ./ ((1 + xi_v) .* (1 - xi_v));

    % Thrust force vector  (Eq. 19, NED: -m*g_vec mirrors ctrl_Lin2022)
    F_vec  = -K.k2 * (q_v .* eps_v) - m * g_vec;
    f_mag  = norm(F_vec);
    I_a_cd = F_vec / m;

    %% ---------------------------------------------------------------
    % INNER LOOP: geometric SO(3) attitude controller  (Eqs. 20-24)
    % ---------------------------------------------------------------
    T = f_mag;
    if f_mag < 1e-6
        Rd = eye(3);
    else
        rd3 = -F_vec / f_mag;                      % body-z = -thrust dir (FRD)
        a   = [cos(psi_des); sin(psi_des); 0];     % Eq. 20
        rd2_raw = cross(rd3, a);
        n2 = norm(rd2_raw);
        if n2 < 1e-6, rd2_raw = [0;1;0]; n2 = 1; end
        rd2 = rd2_raw / n2;
        rd1 = cross(rd2, rd3);
        Rd  = [rd1, rd2, rd3];
    end

    Omega_d = zeros(3,1);                          % slowly-varying Rd
    eR      = vee_map(0.5 * (Rd'*R_c - R_c'*Rd));  % Eq. 22
    eOmega  = B_w_c - R_c'*Rd*Omega_d;             % Eq. 23

    tau_cd = -K.kR*eR - K.kOmega*eOmega + cross(B_w_c, J*B_w_c);   % Eq. 24
    tau_cd(1:2) = min(max(tau_cd(1:2), -tau_xy_max), tau_xy_max);
    tau_cd(3)   = min(max(tau_cd(3),   -tau_z_max),  tau_z_max);
    T           = max(min(T, T_max), T_min);

    u_2 = [tau_cd; T];
end

function v = vee_map(S)
    v = [S(3,2); S(1,3); S(2,1)];
end

function y = clamp_vec(x, lo, hi)
    y = max(lo, min(hi, x));
end
