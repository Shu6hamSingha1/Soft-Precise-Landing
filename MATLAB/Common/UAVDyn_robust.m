%--------------------------------------------------------------------------
% UAVDyn_robust: quadrotor dynamics with parametric uncertainty + wind.
%
% Inputs:
%   x        - state [px py pz qw qx qy qz vx vy vz wx wy wz]
%   u        - control [taux; tauy; tauz; T]
%   m_p      - perturbed mass [kg]
%   J_p      - perturbed inertia matrix [3x3]
%   F_wind_I - wind force in inertial frame [N, 3x1] (includes turbulence)
%   r_cog    - center-of-gravity offset in body frame [m, 3x1]
%--------------------------------------------------------------------------
function [dxdt] = UAVDyn_robust(~, x, u, m_p, J_p, F_wind_I, r_cog)
    quat  = x(4:7);
    vel   = x(8:10);
    omega = x(11:13);

    R = quat2rotm(quat');

    f_b = [0; 0; -u(4)];
    tau = u(1:3) + cross(r_cog, f_b);   % CoG offset -> thrust-induced torque

    g = [0; 0; 9.81];

    dpos = vel;
    dvel = (1/m_p) * R * f_b + g + F_wind_I / m_p;

    Omega = [ 0       -omega'
              omega   -skew(omega) ];
    dquat = 0.5 * Omega * quat;

    D = diag([0.05, 0.05, 0.02]);
    domega = J_p \ (tau - cross(omega, J_p*omega)) - D * omega;

    dxdt = [dpos; dquat; dvel; domega];
end
