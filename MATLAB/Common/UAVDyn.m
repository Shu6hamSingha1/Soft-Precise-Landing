%--------------------------------------------------------------------------
%
% UAV Flight Dynamics: Defines the ordinary differential equations of 
% the flight dynamics of the UAV in body-fixed frame.
% x     - > State vector (px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz)
% u     - > Control Input (T, taux, tauy, tauz)
%    
%--------------------------------------------------------------------------
function [dxdt] = UAVDyn(~, x, u)
    Constants;

    %--------------------------------------------------
    % State unpacking
    %--------------------------------------------------
    pos   = x(1:3);
    quat  = x(4:7);     % [qw qx qy qz]
    vel   = x(8:10);
    omega = x(11:13);   % body angular velocity

    %--------------------------------------------------
    % Rotation matrix
    %--------------------------------------------------
    R = quat2rotm(quat');   % body -> inertial

    %--------------------------------------------------
    % Forces & torques
    %--------------------------------------------------
    f_b   = [0; 0; -u(4)];
    tau   = u(1:3);

    %--------------------------------------------------
    % Translational dynamics
    %--------------------------------------------------
    dpos = vel;
    dvel = (1/m) * R * f_b + g;

    %--------------------------------------------------
    % Quaternion kinematics
    %--------------------------------------------------
    Omega = [ 0         -omega'
              omega    -skew(omega) ];

    dquat = 0.5 * Omega * quat;

    %--------------------------------------------------
    % Rotational dynamics
    %--------------------------------------------------
    % Add angular damping
    D = diag([0.05, 0.05, 0.02]);
    domega = J \ (tau - cross(omega, J*omega)) - D * omega;

    %--------------------------------------------------
    % Assemble derivative
    %--------------------------------------------------
    dxdt = [dpos;
            dquat;
            dvel;
            domega];
end