% *************************************************************************
% Trajectory Generator
%
% Generates platform trajectories for:
% 1. Static
% 2. Linear
% 3. Circular
% 4. EightShape
% 5. Sinusoidal
% 6. Lissajous
% 7. CircularYaw
%
% t     -> Time
% type  -> Trajectory type (string)
%
% Output:
% X = [x, dx]
% x  = [p; q]
% dx = [v; w; 0]
% *************************************************************************

function [X] = traj_Gen(t, type, speed_mult)

    if nargin < 3 || isempty(speed_mult)
        speed_mult = 1.0;
    end

    switch type

        %% ===================== 1. STATIC =====================
        case "Static"

            p = [0; 0; 0];
            v = [0; 0; 0];
            w = [0; 0; 0];
            q = [1; 0; 0; 0];

        %% ===================== 2. LINEAR (ship deck) =====================
        %  Ship deck moving at a fixed forward speed while undergoing large
        %  roll and pitch oscillations plus a Lin 2022 heave sinusoid
        %  z_r = 0.2 sin(0.5 t). 3-2-1 (yaw-pitch-roll) attitude with yaw=0.
        case "Linear"

            A_z = 0.2;                   % heave amplitude [m]  (Lin 2022)
            w_z = 0.5;                   % heave angular freq [rad/s]

            p = [1.0*speed_mult*t;
                 1.0*speed_mult*t;
                 A_z*sin(w_z*t)];

            v = [1.0*speed_mult;
                 1.0*speed_mult;
                 A_z*w_z*cos(w_z*t)];

            phi_max   = deg2rad(15);
            theta_max = deg2rad(8);
            w_r       = 0.9;            % roll  (~7 s period)
            w_p       = 0.6;            % pitch (~10 s period)
            phase_p   = pi/3;           % pitch lag behind roll

            phi   = phi_max   * sin(w_r * t);
            theta = theta_max * sin(w_p * t + phase_p);
            psi   = 0;

            dphi   = phi_max   * w_r * cos(w_r * t);
            dtheta = theta_max * w_p * cos(w_p * t + phase_p);
            dpsi   = 0;

            q = euler321_to_quat(phi, theta, psi);
            w = euler_rates_to_bodyrates(phi, theta, dphi, dtheta, dpsi);

        %% ===================== 3. CIRCULAR (ship deck) =====================
        %  Circular surge on top of large roll/pitch deck oscillation plus
        %  a Lin 2022 heave sinusoid z_r = 0.2 sin(0.5 t); yaw follows the
        %  tangent heading wz*t.
        case "Circular"

            r  = 0.5;                      % rescaled to match prior 5/5 multi-init baseline
            wz = 0.25 * speed_mult;         % backed off from 0.5: r*wz^2 centripetal broke Run 5

            A_z = 0.2;                     % heave amplitude [m]  (Lin 2022)
            w_hz = 0.5;                    % heave angular freq [rad/s]

            p = [-r*(cos(wz*t)-1);
                 r*sin(wz*t);
                 A_z*sin(w_hz*t)];

            v = [r*wz*sin(wz*t);
                  r*wz*cos(wz*t);
                  A_z*w_hz*cos(w_hz*t)];

            phi_max   = deg2rad(15);
            theta_max = deg2rad(8);
            w_r       = 0.9;
            w_p       = 0.6;
            phase_p   = pi/3;

            phi   = phi_max   * sin(w_r * t);
            theta = theta_max * sin(w_p * t + phase_p);
            psi   = wz * t;

            dphi   = phi_max   * w_r * cos(w_r * t);
            dtheta = theta_max * w_p * cos(w_p * t + phase_p);
            dpsi   = wz;

            q = euler321_to_quat(phi, theta, psi);
            w = euler_rates_to_bodyrates(phi, theta, dphi, dtheta, dpsi);

        %% ===================== 4. EIGHT SHAPE =====================
        case "EightShape"

            a = 1.0;
            w0 = 0.3;

            p = [a*sin(w0*t);
                 a*sin(w0*t).*cos(w0*t);
                 0];

            v = [a*w0*cos(w0*t);
                 a*w0*(cos(2*w0*t));
                 0];

            w = [0; 0; 0];
            q = [1; 0; 0; 0];

        %% ===================== 5. SINUSOIDAL =====================
        case "Sinusoidal"

            A  = 0.5;
            w0 = 0.8 * speed_mult;   % x-axis oscillation frequency
            v0 = 0.3 * speed_mult;   % y-axis drift velocity

            p = [A*sin(w0*t);
                 v0*t;
                 0];

            v = [A*w0*cos(w0*t);
                 v0;
                 0];

            w = [0; 0; 0];
            q = [1; 0; 0; 0];

        %% ===================== 6. LISSAJOUS =====================
        case "Lissajous"

            A  = 0.4;
            B  = 0.8;
            w1 = -0.8 * speed_mult;
            w2 =  0.4 * speed_mult;

            p = [A*sin(w1*t);
                 B*sin(w2*t);
                 0];

            v = [A*w1*cos(w1*t);
                 B*w2*cos(w2*t);
                 0];

            w = [0; 0; 0];
            q = [1; 0; 0; 0];

        %% ===================== 7. CIRCULAR + YAW ROTATION =====================
        case "CircularYaw"

            r  = 1.0;
            w_tr = 0.2;   % translation angular rate
            w_yaw = 0.4;  % independent yaw rate

            p = [r*(cos(w_tr*t)-1);
                 r*sin(w_tr*t);
                 0];

            v = [-r*w_tr*sin(w_tr*t);
                  r*w_tr*cos(w_tr*t);
                  0];

            w = [0; 0; w_yaw];

            q = [cos(w_yaw*t/2);
                 0;
                 0;
                 sin(w_yaw*t/2)];

        otherwise
            error("Unknown trajectory type");

    end

    % State vector
    x  = [p; q];

    % Derivative
    dx = [v; w; 0];

    % Output
    X = [x, dx];

end

% ---- Local helpers for deck-motion trajectories ----

function q = euler321_to_quat(phi, theta, psi)
    % 3-2-1 (yaw-pitch-roll) Tait-Bryan composition: q = q_psi * q_theta * q_phi
    cph = cos(phi/2);   sph = sin(phi/2);
    cth = cos(theta/2); sth = sin(theta/2);
    cps = cos(psi/2);   sps = sin(psi/2);

    q = [ cps*cth*cph + sps*sth*sph;
          cps*cth*sph - sps*sth*cph;
          cps*sth*cph + sps*cth*sph;
          sps*cth*cph - cps*sth*sph ];
end

function w = euler_rates_to_bodyrates(phi, theta, dphi, dtheta, dpsi)
    % 3-2-1 Euler rates -> body angular velocity
    w = [ dphi - dpsi*sin(theta);
          dtheta*cos(phi) + dpsi*cos(theta)*sin(phi);
         -dtheta*sin(phi) + dpsi*cos(theta)*cos(phi) ];
end