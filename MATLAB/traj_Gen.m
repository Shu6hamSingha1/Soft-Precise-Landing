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

function [X] = traj_Gen(t, type)

    switch type

        %% ===================== 1. STATIC =====================
        case "Static"

            p = [0; 0; 0];
            v = [0; 0; 0];
            w = [0; 0; 0];
            q = [1; 0; 0; 0];

        %% ===================== 2. LINEAR =====================
        case "Linear"

            v = [0.5; 0.5; 0.0];
            p = v * t;

            w = [0; 0; 0];
            q = [1; 0; 0; 0];

        %% ===================== 3. CIRCULAR =====================
        case "Circular"

            r  = 10.0;        % radius
            wz = 0.1;        % angular rate
            vz = 0.0;

            p = [-r*(cos(wz*t)-1);
                 r*sin(wz*t);
                 vz*t];

            v = [r*wz*sin(wz*t);
                  r*wz*cos(wz*t);
                  vz];

            w = [0; 0; wz];

            q = [cos(wz*t/2);
                 0;
                 0;
                 sin(wz*t/2)];

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

            A = 0.5;
            w0 = 0.8;
            v0 = 0.1;

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

            A = 0.5;
            B = 0.8;
            w1 = -0.8;
            w2 = 0.5;

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