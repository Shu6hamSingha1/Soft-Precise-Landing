%% Quad-rotor details
r = 0.127;                              % Rotor radius in meters (10-inch props, X500)
zf = 0.2;                              % Landing height in meters (shared across multi-init + comparison)
J = diag([0.029125,0.029125,0.055225]); % X500 Inertia Matrix (Gazebo SDF)
m = 2.114;   % Mass
M = [0.174, -0.174, -0.174, 0.174;            % Motor Mixer Matrix.
    -0.174, 0.174, -0.174, 0.174;       % Values taken from Gazebo X500
    9.44, 9.44, -9.44, -9.44;        % SDF File.
    1, 1, 1, 1;];

%% Camera details
f = 135;  % Camera focal length in pixels

res = [320;240];  % Image resolution

%% Defining system constants
g = [0;0;9.81];
h_rd = -0.46;                     % slowed 8% to shave Circ R3 / Linear R5 terminal v_rel

e3 = [0;0;1];

%% Defining parameter bounds (from x500)
w_max = [4.0;4.0;4.0];   % rad/s (roll, pitch, yaw)
% w_max = [10.0;10.0;10.0];   % rad/s (roll, pitch, yaw)
T_max = 60.0; T_min = 0.00;   % N

tau_xy_max = 1.85;   % N·m
tau_z_max  = 1.0;    % N·m
% tau_xy_max = 10;   % N·m
% tau_z_max  = 10;    % N·m

K.p_10 = [res(2)/2/f; res(1)/2/f];

FILTER_WINDOW = 21;

eps = 1e-6;