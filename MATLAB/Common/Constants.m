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
h_rd = -0.42;                     % slowed further (-0.50->-0.46->-0.42) to close Linear IC5 soft threshold; -0.38 over-slows and kills IC4 hover-authority
% Honor batch-wrapper global if set (e.g. to match PX4's -0.70 default).
global H_RD_OVERRIDE;
if ~isempty(H_RD_OVERRIDE); h_rd = H_RD_OVERRIDE; end

e3 = [0;0;1];

%% Defining parameter bounds (from x500)
w_max = [4.0;4.0;4.0];   % rad/s (roll, pitch, yaw)
% w_max = [10.0;10.0;10.0];   % rad/s (roll, pitch, yaw)
% T_max PORTED FROM PX4 2026-09-03 (was 60.0 N = 2.89 g): PX4's measured x500 ceiling is
% THRUST_MAX_N*THRUST_MARGIN = 33.85*0.85 = 28.7725 N (A_CAP = 13.610 m/s^2 = 1.389 g at m=2.114).
% MATLAB had been modelling ~2.09x the real airframe's authority. NB this is a PLANT constant --
% it applies to ALL FIVE controllers (PLASMC + the four baselines all clamp T against it), so the
% full comparison must be regenerated. Revert to 60.0 to restore the pre-port airframe.
T_max = 28.7725; T_min = 0.00;   % N

tau_xy_max = 1.85;   % N·m
tau_z_max  = 1.0;    % N·m
% tau_xy_max = 10;   % N·m
% tau_z_max  = 10;    % N·m

K.p_10 = [res(2)/2/f; res(1)/2/f];

FILTER_WINDOW = 11;

eps = 1e-6;