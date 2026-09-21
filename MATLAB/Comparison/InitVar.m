%% Simulation conditions
NOISE = 1; GE = 1; delay = 1;
% ZOH = 1;
ZOH = floor(100/30);
ACTUAL = 1;

%% Sensor noise parameters (active when NOISE = 1)
SNR_IBVS  = 50;       % IBVS pixel noise SNR [dB] for awgn (50=original, 60=baseline)
sigma_pos = 0.01;     % PBVS position measurement noise std [m]
sigma_vel = 0.02;     % PBVS velocity measurement noise std [m/s]

%% Initialization Time Trajectory
% step size
dt = 1/100; % [s]                 % 30 Hz becaz the camera works in 30 Hz
% Initial values
t0 = 0;
tend = 40.0; % end time [s]
tRange = t0:dt:tend;

%% Initializing State
% Initial Absolute Pose of Camera wrt Global Origin in Inertial Reference Frame
I_px_c = 2.0; I_py_c = 2.0; I_pz_c = -5.0;
I_p_c = [I_px_c; I_py_c; I_pz_c];
q_cw = 1.0; q_cx = 0.0; q_cy = 0.0; q_cz = 0.0;
q_c = [q_cw; q_cx; q_cy; q_cz];
q_c = q_c / norm(q_c);

% Initial Absolute Linear and Angular Velocities of Camera in Inertial 
% Reference Frame
I_vx_c = 0.0; I_vy_c = 0.0; I_vz_c = -0.00;
I_v_c = [I_vx_c; I_vy_c; I_vz_c];
w_cx = 0.00; w_cy = 0.00; w_cz = 0.00;
B_w_c = [w_cx; w_cy; w_cz];

x_c = [I_p_c; q_c; I_v_c; B_w_c];

% *************************************************************************
% Computing Desired Image Features Parameters
% *************************************************************************
% Defining Desired Feature Points wrt to Target Origin in Target Reference Frame
% 5-point CROSS marker (matches Multi_init_cond/InitVar.m and the PX4 SITL cross):
% cols 1-4 = symmetric X-arm tips (arm-LINES are the diagonals, col1/col2 and
% col3/col4 opposite pairs, 90deg apart), col 5 = STUB extending horizontally
% (+x), at 45deg RELATIVE to the nearest arm -- the sole asymmetry -> full 2pi
% image orientation. Column order (stub LAST) is a contract with
% image_feature.m's N==5 branch; do not permute.
% CORRECTED 2026-09-15 (matches Multi_init_cond/InitVar.m's same-date fix): was
% previously an axis-aligned + with the stub collinear (0deg relative) with the
% +x arm -- wrong on both counts vs the real PX4 marker (src/cross_marker_detector.py:
% "two arms meet near 90deg" + STUB_REL_ANGLE_DEG=45, PX4_Gazebo/Images/cross_marker.png).
% Legacy 4-point trapezoid (pre-2026-09-10): [-20 15 15 -15; 20 15 -15 -15; 0 0 0 0]/250
% MARKER SIZE (2026-09-19): 2x the original 12 cm cross (arm tips at 15/250 m, stub 22/250 m -> 30/250, 44/250). The
% cross marker only needs its CENTRE in view (centroid-visibility CBF), so it no longer has to fit the old 4-corner
% FoV margin; a larger marker also keeps the singular values of the interaction matrix above pinv_tol early in the
% flight so w_z is observable (see project_ic2_speed_sweep_failure_2026_09_17 memory). NB the PX4/Gazebo marker is a
% separate asset and is NOT changed by this.
% 2026-09-21: SINGLE size variable = global MARKER_SCALE (absolute, x the 12 cm cross; default [] -> 26), same as
% Multi_init_cond/InitVar.m. 26x => 3.12 m tip-to-tip >= 60 px at 7 m (f=135). PRIOR: base 2.0 x hook.
global MARKER_SCALE %#ok<GVMIS>
if isempty(MARKER_SCALE), marker_scale = 26; else, marker_scale = MARKER_SCALE; end
T_nP3 = [ 15/sqrt(2), -15/sqrt(2), -15/sqrt(2),  15/sqrt(2),  22 ;
          15/sqrt(2), -15/sqrt(2),  15/sqrt(2), -15/sqrt(2),   0 ;
                   0,           0,           0,           0,   0 ] * marker_scale / 250;

% Removing offset due to unsymmetry (the stub biases the geometric centroid)
T_nP3 = T_nP3-mean(T_nP3,2);

% Computing Desired Feature Points wrt Target Origin in Virtual Camera Reference Frame
V_nP3 = T_nP3;

% Computing desired Feature Points in Image Plane    
V_nP_d = (f/(2*zf))*V_nP3(1:2,:);

% Computing desired Features Parameters in Image Plane (without 'z')
V_s_d = image_feature(V_nP_d/f);

% Desired area moment a* for the IBVS baselines (Eq. 8-9: a* = mu_20 + mu_02 at desired pose)
nP_d_norm = V_nP_d / f;                     % normalised desired feature points
cg_d      = mean(nP_d_norm, 2);             % centroid
a_star    = sum(sum((nP_d_norm - cg_d).^2));  % mu_20 + mu_02

%% Data Logging
U_DS = [];
X_DS = x_c;
V_X_DS = [];
D_DS = [];

%% Constants related to Low-pass filter for B_w_c
tau_w   = 0.08;        % time constant [s] (20–50 ms recommended)
alpha_w = tau_w / (tau_w + dt);

% Constants related to Low-pass filter for B_dw_c
tau_dw   = 0.08;        % time constant [s] (20–50 ms recommended)
alpha_dw = tau_dw / (tau_dw + dt);