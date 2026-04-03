%% Simulation conditions
NOISE = 1; GE = 1; delay = 1;
% ZOH = 1;
ZOH = floor(100/30);
ACTUAL = 1;

%% Initialization Time Trajectory
% step size
dt = 1/100; % [s]                 % 30 Hz becaz the camera works in 30 Hz
% Initial values
t0 = 0;
tend = 40.0; % end time [s]
tRange = t0:dt:tend;

%% Initializing State
% Initial Absolute Pose of Camera wrt Global Origin in Inertial Reference Frame
I_px_c = -2.0; I_py_c = -2.0; I_pz_c = -5.0;
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
% T_nP3 = [-30, 15, 15, -15; 30, 15, -15, -15; 0, 0, 0, 0]/150; 
% T_nP3 = [-15, 15, 15, -15; 15, 15, -15, -15; 0, 0, 0, 0]/2; 
T_nP3 = [-20, 15, 15, -15; 20, 15, -15, -15; 0, 0, 0, 0]/250; 

% Removing offset due to unsymmetry
T_nP3 = T_nP3-mean(T_nP3,2);

% Computing Desired Feature Points wrt Target Origin in Virtual Camera Reference Frame
V_nP3 = T_nP3;

% Computing desired Feature Points in Image Plane    
V_nP_d = (f/zf)*V_nP3(1:2,:);

% Computing desired Features Parameters in Image Plane (without 'z')
V_s_d = image_feature(V_nP_d/f);

%% Data Logging
U_DS = [];
X_DS = x_c;
V_X_DS = [];
D_DS = [];

kappa = K.kappa_0;

%% Constants related to Low-pass filter for B_w_c
tau_w   = 0.08;        % time constant [s] (20–50 ms recommended)
alpha_w = tau_w / (tau_w + dt);

% Constants related to Low-pass filter for B_dw_c
tau_dw   = 0.08;        % time constant [s] (20–50 ms recommended)
alpha_dw = tau_dw / (tau_dw + dt);