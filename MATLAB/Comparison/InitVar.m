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
T_tip = [ 15/sqrt(2), -15/sqrt(2), -15/sqrt(2),  15/sqrt(2),  22 ;     % 4 arm tips + stub tip (legacy 5-point cross)
         15/sqrt(2), -15/sqrt(2),  15/sqrt(2), -15/sqrt(2),   0 ;
                  0,           0,           0,           0,   0 ] * marker_scale / 250;
% 2026-09-21 PORT of Multi_init_cond/InitVar.m: LINE-SAMPLED CROSS. Each arm and the stub is a line of N samples centre->tip,
% N_arm = max(1,round(scale/2)), N_stub = round(N_arm*22/15) (26x: 13/arm, 19 stub, 71 points); N_arm=1 (scale<~3) is EXACTLY the
% legacy 5-point cross. Column order [arm1|arm2|arm3|arm4|stub]; T_wq = stub weight 3 / arm 1; T_ang = line direction; T_key = the 5 KEY
% points (4 arm tips + stub tip). Controller 1 (PLASMC) sees all samples (in-view mask + direction-based alpha, image_features.m);
% the baselines (Lin2023, Cho2022; Lin2022/Zhang2026 are PBVS) see only the 5 key points -- the same physical marker, its distinctive
% points. Origin = the cross INTERSECTION (the landing point) unless global PX_CENTER_FEATURE=false (then recentred on the centroid).
N_arm = max(1, round(marker_scale/2));  N_stub = max(1, round(N_arm*22/15));
T_all = zeros(3,0);  T_wq = zeros(1,0);  T_ang = zeros(1,0);  T_key_idx = zeros(1,5);
for ln = 1:5
    if ln <= 4, Nl = N_arm; wl = 1; else, Nl = N_stub; wl = 3; end
    for kk = 1:Nl
        T_all(:,end+1) = T_tip(:,ln)*kk/Nl;               %#ok<SAGROW>
        T_wq(end+1)    = wl;                              %#ok<SAGROW>
        T_ang(end+1)   = atan2(T_tip(2,ln), T_tip(1,ln));  %#ok<SAGROW>
    end
    T_key_idx(ln) = size(T_all,2);                        % last sample of the line = its tip
end
T_key = T_all(:, T_key_idx);
% Controller-dependent point set (CTRL_SEL is set by visualControl_comparison before InitVar runs)
if exist('CTRL_SEL','var') && CTRL_SEL ~= 1
    T_nP3 = T_key;  T_lines = false;
else
    T_nP3 = T_all;  T_lines = size(T_all,2) > 5;
end
global PX_CENTER_FEATURE %#ok<GVMIS>
if ~isempty(PX_CENTER_FEATURE) && ~PX_CENTER_FEATURE, T_nP3 = T_nP3-mean(T_nP3,2); end   % legacy centroid origin (opt-out)

% Computing Desired Feature Points wrt Target Origin in Virtual Camera Reference Frame
V_nP3 = T_nP3;

% Computing desired Feature Points in Image Plane    
V_nP_d = (f/(2*zf))*V_nP3(1:2,:);

% Computing desired Features Parameters in Image Plane (without 'z')
if T_lines, V_s_d = image_feature(V_nP_d/f, T_wq); else, V_s_d = image_feature(V_nP_d/f); end
if isempty(PX_CENTER_FEATURE) || PX_CENTER_FEATURE, V_s_d(1:2) = 0; end   % desired centre = image centre (run_simulation parity)

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