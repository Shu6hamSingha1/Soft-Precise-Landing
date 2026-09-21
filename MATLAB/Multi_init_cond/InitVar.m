%% Simulation conditions
NOISE = 1; GE = 1; delay = 1;
% ZOH = 1;
ZOH = floor(100/30);
ACTUAL = 1;

% Allow external batch wrappers (phase1_baseline_sweep.m, etc.) to override
% IC and noise via MATLAB globals declared BEFORE calling the canonical
% script.  `clear` at the top of the canonical script wipes locals but
% leaves `global` declarations intact, so this is the safe override path.
global IC_OVERRIDE NOISE_OVERRIDE IC_VEL_OVERRIDE;
if ~isempty(NOISE_OVERRIDE); NOISE = NOISE_OVERRIDE; end

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
if ~isempty(IC_OVERRIDE)
    I_px_c = IC_OVERRIDE(1); I_py_c = IC_OVERRIDE(2); I_pz_c = IC_OVERRIDE(3);
end
I_p_c = [I_px_c; I_py_c; I_pz_c];
q_cw = 1.0; q_cx = 0.0; q_cy = 0.0; q_cz = 0.0;
q_c = [q_cw; q_cx; q_cy; q_cz];
q_c = q_c / norm(q_c);

% Initial Absolute Linear and Angular Velocities of Camera in Inertial
% Reference Frame
I_vx_c = 0.0; I_vy_c = 0.0; I_vz_c = -0.00;
if ~isempty(IC_VEL_OVERRIDE)
    I_vx_c = IC_VEL_OVERRIDE(1); I_vy_c = IC_VEL_OVERRIDE(2); I_vz_c = IC_VEL_OVERRIDE(3);
end
I_v_c = [I_vx_c; I_vy_c; I_vz_c];
w_cx = 0.00; w_cy = 0.00; w_cz = 0.00;
B_w_c = [w_cx; w_cy; w_cz];

x_c = [I_p_c; q_c; I_v_c; B_w_c];

% *************************************************************************
% Computing Desired Image Features Parameters
% *************************************************************************
% Defining Desired Feature Points wrt to Target Origin in Target Reference Frame
% 5-point CROSS marker (abstraction of the PX4 SITL cross marker, same camera
% model f=135 / res=[320;240]): cols 1-4 are the four arm tips of a SYMMETRIC
% X (the two arm-LINES are the diagonals, col1/col2 opposite, col3/col4
% opposite, 90deg apart); col 5 is the STUB, a fifth point extending
% horizontally (+x) -- at 45deg RELATIVE to the nearest arm (col1/col4), not
% collinear with any arm. This matches the real PX4 marker geometry: see
% src/cross_marker_detector.py's own detection-geometry comments ("two arms
% meet near 90deg" + STUB_REL_ANGLE_DEG=45) and PX4_Gazebo/Images/cross_marker.png.
% CORRECTED 2026-09-15: the previous definition here had the arms axis-aligned
% (a +, not an X) with the stub COLLINEAR (0deg relative) with the +x arm --
% wrong on both counts, caught by comparing a generated figure against the
% actual PX4 marker image. This changes the real simulated feature geometry
% (image_feature.m's alpha depends on it), not just how it plots -- every
% MATLAB result was re-run after this fix (see run history/memory).
% The stub is the only asymmetry and is what makes the image orientation a full
% 2pi direction (yaw observable past the +-90deg principal-axis fold) --
% consumed by the N==5 weighted-centroid branch in image_feature.m. Column
% order (stub LAST) is a contract with that branch; do not permute. Aligned
% cross -> alpha = 0.
% Legacy 4-point trapezoid (pre-2026-09-09): [-20 15 15 -15; 20 15 -15 -15; 0 0 0 0]/250
% Sized so the recentred half-extent (~19.4/250) matches the legacy marker's (~18.8/250).
% MARKER SIZE (2026-09-19): 2x the original 12 cm cross (arm tips at 15/250 m, stub 22/250 m -> 30/250, 44/250). The
% cross marker only needs its CENTRE in view (centroid-visibility CBF), so it no longer has to fit the old 4-corner
% FoV margin; a larger marker also keeps the singular values of the interaction matrix above pinv_tol early in the
% flight so w_z is observable (see project_ic2_speed_sweep_failure_2026_09_17 memory). NB the PX4/Gazebo marker is a
% separate asset and is NOT changed by this.
% 2026-09-21: SINGLE size variable = global MARKER_SCALE (absolute, x the 12 cm cross; default [] -> 26). 26x => tip-to-tip span
% 0.12*26 = 3.12 m >= 60 px at 7 m (f=135: 3.12*135/7 = 60.2 px), matching the PX4 >=60 px@7 m rule. PRIOR: base 2.0 x hook
% (old MARKER_SCALE=s meant total 2*s; use MARKER_SCALE=2*s now). The cross only needs its CENTRE in view (centroid-visibility CBF);
% run_simulation.m aborts only when the marker CENTRE leaves the physical frame.
global MARKER_SCALE %#ok<GVMIS>
if isempty(MARKER_SCALE), marker_scale = 26; else, marker_scale = MARKER_SCALE; end
T_nP3 = [ 15/sqrt(2), -15/sqrt(2), -15/sqrt(2),  15/sqrt(2),  22 ;
          15/sqrt(2), -15/sqrt(2),  15/sqrt(2), -15/sqrt(2),   0 ;
                   0,           0,           0,           0,   0 ] * marker_scale / 250;

% Removing offset due to unsymmetry (the stub biases the geometric centroid)
global PX_CENTER_FEATURE %#ok<GVMIS>
% With PX_CENTER_FEATURE the target origin stays at the cross INTERSECTION (the tracked centre / landing point, as in PX4); otherwise the
% legacy recentring puts the origin at the 5-point centroid (offset 0.0176*scale m from the intersection: 3.5 cm @2x, 42 cm @24x).
if ~isempty(PX_CENTER_FEATURE) && ~PX_CENTER_FEATURE, T_nP3 = T_nP3-mean(T_nP3,2); end

% Computing Desired Feature Points wrt Target Origin in Virtual Camera Reference Frame
V_nP3 = T_nP3;

% Computing desired Feature Points in Image Plane    
V_nP_d = (f/(2*zf))*V_nP3(1:2,:);

% Computing desired Features Parameters in Image Plane (without 'z')
V_s_d = image_feature(V_nP_d/f);
if isempty(PX_CENTER_FEATURE) || PX_CENTER_FEATURE, V_s_d(1:2) = 0; end   % desired centre = image centre

%% Data Logging
U_DS = [];
X_DS = x_c;
V_X_DS = [];
D_DS = [];

% K.kappa_0 / K.kappa_a_0 aren't defined by Constants.m in the current
% repo state.  The canonical script overwrites both `kappa` and `kappa_a`
% at its lines 172-173 from K_ctrl.kappa_0 anyway, so this is just a safe
% pre-init.  Honor K.kappa_0 if a caller (e.g. Adapt_Control_Params)
% defined it; otherwise fall back to the canonical default [0.125;0.125;0.25].
if isfield(K, 'kappa_0');   kappa   = K.kappa_0;   else; kappa   = [0.125; 0.125; 0.25]; end
if isfield(K, 'kappa_a_0'); kappa_a = K.kappa_a_0; else; kappa_a = 2.0;                  end

%% Constants related to Low-pass filter for B_w_c
tau_w   = 0.08;        % time constant [s] (20–50 ms recommended)
alpha_w = tau_w / (tau_w + dt);

% Constants related to Low-pass filter for B_dw_c
tau_dw   = 0.08;        % time constant [s] (20–50 ms recommended)
alpha_dw = tau_dw / (tau_dw + dt);