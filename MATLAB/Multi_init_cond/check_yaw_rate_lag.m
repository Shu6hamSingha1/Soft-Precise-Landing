%% CHECK_YAW_RATE_LAG  Step response of MATLAB's yaw rate loop (PX4-parity check, 2026-09-21).
%   PX4 measured a ~287 ms lag from commanded to achieved yaw rate (GT 275 ms; PX4_Gazebo/tools/
%   analyze_gt_rate_lag.py). In MATLAB the direct-rate yaw (so3_tracker, P.yaw_direct_rate) is a
%   torque-level P rate loop, so the achieved rate is first order with tau = J_z/kOmega_z. This
%   script applies a yaw-rate command u_a in hover through the REAL so3_tracker + saturation +
%   actuator delay + UAVDyn + RK5 and reports the 63% / 95% response times.
%   Expected (kOmega_z=0.2, J_z=0.0552): tau = 0.276 s, 63% at ~0.27 s, 95% at ~0.8 s, gain ~0.99.
%   If you change kOmega_z or J and want to keep PX4's lag, tau must stay ~0.27-0.29 s.
%   Run:  cd MATLAB/Multi_init_cond; check_yaw_rate_lag
clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','Common')); addpath(fullfile(here,'..','VDF_ASMC'));
Constants; P = vdf_params(); dt = P.dt; g = P.g;
fprintf('J_z=%.4f kOmega_z=%.2f -> tau = J/k = %.3f s ; tau_z_max=%.2f ; dt=%.3f\n', ...
        J(3,3), P.kOmega(3,3), J(3,3)/P.kOmega(3,3), tau_z_max, dt);
for delay = [0 1]                                  % actuator delay in steps (run_simulation uses 1)
    for ustep = [0.3 1.0]                          % commanded yaw rate [rad/s]
        x  = [0;0;-5;1;0;0;0;zeros(6,1)];
        cs = struct('ie_R',zeros(3,1),'thetahat',zeros(2,1),'yrl_cmd',ustep);
        N = round(3/dt); buf = zeros(4,N); wz = zeros(1,N);
        for k = 1:N
            q = x(4:7)/norm(x(4:7)); R = quat2rotm(q');
            yaw = atan2(2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2)); w = x(11:13);
            [tau,T,cs] = blocks.so3_tracker([0;0;-g],[0;0],R(3,3),yaw,yaw,R,w,P,cs);
            tau(1:2) = min(max(tau(1:2),-tau_xy_max),tau_xy_max);
            tau(3)   = min(max(tau(3),  -tau_z_max), tau_z_max);
            T = max(min(T,T_max),T_min);
            buf(:,k) = [tau;T];
            if k > delay, u = buf(:,k-delay); else, u = [zeros(3,1); m*g]; end
            x = RK5(@(t,xx) UAVDyn(t,xx,u), (k-1)*dt, x, dt);  wz(k) = x(13);
        end
        t = (1:N)*dt; ss = wz(end); i63 = find(wz >= 0.632*ss,1); i95 = find(wz >= 0.95*ss,1);
        fprintf('delay=%d step=%.1f rad/s: steady gain %.3f | 63%% at %.3f s | 95%% at %.3f s\n', ...
                delay, ustep, ss/ustep, t(i63), t(i95));
    end
end
