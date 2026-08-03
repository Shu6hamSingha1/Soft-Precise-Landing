%% CB_GAMMA_WHY  Why did raising Gamma fail? Isolate Gamma on the BAKED (stable) funnel
%   -> is the destabilization intrinsic to high Gamma (high-gain overshoot), independent
%   of the tight funnel? Trace h_e / sigma / a_u to see oscillation vs smooth convergence.
%   Gamma = REACHING gain (sets sigma->0 rate); h_e convergence is bandwidth-limited
%   (tilt->lateral-accel chain + 1-step actuator delay + inner-loop). Demanding faster
%   than the bandwidth -> overshoot -> oscillation -> lateral limit cycle.
%
% Run:  cd MATLAB/Multi_init_cond; cb_gamma_why
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE %#ok<GVMIS>

% COMBINED with the BAKED (loose, stable) funnel -- isolate Gamma from funnel tightness
C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

for gxy = [0.4375, 2.0]
    cfg = C; cfg.Gamma = diag([gxy,gxy,0.75]);
    VDF_OVERRIDE = cfg;
    x0=[2;2;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];
    r=run_simulation(x0,"Sinusoidal",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
    d=r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)';
    he = vecnorm(d.V_h_e(1:2,1:idx));
    sg = vecnorm(d.sigma(1:2,1:idx));
    au = vecnorm(d.I_a_cd(1:2,1:idx));
    % oscillation metric: sign changes of d|sigma|/dt over the flight (chatter/limit cycle)
    dsg = diff(sg); flips = sum(abs(diff(sign(dsg)))>0);
    fprintf('\n=== BAKED funnel, Gamma_xy=%.3f (landed=%d xy=%.3f) | sigma sign-flips=%d ===\n', ...
        gxy, r.success, r.final_xy, flips);
    fprintf('     t   | |h_e|   |sigma|  |a_u_xy|\n');
    for tt=0:1.0:t(end)
        [~,j]=min(abs(t-tt));
        fprintf('   %4.1f | %5.3f   %5.3f   %6.2f\n', t(j), he(j), sg(j), au(j));
    end
end
clear global VDF_OVERRIDE
