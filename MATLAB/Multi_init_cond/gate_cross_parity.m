%% GATE_CROSS_PARITY  Re-gate after the image_feature.m N==5 change (2026-09-10):
%   the 5-pt cross principal axis now comes from PLAIN unweighted centered
%   moments (PX4 _unweighted_principal_angle parity) instead of the weighted
%   wq=[1 1 1 1 3] moments. Stub weight kept ONLY for the disambiguation vector.
%
%   Manuscript realistic gate: 5 traj x 5 IC = 25, NOISE/GE/delay=1, seed=1,
%   VDF_OVERRIDE.theta_per_axis=true, all gains from the current vdf_params.m
%   (h_rd=-0.38, chi_r=2.0, yaw_rate_law=1, two-tier CBF, cbf_drift_tau=0.15).
%
%   PASS = 25/25 soft-precise, 0 FoV, worst xy ~<=0.056 (matches the 2026-09-09
%   replica in project_manuscript_lyapunov_radial_D_fix_2026_09_08).
%
% Run:  cd MATLAB/Multi_init_cond; gate_cross_parity
% Saves: gate_cross_parity.mat
clc; clear;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','Common'));
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

R = struct('traj',{},'ic',{},'sp',{},'precise',{},'soft',{},'xy',{},'v',{},'t',{},'fov',{});
sp=0; fov=0; xy=[]; vv=[]; tt=[];
fprintf('\n===== CROSS-PARITY GATE (25 realistic, seed=1) =====\n');
for t = 1:numel(trajList)
    for ic = 1:5
        VDF_OVERRIDE = struct('theta_per_axis',true); STRESS_SCALE = [];
        r = run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)], trajList(t), [], 1.0, ...
                           struct('NOISE',1,'GE',1,'delay',1), 1);
        R(end+1) = struct('traj',trajList(t),'ic',ic,'sp',r.precise&&r.soft, ...
            'precise',r.precise,'soft',r.soft,'xy',r.final_xy,'v',r.final_rel_vel, ...
            't',r.final_t,'fov',r.fov_fail); %#ok<AGROW>
        sp = sp + (r.precise&&r.soft); fov = fov + r.fov_fail;
        xy(end+1)=r.final_xy; vv(end+1)=r.final_rel_vel; tt(end+1)=r.final_t; %#ok<AGROW>
        fprintf('  %-11s IC%d -> SP%d P%d S%d fov%d  xy%.4f v%.4f t%.2f\n', ...
            trajList(t), ic, r.precise&&r.soft, r.precise, r.soft, r.fov_fail, ...
            r.final_xy, r.final_rel_vel, r.final_t);
    end
end
fprintf('-----------------------------------------------------------------\n');
fprintf('  SP %d/25 | FoV %d | meanXY %.4f worstXY %.4f | mean|v| %.4f worst|v| %.4f | mean t_f %.2f\n', ...
    sp, fov, mean(xy), max(xy), mean(vv), max(vv), mean(tt));
fprintf('  PASS: %s\n', string(sp==25 && fov==0));
save(fullfile(here,'gate_cross_parity.mat'),'R','sp','fov','xy','vv','tt');
clear global VDF_OVERRIDE STRESS_SCALE
