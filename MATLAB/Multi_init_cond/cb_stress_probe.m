%% CB_STRESS_PROBE  Does engagement buy REAL disturbance-rejection margin?
%
% Motivation (2026-06-26): the retune is NOT for the figure. In nominal MATLAB the
% errors are so tiny that kappa leakage-decays and the funnels sit idle, so the
% adaptive/barrier stack is dormant. Real PX4/Gazebo disturbances are far larger;
% a dormant adaptive law cannot reject them. This probe asks the two questions that
% decide whether the engaged config is actually MORE robust:
%
%   Q1  Does kappa ADAPT to disturbance now?  (prior handoff: noiseless-vs-noisy
%       kappa gap < 0.005 = did NOT adapt). Compare candidate kappa noiseless vs noisy.
%   Q2  Does engagement buy REJECTION MARGIN?  Run BAKED vs CANDIDATE under 3x
%       amplified disturbance (STRESS_SCALE=3). Does baked fail where candidate holds?
%
% Run from MATLAB:  cd MATLAB/Multi_init_cond; cb_stress_probe
% Saves: ../Datasets/MultiInit/stress_probe.mat
clc; clear;

mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

% ----- the two controller configs (both keep per-axis theta = current law) -----
baked = struct('theta_per_axis', true);
cand  = struct('theta_per_axis', true, ...
               'Xi_h',   diag([0.4,0.4,0.4]), ...
               'p_hinf', [0.6;0.6;1.0], ...
               'Xi_r',   diag([0.2,0.2]), ...
               'p_rinf', [0.85;0.85], ...
               'kappa0', [0.05;0.05;0.10], ...
               'N',      diag([0.06,0.06,0.06]));

% Offset / engaging cells across two trajectories (skip benign centered IC1).
cells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

fprintf('\n=== STRESS PROBE  (3x amplified disturbance) ===\n');
S = struct('traj',{},'ic',{}, 'kp_clean',{},'kp_noisy',{},'kgap',{}, ...
           'cand3_sp',{},'cand3_xy',{},'cand3_v',{}, ...
           'baked3_sp',{},'baked3_xy',{},'baked3_v',{});
for c = 1:size(cells,1)
    traj = cells{c,1}; ic = cells{c,2};

    % Q1: kappa adaptation = candidate clean vs noisy (nominal disturbance)
    rc = onerun(cand, p0, traj, ic, 0, 1);   kpc = kappapeak(rc);
    rn = onerun(cand, p0, traj, ic, 1, 1);   kpn = kappapeak(rn);
    kgap = kpn - kpc;

    % Q2: rejection margin at 3x disturbance, baked vs candidate
    rcs = onerun(cand,  p0, traj, ic, 1, 3);
    rbs = onerun(baked, p0, traj, ic, 1, 3);

    e = numel(S)+1;
    S(e).traj=traj; S(e).ic=ic;
    S(e).kp_clean=kpc; S(e).kp_noisy=kpn; S(e).kgap=kgap;
    S(e).cand3_sp = rcs.precise&&rcs.soft;  S(e).cand3_xy=rcs.final_xy; S(e).cand3_v=rcs.final_rel_vel;
    S(e).baked3_sp= rbs.precise&&rbs.soft;  S(e).baked3_xy=rbs.final_xy; S(e).baked3_v=rbs.final_rel_vel;

    fprintf(['%-11s IC%d | kappa clean=[%.2f %.2f %.2f] noisy=[%.2f %.2f %.2f] gap=[%.3f %.3f %.3f]\n' ...
             '            3x-stress  CAND SP=%d (xy=%.3f v=%.3f)   BAKED SP=%d (xy=%.3f v=%.3f)\n'], ...
        traj, ic, kpc, kpn, kgap, ...
        S(e).cand3_sp, rcs.final_xy, rcs.final_rel_vel, ...
        S(e).baked3_sp, rbs.final_xy, rbs.final_rel_vel);
end

clear global VDF_OVERRIDE STRESS_SCALE   % restore defaults
fprintf('\n--- 3x-stress SP:  CANDIDATE %d/%d   BAKED %d/%d ---\n', ...
    sum([S.cand3_sp]), numel(S), sum([S.baked3_sp]), numel(S));
fprintf('--- mean kappa adaptation gap (noisy-clean): [%.3f %.3f %.3f] ---\n', ...
    mean(reshape([S.kgap],3,[]),2));

outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Datasets', 'MultiInit');
save(fullfile(outDir,'stress_probe.mat'), 'S', 'cells', 'cand', 'baked');
fprintf('\nSaved -> Datasets/MultiInit/stress_probe.mat\n');

% ============================= local functions ===============================
function r = onerun(cfgOv, p0, traj, ic, NOISE, stress)
    global VDF_OVERRIDE STRESS_SCALE      %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv;
    STRESS_SCALE = stress;
    x0 = [p0(ic,:)'; 1;0;0;0; zeros(3,1); zeros(3,1)];
    co = struct('NOISE',NOISE,'GE',1,'delay',1);
    r  = run_simulation(x0, traj, [], 1.0, co, 1);
end

function kp = kappapeak(r)
    kp = max(r.data.kappa_log(:,1:r.data.idx), [], 2)';   % 1x3
end
