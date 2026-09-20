%% Diagnostic: is the w_z fed to the direct yaw-rate law tracking the true image
% rotation rate? Runs Circular @ IC2 x1.4 (realistic config, seed 1) at
% yrl_kp = 0.3 (the failing config) and 0.02 (current) and saves the results,
% incl. the new V_w_log (smoothed measured V_w actually passed to the law), to
% scratch files under Datasets/MultiInit (gitignored). Not part of the gate.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE       %#ok<GVMIS>
VDF_OVERRIDE.theta_per_axis = true;

x0 = [2; 2; -5; 1; 0; 0; 0; zeros(3,1); zeros(3,1)];
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
outDir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit');

for kp = [0.3, 0.02]
    r = run_simulation(x0, "Circular", struct('yaw_rate_law', 1, 'yrl_kp', kp), 1.4, cfg, 1);
    save(fullfile(outDir, sprintf('_diag_wz_kp%03d.mat', round(kp*100))), 'r', 'kp', '-v7');
end
