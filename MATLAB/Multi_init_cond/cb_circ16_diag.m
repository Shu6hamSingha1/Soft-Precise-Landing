%% Diagnose the Circular@IC2 x1.6 breach with the 2x marker. Saves full logs (scratch, gitignored)
% for: A = 2x marker @1.6 (fails), B = 2x @1.5 (lands), C = 1x @1.6 (lands), D = 2x @1.6 NOISE=0.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE MARKER_SCALE   %#ok<GVMIS>
VDF_OVERRIDE = struct('theta_per_axis', true);
x0 = [2; 2; -5; 1; 0; 0; 0; zeros(3,1); zeros(3,1)];
cases = struct('tag', {'A','B','C','D'}, 'sc', {[] , [], 0.5, []}, 'm', {1.6, 1.5, 1.6, 1.6}, 'noise', {1, 1, 1, 0});
outDir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit');
for k = 1:numel(cases)
    MARKER_SCALE = cases(k).sc;
    cfg = struct('NOISE', cases(k).noise, 'GE', 1, 'delay', 1);
    r = run_simulation(x0, "Circular", [], cases(k).m, cfg, 1);
    fprintf('CASE %s: success=%d fail_t=%.2f t_f=%.2f xy=%.3f\n', cases(k).tag, r.success, r.fov_fail_t, r.final_t, r.final_xy);
    save(fullfile(outDir, ['_diag_c16_' cases(k).tag '.mat']), 'r', '-v7');
end
MARKER_SCALE = [];
