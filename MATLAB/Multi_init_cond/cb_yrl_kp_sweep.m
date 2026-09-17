%% Quick yrl_kp grid search for the Circular@IC2@{1.2,1.4}x FoV breach.
% Uses run_simulation's K_override to test gain values WITHOUT touching
% vdf_params.m -- temporary diagnostic script, not part of the gate.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));

p0 = [2, 2, -5];
q0 = [1; 0; 0; 0];
v0 = zeros(3,1);
w0 = zeros(3,1);
x0 = [p0(:); q0; v0; w0];

cfg_override = struct('NOISE', 1, 'GE', 1, 'delay', 1);
kp_grid = [0.02, 0.05, 0.08, 0.10, 0.15, 0.20];
mults   = [1.2, 1.4, 1.5, 1.6, 1.8, 2.0];

fprintf('%-8s %-6s %-8s %-8s %-8s %-8s %-8s %-8s %-10s\n', ...
    'yrl_kp', 'mult', 'landed', 'precise', 'soft', 't_f', 'xy', 'v_rel', 'max|e_a|deg');
for kp = kp_grid
    for m = mults
        K_override = struct('yaw_rate_law', 1, 'yrl_kp', kp);
        r = run_simulation(x0, "Circular", K_override, m, cfg_override, 1);
        if ~isempty(r.data) && isfield(r.data, 'e_a_log')
            idx = r.data.idx; if idx <= 0, idx = numel(r.data.e_a_log); end
            max_ea = max(abs(r.data.e_a_log(1:idx))) * 180/pi;
        else
            max_ea = NaN;
        end
        fprintf('%-8.2f %-6.1f %-8d %-8d %-8d %-8.2f %-8.4f %-8.4f %-10.1f\n', ...
            kp, m, r.success, r.precise, r.soft, r.final_t, r.final_xy, r.final_rel_vel, max_ea);
    end
end
