%% Verify the manuscript's target-yaw-rate-sweep claim (0.7-0.9 rad/s landing
% ceiling) under the CURRENT yrl_kp=0.02 gain. Uses the TARGET_YAW_RATE test
% hook (traj_Gen.m:101,177) to decouple platform yaw rate from translation on
% "Circular" (Case 5) and "CircularYaw" (the manuscript's "independent-yaw
% circular variant"), from the centred IC1, matching manuscript.tex's own
% stated setup. Temporary diagnostic script, not part of the gate.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));

global VDF_OVERRIDE       %#ok<GVMIS>
VDF_OVERRIDE.theta_per_axis = true;

global TARGET_YAW_RATE    %#ok<GVMIS>

p0 = [0, 0, -5];   % centred IC1, matches manuscript's stated setup for this sweep
q0 = [1; 0; 0; 0];
v0 = zeros(3,1);
w0 = zeros(3,1);
x0 = [p0(:); q0; v0; w0];

cfg_override = struct('NOISE', 1, 'GE', 1, 'delay', 1);
w_grid = [0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5];
trajs  = ["Circular", "CircularYaw"];

fprintf('%-12s %-6s %-8s %-8s %-8s %-8s %-10s\n', ...
    'traj', 'w_tz', 'landed', 'precise', 'soft', 't_f', 'max|e_a|deg');
for traj = trajs
    for w = w_grid
        TARGET_YAW_RATE = w;
        r = run_simulation(x0, traj, [], 1.0, cfg_override, 1);
        if ~isempty(r.data) && isfield(r.data, 'e_a_log')
            idx = r.data.idx; if idx <= 0, idx = numel(r.data.e_a_log); end
            max_ea = max(abs(r.data.e_a_log(1:idx))) * 180/pi;
            term_ea = abs(r.data.e_a_log(idx)) * 180/pi;
        else
            max_ea = NaN; term_ea = NaN;
        end
        fprintf('%-12s %-6.1f %-8d %-8d %-8d %-8.2f %-10.1f (term %.1f)\n', ...
            traj, w, r.success, r.precise, r.soft, r.final_t, max_ea, term_ea);
    end
end
TARGET_YAW_RATE = [];
