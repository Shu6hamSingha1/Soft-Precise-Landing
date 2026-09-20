%% Speed-envelope probe (IC2, realistic, seed 1, final formulation, 2x marker baked in InitVar).
% Env MULTS = TOTAL target-speed multipliers to test (vs the ORIGINAL nominal speed), e.g. "1.6 1.8 2.0".
% Runs all 4 moving trajectories at every multiplier and prints pass/fail per cell + barrier-violation
% depth, so the highest nominal speed N with a full +-40% sweep (0.6N..1.4N) can be read off.
% Saves nothing. Not part of the gate.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE MARKER_SCALE       %#ok<GVMIS>
sc = str2double(getenv('SCALE')); if isnan(sc), MARKER_SCALE = []; sc = 1; else, MARKER_SCALE = sc; end   % hook, ON TOP of the baked 2x
trajs = strsplit(strtrim(getenv('TRAJS'))); if isempty(trajs{1}), trajs = {'Linear','Sinusoidal','Lissajous','Circular'}; end
fprintf('### extra marker hook scale=%g (total marker = %gx original)\n', sc, 2*sc);
VDF_OVERRIDE = struct('theta_per_axis', true);
mults = str2num(getenv('MULTS')); %#ok<ST2NM>
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
x0  = [2; 2; -5; 1; 0; 0; 0; zeros(3,1); zeros(3,1)];
fprintf('%-11s %-6s %-6s %-6s %-6s %-7s %-8s %-8s %-9s %-8s\n', ...
    'traj','mult','SP','landed','fail_t','t_f','xy[m]','v[m/s]','term|e_a|','viol[px]');
for t = string(trajs)
    for m = mults
        r = run_simulation(x0, t, [], m, cfg, 1);
        d = r.data; idx = d.idx; if idx <= 0, idx = numel(d.e_a_log); end
        bf = d.P.cbf_buffer_frac; cc = d.cen_px_log(:,1:idx);
        vd = max(0, max(max(abs(cc(1,:)) - (d.P.res(1)/2)*(1-bf), abs(cc(2,:)) - (d.P.res(2)/2)*(1-bf))));
        fprintf('%-11s %-6.2f %-6d %-6d %-6.2f %-7.2f %-8.4f %-8.4f %-9.1f %-8.1f\n', t, m, ...
            r.success && r.soft && r.precise, r.success, r.fov_fail_t, r.final_t, r.final_xy, ...
            r.final_rel_vel, abs(d.e_a_log(idx))*180/pi, vd);
    end
end
