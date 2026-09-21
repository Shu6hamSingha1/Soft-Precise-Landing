%% Full-suite check of the tracker yaw-rate feedforward at yrl_kp=0.3:
%  25 multi-init (5 traj x 5 IC, speed 1.0) + 20 speed sweep (4 traj x 5 mult, IC2),
%  realistic config, seed 1. Saves NOTHING (no dataset clobbering); prints a table.
%  cfg via FF env: FF=1 (default) feedforward on, FF=0 control.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE MARKER_SCALE      %#ok<GVMIS>
sc = str2double(getenv('SCALE'));  if isnan(sc), sc = 1; end, MARKER_SCALE = 2*sc;   % SCALE = old multiplier on the 2x base; MARKER_SCALE is now absolute
ff  = ~strcmp(getenv('FF'), '0');
kp  = str2double(getenv('KP')); if isnan(kp), kp = 0.3; end
VDF_OVERRIDE = struct('theta_per_axis', true, 'yaw_omega_d_ff', ff);
K   = struct('yaw_rate_law', 1, 'yrl_kp', kp);
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
q0  = [1;0;0;0]; z3 = zeros(3,1);
fprintf('### FF=%d  yrl_kp=%.3f  marker scale=%g\n', ff, kp, sc);

% ---- 25 multi-init ----
p0s = [0 0 -5; 2 2 -5; 2 -2 -5; 2 2 -7; 2 2 -3];
trajs = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
ok = 0; wxy = 0; wv = 0; wea = 0; n = 0; fails = {}; nv = 0; wvd = 0; wvt = 0;
for t = trajs
    for i = 1:5
        r = run_simulation([p0s(i,:)'; q0; z3; z3], t, K, 1.0, cfg, 1);
        d = r.data; idx = d.idx; if idx <= 0, idx = numel(d.e_a_log); end
        ea = abs(d.e_a_log(idx))*180/pi;
        [vd, vt] = barrier_viol(d, idx);  nv = nv + (vd > 0); wvd = max(wvd, vd); wvt = max(wvt, vt);
        n = n+1; ok = ok + (r.success && r.soft && r.precise);
        if r.success, wxy = max(wxy, r.final_xy); wv = max(wv, r.final_rel_vel); wea = max(wea, ea); end
        if ~(r.success && r.soft && r.precise), fails{end+1} = sprintf('%s IC%d', t, i); end %#ok<SAGROW>
    end
end
fprintf('MULTI-INIT  SP %d/%d | worst xy %.4f  worst v %.4f  worst terminal|e_a| %.1f deg | fails: %s\n', ...
    ok, n, wxy, wv, wea, strjoin(fails, ', '));
fprintf('  barrier violations: %d/%d runs | worst depth %.1f px | longest %.2f s\n', nv, n, wvd, wvt);

% ---- 20 speed sweep (IC2) ----
ok = 0; wxy = 0; wv = 0; wea = 0; n = 0; fails = {}; nv = 0; wvd = 0; wvt = 0;
for t = ["Linear","Sinusoidal","Lissajous","Circular"]
    for m = [0.6 0.8 1.0 1.2 1.4]
        r = run_simulation([2;2;-5; q0; z3; z3], t, K, m, cfg, 1);
        d = r.data; idx = d.idx; if idx <= 0, idx = numel(d.e_a_log); end
        ea = abs(d.e_a_log(idx))*180/pi;
        [vd, vt] = barrier_viol(d, idx);  nv = nv + (vd > 0); wvd = max(wvd, vd); wvt = max(wvt, vt);
        n = n+1; ok = ok + (r.success && r.soft && r.precise);
        if r.success, wxy = max(wxy, r.final_xy); wv = max(wv, r.final_rel_vel); wea = max(wea, ea); end
        if ~(r.success && r.soft && r.precise), fails{end+1} = sprintf('%s x%.1f', t, m); end %#ok<SAGROW>
    end
end
fprintf('SPEED SWEEP SP %d/%d | worst xy %.4f  worst v %.4f  worst terminal|e_a| %.1f deg | fails: %s\n', ...
    ok, n, wxy, wv, wea, strjoin(fails, ', '));
fprintf('  barrier violations: %d/%d runs | worst depth %.1f px | longest %.2f s\n', nv, n, wvd, wvt);

function [depth, dur] = barrier_viol(d, idx)
%BARRIER_VIOL  How far / how long the marker centre sat outside the CBF's buffered safe set.
%   depth [px] = max excess beyond (res/2)*(1-cbf_buffer_frac) on either axis (0 if never);
%   dur [s]    = total time with any excess. Soft objective: reported, not a failure.
    b  = d.P.cbf_buffer_frac;  Lx = (d.P.res(1)/2)*(1-b);  Ly = (d.P.res(2)/2)*(1-b);
    c  = d.cen_px_log(:, 1:idx);
    ex = max(abs(c(1,:)) - Lx, abs(c(2,:)) - Ly);
    depth = max(0, max(ex));  dur = sum(ex > 0) * d.P.dt;
end
