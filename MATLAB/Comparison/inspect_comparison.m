%% inspect_comparison.m — per-controller failure diagnostics
% Loads <traj>_comparison.mat and prints key telemetry per controller:
%   - final position, final velocity, final pos/vel relative to target
%   - idx (termination step)
%   - peak lateral thrust cmd, peak torque cmd
%   - NOTE: Lin=xi_p/xi_v saturation, Zhang=descent rate,
%           Chen=depth estimate, Cho=Kv-saturation diagnostic

clc;
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Common'));
trajList = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"];

for ti = 1:numel(trajList)
    traj = trajList(ti);
    f = fullfile(fileparts(mfilename('fullpath')), 'Datasets', sprintf('%s_comparison.mat', traj));
    if ~isfile(f), continue; end
    S = load(f);
    fprintf('\n==========  %s  ==========\n', traj);
    fprintf('Ctrl  Name           idx    t[s]   p_xy[m]  v_xy[m/s]  v_z[m/s]  maxT[N]  max|tau|[Nm]\n');
    for c = 1:5
        r = S.all_results(c);
        if isempty(r.data), continue; end
        d = r.data;
        idx = d.idx;
        t_end = d.tRange(idx);
        p_uav = d.X_DS(1:3, idx);
        v_uav = d.X_DS(8:10, idx);
        p_tgt = d.x_t(1:3, idx);
        v_tgt = d.dx_t(1:3, idx);
        p_rel = p_uav - p_tgt;
        v_rel = v_uav - v_tgt;
        maxT   = max(abs(d.U_DS(4, 1:idx)));
        maxTau = max(max(abs(d.U_DS(1:3, 1:idx))));
        fprintf('  %d   %-13s %4d  %6.2f   %6.3f   %6.3f     %6.3f    %5.2f    %5.3f\n', ...
            c, r.ctrl_name, idx, t_end, ...
            norm(p_rel(1:2)), norm(v_rel(1:2)), v_rel(3), maxT, maxTau);
    end
end
