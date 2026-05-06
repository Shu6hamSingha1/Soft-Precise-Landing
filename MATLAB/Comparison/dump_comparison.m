trajList = {'Static','Linear','Sinusoidal','Lissajous','Circular'};
for ti = 1:numel(trajList)
    traj = trajList{ti};
    f = fullfile(fileparts(mfilename('fullpath')),'Datasets',sprintf('%s_comparison.mat',traj));
    if ~isfile(f), continue; end
    S = load(f);
    fprintf('=== %s ===\n', traj);
    for c = 1:5
        r = S.all_results(c);
        if isempty(r.data), continue; end
        d = r.data;
        idx = d.idx;
        t_end = d.tRange(idx);
        xy_e = norm(d.X_DS(1:2,idx) - d.x_t(1:2,idx));
        z_f  = -d.X_DS(3,idx);
        v_rel = norm(d.X_DS(8:10,idx) - d.dx_t(1:3,idx));
        fprintf('  %d %-18s t=%6.2f xy=%.3f z=%.3f vrel=%.3f\n', ...
                c, r.ctrl_name, t_end, xy_e, z_f, v_rel);
    end
end
