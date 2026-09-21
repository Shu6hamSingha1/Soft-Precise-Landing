%% Marker-size experiment: does a bigger cross (MARKER_SCALE) change the Circular@IC2
% x1.4 result under three yaw configs? Also reports how much of the true image yaw
% rate w_z the estimator delivers early (t in [0.2,1.5]s) -- the pinv_tol starvation.
% Scratch results go to Datasets/MultiInit/_diag_mk_*.mat (gitignored). Not part of the gate.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE MARKER_SCALE   %#ok<GVMIS>

x0  = [2; 2; -5; 1; 0; 0; 0; zeros(3,1); zeros(3,1)];
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
cfgs = struct('name', {'kp0.02 ff0','kp0.30 ff0','kp0.30 ff1'}, ...
              'kp',   {0.02, 0.30, 0.30}, 'ff', {false, false, true});
scales = [1 2];

fprintf('%-6s %-11s %-7s %-6s %-6s %-8s %-8s %-10s %-9s | %-9s %-9s %-9s\n', ...
    'scale','yaw cfg','landed','soft','prec','t_f','xy[m]','max|e_a|','term|e_a|','wz_seen','wz_true','viol[px]');
for s = scales
    for c = 1:numel(cfgs)
        MARKER_SCALE = 2*s;  % s = old multiplier on the 2x base
        VDF_OVERRIDE = struct('theta_per_axis', true, 'yaw_omega_d_ff', cfgs(c).ff);
        r = run_simulation(x0, "Circular", struct('yaw_rate_law',1,'yrl_kp',cfgs(c).kp), 1.4, cfg, 1);
        d = r.data; idx = d.idx; if idx <= 0, idx = numel(d.e_a_log); end
        t  = d.tRange(1:idx); w = (t >= 0.2) & (t <= 1.5);
        wzs = d.V_w_log(3,1:idx);  wzt = d.V_X_DS(21,1:idx);
        wz_seen = mean(abs(wzs(w)));
        wz_true = mean(abs(wzt(w)));
        bf = d.P.cbf_buffer_frac;  cc = d.cen_px_log(:,1:idx);      % CBF-buffer violation depth [px]
        vd = max(0, max(max(abs(cc(1,:)) - (d.P.res(1)/2)*(1-bf), abs(cc(2,:)) - (d.P.res(2)/2)*(1-bf))));
        fprintf('%-6g %-11s %-7d %-6d %-6d %-8.2f %-8.4f %-10.1f %-9.1f | %-9.3f %-9.3f %-9.1f\n', s, cfgs(c).name, ...
            r.success, r.soft, r.precise, r.final_t, r.final_xy, ...
            max(abs(d.e_a_log(1:idx)))*180/pi, abs(d.e_a_log(idx))*180/pi, wz_seen, wz_true, vd);
        save(fullfile(mfile_dir,'..','Datasets','MultiInit', sprintf('_diag_mk_%d_%d.mat', s, c)), 'r', '-v7');
    end
end
MARKER_SCALE = [];  VDF_OVERRIDE = struct('theta_per_axis', true);
