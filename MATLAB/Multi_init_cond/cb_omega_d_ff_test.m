%% Test: does a tracker yaw-rate feedforward (Omega_d = R'*[0;0;u_a]) remove the
% Circular@IC2 FoV breach at the ORIGINAL yrl_kp=0.3? Control row (ff off) must
% reproduce the known failure (Circular x1.4 breaks at t=6.69s). Temporary
% diagnostic script; uses VDF_OVERRIDE so vdf_params defaults stay untouched.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE       %#ok<GVMIS>

x0  = [2; 2; -5; 1; 0; 0; 0; zeros(3,1); zeros(3,1)];
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
cases = [ 0 1.4;  1 1.4;  1 1.2;  1 1.6;  1 1.8;  1 2.0 ];   % [ff mult]

fprintf('%-4s %-5s %-7s %-7s %-6s %-7s %-8s %-9s %-10s %-10s\n', ...
    'ff','mult','landed','soft','prec','t_f','xy[m]','v[m/s]','max|e_a|','peak_wz');
for c = 1:size(cases,1)
    ff = cases(c,1);  m = cases(c,2);
    VDF_OVERRIDE = struct('theta_per_axis', true, 'yaw_omega_d_ff', logical(ff));
    r = run_simulation(x0, "Circular", struct('yaw_rate_law',1,'yrl_kp',0.3), m, cfg, 1);
    d = r.data;  idx = d.idx; if idx <= 0, idx = numel(d.e_a_log); end
    fprintf('%-4d %-5.1f %-7d %-7d %-6d %-7.2f %-8.4f %-9.4f %-10.1f %-10.3f\n', ff, m, ...
        r.success, r.soft, r.precise, r.final_t, r.final_xy, r.final_rel_vel, ...
        max(abs(d.e_a_log(1:idx)))*180/pi, max(abs(d.X_DS(13,1:idx))));
    save(fullfile(mfile_dir,'..','Datasets','MultiInit', sprintf('_diag_omegaff_%d_%d.mat',ff,round(m*10))), 'r','-v7');
end
VDF_OVERRIDE = struct('theta_per_axis', true);
