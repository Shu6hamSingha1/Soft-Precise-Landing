% Capture a single MATLAB landing run's sliding-variable trajectory plus a
% few companion signals so we can compare against PX4 Control_Data['sigma(t)'].
%
% Saves a small .mat file (just the time-series we need, ~50 KB) suitable
% for committing alongside Phase 1 results.
%
% Run from MATLAB/Multi_init_cond:  >> capture_sigma_trace
% Output: ../Datasets/Phase3/matlab_sigma_trace.mat

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|IC_VEL_OVERRIDE|WS).*$';
close all; clc;

global IC_OVERRIDE NOISE_OVERRIDE IC_VEL_OVERRIDE H_RD_OVERRIDE;
IC_OVERRIDE     = [0; 0; -5];    % PX4 IC1
NOISE_OVERRIDE  = 1;             % 50 dB pixel noise (matches PX4)
IC_VEL_OVERRIDE = [0; 0; 0];     % nominal IC velocity (no perturbation)
H_RD_OVERRIDE   = -0.70;         % match PX4's aggressive descent (was -0.42 default)

fprintf('Running MATLAB sim at PX4 IC for σ trajectory capture...\n');
visualControl_IBVS_adaptive;

% Final outcome (for sanity)
xy_err  = norm(I_p_c(1:2) - x_t(1:2,idx));
rel_vel = norm(I_v_c - dx_t(1:3,idx));
fprintf('  landed=%d  t=%.2fs  xy=%.4f  vel=%.4f\n', landed, tRange(idx), xy_err, rel_vel);

% Per-step time series we want to compare.  The canonical script keeps these
% in (channel, time) layout — typically (3, N_steps).
OUTDIR = '../Datasets/Phase5/';
if ~exist(OUTDIR, 'dir'); mkdir(OUTDIR); end
out_path = [OUTDIR 'matlab_sigma_h070.mat'];

% Save the variables we want.  Time vector is tRange(1:idx) (the actual
% simulated landing range).
n = idx;
t_log = tRange(1:n);

% Find the sliding-variable log (sigma or s_e).  In the canonical script
% it goes into U_DS or a separate sigma log.  We use whatever's available.
to_save = struct();
to_save.t        = t_log;
to_save.xy_err   = xy_err;
to_save.rel_vel  = rel_vel;
to_save.IC       = [0; 0; -5];
to_save.landed   = landed;
to_save.t_land   = tRange(idx);

% Look for any per-step variables that look like sliding/feature-error logs.
candidates = {'sigma_log', 'sigma', 'sigma_t', 'zeta_log', 'zeta', ...
              'V_h_log', 'V_h', 'h_log', 'h', 'V_s_log', 'V_s', ...
              's_e_log', 's_e', 'a_v_log', 'a_v', 'a_u_log', 'a_u', ...
              'I_a_log', 'I_a', 'I_a_cd', 'I_a_cd_log', ...
              'kappa_log', 'kappa', 'kappa_a_log', 'kappa_a', ...
              'u_a_log', 'u_a', 'e_R_log', 'e_R', 'B_T_log', 'B_T'};
fprintf('\nCapturing available trajectory channels:\n');
for k = 1:numel(candidates)
    nm = candidates{k};
    if evalin('base', sprintf('exist(''%s'', ''var'')', nm))
        try
            val = evalin('base', nm);
            if isnumeric(val) && size(val, 2) >= n
                to_save.(nm) = val(:, 1:n);
                fprintf('  %s  size=%s  (kept)\n', nm, mat2str(size(val(:, 1:n))));
            elseif isnumeric(val) && size(val, 1) >= n && size(val, 2) <= 8
                to_save.(nm) = val(1:n, :);
                fprintf('  %s  size=%s  (kept, transposed)\n', nm, mat2str(size(val(1:n, :))));
            end
        catch
        end
    end
end

save(out_path, '-struct', 'to_save');
fprintf('\nSaved: %s\n', out_path);
fid = fopen([OUTDIR 'sigma_done.flag'], 'w');
if fid ~= -1; fprintf(fid, '%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS')); fclose(fid); end
