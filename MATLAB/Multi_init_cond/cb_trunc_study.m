%% Truncation study: closed-loop outcome AND optic-flow estimate quality vs pinv_tol, marker size, pixel noise.
% Env: NOISEFIX=0/1 (global PX_NOISE_FIX: documented pixel noise vs legacy /f), SCALE (old-style multiplier; sets the absolute
% global MARKER_SCALE = 2*SCALE, x the 12 cm cross: 0.5 -> 1x marker, 1 -> 2x, 13 -> 26x = InitVar default), TOLS, SEEDS. Cases: Circular x1.4 and Linear x1.4 at IC2 (realistic cfg).
% Estimator metrics over t in [0.3, min(t_end,8)]: LS slope of measured h_x vs analytic, rms|h_xy err|/rms|h_xy true|,
% rms(w_z err)/rms(w_z true). Saves nothing.
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
global VDF_OVERRIDE MARKER_SCALE PX_NOISE_FIX PX_OUTLIER_OFF PX_NOISE_PARAMS PX_EXACT_PERSP PX_CENTER_FEATURE   %#ok<GVMIS>
PX_CENTER_FEATURE = ~strcmp(getenv('CENTER'), '0');   % CENTER=1 -> marker-centre feature (arm intersection)
PX_EXACT_PERSP = ~strcmp(getenv('EXACTP'), '0');   % EXACTP=1 -> exact per-point pinhole projection
if strcmp(getenv('PXNOISE'), '1'), PX_NOISE_PARAMS = [0.027 0.175 0.5 0 0]; PX_NOISE_FIX = true; else, PX_NOISE_PARAMS = []; end   % measured PX4 model
PX_OUTLIER_OFF = strcmp(getenv('OUTLIERS'), '0');   % OUTLIERS=0 -> Gaussian noise only
nfv = str2double(getenv('NOISEFIX')); if strcmp(getenv('PXNOISE'),'1'), nfv = 1; end; if isnan(nfv) || nfv == 0, nf = 0; PX_NOISE_FIX = false; else, nf = nfv; PX_NOISE_FIX = nfv; end   % numeric = x documented level
sc = str2double(getenv('SCALE')); if isnan(sc), sc = 1; end;  MARKER_SCALE = 2*sc;
tols  = str2num(getenv('TOLS'));  if isempty(tols),  tols = [1 4 8]; end %#ok<ST2NM>
seeds = str2num(getenv('SEEDS')); if isempty(seeds), seeds = [1 2 3]; end %#ok<ST2NM>
cfg = struct('NOISE', 1, 'GE', 1, 'delay', 1);
z0 = str2double(getenv('Z0')); if isnan(z0), z0 = 5; end
x0  = [2; 2; -z0; 1; 0; 0; 0; zeros(3,1); zeros(3,1)];
tj = getenv('TRAJS'); if isempty(tj), tj = 'Circular Linear'; end
tn = strsplit(strtrim(tj)); cases = cell(numel(tn),2);
for q = 1:numel(tn), cases{q,1} = string(tn{q}); if strcmp(tn{q},'Static'), cases{q,2} = 1.0; else, cases{q,2} = 1.4; end, end
fprintf('### pixel-noise fix=%d  marker=%gx original\n', nf, 2*sc);
fprintf('%-9s %-5s %-5s %-4s %-6s %-7s %-7s %-7s | %-8s %-9s %-9s\n', ...
    'traj','tol','seed','SP','fail_t','t_f','xy[m]','v[m/s]','hx_slope','hxy_err/sig','wz_err/sig');
for c = 1:size(cases,1)
  for tol = tols
    for sd = seeds
        VDF_OVERRIDE = struct('theta_per_axis', true, 'pinv_tol', tol, 'flow_reduced', ~strcmp(getenv('REDUCED'), '0'), 'flow_omega_corr', ~strcmp(getenv('OMCORR'), '0'), 'flow_gyro_variant', max(1,str2double(getenv('GV'))));
        r = run_simulation(x0, cases{c,1}, [], cases{c,2}, cfg, sd);
        d = r.data; idx = d.idx; if idx <= 0, idx = numel(d.e_a_log); end
        t = d.tRange(1:idx); s = (t >= 0.3) & (t <= 8);
        hm = d.V_X_DS(4:5,1:idx);  ht = d.V_X_DS(16:17,1:idx);
        wm = d.V_X_DS(9,1:idx);    wt = d.V_X_DS(21,1:idx);
        hx_slope = (hm(1,s)*ht(1,s)') / (ht(1,s)*ht(1,s)');
        hxy = sqrt(mean(sum((hm(:,s)-ht(:,s)).^2,1))) / sqrt(mean(sum(ht(:,s).^2,1)));
        wz  = sqrt(mean((wm(s)-wt(s)).^2)) / sqrt(mean(wt(s).^2));
        fprintf('%-9s %-5g %-5d %-4d %-6.2f %-7.2f %-7.3f %-7.3f | %-8.2f %-9.2f %-9.2f\n', cases{c,1}, tol, sd, ...
            r.success && r.soft && r.precise, r.fov_fail_t, r.final_t, r.final_xy, r.final_rel_vel, hx_slope, hxy, wz);
    end
  end
end
