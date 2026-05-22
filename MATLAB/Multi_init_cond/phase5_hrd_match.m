% Phase 5 — does MATLAB still hit SP at PX4's aggressive descent rate?
%
% Phase 1 was at MATLAB's canonical h_rd=-0.42 (descent ~19s).  PX4 uses
% h_rd=-0.70 (descent ~4s).  This script tests MATLAB at h_rd=-0.70 to
% see whether the controller still achieves SP when given the same short
% descent budget PX4 has.
%
% n=10 reps with 50 dB pixel noise + the PX4-aligned IC.
% Also captures σ trace from rep 1 for the trajectory diff against PX4.

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|IC_VEL_OVERRIDE|H_RD_OVERRIDE|WS).*$';
close all; clc;

global IC_OVERRIDE NOISE_OVERRIDE IC_VEL_OVERRIDE H_RD_OVERRIDE WS;

OUTDIR = '../Datasets/Phase5/';
if ~exist(OUTDIR, 'dir'); mkdir(OUTDIR); end
old = dir([OUTDIR 'rep_*.mat']); for k=1:numel(old); delete([OUTDIR old(k).name]); end
if exist([OUTDIR 'done.flag'], 'file'); delete([OUTDIR 'done.flag']); end

WS = struct();
WS.outdir = OUTDIR;
WS.N = 10;
WS.h_rd_test = -0.70;            % PX4's aggressive descent

for r = 1:WS.N
  fprintf('\n=== Phase 5 rep %d/%d  h_rd=%.2f  IC=(0,0,-5) ===\n', ...
          r, WS.N, WS.h_rd_test);

  IC_OVERRIDE     = [0; 0; -5];
  NOISE_OVERRIDE  = 1;
  IC_VEL_OVERRIDE = [0; 0; 0];
  H_RD_OVERRIDE   = WS.h_rd_test;
  WS.cur_r = r;

  try
    visualControl_IBVS_adaptive;
    rep_alt    = abs(I_p_c(3) - x_t(3,idx));
    rep_xy     = norm(I_p_c(1:2) - x_t(1:2,idx));
    rep_vel    = norm(I_v_c - dx_t(1:3,idx));
    rep_prec   = double(rep_xy <= 0.08);
    rep_soft   = double(rep_vel <= 0.2);
    rep_landed = double(landed);
    rep_t      = tRange(idx);
    rep_err    = '';
    % For rep 1, capture σ trajectory too
    if r == 1
      n = idx;
      sigma_trace.t       = tRange(1:n);
      sigma_trace.sigma   = sigma(:, 1:n);
      sigma_trace.I_a_cd  = I_a_cd(:, 1:n);
      sigma_trace.kappa   = kappa(:, 1:n);
      sigma_trace.xy_err  = rep_xy;
      sigma_trace.h_rd    = H_RD_OVERRIDE;
      save([WS.outdir 'matlab_sigma_h070.mat'], '-struct', 'sigma_trace');
    end
  catch ME
    rep_err = sprintf('%s (%s)', ME.message, ME.identifier);
    fprintf('  ERR: %s\n', rep_err);
  end

  global IC_OVERRIDE NOISE_OVERRIDE IC_VEL_OVERRIDE H_RD_OVERRIDE WS;
  if ~exist('rep_landed', 'var'); rep_landed = 0;   end
  if ~exist('rep_xy',     'var'); rep_xy     = NaN; end
  if ~exist('rep_vel',    'var'); rep_vel    = NaN; end
  if ~exist('rep_alt',    'var'); rep_alt    = NaN; end
  if ~exist('rep_t',      'var'); rep_t      = NaN; end
  if ~exist('rep_prec',   'var'); rep_prec   = 0;   end
  if ~exist('rep_soft',   'var'); rep_soft   = 0;   end
  if ~exist('rep_err',    'var'); rep_err    = '';  end

  cr = WS.cur_r;
  fname = sprintf('%srep_%02d.mat', WS.outdir, cr);
  save(fname, 'rep_landed', 'rep_xy', 'rep_vel', 'rep_alt', 'rep_t', ...
              'rep_prec', 'rep_soft', 'rep_err');
  fprintf('  → landed=%d  t=%.2fs  xy=%.4f  vel=%.4f  P=%d S=%d\n', ...
          rep_landed, rep_t, rep_xy, rep_vel, rep_prec, rep_soft);
  r = cr;
end

global WS;
OUTDIR = WS.outdir;
results = struct('rep', {}, 'landed', {}, 't_land', {}, 'xy_err', {}, ...
                 'rel_vel', {}, 'precise', {}, 'soft', {});
files = dir([OUTDIR 'rep_*.mat']);
for k = 1:numel(files)
  m = regexp(files(k).name, '^rep_(\d+)\.mat$', 'tokens', 'once');
  if isempty(m); continue; end
  d = load([OUTDIR files(k).name]);
  results(end+1) = struct('rep', str2double(m{1}), 'landed', d.rep_landed, ...
                          't_land', d.rep_t, 'xy_err', d.rep_xy, ...
                          'rel_vel', d.rep_vel, 'precise', d.rep_prec, ...
                          'soft', d.rep_soft); %#ok<SAGROW>
end

fprintf('\n=================== PHASE 5 SUMMARY (h_rd=%.2f) ===================\n', ...
        WS.h_rd_test);
xy = [results.xy_err]; vel = [results.rel_vel];
fprintf('n=%d  xy_mean=%.4f  xy_std=%.4f  xy_min=%.4f  xy_max=%.4f\n', ...
        numel(results), mean(xy, 'omitnan'), std(xy, 'omitnan'), ...
        min(xy, [], 'omitnan'), max(xy, [], 'omitnan'));
fprintf('vel_mean=%.4f  PREC=%d/%d  SOFT=%d/%d  SP=%d/%d\n', ...
        mean(vel, 'omitnan'), sum([results.precise]), numel(results), ...
        sum([results.soft]), numel(results), ...
        sum([results.precise] & [results.soft]), numel(results));
fprintf('t_land_mean=%.2fs (compare h_rd=-0.42: ~19s, PX4 h_rd=-0.70: ~4s)\n', ...
        mean([results.t_land], 'omitnan'));

save([OUTDIR 'phase5_summary.mat'], 'results');
fid = fopen([OUTDIR 'done.flag'], 'w');
if fid ~= -1; fprintf(fid, '%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS')); fclose(fid); end
fprintf('\nSaved: %sphase5_summary.mat\n', OUTDIR);
