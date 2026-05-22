% Phase 3 — IC sensitivity test for MATLAB
%
% Tests whether MATLAB still hits soft+precise when given the same kind of
% IC variance PX4 SITL exhibits.  If yes → PX4's IC sensitivity (per
% feedback_instability_mechanism.md) is amplified by loop lag, not
% intrinsic to the controller.  If MATLAB starts missing SP at PX4-style
% IC noise, the gap is partly IC-sensitivity + partly lag.
%
% PX4 IC variance (measured 2026-05-21, LOOSE config, n=10):
%   vh_mean: 0.089 m/s    vh_std: 0.055 m/s   range [0.04, 0.22]
%   vh_max  was 0.10-0.25 m/s depending on rep
%
% Sweep: nominal IC (0,0,-5) plus random velocity perturbation drawn from
% N(0, 0.15) m/s on each of x/y axes (matches PX4 vh distribution).

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|IC_VEL_OVERRIDE|WS).*$';
close all; clc;

OUTDIR = '../Datasets/Phase3/';
if ~exist(OUTDIR, 'dir'); mkdir(OUTDIR); end
old = dir([OUTDIR 'rep_*.mat']); for k=1:numel(old); delete([OUTDIR old(k).name]); end
if exist([OUTDIR 'done.flag'], 'file'); delete([OUTDIR 'done.flag']); end

global IC_OVERRIDE NOISE_OVERRIDE IC_VEL_OVERRIDE WS;

WS = struct();
WS.outdir = OUTDIR;
WS.N = 15;                                 % 15 reps with random IC velocity
rng(42);                                   % reproducible velocity samples
WS.vel_samples = randn(WS.N, 3) * 0.15;    % N(0, 0.15) m/s on x,y,z
WS.vel_samples(:, 3) = 0;                  % zero vertical perturbation

for r = 1:WS.N
  fprintf('\n=== Phase 3 rep %d/%d  IC_vel=[%.3f,%.3f,%.3f] m/s ===\n', ...
          r, WS.N, WS.vel_samples(r, 1), WS.vel_samples(r, 2), 0.0);

  IC_OVERRIDE     = [0; 0; -5];
  NOISE_OVERRIDE  = 1;                     % 50 dB noise on
  IC_VEL_OVERRIDE = WS.vel_samples(r, :)';

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
  catch ME
    rep_err = sprintf('%s (%s)', ME.message, ME.identifier);
    fprintf('  ERR: %s\n', rep_err);
  end

  global IC_OVERRIDE NOISE_OVERRIDE IC_VEL_OVERRIDE WS;

  if ~exist('rep_landed', 'var'); rep_landed = 0; end
  if ~exist('rep_xy',     'var'); rep_xy     = NaN; end
  if ~exist('rep_vel',    'var'); rep_vel    = NaN; end
  if ~exist('rep_alt',    'var'); rep_alt    = NaN; end
  if ~exist('rep_t',      'var'); rep_t      = NaN; end
  if ~exist('rep_prec',   'var'); rep_prec   = 0;   end
  if ~exist('rep_soft',   'var'); rep_soft   = 0;   end
  if ~exist('rep_err',    'var'); rep_err    = '';  end

  cr = WS.cur_r;
  vh_inj = norm(WS.vel_samples(cr, 1:2));
  fname = sprintf('%srep_%02d.mat', WS.outdir, cr);
  save(fname, 'rep_landed', 'rep_xy', 'rep_vel', 'rep_alt', 'rep_t', ...
              'rep_prec', 'rep_soft', 'rep_err', 'vh_inj');
  fprintf('  → vh_inj=%.3f  landed=%d  t=%.2fs  xy=%.4f  vel=%.4f  P=%d S=%d\n', ...
          vh_inj, rep_landed, rep_t, rep_xy, rep_vel, rep_prec, rep_soft);
  r = cr;
end

% Aggregate
global WS;
OUTDIR = WS.outdir;
results = struct('rep', {}, 'vh_inj', {}, 'landed', {}, 't_land', {}, ...
                 'alt_above', {}, 'xy_err', {}, 'rel_vel', {}, ...
                 'precise', {}, 'soft', {});
files = dir([OUTDIR 'rep_*.mat']);
for k = 1:numel(files)
  m = regexp(files(k).name, '^rep_(\d+)\.mat$', 'tokens', 'once');
  if isempty(m); continue; end
  d = load([OUTDIR files(k).name]);
  results(end+1) = struct('rep', str2double(m{1}), 'vh_inj', d.vh_inj, ...
                          'landed', d.rep_landed, 't_land', d.rep_t, ...
                          'alt_above', d.rep_alt, 'xy_err', d.rep_xy, ...
                          'rel_vel', d.rep_vel, 'precise', d.rep_prec, ...
                          'soft', d.rep_soft); %#ok<SAGROW>
end

fprintf('\n=================== PHASE 3 SUMMARY ===================\n');
xy   = [results.xy_err];
vel  = [results.rel_vel];
vh   = [results.vh_inj];
prec = sum([results.precise]);
soft = sum([results.soft]);
sp   = sum([results.precise] & [results.soft]);
fprintf('n=%d  vh_inj_mean=%.3f m/s  vh_inj_max=%.3f m/s\n', ...
        numel(results), mean(vh), max(vh));
fprintf('xy_mean=%.4f  xy_std=%.4f  xy_min=%.4f  xy_max=%.4f\n', ...
        mean(xy, 'omitnan'), std(xy, 'omitnan'), ...
        min(xy, [], 'omitnan'), max(xy, [], 'omitnan'));
fprintf('vel_mean=%.4f m/s\n', mean(vel, 'omitnan'));
fprintf('PREC=%d/%d  SOFT=%d/%d  SP=%d/%d\n', ...
        prec, numel(results), soft, numel(results), sp, numel(results));

% Correlate xy_err with injected vh — does IC velocity predict outcome?
fprintf('\nCorrelation analysis (does IC velocity predict outcome?)\n');
if std(vh) > 0
    rho = corrcoef(vh, xy);
    fprintf('  ρ(vh_inj, xy_err) = %+.3f\n', rho(1,2));
end

save([OUTDIR 'phase3_summary.mat'], 'results');
fprintf('\nSaved: %sphase3_summary.mat\n', OUTDIR);
fid = fopen([OUTDIR 'done.flag'], 'w');
if fid ~= -1; fprintf(fid, '%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS')); fclose(fid); end
