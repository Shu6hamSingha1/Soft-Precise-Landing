% Phase 1 baseline sweep — establish MATLAB ground-truth performance
% at the same IC the PX4 SITL pipeline uses.
%
% Note: visualControl_IBVS_adaptive.m calls `clear` at its top, which
% wipes ALL local variables in our workspace.  To survive that, we keep
% every piece of iteration state in a single global struct WS, and we
% re-declare it as global after every canonical call.

clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE|WS).*$';
close all; clc;

global IC_OVERRIDE NOISE_OVERRIDE WS;

WS = struct();
WS.outdir = '../Datasets/Phase1/';
if ~exist(WS.outdir, 'dir'); mkdir(WS.outdir); end

% Wipe stale artifacts
old = dir([WS.outdir 'rep_*.mat']);
for k = 1:numel(old); delete([WS.outdir old(k).name]); end
if exist([WS.outdir 'done.flag'], 'file'); delete([WS.outdir 'done.flag']); end

WS.batches = struct( ...
  'name',  {'A_PX4ic_noise',  'B_PX4ic_noiseless', 'C_canonical_noise'}, ...
  'IC',    {[ 0.0,  0.0, -5.0], [ 0.0,  0.0, -5.0], [ 2.0,  2.0, -5.0]}, ...
  'noise', {1, 0, 1}, ...
  'N',     {10, 1, 5});

for b = 1:numel(WS.batches)
  for r = 1:WS.batches(b).N
    fprintf('\n=== Batch %s, rep %d/%d  IC=[%.1f,%.1f,%.1f]  noise=%d ===\n', ...
            WS.batches(b).name, r, WS.batches(b).N, ...
            WS.batches(b).IC(1), WS.batches(b).IC(2), WS.batches(b).IC(3), ...
            WS.batches(b).noise);

    IC_OVERRIDE    = WS.batches(b).IC(:);
    NOISE_OVERRIDE = WS.batches(b).noise;
    % Persist loop counters into the global — canonical script's `clear`
    % wipes the loop variables b and r along with all other locals.
    WS.cur_b = b; WS.cur_r = r;

    try
      visualControl_IBVS_adaptive;
      % Variables now in workspace: idx, tRange, I_p_c, x_t, dx_t, I_v_c, zf, landed
      rep_alt    = abs(I_p_c(3) - x_t(3,idx));
      rep_xy     = norm(I_p_c(1:2) - x_t(1:2,idx));
      rep_vel    = norm(I_v_c - dx_t(1:3,idx));
      rep_prec   = double(rep_xy <= 0.08);
      rep_soft   = double(rep_vel <= 0.2);
      rep_landed = double(landed);
      rep_t      = tRange(idx);
    catch ME
      rep_err = sprintf('%s (%s)', ME.message, ME.identifier);
      fprintf('  ERR: %s\n', rep_err);
    end

    % Re-declare globals; the canonical script's `clear` just wiped our locals
    global IC_OVERRIDE NOISE_OVERRIDE WS;

    % Defaults for any rep_* the canonical script's `clear` wiped (when an
    % exception fires before the try-block can fill them in)
    if ~exist('rep_landed', 'var'); rep_landed = 0; end
    if ~exist('rep_xy',     'var'); rep_xy     = NaN; end
    if ~exist('rep_vel',    'var'); rep_vel    = NaN; end
    if ~exist('rep_alt',    'var'); rep_alt    = NaN; end
    if ~exist('rep_t',      'var'); rep_t      = NaN; end
    if ~exist('rep_prec',   'var'); rep_prec   = 0;   end
    if ~exist('rep_soft',   'var'); rep_soft   = 0;   end
    if ~exist('rep_err',    'var'); rep_err    = '';  end

    % Persist this rep (use WS.cur_b/cur_r — locals b/r were wiped)
    cb = WS.cur_b; cr = WS.cur_r;
    fname = sprintf('%srep_%s_%02d.mat', WS.outdir, WS.batches(cb).name, cr);
    save(fname, 'rep_landed', 'rep_xy', 'rep_vel', 'rep_alt', 'rep_t', ...
                'rep_prec', 'rep_soft', 'rep_err');
    fprintf('  → landed=%d  t=%.2fs  alt=%.3f  xy=%.4f  vel=%.4f  P=%d S=%d  → %s\n', ...
            rep_landed, rep_t, rep_alt, rep_xy, rep_vel, rep_prec, rep_soft, fname);
    % Restore loop counter so the for-loop continues correctly
    b = cb; r = cr;
  end
end

% =========================================================================
% Aggregate
% =========================================================================
global WS;
OUTDIR = WS.outdir; batches_meta = WS.batches;

results = struct('batch', {}, 'rep', {}, 'IC', {}, 'noise', {}, ...
                 'landed', {}, 't_land', {}, 'alt_above', {}, ...
                 'xy_err', {}, 'rel_vel', {}, 'precise', {}, 'soft', {}, 'err', {});

files = dir([OUTDIR 'rep_*.mat']);
for k = 1:numel(files)
  fname = files(k).name;
  m = regexp(fname, '^rep_(.+)_(\d+)\.mat$', 'tokens', 'once');
  if isempty(m); continue; end
  batch_name = m{1}; rep_n = str2double(m{2});
  d = load([OUTDIR fname]);
  bm = batches_meta(strcmp({batches_meta.name}, batch_name));
  results(end+1) = struct( ...
    'batch',     batch_name, ...
    'rep',       rep_n, ...
    'IC',        bm.IC, ...
    'noise',     bm.noise, ...
    'landed',    d.rep_landed, ...
    't_land',    d.rep_t, ...
    'alt_above', d.rep_alt, ...
    'xy_err',    d.rep_xy, ...
    'rel_vel',   d.rep_vel, ...
    'precise',   d.rep_prec, ...
    'soft',      d.rep_soft, ...
    'err',       d.rep_err); %#ok<SAGROW>
end

fprintf('\n=================== PHASE 1 SUMMARY ===================\n');
for b = 1:numel(batches_meta)
  bb = batches_meta(b);
  rs = results(strcmp({results.batch}, bb.name));
  if isempty(rs); continue; end
  xy = [rs.xy_err]; vel = [rs.rel_vel];
  prec = sum([rs.precise]); soft = sum([rs.soft]);
  sp = sum([rs.precise] & [rs.soft]);
  err_count = sum(~cellfun(@isempty, {rs.err}));
  fprintf(['%-22s  n=%d  xy_mean=%.4f  xy_std=%.4f  xy_min=%.4f  ' ...
           'vel_mean=%.4f  PREC=%d SOFT=%d SP=%d  ERR=%d\n'], ...
          bb.name, numel(rs), mean(xy, 'omitnan'), std(xy, 'omitnan'), ...
          min(xy, [], 'omitnan'), mean(vel, 'omitnan'), prec, soft, sp, err_count);
end

save([OUTDIR 'phase1_summary.mat'], 'results');
fprintf('\nFull results: %sphase1_summary.mat\n', OUTDIR);

fid = fopen([OUTDIR 'done.flag'], 'w');
if fid ~= -1
  fprintf(fid, '%s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
  fclose(fid);
end
fprintf('Wrote sentinel: %sdone.flag\n', OUTDIR);
