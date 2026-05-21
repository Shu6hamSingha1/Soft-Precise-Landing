% Phase 1 baseline sweep — establish MATLAB ground-truth performance
% at the same IC the PX4 SITL pipeline uses.
%
% Sets globals IC_OVERRIDE and NOISE_OVERRIDE which InitVar.m honors (see
% the patch added on 2026-05-22).  The canonical script's `clear` at the
% top wipes locals but leaves globals, so this is the safe override path.
%
% Run from MATLAB/Multi_init_cond:  >> phase1_baseline_sweep
%
% Output:  ../Datasets/Phase1/phase1_<batch>_repNN.mat (one per run) +
%          ../Datasets/Phase1/phase1_summary.mat (aggregated)

% Don't `clear all` — we WANT globals to persist.  But clean the workspace.
clear -regexp '^(?!IC_OVERRIDE|NOISE_OVERRIDE).*$';
close all; clc;

OUTDIR = '../Datasets/Phase1/';
if ~exist(OUTDIR, 'dir'); mkdir(OUTDIR); end

global IC_OVERRIDE NOISE_OVERRIDE;

% Three batches:
%   A: PX4-aligned IC (0,0,-5) + 50 dB pixel noise, N=10  — matches PX4 IC1
%   B: PX4-aligned IC (0,0,-5) + NO noise, N=1             — deterministic floor
%   C: Canonical IC (2,2,-5) + 50 dB noise, N=5            — sanity check
batches = struct( ...
  'name',  {'A_PX4ic_noise',  'B_PX4ic_noiseless', 'C_canonical_noise'}, ...
  'IC',    {[ 0.0,  0.0, -5.0], [ 0.0,  0.0, -5.0], [ 2.0,  2.0, -5.0]}, ...
  'noise', {1, 0, 1}, ...
  'N',     {10, 1, 5});

% Pre-allocate result struct
n_total = sum([batches.N]);
results(n_total) = struct('batch', '', 'rep', 0, 'IC', [], 'noise', 0, ...
                          'landed', 0, 't_land', 0, 'alt_above', 0, ...
                          'xy_err', 0, 'rel_vel', 0, 'precise', 0, 'soft', 0);
ri = 0;

for b = 1:numel(batches)
  bb = batches(b);
  for r = 1:bb.N
    fprintf('\n=== Batch %s, rep %d/%d  IC=[%.1f,%.1f,%.1f]  noise=%d ===\n', ...
            bb.name, r, bb.N, bb.IC(1), bb.IC(2), bb.IC(3), bb.noise);

    IC_OVERRIDE    = bb.IC(:);
    NOISE_OVERRIDE = bb.noise;

    try
      visualControl_IBVS_adaptive;
    catch ME
      fprintf('  ERR rep %d: %s (%s)\n', r, ME.message, ME.identifier);
      ri = ri + 1;
      results(ri) = struct('batch', bb.name, 'rep', r, 'IC', bb.IC, ...
                           'noise', bb.noise, 'landed', 0, 't_land', 0, ...
                           'alt_above', NaN, 'xy_err', NaN, 'rel_vel', NaN, ...
                           'precise', 0, 'soft', 0);
      % Re-declare globals — `clear` inside canonical script wiped locals.
      global IC_OVERRIDE NOISE_OVERRIDE;
      continue;
    end

    % After the canonical script returns, the variables it used (idx,
    % tRange, I_p_c, x_t, dx_t, I_v_c, zf, landed) are in our workspace
    % UNTIL the next iteration's `clear` inside the canonical script.
    % Capture them NOW.
    alt_above_v = abs(I_p_c(3) - x_t(3,idx));
    xy_err_v    = norm(I_p_c(1:2) - x_t(1:2,idx));
    rel_vel_v   = norm(I_v_c - dx_t(1:3,idx));
    precise_v   = xy_err_v <= 0.08;
    soft_v      = rel_vel_v <= 0.2;
    landed_v    = landed;
    t_land_v    = tRange(idx);
    fprintf('  → landed=%d  t=%.2fs  alt=%.3f  xy=%.4f  vel=%.4f  P=%d S=%d\n', ...
            landed_v, t_land_v, alt_above_v, xy_err_v, rel_vel_v, ...
            precise_v, soft_v);

    ri = ri + 1;
    results(ri) = struct('batch', bb.name, 'rep', r, 'IC', bb.IC, ...
                         'noise', bb.noise, 'landed', landed_v, ...
                         't_land', t_land_v, 'alt_above', alt_above_v, ...
                         'xy_err', xy_err_v, 'rel_vel', rel_vel_v, ...
                         'precise', precise_v, 'soft', soft_v);

    % Per-rep dumps are skipped — saving the full workspace per run produces
    % 50-200 MB files, too heavy for git.  If Phase 3 (trajectory replay)
    % is needed later, re-run with a single batch and uncomment below.
    %
    % fname = sprintf('%sphase1_%s_rep%02d.mat', OUTDIR, bb.name, r);
    % save(fname);  % full workspace dump for trajectory replay

    % Re-declare globals — `clear` inside canonical script wiped locals
    global IC_OVERRIDE NOISE_OVERRIDE;
  end
end

% Summary
fprintf('\n\n=================== PHASE 1 SUMMARY ===================\n');
for b = 1:numel(batches)
  bb = batches(b);
  rs = results(strcmp({results.batch}, bb.name));
  if isempty(rs); continue; end
  xy = [rs.xy_err]; vel = [rs.rel_vel];
  prec = sum([rs.precise]); soft = sum([rs.soft]);
  sp = sum([rs.precise] & [rs.soft]);
  fprintf(['%-22s  n=%d  xy_mean=%.4f  xy_std=%.4f  xy_min=%.4f  ' ...
           'vel_mean=%.4f  PREC=%d SOFT=%d SP=%d\n'], ...
          bb.name, numel(rs), mean(xy, 'omitnan'), std(xy, 'omitnan'), ...
          min(xy, [], 'omitnan'), mean(vel, 'omitnan'), prec, soft, sp);
end

save([OUTDIR 'phase1_summary.mat'], 'results');
fprintf('\nFull results: %sphase1_summary.mat\n', OUTDIR);
fprintf('Per-rep trajectories: %sphase1_*_rep*.mat\n', OUTDIR);
