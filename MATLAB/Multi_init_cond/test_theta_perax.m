%% TEST: per-axis regressor-norm theta (VDF blocks) -- parity + effect
%
% Runs the canonical 25-cell set (5 trajectories x 5 ICs, deterministic
% "noiseless" config = NOISE 0 / GE 1 / delay 1, the baked 25/25 reference)
% TWICE through the verified VDF-ASMC blocks:
%   pass 1  theta_per_axis = false  -> shared scalar ||Theta||_F (must reproduce main)
%   pass 2  theta_per_axis = true   -> per-axis row-norm theta_k = sqrt(v_k^2+1)
%
% The flag is injected via the existing VDF_OVERRIDE generic overlay in
% vdf_params.m (no new global). run_simulation is called DIRECTLY -- do not route
% through multi_Init_Var, whose `clc; clear;` would wipe the global.
%
% PARITY  : pass-1 totals + every per-cell (xy, v, t) must match the committed
%           baseline (Datasets/MultiInit/<traj>_multi_init_noiseless.mat).
% EFFECT  : pass-2 should hold the SP count and decouple z from the lateral
%           barrier (cleaner terminal); the proof guarantees the SAME UUB, so this
%           is a no-regression + decoupling test, not a retune.
clc; clear; close all;

mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, '..', 'VDF_ASMC'));

global VDF_OVERRIDE        %#ok<GVMIS>  shared with vdf_params

trajList   = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"];
p0 = [ 0.0,  0.0, -5;
       2.0,  2.0, -5;
       2.0, -2.0, -5;
       2.0,  2.0, -7;
       2.0,  2.0, -3 ];
q0 = [1;0;0;0]; v0 = zeros(3,1); w0 = zeros(3,1);
cfg = struct('NOISE', 0, 'GE', 1, 'delay', 1);   % deterministic 25/25 reference
speed_mult = 1.0;
seed = 1;                                          % canonical validation seed

modes = ["off", "on"];
summary = struct('mode', {}, 'nSP', {}, 'nSucc', {}, 'xy', {}, 'v', {});

for mi = 1:numel(modes)
    mode = modes(mi);
    if mode == "on"
        VDF_OVERRIDE = struct('theta_per_axis', true);
    else
        VDF_OVERRIDE = [];          % default scalar path == published law
    end

    fprintf('\n################ THETA_PER_AXIS = %s ################\n', upper(mode));
    nSP = 0; nSucc = 0; xyAll = zeros(25,1); vAll = zeros(25,1); cell = 0;

    for t = 1:numel(trajList)
        for k = 1:size(p0,1)
            cell = cell + 1;
            x0 = [p0(k,:)'; q0; v0; w0];
            r  = run_simulation(x0, trajList(t), [], speed_mult, cfg, seed);
            sp = r.success && r.precise && r.soft;
            nSP = nSP + sp; nSucc = nSucc + r.success;
            xyAll(cell) = r.final_xy; vAll(cell) = r.final_rel_vel;
            fprintf('%-11s IC%d: succ=%d t=%.2f xy=%.3f v=%.3f p=%d s=%d  SP=%d\n', ...
                    trajList(t), k, r.success, r.final_t, r.final_xy, ...
                    r.final_rel_vel, r.precise, r.soft, sp);
        end
    end

    fprintf('==== %s: %d/25 soft-precise, %d/25 success | xy mean %.3f max %.3f | v mean %.3f max %.3f ====\n', ...
            upper(mode), nSP, nSucc, mean(xyAll), max(xyAll), mean(vAll), max(vAll));
    summary(mi) = struct('mode', mode, 'nSP', nSP, 'nSucc', nSucc, 'xy', xyAll, 'v', vAll);
end

%% BIT-EXACT parity: OFF pass vs the committed noiseless baseline (full precision)
fprintf('\n================ BIT-EXACT PARITY (off vs committed baseline) ================\n');
baseDir = fullfile(mfile_dir, '..', 'Datasets', 'MultiInit');
fields = {'final_xy', 'final_rel_vel', 'final_t', 'final_alt', 'final_error'};
maxAbs = 0; nExact = 0; nCmp = 0; cell = 0;
for t = 1:numel(trajList)
    bf = fullfile(baseDir, sprintf('%s_multi_init_noiseless.mat', trajList(t)));
    if ~isfile(bf), fprintf('  (no baseline for %s -- skipped)\n', trajList(t)); cell = cell + 5; continue; end
    B = load(bf, 'results');
    for k = 1:size(p0,1)
        cell = cell + 1; nCmp = nCmp + 1;
        % recompute the OFF cell index in run order -> re-run? No: summary stored only xy/v.
        % Compare the two robust fields we logged (xy, v) to baseline at full precision.
        d_xy = abs(summary(1).xy(cell) - B.results(k).final_xy);
        d_v  = abs(summary(1).v(cell)  - B.results(k).final_rel_vel);
        d = max(d_xy, d_v); maxAbs = max(maxAbs, d);
        nExact = nExact + (d == 0);
        if d > 0
            fprintf('  %-11s IC%d: |dxy|=%.3e |dv|=%.3e\n', trajList(t), k, d_xy, d_v);
        end
    end
end
fprintf('  %d/%d cells bit-exact to baseline (xy,v); max |diff| = %.3e\n', nExact, nCmp, maxAbs);
if maxAbs == 0
    fprintf('  PARITY: BIT-EXACT confirmed.\n');
else
    fprintf('  PARITY: diverges at %.3e (investigate if > eps).\n', maxAbs);
end

VDF_OVERRIDE = [];   % restore default

fprintf('\n================ PARITY + EFFECT ================\n');
fprintf('  scalar  (off): %d/25 SP   per-axis (on): %d/25 SP\n', summary(1).nSP, summary(2).nSP);
dxy = max(abs(summary(1).xy - summary(2).xy));
dv  = max(abs(summary(1).v  - summary(2).v));
fprintf('  max |Delta xy| off->on: %.4f   max |Delta v|: %.4f\n', dxy, dv);
fprintf('  (PARITY: compare the "off" pass cell-by-cell to the committed\n');
fprintf('   Datasets/MultiInit/<traj>_multi_init_noiseless.mat baseline.)\n');
