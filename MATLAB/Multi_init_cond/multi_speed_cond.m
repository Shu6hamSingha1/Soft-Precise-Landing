%% MULTI SPEED CONDITION TEST
%
% Single baseline initial condition [0,0,-5] m, symmetric speed-multiplier
% sweep across all four moving trajectories under both the clean and the
% realistic disturbance configs (mirrors multi_Init_Var.m).
%
%   mults      : [0.6, 0.8, 1.0, 1.2, 1.4]
%   Trajectory : Linear, Sinusoidal, Lissajous, Circular  (Static has no speed knob)
%   Config     : noiseless (NOISE=0, GE=0, delay=0) and realistic (all on)
%
% Each (trajectory, config) pair saves a results struct array of length
% numel(mults), indexed by speed multiplier, to:
%   Datasets/<trajType>_multi_speed.mat            -- realistic
%   Datasets/<trajType>_multi_speed_noiseless.mat  -- clean
%
% Each row carries the same fields as multi_Init_Var (success, final_t,
% final_xy, final_rel_vel, precise, soft, data) plus the scalar `mult`
% that generated it.
clc; clear;

% Shared helpers live in ../Common
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
addpath(fullfile(mfile_dir, 'plotters'));
clear mfile_dir;

trajList = ["Linear", "Sinusoidal", "Lissajous", "Circular"];
mults    = [0.6, 0.8, 1.0, 1.2, 1.4];

% Single deterministic baseline IC
p0 = [0, 0, -5];
q0 = [1; 0; 0; 0];  q0 = q0 / norm(q0);
v0 = zeros(3,1);
w0 = zeros(3,1);
x0 = [p0(:); q0; v0; w0];

cfgList = struct( ...
    'tag',   {'noiseless',     ''}, ...
    'NOISE', {0,               1}, ...
    'GE',    {0,               1}, ...
    'delay', {0,               1});

datasetDir = fullfile(fileparts(mfilename('fullpath')), 'Datasets');
if ~exist(datasetDir, 'dir')
    mkdir(datasetDir);
end

nMults = numel(mults);

for tIdx = 1:numel(trajList)
    trajType = trajList(tIdx);

    for cIdx = 1:numel(cfgList)
        cfg = cfgList(cIdx);
        cfg_override = struct('NOISE', cfg.NOISE, ...
                              'GE',    cfg.GE, ...
                              'delay', cfg.delay);

        if isempty(cfg.tag)
            tagStr = 'realistic';
        else
            tagStr = cfg.tag;
        end
        fprintf('\n=== %s | %s (NOISE=%d, GE=%d, delay=%d) ===\n', ...
                trajType, tagStr, cfg.NOISE, cfg.GE, cfg.delay);

        results = repmat(struct( ...
            'mult',          0, ...
            'success',       false, ...
            'final_error',   0, ...
            'final_t',       0, ...
            'final_xy',      0, ...
            'final_alt',     0, ...
            'final_rel_vel', 0, ...
            'precise',       false, ...
            'soft',          false, ...
            'fov_fail',      false, ...
            'fov_fail_t',    NaN, ...
            'data',          []), 1, nMults);

        for k = 1:nMults
            m = mults(k);
            fprintf('\nMult %d/%d  (x%.2f)\n', k, nMults, m);

            tmp = run_simulation(x0, trajType, [], m, cfg_override, 1000+k);

            tmp.mult = m;
            results(k) = orderfields(tmp, results(k));
        end

        % Save under Datasets/<trajType>_multi_speed[_noiseless].mat
        if isempty(cfg.tag)
            saveFile = fullfile(datasetDir, ...
                sprintf('%s_multi_speed.mat', trajType));
        else
            saveFile = fullfile(datasetDir, ...
                sprintf('%s_multi_speed_%s.mat', trajType, cfg.tag));
        end
        save(saveFile, 'results', 'mults', 'p0', 'trajType', ...
                       'nMults', 'cfg_override');
        fprintf('\nSaved results to %s\n', saveFile);

        clear results
    end
end

% --------------------- REALISTIC SUMMARY TABLE ---------------------
fprintf('\n\n================= REALISTIC SUMMARY =================\n');
fprintf('%-12s %-6s %-8s %-8s %-8s %-9s %-9s %-9s\n', ...
        'traj','mult','landed','precise','soft','t [s]','xy [m]','v_rel');
fprintf('-----------------------------------------------------------------\n');
for tIdx = 1:numel(trajList)
    trajType = trajList(tIdx);
    saveFile = fullfile(datasetDir, sprintf('%s_multi_speed.mat', trajType));
    S = load(saveFile, 'results', 'mults');
    for k = 1:numel(S.mults)
        r = S.results(k);
        if isfield(r,'final_rel_vel')
            vrel = r.final_rel_vel;
        else
            vrel = NaN;
        end
        fprintf('%-12s %-6.2f %-8d %-8d %-8d %-9.2f %-9.4f %-9.3f\n', ...
                trajType, S.mults(k), ...
                logical(r.success), logical(r.precise), logical(r.soft), ...
                r.final_t, r.final_xy, vrel);
    end
end

fprintf(['\nAll speed sweeps complete.  Run' ...
         '\n    python Soft_Precise_Landing/make_multi_speed_plots.py' ...
         '\nto regenerate the manuscript figures.\n']);
