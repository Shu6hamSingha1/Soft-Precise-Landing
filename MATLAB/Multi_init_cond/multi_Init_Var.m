%% MULTI INITIAL CONDITION TEST  (full sweep: 5 trajectories x 2 configs)
%
% Sweeps the 5 nominal initial conditions on every trajectory under both
%   - clean    : NOISE = 0, GE = 0, delay = 0      (idealized)
%   - realistic: NOISE = 1, GE = 1, delay = 1      (full disturbance set)
%
% Saved as:
%   Datasets/<trajType>_multi_init.mat            -- realistic
%   Datasets/<trajType>_multi_init_noiseless.mat  -- clean
%
% Plotting is delegated to Soft_Precise_Landing/make_multi_init_plots.py so
% that the manuscript figures stay visually uniform with the rest of the
% Python-generated plots.
clc; clear;

trajList = ["Static", "Linear", "Sinusoidal", "Lissajous", "Circular"];

% Global target-speed multiplier for moving trajectories. Static ignores it.
speed_mult = 1.0;

p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2.0,2.0,-5;
    2.0,-2.0,-5
    ];

numRuns = size(p0,1);

cfgList = struct( ...
    'tag',       {'noiseless',                    ''}, ...
    'NOISE',     {0,                              1}, ...
    'GE',        {0,                              1}, ...
    'delay',     {0,                              1});

datasetDir = fullfile(fileparts(mfilename('fullpath')), 'Datasets');
if ~exist(datasetDir, 'dir')
    mkdir(datasetDir);
end

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
            'success',       false, ...
            'final_error',   0, ...
            'final_t',       0, ...
            'final_xy',      0, ...
            'final_alt',     0, ...
            'final_rel_vel', 0, ...
            'precise',       false, ...
            'soft',          false, ...
            'data',          []), 1, numRuns);

        for k = 1:numRuns
            fprintf('\nRun %d/%d\n', k, numRuns);

            q0w = 1.0; q0x = 0.0; q0y = 0.0; q0z = 0.0;
            q0 = [q0w; q0x; q0y; q0z];
            q0 = q0 / norm(q0);

            v0 = zeros(3,1);
            w0 = zeros(3,1);

            x0 = [p0(k,:)'; q0; v0; w0];

            tmp = run_simulation(x0, trajType, [], speed_mult, cfg_override, 1000+k);
            results(k) = tmp;
        end

        % Save under Datasets/<trajType>_multi_init[_noiseless].mat
        if isempty(cfg.tag)
            saveFile = fullfile(datasetDir, ...
                sprintf('%s_multi_init.mat', trajType));
        else
            saveFile = fullfile(datasetDir, ...
                sprintf('%s_multi_init_%s.mat', trajType, cfg.tag));
        end
        save(saveFile, 'results', 'p0', 'trajType', 'numRuns', 'cfg_override');
        fprintf('\nSaved results to %s\n', saveFile);

        clear results
    end
end

fprintf(['\nAll sweeps complete.  Run' ...
         '\n    python Soft_Precise_Landing/make_multi_init_plots.py' ...
         '\nto regenerate the manuscript figures.\n']);
