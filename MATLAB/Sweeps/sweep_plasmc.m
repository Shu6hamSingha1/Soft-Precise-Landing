%% PLASMC PARAMETER SWEEP HARNESS
% Sweeps each PLASMC control parameter ± perturbations and reports
% multi-init success rate + mean landing time + mean final xy.
%
% Usage: edit `trajType` and `param_list` below, then run.
% Output: sweep_<trajType>_<group>.mat in Datasets/

clc; clear;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'Multi_init_cond'));
addpath(fullfile(this_dir, '..', 'Common'));

trajType = "Linear";   % change to test different trajectory
group    = "outer";    % "outer" | "inner" — for filename

% Standard 5 ICs (matches multi_Init_Var.m)
p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2,2,-5;
    -2,-2,-5
];
numRuns = size(p0,1);

% Multipliers for each parameter sweep
mults = [0.5, 0.75, 1.25, 1.5];

% Parameter list — each row: {name, baseline_diag_or_scalar, index_or_all}
% For diagonal matrices we sweep the diagonal entries.
% index = 0 means scale ALL diagonal entries together.
% index > 0 means scale only that entry (1-based).
param_list = {
    % --- Visibility funnel ---
    'gamma_1', [0.2,0.2],    0;
    'p_1inf',  [0.2;0.2],    0;
    % --- Outer PID on zeta_1 ---
    'rp',      diag([4,4]),  0;
    'ri',      diag([0.1,0.1]), 0;
    'rd',      diag([1.3,1.3]), 0;
    % --- Optical flow funnel ---
    'gamma_2', [0.2,0.2,0.2], 0;
    'p_20',    [12;12;5],    1;   % horizontal x
    'p_20',    [12;12;5],    3;   % vertical
    'p_2inf',  [1.5;1.5;2],  1;   % horizontal x
    'p_2inf',  [1.5;1.5;2],  3;   % vertical
    % --- Sliding mode ---
    'Omega',   diag([0.005,0.005,0.01]), 1;  % horizontal x
    'Omega',   diag([0.005,0.005,0.01]), 3;  % vertical
    'Gamma',   diag([0.25,0.25,0.5]), 1;     % horizontal x
    'Gamma',   diag([0.25,0.25,0.5]), 3;     % vertical
    'E',       diag([2.5,2.5,0.5]),  1;      % horizontal x
    'E',       diag([2.5,2.5,0.5]),  3;      % vertical
    % --- Adaptive law ---
    'N',       diag([0.02,0.02,0.05]), 0;
    'P',       diag([1.5,1.5,5]),  0;
    'kappa_0', [0.1;0.1;0.2], 0;
};

n_params = size(param_list,1);
fprintf('=== PLASMC sweep on %s, %d params x %d mults x %d ICs ===\n', ...
        trajType, n_params, numel(mults), numRuns);

% Pre-allocate result table
sweep = struct( ...
    'param', {}, 'index', {}, 'mult', {}, ...
    'value', {}, ...
    'n_land', {}, 'mean_t', {}, 'mean_xy', {}, 'max_xy', {});

row = 0;

% First: BASELINE (mult=1)
row = row + 1;
sweep(row).param = 'BASELINE';
sweep(row).index = 0;
sweep(row).mult  = 1;
sweep(row).value = NaN;
fprintf('\n--- BASELINE ---\n');
[nl, mt, mxy, maxxy] = run_5ic(p0, numRuns, trajType, struct());
sweep(row).n_land  = nl;
sweep(row).mean_t  = mt;
sweep(row).mean_xy = mxy;
sweep(row).max_xy  = maxxy;
fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f\n', nl, mt, mxy, maxxy);

% Loop over parameters
for p = 1:n_params
    name  = param_list{p,1};
    base  = param_list{p,2};
    idx   = param_list{p,3};

    for m = mults
        row = row + 1;
        sweep(row).param = name;
        sweep(row).index = idx;
        sweep(row).mult  = m;

        % Build override
        ovr = struct();
        if isvector(base) && ~isscalar(base)
            v = base;
            if idx == 0
                v = v * m;
            else
                v(idx) = v(idx) * m;
            end
            ovr.(name) = v;
            sweep(row).value = v(max(idx,1));
        elseif ismatrix(base) && size(base,1) == size(base,2) && ~isscalar(base)
            % Diagonal matrix
            d = diag(base);
            if idx == 0
                d = d * m;
            else
                d(idx) = d(idx) * m;
            end
            ovr.(name) = diag(d);
            sweep(row).value = d(max(idx,1));
        else
            ovr.(name) = base * m;
            sweep(row).value = base * m;
        end

        fprintf('\n--- %s (idx=%d) x%.2f -> %.4f ---\n', name, idx, m, sweep(row).value);
        [nl, mt, mxy, maxxy] = run_5ic(p0, numRuns, trajType, ovr);
        sweep(row).n_land  = nl;
        sweep(row).mean_t  = mt;
        sweep(row).mean_xy = mxy;
        sweep(row).max_xy  = maxxy;
        fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f\n', ...
                nl, mt, mxy, maxxy);
    end
end

% Save
datasetDir = fullfile(fileparts(mfilename('fullpath')), 'Datasets');
if ~exist(datasetDir, 'dir'), mkdir(datasetDir); end
saveFile = fullfile(datasetDir, sprintf('sweep_%s_%s.mat', trajType, group));
save(saveFile, 'sweep', 'trajType', 'param_list', 'mults');
fprintf('\nSaved sweep to %s\n', saveFile);

% Print final table
fprintf('\n========================================================\n');
fprintf('%-12s %-5s %-7s %-9s %-7s %-7s %-9s %-9s\n', ...
        'param','idx','mult','value','land','mean_t','mean_xy','max_xy');
fprintf('--------------------------------------------------------\n');
for r = 1:numel(sweep)
    fprintf('%-12s %-5d %-7.2f %-9.4f %-7s %-7.2f %-9.4f %-9.4f\n', ...
            sweep(r).param, sweep(r).index, sweep(r).mult, sweep(r).value, ...
            sprintf('%d/5', sweep(r).n_land), ...
            sweep(r).mean_t, sweep(r).mean_xy, sweep(r).max_xy);
end


% =====================================================================
% Helper: run 5 ICs, return aggregate stats
% =====================================================================
function [n_land, mean_t, mean_xy, max_xy] = run_5ic(p0, numRuns, trajType, ovr)
    landed   = false(numRuns,1);
    t_land   = nan(numRuns,1);
    xy_final = nan(numRuns,1);

    for k = 1:numRuns
        q0 = [1;0;0;0];
        v0 = zeros(3,1);
        w0 = zeros(3,1);
        x0 = [p0(k,:)'; q0; v0; w0];

        % Deterministic seed per IC (same noise across all perturbations)
        rng(1000 + k);

        try
            tmp = run_simulation(x0, trajType, ovr);
        catch ME
            fprintf('  Run %d ERR: %s\n', k, ME.message);
            continue;
        end

        if isfield(tmp,'success'),     landed(k) = tmp.success;       end
        if isfield(tmp,'final_t'),     t_land(k) = tmp.final_t;       end
        if isfield(tmp,'final_xy'),    xy_final(k) = tmp.final_xy;    end
    end

    n_land  = sum(landed);
    mean_t  = mean(t_land(landed),  'omitnan');
    mean_xy = mean(xy_final(landed), 'omitnan');
    max_xy  = max(xy_final(landed), [], 'omitnan');
    if isempty(mean_t)  || isnan(mean_t),  mean_t  = NaN; end
    if isempty(mean_xy) || isnan(mean_xy), mean_xy = NaN; end
    if isempty(max_xy)  || isnan(max_xy),  max_xy  = NaN; end
end
