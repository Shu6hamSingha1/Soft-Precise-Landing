%% PLASMC DEEP PARAMETER SWEEP — CROSS-TRAJECTORY
% Sweeps every outer + inner control parameter at mults [0.5, 0.75, 1.25, 1.5]
% across all 4 moving trajectories (Linear, Sinusoidal, Lissajous, Circular)
% using 5 deterministic ICs.  Scores each (param, mult) by:
%   - min land rate across trajs (primary — must be 5/5 on all 4 for "safe")
%   - aggregate mean_t across trajs (faster = better)
%   - aggregate max_xy across trajs (tighter = better)
%
% A candidate is a true improvement only if it is 5/5 × 4 trajs AND strictly
% better than baseline on either mean_t or max_xy without degrading the other.
%
% Post S_2 funnel guard fix: baseline is 25/25 on all 4 moving trajs.
%
% Output: Datasets/sweep_deep.mat  +  sweep_deep.log

clc; clear;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'Multi_init_cond'));  % run_simulation, InitVar
addpath(fullfile(this_dir, '..', 'Common'));

% Synced to multi_Init_Var.m (2026-04-16): same trajList, same IC set, and
% same per-run cfg_override/seed injection so any cell that passes the sweep
% gate will also pass the 50/50 multi-init gate.
trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];

p0 = [
    0,  0, -5;
    2,  2, -5;
    2, -2, -5;
    2,  2, -7;
    2,  2, -3
];
numRuns = size(p0,1);

% Realistic disturbance leg (matches multi_Init_Var.m realistic cfg).
cfg_override = struct('NOISE', 1, 'GE', 1, 'delay', 1);

mults = [0.5, 0.75, 1.25, 1.5];

% ---------------------------------------------------------------------
% Parameter list: {name, baseline, index, group}
%   index = 0  -> scale all diag/vec entries together
%   index > 0  -> scale only that 1-based entry
% ---------------------------------------------------------------------
% NOTE: Baselines below are placeholders — auto-synced from K_PLASMC (outer+
% inner) and Constants workspace (const) below before the sweep runs.
% Post-Approach 2 (2026-04-18): visibility-funnel params (gamma_1, p_1inf)
% removed; funnel-margin cone params (rho_fov_inf, l_fov, theta_cap) added;
% Constants-level h_rd and FILTER_WINDOW added as sweepable knobs.
param_list = {
    % ============== OUTER ==============
    % Raw-error PID on V_s_e_n (Approach 2)
    'rp',      diag([6,6]),               0, 'outer';
    'ri',      diag([0.1,0.1]),           0, 'outer';
    'rd',      diag([1.15,1.15]),         0, 'outer';
    % Optical-flow funnel (zeta_2)
    'gamma_2', [0.2,0.2,0.2],             0, 'outer';
    'p_20',    [25;25;4],                 1, 'outer';   % lateral
    'p_20',    [25;25;4],                 3, 'outer';   % vertical
    'p_2inf',  [2.5;2.5;1.5],             1, 'outer';   % lateral
    'p_2inf',  [2.5;2.5;1.5],             3, 'outer';   % vertical
    % Sliding-mode law
    'Omega',   diag([0.05,0.05,0.025]),   1, 'outer';   % lateral
    'Omega',   diag([0.05,0.05,0.025]),   3, 'outer';   % vertical
    'Gamma',   diag([0.4375,0.5,0.75]),   1, 'outer';   % x
    'Gamma',   diag([0.4375,0.5,0.75]),   2, 'outer';   % y
    'Gamma',   diag([0.4375,0.5,0.75]),   3, 'outer';   % z
    'E',       diag([1.0,1.0,1.0]),       1, 'outer';
    'E',       diag([1.0,1.0,1.0]),       3, 'outer';
    % Adaptive law
    'N',       diag([0.02,0.02,0.05]),    0, 'outer';
    'P',       diag([1.5,1.5,5]),         0, 'outer';
    'kappa_0', [0.125;0.125;0.25],        0, 'outer';
    % funnel-margin cone clamp (Approach 2)
    'rho_fov_inf', [40;40],               0, 'outer';
    'l_fov',       0.1,                   0, 'outer';
    'theta_cap',   deg2rad(60),           0, 'outer';

    % ============== INNER ==============
    % Geometric SO(3) attitude
    'kR',        diag([1.5, 1.5, 0.5]),   1, 'inner';   % roll/pitch
    'kR',        diag([1.5, 1.5, 0.5]),   3, 'inner';   % yaw
    'kOmega',    diag([0.3, 0.3, 0.1]),   1, 'inner';
    'kOmega',    diag([0.3, 0.3, 0.1]),   3, 'inner';
    % Yaw adaptive SMC
    'Omega_a',   0.5,   0, 'inner';
    'Gamma_a',   0.5,   0, 'inner';
    'n_a',       1.0,   0, 'inner';
    'p_a',       2,     0, 'inner';
    'kappa_a_0', 2.0,   0, 'inner';
    'E_a',       3.0,   0, 'inner';

    % ============== CONSTANTS (Common/Constants.m) ==============
    'h_rd',          -0.42, 0, 'const';                 % descent-profile scalar
    'FILTER_WINDOW', 21,    0, 'const';                 % Sav-Gol window (must stay odd)
};

% ---------------------------------------------------------------------
% AUTO-SYNC: overwrite column 2 of param_list with the currently-locked
% values from K_PLASMC (InitGains_Comparison.m). This prevents the sweep
% from perturbing a stale reference point whenever the locked gains drift.
% Only the 'base' column is replaced; index/group remain as declared.
% ---------------------------------------------------------------------
addpath(fullfile(this_dir, '..', 'Comparison'));
Constants;             % populates K.p_10, h_rd, FILTER_WINDOW in workspace
InitGains_Comparison;  % populates K_PLASMC using K.p_10
for pp = 1:size(param_list,1)
    fname = param_list{pp,1};
    if isfield(K_PLASMC, fname)
        param_list{pp,2} = K_PLASMC.(fname);
    elseif exist(fname, 'var') == 1
        % Constants-workspace scalar (h_rd, FILTER_WINDOW)
        param_list{pp,2} = eval(fname);
    else
        warning('sweep_deep: no source for %s — keeping declared base', fname);
    end
end
clear pp fname;

n_params = size(param_list,1);
n_trajs  = numel(trajList);
n_mults  = numel(mults);

fprintf('=== DEEP sweep: %d params x %d mults x %d trajs x %d ICs = %d sims ===\n', ...
        n_params, n_mults, n_trajs, numRuns, ...
        (1 + n_params*n_mults)*n_trajs*numRuns);

% sweep(param_idx, mult_idx, traj_idx) = struct of stats
% Special row 0 = baseline; we store in separate baseline(traj_idx)
baseline = struct('n_land',{},'mean_t',{},'mean_xy',{},'max_xy',{});
sweep    = struct('param',{},'index',{},'group',{},'mult',{},'value',{}, ...
                  'traj',{},'n_land',{},'mean_t',{},'mean_xy',{},'max_xy',{});

% -------------- BASELINE per trajectory --------------
fprintf('\n=========== BASELINE ===========\n');
for t = 1:n_trajs
    trajType = trajList(t);
    fprintf('\n--- baseline %s ---\n', trajType);
    [nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,struct(),cfg_override);
    baseline(t).traj    = trajType;
    baseline(t).n_land  = nl;
    baseline(t).mean_t  = mt;
    baseline(t).mean_xy = mxy;
    baseline(t).max_xy  = maxxy;
    fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f\n', ...
            nl,mt,mxy,maxxy);
end

% -------------- SWEEPS --------------
row = 0;
for p = 1:n_params
    name  = param_list{p,1};
    base  = param_list{p,2};
    idx   = param_list{p,3};
    grp   = param_list{p,4};

    for im = 1:n_mults
        m = mults(im);

        % Build override (same logic as original harness)
        ovr = struct();
        if isscalar(base)
            new_v = base * m;
            % FILTER_WINDOW must be an odd integer (sgolayfilt requirement)
            if strcmp(name, 'FILTER_WINDOW')
                new_v = round(new_v);
                if mod(new_v,2) == 0, new_v = new_v + 1; end
            end
            ovr.(name) = new_v;
            value_log = new_v;
        elseif isvector(base)
            v = base;
            if idx == 0, v = v * m; else, v(idx) = v(idx) * m; end
            ovr.(name) = v;
            value_log = v(max(idx,1));
        elseif ismatrix(base) && size(base,1)==size(base,2)
            d = diag(base);
            if idx == 0, d = d * m; else, d(idx) = d(idx) * m; end
            ovr.(name) = diag(d);
            value_log = d(max(idx,1));
        end

        for t = 1:n_trajs
            trajType = trajList(t);
            row = row + 1;
            sweep(row).param   = name;
            sweep(row).index   = idx;
            sweep(row).group   = grp;
            sweep(row).mult    = m;
            sweep(row).value   = value_log;
            sweep(row).traj    = trajType;

            fprintf('\n--- %s[%s] (idx=%d) x%.2f -> %.4f  on %s ---\n', ...
                    name, grp, idx, m, value_log, trajType);
            [nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,ovr,cfg_override);
            sweep(row).n_land  = nl;
            sweep(row).mean_t  = mt;
            sweep(row).mean_xy = mxy;
            sweep(row).max_xy  = maxxy;
            fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f\n', ...
                    nl,mt,mxy,maxxy);
        end
    end
end

% Save
datasetDir = fullfile(fileparts(mfilename('fullpath')),'Datasets');
if ~exist(datasetDir,'dir'), mkdir(datasetDir); end
saveFile = fullfile(datasetDir,'sweep_deep.mat');
save(saveFile,'sweep','baseline','param_list','mults','trajList','p0');
fprintf('\nSaved sweep to %s\n', saveFile);

% --------------- CROSS-TRAJ SCORING ---------------
fprintf('\n\n================= CROSS-TRAJ SUMMARY =================\n');
% Aggregate baseline
base_mean_t  = mean([baseline.mean_t], 'omitnan');
base_max_xy  = max([baseline.max_xy]);
base_minland = min([baseline.n_land]);
fprintf('Baseline: min land=%d/5 across trajs, agg mean_t=%.3f, agg max_xy=%.4f\n', ...
        base_minland, base_mean_t, base_max_xy);

% Group sweep rows by (param, idx, mult)
keys   = cell(0,1);
agg    = struct('param',{},'index',{},'group',{},'mult',{},'value',{}, ...
                'min_land',{},'mean_t_agg',{},'max_xy_agg',{});
for r = 1:numel(sweep)
    k = sprintf('%s_idx%d_x%.2f', sweep(r).param, sweep(r).index, sweep(r).mult);
    ix = find(strcmp(keys,k),1);
    if isempty(ix)
        keys{end+1,1} = k;  %#ok<AGROW>
        ix = numel(keys);
        agg(ix).param = sweep(r).param;
        agg(ix).index = sweep(r).index;
        agg(ix).group = sweep(r).group;
        agg(ix).mult  = sweep(r).mult;
        agg(ix).value = sweep(r).value;
        agg(ix).min_land   = sweep(r).n_land;
        agg(ix).mean_t_agg = sweep(r).mean_t;
        agg(ix).max_xy_agg = sweep(r).max_xy;
        agg(ix).n_seen     = 1;
    else
        agg(ix).min_land   = min(agg(ix).min_land, sweep(r).n_land);
        agg(ix).mean_t_agg = agg(ix).mean_t_agg + sweep(r).mean_t;
        agg(ix).max_xy_agg = max(agg(ix).max_xy_agg, sweep(r).max_xy);
        agg(ix).n_seen     = agg(ix).n_seen + 1;
    end
end
for r = 1:numel(agg)
    agg(r).mean_t_agg = agg(r).mean_t_agg / agg(r).n_seen;
end

% Print full table
fprintf('\n%-12s %-5s %-6s %-5s %-9s %-9s %-9s %-9s\n', ...
        'param','grp','idx','mult','value','minL','aggT','aggMaxXY');
fprintf('-----------------------------------------------------------------\n');
for r = 1:numel(agg)
    mark = ' ';
    if agg(r).min_land >= numRuns && ...
       (agg(r).mean_t_agg < base_mean_t - 1e-3 || ...
        agg(r).max_xy_agg < base_max_xy - 1e-3)
        mark = '*';
    end
    fprintf('%c%-11s %-5s %-6d %-5.2f %-9.4f %-9s %-9.3f %-9.4f\n', ...
            mark, agg(r).param, agg(r).group, agg(r).index, agg(r).mult, ...
            agg(r).value, sprintf('%d/5',agg(r).min_land), ...
            agg(r).mean_t_agg, agg(r).max_xy_agg);
end

% Winners only
fprintf('\n=============== CANDIDATE WINNERS (safe + better) ===============\n');
fprintf('%-12s %-5s %-6s %-5s %-9s %-9s %-9s\n', ...
        'param','grp','idx','mult','value','aggT','aggMaxXY');
fprintf('-----------------------------------------------------------\n');
found = false;
for r = 1:numel(agg)
    if agg(r).min_land >= numRuns && ...
       (agg(r).mean_t_agg < base_mean_t - 1e-3 || ...
        agg(r).max_xy_agg < base_max_xy - 1e-3)
        fprintf('%-12s %-5s %-6d %-5.2f %-9.4f %-9.3f %-9.4f\n', ...
                agg(r).param, agg(r).group, agg(r).index, agg(r).mult, ...
                agg(r).value, agg(r).mean_t_agg, agg(r).max_xy_agg);
        found = true;
    end
end
if ~found
    fprintf('(none — current baseline is locally Pareto-optimal across trajs)\n');
end

% =====================================================================
function [n_land,mean_t,mean_xy,max_xy] = run_5ic(p0,numRuns,trajType,ovr,cfg_override)
    landed   = false(numRuns,1);
    t_land   = nan(numRuns,1);
    xy_final = nan(numRuns,1);

    for k = 1:numRuns
        q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        try
            tmp = run_simulation(x0,trajType,ovr,1.0,cfg_override,1000+k);
        catch ME
            fprintf('  Run %d ERR: %s\n',k,ME.message);
            continue;
        end
        if isfield(tmp,'success'),  landed(k)  = tmp.success;  end
        if isfield(tmp,'final_t'),  t_land(k)  = tmp.final_t;  end
        if isfield(tmp,'final_xy'), xy_final(k)= tmp.final_xy; end
    end

    n_land  = sum(landed);
    mean_t  = mean(t_land(landed),'omitnan');
    mean_xy = mean(xy_final(landed),'omitnan');
    max_xy  = max(xy_final(landed),[],'omitnan');
    if isempty(mean_t)||isnan(mean_t),  mean_t  = NaN; end
    if isempty(mean_xy)||isnan(mean_xy),mean_xy = NaN; end
    if isempty(max_xy)||isnan(max_xy),  max_xy  = NaN; end
end
