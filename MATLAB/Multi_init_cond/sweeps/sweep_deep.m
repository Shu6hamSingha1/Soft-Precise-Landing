%% PLASMC DEEP PARAMETER SWEEP — CROSS-TRAJECTORY
% Sweeps every outer + inner control parameter at mults [0.5, 0.75, 1.25, 1.5]
% across all 4 moving trajectories (Linear, Sinusoidal, Circular, Lissajous)
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
addpath(fullfile(this_dir, '..'));            % run_simulation, InitVar, etc.
addpath(fullfile(this_dir, '..', '..', 'Common'));  % traj_Gen canonical

trajList = ["Linear","Sinusoidal","Circular","Lissajous"];

p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2,2,-5;
    -2,-2,-5
];
numRuns = size(p0,1);

mults = [0.5, 0.75, 1.25, 1.5];

% ---------------------------------------------------------------------
% Parameter list: {name, baseline, index, group}
%   index = 0  -> scale all diag/vec entries together
%   index > 0  -> scale only that 1-based entry
% ---------------------------------------------------------------------
% NOTE: Baselines below must match current K_ctrl in run_simulation.m.
% 2026-04-14 post-retune lock: widened p_20, tightened p_1inf/p_2inf, raised
% Gamma/kappa_0, replaced attitude PID with geometric SO(3) (kR, kOmega),
% and retuned yaw ASMC (Omega_a, Gamma_a, n_a, kappa_a_0, E_a).
param_list = {
    % ============== OUTER ==============
    % Visibility funnel (zeta_1 / image error)
    'gamma_1', [0.2,0.2],              0, 'outer';
    'p_1inf',  [0.08;0.08],            0, 'outer';
    % Outer PID on zeta_1
    'zp',      diag([6,6]),            0, 'outer';
    'zi',      diag([0.1,0.1]),        0, 'outer';
    'zd',      diag([1.3,1.3]),        0, 'outer';
    % Optical-flow funnel (zeta_2)
    'gamma_2', [0.2,0.2,0.2],          0, 'outer';
    'p_20',    [25;25;8],              1, 'outer';   % lateral
    'p_20',    [25;25;8],              3, 'outer';   % vertical
    'p_2inf',  [0.8;0.8;1.0],          1, 'outer';
    'p_2inf',  [0.8;0.8;1.0],          3, 'outer';
    % Sliding-mode law
    'Omega',   diag([0.003,0.003,0.006]),  1, 'outer'; % lateral
    'Omega',   diag([0.003,0.003,0.006]),  3, 'outer'; % vertical
    'Gamma',   diag([0.4375,0.5,0.75]),    1, 'outer'; % x (lock-broken)
    'Gamma',   diag([0.4375,0.5,0.75]),    2, 'outer'; % y
    'Gamma',   diag([0.4375,0.5,0.75]),    3, 'outer'; % z
    'E',       diag([1.0,1.0,0.5]),        1, 'outer';
    'E',       diag([1.0,1.0,0.5]),        3, 'outer';
    % Adaptive law
    'N',       diag([0.02,0.02,0.05]),     0, 'outer';
    'P',       diag([1.5,1.5,5]),          0, 'outer';
    'kappa_0', [0.125;0.125;0.25],         0, 'outer';

    % ============== INNER ==============
    % Geometric SO(3) attitude (replaces former PID block)
    'kR',        diag([1.5, 1.5, 0.5]),    1, 'inner'; % roll/pitch (x=y)
    'kR',        diag([1.5, 1.5, 0.5]),    3, 'inner'; % yaw
    'kOmega',    diag([0.3, 0.3, 0.1]),    1, 'inner';
    'kOmega',    diag([0.3, 0.3, 0.1]),    3, 'inner';
    % Yaw adaptive SMC
    'Omega_a',   0.5,   0, 'inner';
    'Gamma_a',   0.5,   0, 'inner';
    'n_a',       1.0,   0, 'inner';
    'p_a',       2,     0, 'inner';
    'kappa_a_0', 2.0,   0, 'inner';
    'E_a',       3.0,   0, 'inner';
};

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
    [nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,struct());
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
            [nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,ovr);
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
function [n_land,mean_t,mean_xy,max_xy] = run_5ic(p0,numRuns,trajType,ovr)
    landed   = false(numRuns,1);
    t_land   = nan(numRuns,1);
    xy_final = nan(numRuns,1);

    for k = 1:numRuns
        q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        rng(1000+k);
        try
            tmp = run_simulation(x0,trajType,ovr);
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
