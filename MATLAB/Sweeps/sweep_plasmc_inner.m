%% PLASMC INNER-LOOP PARAMETER SWEEP
% Sweeps attitude PID (roll/pitch) + yaw ASMC parameters.
% Same harness as sweep_plasmc.m but for inner-loop knobs.
%
% Output: Datasets/sweep_<trajType>_inner.mat

clc; clear;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'Multi_init_cond'));
addpath(fullfile(this_dir, '..', 'Common'));

trajType = "Linear";
group    = "inner";

% 5 ICs (matches multi_Init_Var.m)
p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2,2,-5;
    -2,-2,-5
];
numRuns = size(p0,1);

mults = [0.5, 0.75, 1.25, 1.5];

% --- Parameter list ---
% {name, baseline, index}  index=0 -> scale all diag entries
param_list = {
    % --- Attitude PID (roll/pitch) ---
    'ep', diag([5.0, 5.0]),       0;
    'ei', diag([0.1, 0.1]),       0;
    'ed', diag([0.1, 0.1]),       0;
    % --- Body-rate PID (full 3-axis) ---
    'wp', diag([5.0, 5.0, 5.0]),  0;
    'wi', diag([0.01,0.01,0.1]),  0;
    'wd', diag([0.1, 0.1, 0.2]),  0;
    'ff', diag([0.1, 0.1, 0.1]),  0;
    % --- Yaw adaptive SMC (scalars) ---
    'Omega_a',   1.5,  0;
    'Gamma_a',   0.3,  0;
    'n_a',       0.05, 0;
    'p_a',       2,    0;
    'kappa_a_0', 0.1,  0;
    'E_a',       2.5,  0;
};

n_params = size(param_list,1);
fprintf('=== PLASMC INNER sweep on %s, %d params x %d mults x %d ICs ===\n', ...
        trajType, n_params, numel(mults), numRuns);

sweep = struct('param',{},'index',{},'mult',{},'value',{}, ...
               'n_land',{},'mean_t',{},'mean_xy',{},'max_xy',{});

row = 0;

% Baseline
row = row + 1;
sweep(row).param = 'BASELINE';
sweep(row).index = 0;
sweep(row).mult  = 1;
sweep(row).value = NaN;
fprintf('\n--- BASELINE ---\n');
[nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,struct());
sweep(row).n_land=nl; sweep(row).mean_t=mt;
sweep(row).mean_xy=mxy; sweep(row).max_xy=maxxy;
fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f\n',nl,mt,mxy,maxxy);

for p = 1:n_params
    name = param_list{p,1};
    base = param_list{p,2};
    idx  = param_list{p,3};

    for m = mults
        row = row + 1;
        sweep(row).param = name;
        sweep(row).index = idx;
        sweep(row).mult  = m;

        ovr = struct();
        if isscalar(base)
            ovr.(name) = base * m;
            sweep(row).value = base * m;
        elseif isvector(base)
            v = base;
            if idx == 0, v = v * m; else, v(idx) = v(idx) * m; end
            ovr.(name) = v;
            sweep(row).value = v(max(idx,1));
        elseif ismatrix(base) && size(base,1)==size(base,2)
            d = diag(base);
            if idx == 0, d = d * m; else, d(idx) = d(idx) * m; end
            ovr.(name) = diag(d);
            sweep(row).value = d(max(idx,1));
        end

        fprintf('\n--- %s (idx=%d) x%.2f -> %.4f ---\n', name, idx, m, sweep(row).value);
        [nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,ovr);
        sweep(row).n_land=nl; sweep(row).mean_t=mt;
        sweep(row).mean_xy=mxy; sweep(row).max_xy=maxxy;
        fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f\n',nl,mt,mxy,maxxy);
    end
end

datasetDir = fullfile(fileparts(mfilename('fullpath')),'Datasets');
if ~exist(datasetDir,'dir'), mkdir(datasetDir); end
saveFile = fullfile(datasetDir, sprintf('sweep_%s_%s.mat',trajType,group));
save(saveFile,'sweep','trajType','param_list','mults');
fprintf('\nSaved sweep to %s\n',saveFile);

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
