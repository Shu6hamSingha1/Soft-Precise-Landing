%% VALIDATE TOP SWEEP CANDIDATES ACROSS TRAJECTORIES
% Runs baseline + top-3 candidates from sweep_Linear_outer
% on Static, Linear, Sinusoidal, Circular, Lissajous (5 ICs each).
%
% Output: Datasets/validate_candidates.mat

clc; clear;

trajList = ["Static","Linear","Sinusoidal","Circular","Lissajous"];

% 5 ICs (matches multi_Init_Var.m)
p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2,2,-5;
    -2,-2,-5
];
numRuns = size(p0,1);

% Candidates: name + override struct
cand(1).name = 'BASELINE_S2guard';
cand(1).ovr  = struct();

n_cand = numel(cand);
n_traj = numel(trajList);

% results(c,t) -> aggregate
results = struct('cand',{},'traj',{},'n_land',{},'mean_t',{}, ...
                 'mean_xy',{},'max_xy',{},'fail_ICs',{});

row = 0;
for t = 1:n_traj
    trajType = trajList(t);
    for c = 1:n_cand
        row = row + 1;
        results(row).cand = cand(c).name;
        results(row).traj = trajType;

        fprintf('\n=== %-14s on %-10s ===\n', cand(c).name, trajType);
        [nl,mt,mxy,maxxy,failIC] = run_5ic(p0,numRuns,trajType,cand(c).ovr);
        results(row).n_land   = nl;
        results(row).mean_t   = mt;
        results(row).mean_xy  = mxy;
        results(row).max_xy   = maxxy;
        results(row).fail_ICs = failIC;
        fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f  fails=[%s]\n', ...
                nl,mt,mxy,maxxy,num2str(failIC));
    end
end

% Save
datasetDir = fullfile(fileparts(mfilename('fullpath')),'Datasets');
if ~exist(datasetDir,'dir'), mkdir(datasetDir); end
saveFile = fullfile(datasetDir,'validate_candidates.mat');
save(saveFile,'results','cand','trajList');
fprintf('\nSaved to %s\n',saveFile);

% Print final pivot table
fprintf('\n========================================================\n');
fprintf('%-14s', 'candidate');
for t = 1:n_traj, fprintf(' %-12s', trajList(t)); end
fprintf('\n--------------------------------------------------------\n');
for c = 1:n_cand
    fprintf('%-14s', cand(c).name);
    for t = 1:n_traj
        idx = (t-1)*n_cand + c;
        fprintf(' %d/5 %5.2fs ', results(idx).n_land, results(idx).mean_t);
    end
    fprintf('\n');
end

% =====================================================================
function [n_land,mean_t,mean_xy,max_xy,fail_ICs] = run_5ic(p0,numRuns,trajType,ovr)
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

    n_land   = sum(landed);
    mean_t   = mean(t_land(landed),'omitnan');
    mean_xy  = mean(xy_final(landed),'omitnan');
    max_xy   = max(xy_final(landed),[],'omitnan');
    fail_ICs = find(~landed)';
    if isempty(mean_t)||isnan(mean_t),  mean_t  = NaN; end
    if isempty(mean_xy)||isnan(mean_xy),mean_xy = NaN; end
    if isempty(max_xy)||isnan(max_xy),  max_xy  = NaN; end
end
