%% PLASMC TARGET-SPEED STRESS TEST
% For each moving trajectory (Linear, Sinusoidal, Circular, Lissajous),
% scale the target's motion rate by increasing multipliers until
% land rate drops from 5/5. Uses deterministic seeds (rng(1000+k))
% so the failure edge is repeatable.
%
% Scaling rule per trajectory (see traj_Gen.m):
%   Linear     : v = [0.5;0.5;0] * mult          (linear speedup)
%   Circular   : wz = 0.11 * mult                (chase v = r*wz also scales)
%   Sinusoidal : w0 = 0.8 * mult, v0 = 0.1 * mult (both axes scale)
%   Lissajous  : w1 = -0.8 * mult, w2 = 0.5 * mult
%
% Output: Datasets/sweep_speed.mat

clc; clear;

trajList = ["Linear","Sinusoidal","Circular","Lissajous"];

% 5 ICs (same as multi_Init_Var.m)
p0 = [
    0,0,-5;
    0,0,-7;
    0,0,-3;
    2,2,-5;
    -2,-2,-5
];
numRuns = size(p0,1);

% Step up speed multiplier, stop per-trajectory when land rate < 5/5
mults = [1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0];

results = struct('traj',{},'mult',{},'n_land',{},'mean_t',{}, ...
                 'mean_xy',{},'max_xy',{},'fail_ICs',{});
row = 0;

for t = 1:numel(trajList)
    trajType = trajList(t);
    fprintf('\n########## %s ##########\n', trajType);
    stopped = false;
    for m = mults
        row = row + 1;
        results(row).traj = trajType;
        results(row).mult = m;

        fprintf('\n--- %s  mult=%.2f ---\n', trajType, m);
        [nl,mt,mxy,maxxy,failIC] = run_5ic(p0,numRuns,trajType,m);

        results(row).n_land   = nl;
        results(row).mean_t   = mt;
        results(row).mean_xy  = mxy;
        results(row).max_xy   = maxxy;
        results(row).fail_ICs = failIC;

        fprintf('  land=%d/5  mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f  fails=[%s]\n', ...
                nl,mt,mxy,maxxy,num2str(failIC));

        % Stop this trajectory once we see a failure
        if nl < numRuns
            fprintf('  -> first failure at mult=%.2f, stopping %s sweep\n', m, trajType);
            stopped = true;
            break;
        end
    end
    if ~stopped
        fprintf('  -> %s still 5/5 at max mult=%.2f\n', trajType, mults(end));
    end
end

% Save
datasetDir = fullfile(fileparts(mfilename('fullpath')),'Datasets');
if ~exist(datasetDir,'dir'), mkdir(datasetDir); end
saveFile = fullfile(datasetDir,'sweep_speed.mat');
save(saveFile,'results','mults','trajList');
fprintf('\nSaved sweep to %s\n', saveFile);

% Summary table
fprintf('\n========================================================\n');
fprintf('%-12s %-6s %-7s %-7s %-9s %-9s\n','traj','mult','land','mean_t','mean_xy','max_xy');
fprintf('--------------------------------------------------------\n');
for r = 1:numel(results)
    fprintf('%-12s %-6.2f %-7s %-7.2f %-9.4f %-9.4f\n', ...
            results(r).traj, results(r).mult, ...
            sprintf('%d/5', results(r).n_land), ...
            results(r).mean_t, results(r).mean_xy, results(r).max_xy);
end

% Per-trajectory summary
fprintf('\n========= FAILURE THRESHOLDS =========\n');
for t = 1:numel(trajList)
    tr = trajList(t);
    lastPass = NaN; firstFail = NaN;
    for r = 1:numel(results)
        if results(r).traj ~= tr, continue; end
        if results(r).n_land == numRuns, lastPass = results(r).mult; end
        if results(r).n_land < numRuns && isnan(firstFail)
            firstFail = results(r).mult;
        end
    end
    fprintf('%-12s  last 5/5 at x%.2f,  first failure at x%s\n', ...
            tr, lastPass, num2str(firstFail));
end

% =====================================================================
function [n_land,mean_t,mean_xy,max_xy,fail_ICs] = run_5ic(p0,numRuns,trajType,speed_mult)
    landed   = false(numRuns,1);
    t_land   = nan(numRuns,1);
    xy_final = nan(numRuns,1);

    for k = 1:numRuns
        q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        rng(1000+k);
        try
            tmp = run_simulation(x0,trajType,[],speed_mult);
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
