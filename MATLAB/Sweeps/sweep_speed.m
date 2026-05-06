%% PLASMC TARGET-SPEED STRESS TEST
% For each moving trajectory (Linear, Sinusoidal, Lissajous, Circular),
% scale the target's motion rate by increasing multipliers until
% land rate drops from 5/5. Uses deterministic seeds (rng(1000+k))
% so the failure edge is repeatable.
%
% Scaling rule per trajectory (see traj_Gen.m):
%   Linear     : v = [0.5;0.5;0] * mult          (linear speedup)
%   Circular   : wz = 0.3  * mult                (chase v = r*wz also scales)
%   Sinusoidal : w0 = 0.8  * mult, v0 = 0.1 * mult
%   Lissajous  : w1 = -0.8 * mult, w2 = 0.4 * mult
%
% Per-run outcome tracks precise (xy<=0.08m) and soft (v_rel<=0.2 m/s)
% flags in addition to the raw (t,xy) pair.
%
% Output: Datasets/sweep_speed.mat  +  sweep_speed.log

clc; clear;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'Multi_init_cond'));  % run_simulation, InitVar
addpath(fullfile(this_dir, '..', 'Common'));

trajList = ["Linear","Sinusoidal","Lissajous","Circular"];

% 5 ICs (matched to multi_Init_Var.m — canonical PLASMC validation set)
p0 = [
    0,   0,  -5;
    2.0, 2.0, -5;
    2.0,-2.0, -5;
    2.0, 2.0, -7;
    2.0, 2.0, -3
];
numRuns = size(p0,1);

% Step up speed multiplier, stop per-trajectory when land rate < 5/5
mults = [1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0];

results = struct('traj',{},'mult',{},'n_land',{},'n_precise',{},'n_soft',{}, ...
                 'mean_t',{},'mean_xy',{},'max_xy',{},'fail_ICs',{}, ...
                 'n_fov_fail',{},'fov_fail_ICs',{});
row = 0;

% Match multi_Init_Var.m: pass realistic cfg explicitly so the two
% harnesses stay locked even if InitVar.m defaults ever drift.
cfg_override = struct('NOISE', 1, 'GE', 1, 'delay', 1);

for t = 1:numel(trajList)
    trajType = trajList(t);
    fprintf('\n########## %s ##########\n', trajType);
    stopped = false;
    for m = mults
        row = row + 1;
        results(row).traj = trajType;
        results(row).mult = m;

        fprintf('\n--- %s  mult=%.2f ---\n', trajType, m);
        [nl,npr,nso,mt,mxy,maxxy,failIC,nfov,fovIC] = ...
                run_5ic(p0,numRuns,trajType,m,cfg_override);

        results(row).n_land        = nl;
        results(row).n_precise     = npr;
        results(row).n_soft        = nso;
        results(row).mean_t        = mt;
        results(row).mean_xy       = mxy;
        results(row).max_xy        = maxxy;
        results(row).fail_ICs      = failIC;
        results(row).n_fov_fail    = nfov;
        results(row).fov_fail_ICs  = fovIC;

        fprintf(['  land=%d/5  precise=%d/5  soft=%d/5  ' ...
                 'mean_t=%.2f  mean_xy=%.4f  max_xy=%.4f  ' ...
                 'fails=[%s]  fov_fail=%d/5 [%s]\n'], ...
                nl,npr,nso,mt,mxy,maxxy,num2str(failIC),nfov,num2str(fovIC));

        % Stop this trajectory once we see a landing failure
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
datasetDir = fullfile(this_dir,'Datasets');
if ~exist(datasetDir,'dir'), mkdir(datasetDir); end
saveFile = fullfile(datasetDir,'sweep_speed.mat');
save(saveFile,'results','mults','trajList','p0');
fprintf('\nSaved sweep to %s\n', saveFile);

% Summary table
fprintf('\n======================================================================\n');
fprintf('%-12s %-6s %-7s %-8s %-6s %-9s %-9s %-9s\n', ...
        'traj','mult','land','precise','soft','mean_t','mean_xy','max_xy');
fprintf('----------------------------------------------------------------------\n');
for r = 1:numel(results)
    fprintf('%-12s %-6.2f %-7s %-8s %-6s %-9.2f %-9.4f %-9.4f\n', ...
            results(r).traj, results(r).mult, ...
            sprintf('%d/5', results(r).n_land), ...
            sprintf('%d/5', results(r).n_precise), ...
            sprintf('%d/5', results(r).n_soft), ...
            results(r).mean_t, results(r).mean_xy, results(r).max_xy);
end

% Per-trajectory failure thresholds
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
function [n_land,n_precise,n_soft,mean_t,mean_xy,max_xy,fail_ICs, ...
          n_fov_fail,fov_fail_ICs] = ...
        run_5ic(p0,numRuns,trajType,speed_mult,cfg_override)
    landed   = false(numRuns,1);
    precise  = false(numRuns,1);
    soft     = false(numRuns,1);
    fovfail  = false(numRuns,1);
    t_land   = nan(numRuns,1);
    xy_final = nan(numRuns,1);

    for k = 1:numRuns
        q0=[1;0;0;0]; q0 = q0 / norm(q0);
        v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        try
            % Seed passed explicitly; run_simulation does rng('shuffle')
            % when both K_override and seed are empty, so a bare rng() here
            % gets clobbered. Reproducibility requires seed via 6th arg.
            tmp = run_simulation(x0,trajType,[],speed_mult,cfg_override,1000+k);
        catch ME
            fprintf('  Run %d ERR: %s\n',k,ME.message);
            continue;
        end
        if isfield(tmp,'success'),  landed(k)  = tmp.success;  end
        if isfield(tmp,'final_t'),  t_land(k)  = tmp.final_t;  end
        if isfield(tmp,'final_xy'), xy_final(k)= tmp.final_xy; end
        if isfield(tmp,'precise'),  precise(k) = logical(tmp.precise); end
        if isfield(tmp,'soft'),     soft(k)    = logical(tmp.soft);    end
        if isfield(tmp,'fov_fail'), fovfail(k) = logical(tmp.fov_fail);end
    end

    n_land       = sum(landed);
    n_precise    = sum(precise);
    n_soft       = sum(soft);
    n_fov_fail   = sum(fovfail);
    mean_t       = mean(t_land(landed),'omitnan');
    mean_xy      = mean(xy_final(landed),'omitnan');
    max_xy       = max(xy_final(landed),[],'omitnan');
    fail_ICs     = find(~landed)';
    fov_fail_ICs = find(fovfail)';
    if isempty(mean_t)||isnan(mean_t),  mean_t  = NaN; end
    if isempty(mean_xy)||isnan(mean_xy),mean_xy = NaN; end
    if isempty(max_xy)||isnan(max_xy),  max_xy  = NaN; end
end
