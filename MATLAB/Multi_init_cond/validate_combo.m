%% VALIDATE COMBINED CANDIDATE
% Lock-in test for the two deep-sweep winners:
%   kappa_0 = [0.125; 0.125; 0.25]   (×1.25 from baseline)
%   Gamma   = diag([0.1875, 0.1875, 0.5])   (lateral ×0.75, vertical unchanged)
%
% Phase 1: single-knob replays (for reference)
% Phase 2: combined override across all 4 moving trajectories, 5 ICs
% Phase 3: mini speed sweep on the combined controller to confirm the
%          robustness envelope vs baseline multipliers

clc; clear;

trajList = ["Linear","Sinusoidal","Circular","Lissajous"];
p0 = [0,0,-5; 0,0,-7; 0,0,-3; 2,2,-5; -2,-2,-5];
numRuns = size(p0,1);

% Candidate overrides
% NOTE: deep sweep's "Gamma idx=1 x0.75" only scales Gamma(1,1) (x-lateral),
% leaving Gamma(2,2)=0.25 and Gamma(3,3)=0.5 untouched.
ovr_kappa = struct('kappa_0', [0.125; 0.125; 0.25]);
ovr_gamma = struct('Gamma',   diag([0.1875, 0.25, 0.5]));
ovr_combo = struct('kappa_0', [0.125; 0.125; 0.25], ...
                   'Gamma',   diag([0.1875, 0.25, 0.5]));

fprintf('============================================================\n');
fprintf('PHASE 1: SINGLE-KNOB REPLAY (cross-traj)\n');
fprintf('============================================================\n');
phase1 = run_phase(p0, numRuns, trajList, ...
                   {'baseline', struct(); ...
                    'kappa_0 only', ovr_kappa; ...
                    'Gamma(1) only', ovr_gamma});

fprintf('\n============================================================\n');
fprintf('PHASE 2: COMBINED OVERRIDE\n');
fprintf('============================================================\n');
phase2 = run_phase(p0, numRuns, trajList, ...
                   {'COMBO', ovr_combo});

fprintf('\n============================================================\n');
fprintf('PHASE 3: SPEED SWEEP ON COMBO CONTROLLER\n');
fprintf('============================================================\n');
mults = [1.0, 1.25, 1.5, 1.75, 2.0];  % bracket prior speed-sweep edges
phase3 = struct('traj',{},'mult',{},'n_land',{},'mean_t',{},'max_xy',{});
row = 0;
for t = 1:numel(trajList)
    trajType = trajList(t);
    fprintf('\n--- SPEED %s ---\n', trajType);
    stopped = false;
    for m = mults
        row = row + 1;
        phase3(row).traj = trajType;
        phase3(row).mult = m;
        [nl,mt,mxy,maxxy,failIC] = run_5ic_speed(p0,numRuns,trajType,m,ovr_combo);
        phase3(row).n_land = nl;
        phase3(row).mean_t = mt;
        phase3(row).mean_xy = mxy;
        phase3(row).max_xy = maxxy;
        phase3(row).fail_ICs = failIC;
        fprintf('  mult=%.2f  land=%d/5  mean_t=%.2f  max_xy=%.4f  fails=[%s]\n', ...
                m,nl,mt,maxxy,num2str(failIC));
        if nl < numRuns
            fprintf('  -> first failure at mult=%.2f, stopping %s\n', m, trajType);
            stopped = true;
            break;
        end
    end
    if ~stopped
        fprintf('  -> %s still 5/5 at max mult %.2f\n', trajType, mults(end));
    end
end

% Save
dsDir = fullfile(fileparts(mfilename('fullpath')),'Datasets');
if ~exist(dsDir,'dir'), mkdir(dsDir); end
save(fullfile(dsDir,'validate_combo.mat'), 'phase1','phase2','phase3','mults');
fprintf('\nSaved validate_combo.mat\n');

% Final verdict
fprintf('\n================== VERDICT ==================\n');
base_minL = min([phase1(1,:).n_land]);
base_aggT = mean([phase1(1,:).mean_t],'omitnan');
base_maxXY= max([phase1(1,:).max_xy]);
combo_minL = min([phase2(1,:).n_land]);
combo_aggT = mean([phase2(1,:).mean_t],'omitnan');
combo_maxXY= max([phase2(1,:).max_xy]);
fprintf('Baseline     : minL=%d/5  aggT=%.3f  maxXY=%.4f\n', base_minL, base_aggT, base_maxXY);
fprintf('COMBO        : minL=%d/5  aggT=%.3f  maxXY=%.4f\n', combo_minL, combo_aggT, combo_maxXY);
if combo_minL >= numRuns
    dT  = combo_aggT - base_aggT;
    dXY = combo_maxXY - base_maxXY;
    fprintf('delta        : aggT %+.3f (%.1f%%),  maxXY %+.4f (%.1f%%)\n', ...
            dT, 100*dT/base_aggT, dXY, 100*dXY/base_maxXY);
    if dT < -1e-3 && dXY < 1e-3
        fprintf('STATUS: CLEAR IMPROVEMENT — safe to lock in\n');
    elseif dT < 1e-3 && dXY < -1e-3
        fprintf('STATUS: CLEAR IMPROVEMENT — safe to lock in\n');
    elseif dT < -1e-3 && dXY < -1e-3
        fprintf('STATUS: DOUBLE IMPROVEMENT — strong lock-in\n');
    else
        fprintf('STATUS: Trade — degraded one metric, check interaction\n');
    end
else
    fprintf('STATUS: REGRESSION — combined override drops 5/5; single knobs did not\n');
end

% =====================================================================
function results = run_phase(p0, numRuns, trajList, configs)
% configs: cell array of {name, ovr}
    n_cfg = size(configs,1);
    n_traj = numel(trajList);
    results = struct('config',{},'traj',{}, ...
                     'n_land',{},'mean_t',{},'mean_xy',{},'max_xy',{});
    % rows = configs, cols = trajs (but flatten to linear struct array)
    row = 0;
    for c = 1:n_cfg
        name = configs{c,1};
        ovr  = configs{c,2};
        fprintf('\n--- %s ---\n', name);
        for t = 1:n_traj
            trajType = trajList(t);
            [nl,mt,mxy,maxxy] = run_5ic(p0,numRuns,trajType,ovr);
            row = row + 1;
            results(row).config  = name;
            results(row).traj    = trajType;
            results(row).n_land  = nl;
            results(row).mean_t  = mt;
            results(row).mean_xy = mxy;
            results(row).max_xy  = maxxy;
            fprintf('  %-12s  land=%d/5  mean_t=%.2f  max_xy=%.4f\n', ...
                    trajType, nl, mt, maxxy);
        end
        minL = min([results(row-n_traj+1:row).n_land]);
        aggT = mean([results(row-n_traj+1:row).mean_t],'omitnan');
        mxy  = max([results(row-n_traj+1:row).max_xy]);
        fprintf('  AGG: minL=%d/5  aggT=%.3f  maxXY=%.4f\n', minL, aggT, mxy);
    end
end

function [n_land,mean_t,mean_xy,max_xy] = run_5ic(p0,numRuns,trajType,ovr)
    landed=false(numRuns,1); t_l=nan(numRuns,1); xy_f=nan(numRuns,1);
    for k=1:numRuns
        q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        rng(1000+k);
        try
            tmp = run_simulation(x0,trajType,ovr);
        catch ME
            fprintf('  Run %d ERR: %s\n',k,ME.message); continue;
        end
        if isfield(tmp,'success'),  landed(k)=tmp.success;  end
        if isfield(tmp,'final_t'),  t_l(k)=tmp.final_t;     end
        if isfield(tmp,'final_xy'), xy_f(k)=tmp.final_xy;   end
    end
    n_land=sum(landed);
    mean_t=mean(t_l(landed),'omitnan');
    mean_xy=mean(xy_f(landed),'omitnan');
    max_xy=max(xy_f(landed),[],'omitnan');
    if isempty(mean_t)||isnan(mean_t), mean_t=NaN; end
    if isempty(mean_xy)||isnan(mean_xy), mean_xy=NaN; end
    if isempty(max_xy)||isnan(max_xy), max_xy=NaN; end
end

function [n_land,mean_t,mean_xy,max_xy,fail_ICs] = run_5ic_speed(p0,numRuns,trajType,speed_mult,ovr)
    landed=false(numRuns,1); t_l=nan(numRuns,1); xy_f=nan(numRuns,1);
    for k=1:numRuns
        q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
        x0=[p0(k,:)'; q0; v0; w0];
        rng(1000+k);
        try
            tmp = run_simulation(x0,trajType,ovr,speed_mult);
        catch ME
            fprintf('  Run %d ERR: %s\n',k,ME.message); continue;
        end
        if isfield(tmp,'success'),  landed(k)=tmp.success;  end
        if isfield(tmp,'final_t'),  t_l(k)=tmp.final_t;     end
        if isfield(tmp,'final_xy'), xy_f(k)=tmp.final_xy;   end
    end
    n_land=sum(landed);
    mean_t=mean(t_l(landed),'omitnan');
    mean_xy=mean(xy_f(landed),'omitnan');
    max_xy=max(xy_f(landed),[],'omitnan');
    fail_ICs=find(~landed)';
    if isempty(mean_t)||isnan(mean_t), mean_t=NaN; end
    if isempty(mean_xy)||isnan(mean_xy), mean_xy=NaN; end
    if isempty(max_xy)||isnan(max_xy), max_xy=NaN; end
end
