%% VALIDATE COMBO2 — fresh deep-sweep winners (2026-04-16)
% Baseline (post-retune lock): zp=diag([6,6]), Gamma=diag([0.4375,0.5,0.75]),
% p_2inf=[2.5;2.5;1.5].
%
% Top strict dual-Pareto candidates from sweep_deep_rerun.log:
%   zp           x1.50  -> diag([9,9])           (best xy: -32.3%)
%   Gamma(3)     x1.50  -> diag([.4375,.5,1.125])(best dual: -28.3% t, -4.6% xy)
%   p_2inf(3)    x0.50  -> [2.5;2.5;0.75]        (fastest: -29.2% t)
%
% Probe orthogonality of the two axes (precision vs descent rate):
%   COMBO_A = zp*1.5 + Gamma(3)*1.5     (precision + speed-via-Gamma)
%   COMBO_B = zp*1.5 + p_2inf(3)*0.5    (precision + speed-via-funnel)
%   COMBO_C = zp*1.5 + Gamma(3)*1.5 + p_2inf(3)*0.5  (triple)
%
% 4 moving trajectories x 5 ICs = 20 runs per cfg.

clc; clear;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..'));
addpath(fullfile(this_dir, '..', '..', 'Common'));
addpath(fullfile(this_dir, '..', '..', 'Comparison'));

trajList = ["Linear","Sinusoidal","Circular","Lissajous"];
p0 = [0,0,-5; 0,0,-7; 0,0,-3; 2,2,-5; -2,-2,-5];
numRuns = size(p0,1);

% Pull current locked bases straight from K_PLASMC (same auto-sync
% pattern as sweep_deep.m — no hardcoded stale values).
tmp_bp = load(fullfile(this_dir, '..', '..', 'Common', 'bestParam.mat'));
K = tmp_bp.K;  %#ok<NASGU>
InitGains_Comparison;
base_zp     = K_PLASMC.zp;
base_Gamma  = K_PLASMC.Gamma;
base_p2inf  = K_PLASMC.p_2inf;

fprintf('Locked bases pulled from K_PLASMC:\n');
fprintf('  zp     = diag([%g %g])\n', base_zp(1,1), base_zp(2,2));
fprintf('  Gamma  = diag([%g %g %g])\n', base_Gamma(1,1), base_Gamma(2,2), base_Gamma(3,3));
fprintf('  p_2inf = [%g %g %g]\n\n', base_p2inf(1), base_p2inf(2), base_p2inf(3));

% Build overrides
zp_15     = 1.5 * base_zp;
Gamma_z15 = base_Gamma; Gamma_z15(3,3) = 1.5 * Gamma_z15(3,3);
p2inf_z05 = base_p2inf; p2inf_z05(3)   = 0.5 * p2inf_z05(3);

ovr_base  = struct();
ovr_zp    = struct('zp', zp_15);
ovr_gz    = struct('Gamma', Gamma_z15);
ovr_pz    = struct('p_2inf', p2inf_z05);
ovr_A     = struct('zp', zp_15, 'Gamma', Gamma_z15);
ovr_B     = struct('zp', zp_15, 'p_2inf', p2inf_z05);
ovr_C     = struct('zp', zp_15, 'Gamma', Gamma_z15, 'p_2inf', p2inf_z05);

configs = {
    'baseline',          ovr_base;
    'zp x1.5',           ovr_zp;
    'Gamma(z) x1.5',     ovr_gz;
    'p_2inf(z) x0.5',    ovr_pz;
    'COMBO_A (zp+Gz)',   ovr_A;
    'COMBO_B (zp+pz)',   ovr_B;
    'COMBO_C (zp+Gz+pz)',ovr_C;
};

n_cfg  = size(configs,1);
n_traj = numel(trajList);

results = struct('config',{},'traj',{}, ...
                 'n_land',{},'mean_t',{},'mean_xy',{},'max_xy',{});
row = 0;
agg  = struct('config',{},'minL',{},'aggT',{},'maxXY',{});
for c = 1:n_cfg
    name = configs{c,1};
    ovr  = configs{c,2};
    fprintf('\n=== %s ===\n', name);
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
        fprintf('  %-10s  land=%d/5  mean_t=%.2f  max_xy=%.4f\n', ...
                trajType, nl, mt, maxxy);
    end
    slice = results(row-n_traj+1:row);
    agg(c).config = name;
    agg(c).minL   = min([slice.n_land]);
    agg(c).aggT   = mean([slice.mean_t],'omitnan');
    agg(c).maxXY  = max([slice.max_xy]);
    fprintf('  AGG: minL=%d/5  aggT=%.3f  maxXY=%.4f\n', ...
            agg(c).minL, agg(c).aggT, agg(c).maxXY);
end

% Save
dsDir = fullfile(this_dir,'Datasets');
if ~exist(dsDir,'dir'), mkdir(dsDir); end
save(fullfile(dsDir,'validate_combo2.mat'),'results','agg','configs');
fprintf('\nSaved validate_combo2.mat\n');

% Verdict
fprintf('\n================== SUMMARY ==================\n');
fprintf('%-22s %-6s %-9s %-9s %-9s %-9s\n', ...
        'config','minL','aggT','dT%','maxXY','dXY%');
fprintf('---------------------------------------------------------------\n');
baseT = agg(1).aggT; baseXY = agg(1).maxXY;
for c = 1:n_cfg
    dT  = 100*(agg(c).aggT - baseT)/baseT;
    dXY = 100*(agg(c).maxXY - baseXY)/baseXY;
    fprintf('%-22s %-6s %-9.3f %+8.1f  %-9.4f %+8.1f\n', ...
            agg(c).config, sprintf('%d/5',agg(c).minL), ...
            agg(c).aggT, dT, agg(c).maxXY, dXY);
end

% =====================================================================
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
