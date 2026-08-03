%% CB_ENGAGE_PROBE  Funnel / kappa ENGAGEMENT probe (tuning iteration loop)
%
% Goal (2026-06-26): retune so BOTH funnels (p_r position, p_h optic-flow) engage
% and kappa visibly adapts -- "engagement first, revalidate later" (user decision).
% The baked 25/25 config keeps errors so tiny that the wide funnels sit idle and
% kappa just leakage-decays. This probe drives a CANDIDATE config via VDF_OVERRIDE
% (baked vdf_params.m UNTOUCHED -> fully reversible) and scores engagement.
%
% Engagement metrics (per cell, per axis, max over the flown trajectory):
%   opt-flow  engH_k = max |V_h_e(k,:)| / p_h_log(k,:)      (-> 1 = funnel hard at work)
%   position  engR_k = max |r_bar_e(k,:)| / p_r_log(k,:),  r_bar_e = s_e ./ phi_max
%   kappa     dK_k   = max kappa_log(k,:) - kappa0(k)       (-> growth above pre-load)
%
% Run from MATLAB:  cd MATLAB/Multi_init_cond; cb_engage_probe
% Saves: ../Datasets/MultiInit/engage_probe.mat  (read by the analysis side)
clc; clear;

mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

% ===================== CANDIDATE CONFIG (edit me each iteration) ==============
global VDF_OVERRIDE              %#ok<GVMIS>
VDF_OVERRIDE = struct();
VDF_OVERRIDE.theta_per_axis = true;          % current formulation (mandatory)
% Tighten BOTH funnels via the floor p_inf + contraction rate Xi (NOT p0, which is
% FoV-derived). Faster Xi pulls p(t) down to the lower floor sooner; the tighter
% terminal envelope is what the (small) error fills -> engagement + barrier activity.
VDF_OVERRIDE.Xi_h   = diag([0.4, 0.4, 0.4]); % baked diag([.2,.2,.2]) -> faster optic-flow contraction
VDF_OVERRIDE.p_hinf = [0.6; 0.6; 1.0];       % baked [1;1;1.5]       -> lower optic-flow floor
VDF_OVERRIDE.Xi_r   = diag([0.2, 0.2]);      % baked diag([.1,.1])   -> faster position contraction
VDF_OVERRIDE.p_rinf = [0.85; 0.85];          % baked [1;1] -- BELOW Standing Cond. 1 (>=1); test-only
VDF_OVERRIDE.kappa0 = [0.05; 0.05; 0.10];    % baked [.125;.125;.25] -> room to rise
VDF_OVERRIDE.N      = diag([0.06,0.06,0.06]);% baked 0.02            -> faster adaptation
CONFIG_TAG = 'cand1_Xi_pinf_k05_N06';
% =============================================================================

% Subset for fast iteration; scale to all 5 once engagement looks right.
trajList   = ["Linear", "Sinusoidal", "Circular"];
speed_mult = 1.0;
cfg_override = struct('NOISE',1, 'GE',1, 'delay',1);   % realistic only

p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
numRuns = size(p0,1);

probe = struct('traj',{},'ic',{},'p0',{}, ...
               'success',{},'precise',{},'soft',{},'sp',{}, ...
               'xy',{},'vel',{},'t',{}, ...
               'engH',{},'engR',{},'dK',{},'kappa_end',{});

fprintf('\n=== ENGAGEMENT PROBE  [%s] ===\n', CONFIG_TAG);
for tIdx = 1:numel(trajList)
    trajType = trajList(tIdx);
    for k = 1:numRuns
        q0 = [1;0;0;0];
        x0 = [p0(k,:)'; q0; zeros(3,1); zeros(3,1)];
        r  = run_simulation(x0, trajType, [], speed_mult, cfg_override, 1);

        d   = r.data;
        idx = d.idx;
        P   = d.P;

        He  = abs(d.V_h_e(:,1:idx))     ./ max(d.p_h_log(:,1:idx), eps);
        Rbe = abs(d.s_e_log(:,1:idx) ./ P.phi_max(:)) ./ max(d.p_r_log(:,1:idx), eps);
        engH = max(He,  [], 2);     % 3x1
        engR = max(Rbe, [], 2);     % 2x1
        dK   = max(d.kappa_log(:,1:idx), [], 2) - P.kappa0(:);   % 3x1

        sp = r.precise && r.soft;
        e = numel(probe) + 1;
        probe(e).traj=trajType; probe(e).ic=k; probe(e).p0=p0(k,:);
        probe(e).success=r.success; probe(e).precise=r.precise; probe(e).soft=r.soft; probe(e).sp=sp;
        probe(e).xy=r.final_xy; probe(e).vel=r.final_rel_vel; probe(e).t=r.final_t;
        probe(e).engH=engH(:)'; probe(e).engR=engR(:)'; probe(e).dK=dK(:)';
        probe(e).kappa_end=d.kappa_log(:,idx)';

        fprintf(['%-11s IC%d  SP=%d (p=%d s=%d) xy=%.3f v=%.3f t=%.2f | ' ...
                 'engH=[%.2f %.2f %.2f] engR=[%.2f %.2f] dK=[%.3f %.3f %.3f]\n'], ...
                 trajType, k, sp, r.precise, r.soft, r.final_xy, r.final_rel_vel, r.final_t, ...
                 engH(1),engH(2),engH(3), engR(1),engR(2), dK(1),dK(2),dK(3));
    end
end

nSP = sum([probe.sp]);  nTot = numel(probe);
fprintf('\n--- SP %d/%d  |  mean engH=%.2f engR=%.2f  max dK=%.3f ---\n', ...
        nSP, nTot, mean(cellfun(@max,{probe.engH})), ...
        mean(cellfun(@max,{probe.engR})), max(cellfun(@max,{probe.dK})));

outDir = fullfile(fileparts(mfilename('fullpath')), '..', 'Datasets', 'MultiInit');
if ~exist(outDir,'dir'), mkdir(outDir); end
save(fullfile(outDir,'engage_probe.mat'), 'probe', 'CONFIG_TAG', 'p0', 'trajList');
fprintf('\nSaved -> Datasets/MultiInit/engage_probe.mat\n');
