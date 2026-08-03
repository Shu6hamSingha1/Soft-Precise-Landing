%% CB_LOCK_FULLTEST  Full validation of the LOCKED config.
%   LOCKED = COMBINED + pr-tight + E.5 (baked p_h):
%     kappa0=[.05;.05;.05], N=diag(.10), Pleak=[.5;.5;1.5], E=[.5;.5;.5],
%     Xi_r=diag([.3,.3]), p_rinf=[.85;.85].  (p_rinf<1 dips below Standing Cond 1.)
%   Canonical multi-init: 5 traj x 5 IC x {noiseless, realistic} = 50. Plus stress 3/5/7x.
%   Reports SP, precision (xy), softness (v), fly-aways, SP-signal health (engR/engH<1,
%   kappa bounded). Saves to Datasets/MultiInit/LOCKED_test/.
%
% Run:  cd MATLAB/Multi_init_cond; cb_lock_fulltest
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LOCKED = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
                'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
outDir = fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','LOCKED_test');
if ~exist(outDir,'dir'), mkdir(outDir); end

G=struct('traj',{},'tag',{},'ic',{},'sp',{},'xy',{},'v',{},'t',{},'fly',{},'engR',{},'engH',{},'kmax',{});
for cfgL = {{'noiseless',0},{'realistic',1}}
    tag=cfgL{1}{1}; NZ=cfgL{1}{2}; co=struct('NOISE',NZ,'GE',1,'delay',1);
    for t=1:numel(trajList)
        for ic=1:5
            VDF_OVERRIDE=LOCKED; STRESS_SCALE=[];
            r=run_simulation([p0(ic,:)';1;0;0;0;zeros(6,1)],trajList(t),[],1.0,co,1);
            d=r.data; idx=d.idx;
            engR=max(max(abs(d.s_e_log(:,1:idx)./d.P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],1));
            engH=max(max(abs(d.V_h_e(1:2,1:idx))./max(d.p_h_log(1:2,1:idx),eps),[],1));
            kmax=max(max(d.kappa_log(1:2,1:idx),[],2));
            e=numel(G)+1; G(e)=struct('traj',trajList(t),'tag',tag,'ic',ic,'sp',r.precise&&r.soft, ...
                'xy',r.final_xy,'v',r.final_rel_vel,'t',r.final_t,'fly',~r.success||r.final_alt>0.25, ...
                'engR',engR,'engH',engH,'kmax',kmax);
        end
    end
end

fprintf('\n==================== LOCKED FULL TEST ====================\n');
for tg=["noiseless","realistic"]
    m=arrayfun(@(g)strcmp(g.tag,tg),G); g=G(m);
    fprintf('%-10s SP %2d/25 | xy mean %.3f worst %.3f | v mean %.3f worst %.3f | fly %d | engR<1:%d engH<1:%d kmax %.2f\n', ...
        tg, sum([g.sp]),numel(g), mean([g.xy]),max([g.xy]), mean([g.v]),max([g.v]), sum([g.fly]), ...
        all([g.engR]<1), all([g.engH]<1), max([g.kmax]));
end
fprintf('\nFailures:\n');
for g=G, if ~g.sp, fprintf('  %-11s %-9s IC%d xy=%.3f v=%.3f engR=%.2f engH=%.2f\n',g.traj,g.tag,g.ic,g.xy,g.v,g.engR,g.engH); end, end

% ---- stress bracket 3/5/7x (5 engaging cells) ----
scells={'Sinusoidal',2;'Sinusoidal',4;'Sinusoidal',5;'Circular',3;'Circular',5};
fprintf('\n--- stress (5 engaging cells) ---\n');
for s=[3,5,7]
    sp=0; xy=[];
    for c=1:size(scells,1)
        VDF_OVERRIDE=LOCKED; STRESS_SCALE=s;
        r=run_simulation([p0(scells{c,2},:)';1;0;0;0;zeros(6,1)],scells{c,1},[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
        sp=sp+(r.precise&&r.soft); xy(end+1)=r.final_xy; %#ok<AGROW>
    end
    fprintf('  %dx: SP %d/5  mean xy %.3f\n', s, sp, mean(xy));
end
clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(outDir,'LOCKED_grand.mat'),'G','LOCKED');
fprintf('\nSaved -> Datasets/MultiInit/LOCKED_test/\nDONE\n');
