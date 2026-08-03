%% CB_D_FULLTEST  Full multi-init validation of config D (kappa-only engagement).
%   D = prime the adaptive law without touching the funnel:
%       kappa0 = [0.05;0.05;0.05]   (baked [.125;.125;.25])
%       N      = diag([0.10,0.10,0.10])  (baked 0.02)
%   Everything else baked (funnel/chi/Gamma/E/... untouched). Mirrors
%   multi_Init_Var: 5 trajectories x 5 ICs x {noiseless, realistic}.
%   Saves to Datasets/MultiInit/D_test/ so the committed baked .mat are SAFE.
%
% Run:  cd MATLAB/Multi_init_cond; cb_D_fulltest
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;

global VDF_OVERRIDE %#ok<GVMIS>
D_override = struct('theta_per_axis',true, ...
                    'kappa0',[0.05;0.05;0.05], ...
                    'N',diag([0.10,0.10,0.10]));

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cfgList = struct('tag',{'noiseless',''}, 'NOISE',{0,1},'GE',{1,1},'delay',{1,1});

outDir = fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','D_test');
if ~exist(outDir,'dir'), mkdir(outDir); end

grand = struct('traj',{},'tag',{},'ic',{},'sp',{},'precise',{},'soft',{}, ...
               'xy',{},'v',{},'t',{},'fov',{},'engH',{},'engR',{},'dK',{});
for t = 1:numel(trajList)
  traj = trajList(t);
  for cI = 1:numel(cfgList)
    cfg = cfgList(cI);
    tag = cfg.tag; if isempty(tag), tag='realistic'; end
    co = struct('NOISE',cfg.NOISE,'GE',cfg.GE,'delay',cfg.delay);
    fprintf('\n=== %s | %s ===\n', traj, tag);
    results = repmat(struct('success',false,'final_error',0,'final_t',0, ...
        'final_xy',0,'final_alt',0,'final_rel_vel',0,'precise',false,'soft',false, ...
        'fov_fail',false,'fov_fail_t',NaN,'data',[]),1,5);
    for ic = 1:5
      VDF_OVERRIDE = D_override;
      x0 = [p0(ic,:)'; 1;0;0;0; zeros(3,1); zeros(3,1)];
      r = run_simulation(x0, traj, [], 1.0, co, 1);
      results(ic) = r;
      d=r.data; idx=d.idx; P=d.P;
      engH=max(max(abs(d.V_h_e(:,1:idx))./max(d.p_h_log(:,1:idx),eps),[],2));
      engR=max(max(abs(d.s_e_log(:,1:idx)./P.phi_max(:))./max(d.p_r_log(:,1:idx),eps),[],2));
      dK  =max(max(d.kappa_log(:,1:idx),[],2)-P.kappa0(:));
      isSP=r.precise&&r.soft;
      e=numel(grand)+1; grand(e)=struct('traj',traj,'tag',tag,'ic',ic,'sp',isSP, ...
        'precise',r.precise,'soft',r.soft,'xy',r.final_xy,'v',r.final_rel_vel, ...
        't',r.final_t,'fov',r.fov_fail,'engH',engH,'engR',engR,'dK',dK);
      fprintf('  IC%d SP=%d (p=%d s=%d fov=%d) xy=%.3f v=%.3f t=%.2f engH=%.2f engR=%.2f dK=%.3f\n', ...
        ic,isSP,r.precise,r.soft,r.fov_fail,r.final_xy,r.final_rel_vel,r.final_t,engH,engR,dK);
    end
    save(fullfile(outDir,sprintf('%s_multi_init_D%s.mat',traj, ...
         ternary(isempty(cfg.tag),'',['_' cfg.tag]))),'results','p0','traj','co');
  end
end

clear global VDF_OVERRIDE
% ---- summary ----
tags = ["noiseless","realistic"];
fprintf('\n==================== D FULL SUMMARY ====================\n');
for tg = tags
  m = arrayfun(@(g) strcmp(g.tag,tg), grand);
  g = grand(m);
  fprintf('%-10s  SP %2d/%-2d | mean xy=%.3f v=%.3f | engH=%.2f engR=%.2f maxdK=%.3f | fov_fails=%d\n', ...
    tg, sum([g.sp]), numel(g), mean([g.xy]), mean([g.v]), ...
    mean([g.engH]), mean([g.engR]), max([g.dK]), sum([g.fov]));
end
fprintf('\nFailures (non-SP):\n');
for g = grand
  if ~g.sp, fprintf('  %-11s %-9s IC%d  p=%d s=%d fov=%d xy=%.3f v=%.3f\n', ...
      g.traj,g.tag,g.ic,g.precise,g.soft,g.fov,g.xy,g.v); end
end
save(fullfile(outDir,'D_fulltest_grand.mat'),'grand','D_override');
fprintf('\nSaved -> Datasets/MultiInit/D_test/\n');

function out = ternary(c,a,b), if c, out=a; else, out=b; end, end
