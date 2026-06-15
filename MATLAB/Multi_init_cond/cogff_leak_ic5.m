% IC5 [2,2,-3] noisy: CoG-FF with sigma-mod leakage. Goal: faster EARLY theta_hat
% convergence (fix seed 4 hard-land from late convergence t~3.4s, and seed 6 breach
% at t~2.1s) WITHOUT the steady-state over-adapt bias that broke plain gamma=0.02.
% Sweep (GAMMA_COG, COG_LEAK). Baseline = (0.005, 0) = current best.
%   run cogff_leak_ic5
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG COG_LEAK WS;
WS = struct();
WS.cfg = {[0.005 0], [0.015 1], [0.02 1], [0.02 3]};
WS.nm  = {'g.005 s0','g.015 s1','g.02 s1','g.02 s3'};
WS.NS = 12; WS.NC = numel(WS.cfg); WS.R = cell(WS.NC, WS.NS);
for ci = 1:WS.NC
  for sd = 1:WS.NS
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG COG_LEAK WS;
    cfg = WS.cfg{ci};
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd;
    GAMMA_COG=cfg(1); COG_LEAK=cfg(2);
    WS.ci=ci; WS.sd=sd;
    fprintf('\n=== IC5 %s seed %d ===\n', WS.nm{ci}, sd);
    try
      visualControl_IBVS_adaptive;
      rr.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); rr.vel=norm(I_v_c-dx_t(1:3,idx));
      rr.sp=double(landed && rr.xy<=0.08 && rr.vel<=0.2);
      rr.land=double(landed); rr.fov=double(fov_fail); rr.t=tRange(idx);
    catch ME
      rr=struct('xy',NaN,'vel',NaN,'sp',0,'land',0,'fov',0,'t',NaN);
      fprintf('  ERR %s\n',ME.message);
    end
    global WS; WS.R{WS.ci,WS.sd}=rr;
    fprintf('  -> land=%d fov=%d t=%.1f xy=%.3f vel=%.3f SP=%d\n', ...
      rr.land,rr.fov,rr.t,rr.xy,rr.vel,rr.sp);
    ci = WS.ci; sd = WS.sd;
  end
end
global WS;
fprintf('\n===== IC5 noisy: CoG-FF + leakage sweep (n=%d) =====\n', WS.NS);
for ci=1:WS.NC
  rr=WS.R(ci,:);
  sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
  fv=sum(cellfun(@(x)x.fov,rr));
  v=cellfun(@(x)x.vel,rr); xy=cellfun(@(x)x.xy,rr);
  bad=find(cellfun(@(x)~x.land || x.fov,rr));
  hard=find(cellfun(@(x)x.land && ~x.sp && ~x.fov,rr));
  fprintf('  %-9s: SP=%2d/%d land=%2d fov=%d | vel mean=%.3f max=%.3f | xy mean=%.3f\n', ...
    WS.nm{ci}, sp, WS.NS, ld, fv, mean(v,'omitnan'), max(v), mean(xy,'omitnan'));
  if ~isempty(bad);  fprintf('             TL/crash seeds: %s\n', mat2str(bad)); end
  if ~isempty(hard); fprintf('             land-not-SP seeds: %s\n', mat2str(hard)); end
end
fprintf('====================================================\n');
