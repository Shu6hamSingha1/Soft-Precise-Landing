% IC5 [2,2,-3] noisy: thrust-scaled adaptive CoG feed-forward sweep.
% The CoG offset injects tau_d = T*[-dy;dx;0] (thrust-proportional body torque) -
% the pinned root of the IC5 noisy failures. GAMMA_COG>0 enables a Lee-style
% adaptive law: theta_hat -> [-dy;dx] estimated online, tau_ff = -T*theta_hat.
% Sweep GAMMA_COG; compare SP / land / fov / touchdown vs the gamma=0 baseline.
% Expect: seed 6 (CBF-strip, t~5.3s, time to adapt) recoverable; seed 4
% (startup-tilt, fails t~0.4s, before adaptation converges) may persist.
%   run cogff_ic5
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG WS;
WS = struct(); WS.g = {0, 0.005, 0.02, 0.08};
WS.nm = {'gamma=0','gamma=.005','gamma=.02','gamma=.08'};
WS.NS = 12; WS.NC = numel(WS.g); WS.R = cell(WS.NC, WS.NS);
for ci = 1:WS.NC
  for sd = 1:WS.NS
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG WS;
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd; GAMMA_COG=WS.g{ci};
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
    ci = WS.ci; sd = WS.sd;   % restore loop counters (controller's clear wiped them)
  end
end
global WS;
fprintf('\n===== IC5 noisy: adaptive CoG feed-forward sweep (n=%d) =====\n', WS.NS);
for ci=1:WS.NC
  rr=WS.R(ci,:);
  sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
  fv=sum(cellfun(@(x)x.fov,rr));
  v=cellfun(@(x)x.vel,rr); xy=cellfun(@(x)x.xy,rr);
  bad=find(cellfun(@(x)~x.land || x.fov,rr));
  fprintf('  %-11s: SP=%2d/%d land=%2d fov=%d | vel mean=%.3f max=%.3f | xy mean=%.3f\n', ...
    WS.nm{ci}, sp, WS.NS, ld, fv, mean(v,'omitnan'), max(v), mean(xy,'omitnan'));
  if ~isempty(bad); fprintf('               fail seeds: %s\n', mat2str(bad)); end
end
fprintf('=============================================================\n');
