% IC5 [2,2,-3] noisy: default c-term vs C_SIMPLE (first-principles manuscript c).
% C_SIMPLE replaces the spurious s-double-dot block (dw x s, w x(w x s), 2 w x h
% built from the NOISY optic-flow w/dw) with the clean manuscript c_tilde_h =
% -psi_dot_b*(e3 x h) - (e3.h)*h using the IMU yaw rate. Tests whether removing
% the noisy c-term channel kills the seed-4 startup-tilt and seed-6 CBF-strip
% runaways. 12 matched RNG seeds. Reports SP / land / fov_fail / final xy,vel.
%   run csimple_ic5
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE WS;
WS = struct(); WS.cs = {0, 1}; WS.nm = {'c default','C_SIMPLE'};
WS.NS = 12; WS.R = cell(2, WS.NS);
for ci = 1:2
  for sd = 1:WS.NS
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE WS;
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd; C_SIMPLE=WS.cs{ci};
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
fprintf('\n===== IC5 noisy: default c vs C_SIMPLE (n=%d) =====\n', WS.NS);
for ci=1:2
  rr=WS.R(ci,:);
  sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
  fv=sum(cellfun(@(x)x.fov,rr));
  v=cellfun(@(x)x.vel,rr); xy=cellfun(@(x)x.xy,rr);
  bad=find(cellfun(@(x)~x.land || x.fov,rr));
  fprintf('  %-10s: SP=%2d/%d land=%2d fov=%d | vel mean=%.3f max=%.3f | xy mean=%.3f\n', ...
    WS.nm{ci}, sp, WS.NS, ld, fv, mean(v,'omitnan'), max(v), mean(xy,'omitnan'));
  if ~isempty(bad); fprintf('              fail seeds: %s\n', mat2str(bad)); end
end
fprintf('==================================================\n');
