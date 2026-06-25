% IC1 [0,0,-5] noisy: N_z=0.1 vs default N_z=0.02 (n=5 matched seeds).
% N_z = kappa_z adaptation rate; descent-softness lever. Report SP + touchdown vel.
%   run nz_ic1
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE N_Z_OVERRIDE WS;
WS = struct(); WS.nz = {0.02, 0.10}; WS.nm = {'Nz0.02(def)','Nz0.10'};
WS.R = cell(2,5);
for ci = 1:2
  for sd = 1:5
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE N_Z_OVERRIDE WS;
    IC_OVERRIDE=[0;0;-5]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd; N_Z_OVERRIDE=WS.nz{ci};
    WS.ci=ci; WS.sd=sd;
    fprintf('\n=== IC1 %s seed %d ===\n', WS.nm{ci}, sd);
    try
      visualControl_IBVS_adaptive;
      rr.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); rr.vel=norm(I_v_c-dx_t(1:3,idx));
      rr.sp=double(landed && rr.xy<=0.08 && rr.vel<=0.2); rr.land=double(landed); rr.fov=double(fov_fail); rr.t=tRange(idx);
    catch ME
      rr=struct('xy',NaN,'vel',NaN,'sp',0,'land',0,'fov',0,'t',NaN); fprintf('  ERR %s\n',ME.message);
    end
    global WS; WS.R{WS.ci,WS.sd}=rr;
    fprintf('  -> land=%d fov=%d t=%.1f xy=%.3f vel=%.3f SP=%d\n', rr.land,rr.fov,rr.t,rr.xy,rr.vel,rr.sp);
    ci = WS.ci; sd = WS.sd;   % restore loop counters (controller's clear wiped them)
  end
end
global WS;
fprintf('\n===== IC1 noisy: N_z=0.1 vs default (n=5) =====\n');
for ci=1:2
  rr=WS.R(ci,:); sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
  v=cellfun(@(x)x.vel,rr); xy=cellfun(@(x)x.xy,rr);
  fprintf('  %-12s: SP=%d/5 land=%d | vel mean=%.3f max=%.3f | xy mean=%.3f\n', ...
    WS.nm{ci}, sp, ld, mean(v,'omitnan'), max(v), mean(xy,'omitnan'));
end
fprintf('===============================================\n');
