% IC5 noisy, GAMMA_COG=0.005 fixed: does the descent-softness lever N_z=0.1 resolve
% seed 4's hard touchdown (vz=1.3, attributed to terminal under-brake)? Check seed 4
% terminal vel + regression across all 12 seeds. N_z = kappa_z adaptation rate.
%   run cogff_nz_ic5
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG N_Z_OVERRIDE WS;
WS = struct(); WS.nz = {0.02, 0.10}; WS.nm = {'Nz.02(def)','Nz.10'};
WS.NS = 12; WS.NC = 2; WS.R = cell(WS.NC, WS.NS);
for ci = 1:WS.NC
  for sd = 1:WS.NS
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG N_Z_OVERRIDE WS;
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd;
    GAMMA_COG=0.005; N_Z_OVERRIDE=WS.nz{ci};
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
fprintf('\n===== IC5 GAMMA_COG=0.005: N_z=0.02 vs 0.10 (n=%d) =====\n', WS.NS);
for ci=1:WS.NC
  rr=WS.R(ci,:);
  sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
  fv=sum(cellfun(@(x)x.fov,rr)); v=cellfun(@(x)x.vel,rr); xy=cellfun(@(x)x.xy,rr);
  s4=WS.R{ci,4};
  fprintf('  %-11s: SP=%2d/%d land=%2d fov=%d | vel mean=%.3f | xy mean=%.3f | seed4: vel=%.3f xy=%.3f SP=%d\n', ...
    WS.nm{ci}, sp, WS.NS, ld, fv, mean(v,'omitnan'), mean(xy,'omitnan'), s4.vel, s4.xy, s4.sp);
end
fprintf('=========================================================\n');
