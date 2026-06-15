% IC5 [2,2,-3] noisy: does past-breach STRONG recovery (SEN_RECOVER_ST) reduce
% breach depth + recover the breach seeds (4,6)? Sweep ST in {0(off),0.5,0.3}.
% GAMMA_COG default (0.005 baked). Reports SP/land/fov + max breach ratio
% max|s_e_n|/p_s (>=1 = breach) to see if the outer-demand fix shrinks the breach.
%   run sen_recover_ic5
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE SEN_RECOVER_ST WS;
WS = struct(); WS.st = {0, 0.5, 0.3}; WS.nm = {'ST=off','ST=0.5','ST=0.3'};
WS.NS = 12; WS.NC = numel(WS.st); WS.R = cell(WS.NC, WS.NS);
for ci = 1:WS.NC
  for sd = 1:WS.NS
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE SEN_RECOVER_ST WS;
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=sd; SEN_RECOVER_ST=WS.st{ci};
    WS.ci=ci; WS.sd=sd;
    fprintf('\n=== IC5 %s seed %d ===\n', WS.nm{ci}, sd);
    try
      visualControl_IBVS_adaptive;
      rr.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); rr.vel=norm(I_v_c-dx_t(1:3,idx));
      rr.sp=double(landed && rr.xy<=0.08 && rr.vel<=0.2);
      rr.land=double(landed); rr.fov=double(fov_fail);
      rr.brmax=max(max(abs(V_s_e_n(:,1:idx))./p_s(:,1:idx)));   % peak breach ratio
    catch ME
      rr=struct('xy',NaN,'vel',NaN,'sp',0,'land',0,'fov',0,'brmax',NaN);
      fprintf('  ERR %s\n',ME.message);
    end
    global WS; WS.R{WS.ci,WS.sd}=rr;
    fprintf('  -> land=%d fov=%d xy=%.3f vel=%.3f brmax=%.2f SP=%d\n', ...
      rr.land,rr.fov,rr.xy,rr.vel,rr.brmax,rr.sp);
    ci = WS.ci; sd = WS.sd;
  end
end
global WS;
fprintf('\n===== IC5 noisy: SEN past-breach recovery sweep (n=%d) =====\n', WS.NS);
for ci=1:WS.NC
  rr=WS.R(ci,:);
  sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr)); fv=sum(cellfun(@(x)x.fov,rr));
  v=cellfun(@(x)x.vel,rr); xy=cellfun(@(x)x.xy,rr); br=cellfun(@(x)x.brmax,rr);
  bad=find(cellfun(@(x)~x.land||x.fov,rr));
  s4=WS.R{ci,4}; s6=WS.R{ci,6};
  fprintf('  %-7s: SP=%2d/%d land=%2d fov=%d | vel mean=%.3f xy mean=%.3f | seed4 br=%.2f land=%d | seed6 br=%.2f land=%d\n', ...
    WS.nm{ci}, sp, WS.NS, ld, fv, mean(v,'omitnan'), mean(xy,'omitnan'), s4.brmax, s4.land, s6.brmax, s6.land);
  if ~isempty(bad); fprintf('           fail seeds: %s\n', mat2str(bad)); end
end
fprintf('=============================================================\n');
