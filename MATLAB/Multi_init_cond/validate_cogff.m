% IC1-5 no-regression gate for the thrust-scaled adaptive CoG feed-forward.
% base (GAMMA_COG=0) vs g.003 vs g.005, noiseless (n=1) + noisy (n=5) per IC.
% Bake the candidate only if it does NOT regress SP/land vs base on ANY IC --
% especially IC1 (centered), which must stay clean.  run validate_cogff
clear -regexp '^(?!WS)$'; close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG WS;
WS = struct();
WS.ICs = {[0;0;-5], [2;2;-5], [2;-2;-5], [2;2;-7], [2;2;-3]};
WS.nm  = {'IC1','IC2','IC3','IC4','IC5'};
WS.cfg = {0, 0.003, 0.005};  WS.cnm = {'base','g.003','g.005'};
WS.jobs = [];
for ic = 1:5
  for ci = 1:numel(WS.cfg)
    WS.jobs(end+1,:) = [ic ci 0 1];                      % noiseless n=1
    for sd = 1:5; WS.jobs(end+1,:) = [ic ci 1 sd]; end   % noisy n=5
  end
end
WS.R = cell(1, size(WS.jobs,1));
for j = 1:size(WS.jobs,1)
  global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE GAMMA_COG WS;
  jb = WS.jobs(j,:); ic = jb(1); ci = jb(2); noise = jb(3); sd = jb(4);
  IC_OVERRIDE = WS.ICs{ic}; NOISE_OVERRIDE = noise; RNG_SEED_OVERRIDE = sd;
  GAMMA_COG = WS.cfg{ci};
  WS.j = j;
  fprintf('\n=== job %d/%d  %s %s noise=%d seed=%d ===\n', j, size(WS.jobs,1), ...
          WS.nm{ic}, WS.cnm{ci}, noise, sd);
  try
    visualControl_IBVS_adaptive;
    rm.xy  = norm(I_p_c(1:2) - x_t(1:2,idx));
    rm.vel = norm(I_v_c - dx_t(1:3,idx));
    rm.sp  = double(landed && rm.xy<=0.08 && rm.vel<=0.2);
    rm.land= double(landed); rm.fov = double(fov_fail);
  catch ME
    rm = struct('xy',NaN,'vel',NaN,'sp',0,'land',0,'fov',0);
    fprintf('  ERR: %s\n', ME.message);
  end
  global WS;
  rm.ic=WS.jobs(WS.j,1); rm.ci=WS.jobs(WS.j,2); rm.noise=WS.jobs(WS.j,3);
  WS.R{WS.j} = rm;
  fprintf('  -> land=%d fov=%d xy=%.3f vel=%.3f SP=%d\n', rm.land, rm.fov, rm.xy, rm.vel, rm.sp);
end
% ===== SUMMARY =====
global WS; R = WS.R; NC = numel(WS.cfg);
fprintf('\n========= CoG-FF IC1-5 GATE (noiseless n=1 / noisy n=5) =========\n');
hdr = sprintf('%-4s', 'IC'); for ci=1:NC; hdr=[hdr sprintf(' | %-16s', WS.cnm{ci})]; end
fprintf('%s\n', hdr);
for ic = 1:5
  line = sprintf('%-4s', WS.nm{ic});
  for ci = 1:NC
    rr = R(cellfun(@(x) x.ic==ic && x.ci==ci, R));
    nl0 = rr(cellfun(@(x) x.noise==0, rr)); nl1 = rr(cellfun(@(x) x.noise==1, rr));
    sp0=sum(cellfun(@(x)x.sp,nl0)); ld0=sum(cellfun(@(x)x.land,nl0));
    sp1=sum(cellfun(@(x)x.sp,nl1)); ld1=sum(cellfun(@(x)x.land,nl1));
    line=[line sprintf(' | nl SP%d/L%d ny SP%d/L%d', sp0,ld0,sp1,ld1)];
  end
  fprintf('%s\n', line);
end
fprintf('-----------------------------------------------------------------\n');
for ci=1:NC
  sp=sum(cellfun(@(x)x.sp && x.ci==ci, R)); ld=sum(cellfun(@(x)x.land && x.ci==ci, R));
  fprintf('TOTAL %-6s: SP=%2d land=%2d\n', WS.cnm{ci}, sp, ld);
end
% per-IC no-regression check vs base
fprintf('-----------------------------------------------------------------\n');
for ci=2:NC
  ok=true;
  for ic=1:5
    b = R(cellfun(@(x)x.ic==ic && x.ci==1,R)); c = R(cellfun(@(x)x.ic==ic && x.ci==ci,R));
    if sum(cellfun(@(x)x.sp,c))<sum(cellfun(@(x)x.sp,b)) || ...
       sum(cellfun(@(x)x.land,c))<sum(cellfun(@(x)x.land,b)); ok=false;
       fprintf('  %s REGRESSES at %s\n', WS.cnm{ci}, WS.nm{ic}); end
  end
  fprintf('  %s vs base: %s\n', WS.cnm{ci}, string(ok)+" (no-regression on all 5 ICs if true)");
end
fprintf('=================================================================\n');
save('../Datasets/validate_cogff.mat','R');
