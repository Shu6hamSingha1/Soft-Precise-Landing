% Validate DH_D_CAP=20 (V_dh_d spike cap) on the canonical 5 ICs before baking.
% base vs cap20, noiseless (n=1) + noisy (n=5). Bake only if cap20 does NOT
% regress vs base (it only clips c-term derivative spikes; should be a no-op
% noiseless and no-regression noisy).  run validate_cap20

clear -regexp '^(?!WS)$';
close all; clc;
global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS DH_D_CAP;

WS = struct();
WS.ICs = {[0;0;-5], [2;2;-5], [2;-2;-5], [2;2;-7], [2;2;-3]};
WS.nm  = {'IC1','IC2','IC3','IC4','IC5'};
WS.cap = {[], 20};  WS.cnm = {'base','cap20'};
% job list: [ic, ci, noise, seed]
WS.jobs = [];
for ic = 1:5
  for ci = 1:2
    WS.jobs(end+1,:) = [ic ci 0 1];                 % noiseless n=1
    for sd = 1:5; WS.jobs(end+1,:) = [ic ci 1 sd]; end % noisy n=5
  end
end
WS.R = cell(1, size(WS.jobs,1));

for j = 1:size(WS.jobs,1)
  global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE WS DH_D_CAP;
  jb = WS.jobs(j,:); ic = jb(1); ci = jb(2); noise = jb(3); sd = jb(4);
  IC_OVERRIDE = WS.ICs{ic}; NOISE_OVERRIDE = noise; RNG_SEED_OVERRIDE = sd;
  DH_D_CAP = WS.cap{ci};
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
global WS; R = WS.R;
fprintf('\n========= CAP20 5-IC VALIDATION =========\n');
fprintf('%-4s | %-18s | %-18s\n','IC','base (SP/land  noisy)','cap20 (SP/land  noisy)');
for ic = 1:5
  cols = cell(1,2);
  for ci = 1:2
    rr = R(cellfun(@(x) x.ic==ic && x.ci==ci, R));
    nl0 = rr(cellfun(@(x) x.noise==0, rr));   % noiseless
    nl1 = rr(cellfun(@(x) x.noise==1, rr));   % noisy
    sp0 = sum(cellfun(@(x) x.sp, nl0)); ld0 = sum(cellfun(@(x) x.land, nl0));
    sp1 = sum(cellfun(@(x) x.sp, nl1)); ld1 = sum(cellfun(@(x) x.land, nl1));
    cols{ci} = sprintf('nl:SP%d/L%d  ny:SP%d/L%d', sp0, ld0, sp1, ld1);
  end
  fprintf('%-4s | %-18s | %-18s\n', WS.nm{ic}, cols{1}, cols{2});
end
fprintf('=========================================\n');
% verdict
base_sp = sum(cellfun(@(x) x.sp && x.ci==1, R));  cap_sp = sum(cellfun(@(x) x.sp && x.ci==2, R));
base_ld = sum(cellfun(@(x) x.land && x.ci==1, R)); cap_ld = sum(cellfun(@(x) x.land && x.ci==2, R));
fprintf('TOTAL  base: SP=%d land=%d   cap20: SP=%d land=%d  => %s\n', ...
  base_sp, base_ld, cap_sp, cap_ld, ...
  string(cap_sp>=base_sp && cap_ld>=base_ld) + " (cap20 no-regression if true)");
save('../Datasets/validate_cap20.mat','R');
