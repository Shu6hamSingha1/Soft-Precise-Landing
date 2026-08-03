%% CB_LOCK_KAPPA  Does kappa ADAPT to disturbance on the baked LOCKED config?
%   A) ESCALATION: noiseless -> realistic -> 3x/5x/7x stress. If kappa RMS/peak RISES
%      with disturbance magnitude -> it's adapting to the disturbance (not just leakage).
%   B) KNOWN STEP: inject a known lateral force on [2,6]s -> kappa should RISE while on
%      and LEAK-decay after -> demonstrable adaptation + the leakage law.
%   (LOCKED is baked into vdf_params -> bare run = LOCKED.)
%
% Run:  cd MATLAB/Multi_init_cond; cb_lock_kappa
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global STRESS_SCALE KNOWN_DIST %#ok<GVMIS>

% ---- A) escalation: kappa vs disturbance magnitude (Circular IC3, offset cell) ----
fprintf('\n=== A) kappa vs disturbance magnitude (LOCKED, Circular IC3) ===\n');
fprintf('  condition   | kRMS x/y/z        | kpeak x/y/z        | landed xy\n');
conds = {'noiseless',0,[]; 'realistic',1,[]; '3x stress',1,3; '5x stress',1,5; '7x stress',1,7};
for k=1:size(conds,1)
    STRESS_SCALE=conds{k,3}; KNOWN_DIST=[];
    r=run_simulation([2;-2;-5;1;0;0;0;zeros(6,1)],"Circular",[],1.0,struct('NOISE',conds{k,2},'GE',1,'delay',1),1);
    d=r.data; idx=d.idx; ka=d.kappa_log(:,1:idx);
    krms=sqrt(mean(ka.^2,2)); kpk=max(ka,[],2);
    fprintf('  %-11s | %.3f %.3f %.3f | %.3f %.3f %.3f | %d %.3f\n', ...
        conds{k,1}, krms, kpk, r.success, r.final_xy);
end

% ---- B) known-disturbance step: kappa rise + leak-decay ----
fprintf('\n=== B) known lateral step Fx=3N on [2,6]s (Static, noise off) ===\n');
STRESS_SCALE=[]; KNOWN_DIST=struct('force',[3;0;0],'t_on',2.0,'t_off',6.0);
r=run_simulation([0;0;-5;1;0;0;0;zeros(6,1)],"Static",[],1.0,struct('NOISE',1,'GE',1,'delay',1),1);
d=r.data; idx=d.idx; t=d.tRange(1:idx); t=t(:)'; kx=d.kappa_log(1,1:idx); F=d.F_known_log(1,1:idx);
fprintf('  t   | force_on | kappa_x\n');
for tt=[1 2 2.5 3 4 5 6 6.5 7 8]
    if tt>t(end),continue;end
    [~,j]=min(abs(t-tt));
    fprintf('  %4.1f |    %d     | %.3f\n', t(j), F(j)~=0, kx(j));
end
kpre=mean(kx(t<2 & t>1.5)); kon=max(kx(F~=0)); koff=kx(end);
fprintf('  -> pre=%.3f  on-peak=%.3f  end=%.3f  (rises under load? %d ; leaks after? %d)\n', ...
    kpre,kon,koff, kon>kpre+0.01, koff<kon);
clear global STRESS_SCALE KNOWN_DIST
