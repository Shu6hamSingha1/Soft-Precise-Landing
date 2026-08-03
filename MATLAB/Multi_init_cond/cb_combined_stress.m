%% CB_COMBINED_STRESS  Stack the 3 validated levers + prove kappa rejects disturbance.
%   COMBINED = D + L3 + E_xy=0.5:
%       kappa0 = [0.05;0.05;0.05]      (D: prime adaptive law)
%       N      = diag([0.10,0.10,0.10])(D)
%       Pleak  = diag([0.5,0.5,1.5])   (L3: raise sustained kappa)
%       E      = diag([0.5,0.5,0.5])   (E_xy 1.0->0.5: engage lateral switching)
%   vs BAKED (validated 25/25 baseline, theta_per_axis only).
%
%   Part 1  GATE   : 25 realistic cells -- does the stack still land 25/25?
%   Part 2  STRESS : engaging subset x STRESS_SCALE {3,6,10} (amplified sustained
%                    wind+noise+mismatch), COMBINED vs BAKED. Does COMBINED hold SP
%                    where BAKED breaks, and does its kappa grow up to reject?
%
% Run:  cd MATLAB/Multi_init_cond; cb_combined_stress
% Saves: ../Datasets/MultiInit/combined_stress.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

COMB  = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
               'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
BAKED = struct('theta_per_axis',true);

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

% ---------------- Part 1: GATE (25 realistic, COMBINED) ----------------
fprintf('\n========== PART 1: COMBINED gate (25 realistic) ==========\n');
sp=0; n=0; xy=[]; v=[]; fly=0; krms=zeros(3,0); kpk=zeros(3,0);
for t=1:numel(trajList)
    for ic=1:5
        m = onerun(COMB, [], p0(ic,:), trajList(t));
        sp=sp+m.sp; n=n+1; xy(end+1)=m.xy; v(end+1)=m.v;                 %#ok<AGROW>
        if m.fly, fly=fly+1; end
        krms(:,end+1)=m.krms; kpk(:,end+1)=m.kpk;                        %#ok<AGROW>
        fprintf('  %-11s IC%d SP=%d xy=%.3f v=%.3f kRMS=[%.3f %.3f %.3f]\n', ...
            trajList(t),ic,m.sp,m.xy,m.v,m.krms);
    end
end
fprintf('  --> COMBINED %d/25 SP | mXY %.3f mV %.3f maxV %.3f | flyaways %d | kRMS med [%.3f %.3f %.3f]\n', ...
    sp,mean(xy),mean(v),max(v),fly,median(krms,2));

% ---------------- Part 2: STRESS escalation (COMBINED vs BAKED) ----------------
cells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };
scales = [3, 6, 10];
fprintf('\n========== PART 2: STRESS escalation (5 cells) ==========\n');
fprintf('  scale | COMBINED SP  mXY   mV  kRMSxy | BAKED SP  mXY   mV  kRMSxy\n');
ST = struct('scale',{},'comb_sp',{},'comb_xy',{},'comb_k',{},'baked_sp',{},'baked_xy',{});
for s = scales
    cs_sp=0; cs_xy=[]; cs_v=[]; cs_k=[]; bk_sp=0; bk_xy=[]; bk_v=[];
    for c=1:size(cells,1)
        mc = onerun(COMB,  s, p0(cells{c,2},:), cells{c,1});
        mb = onerun(BAKED, s, p0(cells{c,2},:), cells{c,1});
        cs_sp=cs_sp+mc.sp; cs_xy(end+1)=mc.xy; cs_v(end+1)=mc.v; cs_k(end+1)=mean(mc.krms(1:2)); %#ok<AGROW>
        bk_sp=bk_sp+mb.sp; bk_xy(end+1)=mb.xy; bk_v(end+1)=mb.v;          %#ok<AGROW>
    end
    fprintf('   %2dx  |    %d/5     %.3f %.3f  %.3f  |   %d/5    %.3f %.3f\n', ...
        s, cs_sp, mean(cs_xy), mean(cs_v), mean(cs_k), bk_sp, mean(bk_xy), mean(bk_v));
    ST(end+1)=struct('scale',s,'comb_sp',cs_sp,'comb_xy',mean(cs_xy),'comb_k',mean(cs_k), ...
                     'baked_sp',bk_sp,'baked_xy',mean(bk_xy)); %#ok<AGROW>
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','combined_stress.mat'),'ST','COMB');
fprintf('\nSaved -> Datasets/MultiInit/combined_stress.mat\nDONE\n');

% ============================= local functions ===============================
function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    co = struct('NOISE',1,'GE',1,'delay',1);
    r = run_simulation(x0, traj, [], 1.0, co, 1);
    d=r.data; idx=d.idx; ka=d.kappa_log(:,1:idx);
    m.sp = r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly = ~r.success || r.final_alt>0.25;
    m.krms = sqrt(mean(ka.^2,2)); m.kpk = max(ka,[],2);
end
