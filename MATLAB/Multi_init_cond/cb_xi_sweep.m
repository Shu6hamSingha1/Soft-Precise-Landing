%% CB_XI_SWEEP  Increase BOTH funnel contraction rates Xi_h (optic-flow) and Xi_r
%   (image-feature) further, with FLOORS HELD AT BAKED (p_hinf=[1;1;1.5], p_rinf=[1;1]).
%   Isolates the contraction-RATE effect from the floor effect: earlier funnel
%   tightening LOWERED the floor too (the likely destabilizer). Faster Xi just reaches
%   the same baked floor sooner -> does it lift kappa / extend margin WITHOUT the
%   floor-driven instability? Standing Cond 1 respected (p_rinf=1).
%
%   COMBINED base: kappa0 .05, N .10, Pleak [.5;.5;1.5], E [.5;.5;.5].
%
% Run:  cd MATLAB/Multi_init_cond; cb_xi_sweep
% Saves: ../Datasets/MultiInit/xi_sweep.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]), ...
           'p_hinf',[1.0;1.0;1.5],'p_rinf',[1.0;1.0]);   % floors pinned BAKED
mk = @(xih,xir) setfield(setfield(C,'Xi_h',diag([xih xih xih])),'Xi_r',diag([xir xir])); %#ok<SFLD>
F = { 'X0 COMBINED Xih.2 Xir.1', mk(0.2,0.1);
      'X1 Xih.4 Xir.3',          mk(0.4,0.3);
      'X2 Xih.6 Xir.5',          mk(0.6,0.5);
      'X3 Xih.8 Xir.7',          mk(0.8,0.7) };

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

R = struct('name',{},'gate',{},'mxy',{},'maxv',{},'fly',{},'fov',{},'kg',{},'sp7',{},'sp8',{},'k7',{});
for vi=1:size(F,1)
    cfg=F{vi,2};
    g=0; xy=[]; mv=[]; fly=0; fovf=0; kg=zeros(3,0);
    for t=1:numel(trajList)
        for ic=1:5
            m=onerun(cfg,[],p0(ic,:),trajList(t));
            g=g+m.sp; xy(end+1)=m.xy; mv(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
            if m.fov, fovf=fovf+1; end
            kg(:,end+1)=m.krms;                                              %#ok<AGROW>
        end
    end
    sp7=0; sp8=0; k7=[];
    for c=1:size(scells,1)
        m7=onerun(cfg,7,p0(scells{c,2},:),scells{c,1}); sp7=sp7+m7.sp; k7(end+1)=mean(m7.krms(1:2)); %#ok<AGROW>
        m8=onerun(cfg,8,p0(scells{c,2},:),scells{c,1}); sp8=sp8+m8.sp;
    end
    R(vi)=struct('name',F{vi,1},'gate',g,'mxy',mean(xy),'maxv',max(mv),'fly',fly,'fov',fovf, ...
                 'kg',median(kg,2),'sp7',sp7,'sp8',sp8,'k7',mean(k7)); %#ok<AGROW>
    fprintf('  %-24s | gate %2d/25 mXY %.3f maxV %.3f fly %d fov %d | kRMS[%.2f %.2f %.2f] | 7x %d/5 (k%.2f) 8x %d/5\n', ...
        F{vi,1}, g, mean(xy), max(mv), fly, fovf, median(kg,2), sp7, mean(k7), sp8);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','xi_sweep.mat'),'R','F');
fprintf('\n(faster Xi to BAKED floor: does it lift kappa / hold gate / extend 8x without fly-aways?)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; ka=d.kappa_log(:,1:d.idx);
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.final_alt>0.25; m.fov=r.fov_fail; m.krms=sqrt(mean(ka.^2,2));
end
