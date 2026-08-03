%% CB_FUNNEL_TIGHTEN  Tighten the optic-flow funnel on the COMBINED base — the one
%   lever that raises the barrier gain G (kappa* = theta*G*|sigma|/P), so it lifts
%   kappa's EQUILIBRIUM (vs E=delivery, P=leakage). Funnel was the destabilizer
%   earlier (C2/C3 -> FoV breaches + kappa runaway), so GATE HARD on fly-aways.
%
%   Tighten via Xi_h (faster contraction) UP + p_hinf (floor) DOWN. p_h0 is FoV-fixed.
%   COMBINED base: kappa0 .05, N .10, Pleak [.5;.5;1.5], E [.5;.5;.5];
%                  funnel baked Xi_h=[.2;.2;.2], p_hinf=[1;1;1.5].
%
% Run:  cd MATLAB/Multi_init_cond; cb_funnel_tighten
% Saves: ../Datasets/MultiInit/funnel_tighten.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
mk = @(xih,phinf) setfield(setfield(C,'Xi_h',diag(xih)),'p_hinf',phinf(:)); %#ok<SFLD>
F = { 'F0 COMBINED (baked funnel)', C;
      'F1 Xi.3 phinf[.8.8 1]',      mk([.3 .3 .3],[.8;.8;1.0]);
      'F2 Xi.4 phinf[.6.6 1]',      mk([.4 .4 .4],[.6;.6;1.0]);
      'F3 lat Xi.4 phinf[.6.6 1.5]',mk([.4 .4 .2],[.6;.6;1.5]) };

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

R = struct('name',{},'gate',{},'mxy',{},'maxv',{},'fly',{},'kg',{},'sp7',{},'sp8',{},'k7',{});
for vi=1:size(F,1)
    cfg=F{vi,2};
    g=0; xy=[]; mv=[]; fly=0; kg=zeros(3,0);
    for t=1:numel(trajList)
        for ic=1:5
            m=onerun(cfg,[],p0(ic,:),trajList(t));
            g=g+m.sp; xy(end+1)=m.xy; mv(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
            kg(:,end+1)=m.krms;                                              %#ok<AGROW>
        end
    end
    sp7=0; sp8=0; k7=[];
    for c=1:size(scells,1)
        m7=onerun(cfg,7,p0(scells{c,2},:),scells{c,1}); sp7=sp7+m7.sp; k7(end+1)=mean(m7.krms(1:2)); %#ok<AGROW>
        m8=onerun(cfg,8,p0(scells{c,2},:),scells{c,1}); sp8=sp8+m8.sp;
    end
    R(vi)=struct('name',F{vi,1},'gate',g,'mxy',mean(xy),'maxv',max(mv),'fly',fly, ...
                 'kg',median(kg,2),'sp7',sp7,'sp8',sp8,'k7',mean(k7)); %#ok<AGROW>
    fprintf('  %-28s | gate %2d/25 mXY %.3f maxV %.3f fly %d | kRMS[%.2f %.2f %.2f] | 7x %d/5 (k%.2f) 8x %d/5\n', ...
        F{vi,1}, g, mean(xy), max(mv), fly, median(kg,2), sp7, mean(k7), sp8);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','funnel_tighten.mat'),'R','F');
fprintf('\n(want: gate 25/25 + 0 fly-aways + higher kappa + 8x recovered. fly>0 = funnel destabilizing)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; ka=d.kappa_log(:,1:d.idx);
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.final_alt>0.25; m.krms=sqrt(mean(ka.^2,2));
end
