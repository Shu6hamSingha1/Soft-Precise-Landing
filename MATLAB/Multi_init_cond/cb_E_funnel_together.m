%% CB_E_FUNNEL_TOGETHER  Tighten the IMAGE-FEATURE funnel p_r AND the boundary layer E
%   TOGETHER (untested combination). Hypothesis: tight p_r raises |sigma| (error near the
%   tighter barrier) but reaching can't track it -> overtake; tight E engages the SWITCHING
%   on that larger |sigma| (|sigma|/E clears 1 -> full kappa*sat) -> supplies the robust
%   convergence the reaching lacks. So together feasible where neither alone is.
%   p_r tightened via Xi_r (contraction) + p_rinf (floor). E via E_xy. COMBINED base.
%   KEY metric: max engR = |r_bar_e|/p_r  (<1 = funnel HELD, no PPC overtake).
%
% Run:  cd MATLAB/Multi_init_cond; cb_E_funnel_together
% Saves: ../Datasets/MultiInit/E_funnel_together.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

base = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
              'Pleak',diag([0.5,0.5,1.5]));
mk = @(E,xir,prinf) setfield(setfield(setfield(setfield(base, ...
        'E',diag([E,E,0.5])),'Xi_r',diag([xir xir])),'p_rinf',[prinf;prinf]),'theta_per_axis',true); %#ok<*SFLD>
F = { 'ref  E.5  Xir.1 prinf1',  mk(0.5, 0.1, 1.00);   % COMBINED
      'E.25 only (pr baked)',    mk(0.25,0.1, 1.00);
      'pr-tight only E.5',       mk(0.5, 0.3, 0.85);
      'BOTH E.25 + pr-tight',    mk(0.25,0.3, 0.85);
      'BOTH stronger prinf.70',  mk(0.25,0.3, 0.70) };

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

R = struct('name',{},'gate',{},'mxy',{},'maxv',{},'fly',{},'engR',{},'sp7',{},'sp8',{});
for vi=1:size(F,1)
    cfg=F{vi,2};
    g=0; xy=[]; mv=[]; fly=0; eR=[];
    for t=1:numel(trajList)
        for ic=1:5
            m=onerun(cfg,[],p0(ic,:),trajList(t));
            g=g+m.sp; xy(end+1)=m.xy; mv(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
            eR(end+1)=m.engR;                                                %#ok<AGROW>
        end
    end
    sp7=0; sp8=0;
    for c=1:size(scells,1)
        sp7=sp7+onerun(cfg,7,p0(scells{c,2},:),scells{c,1}).sp;
        sp8=sp8+onerun(cfg,8,p0(scells{c,2},:),scells{c,1}).sp;
    end
    R(vi)=struct('name',F{vi,1},'gate',g,'mxy',mean(xy),'maxv',max(mv),'fly',fly, ...
                 'engR',max(eR),'sp7',sp7,'sp8',sp8); %#ok<AGROW>
    fprintf('  %-26s | gate %2d/25 mXY %.3f maxV %.3f fly %d | maxEngR %.2f | 7x %d/5  8x %d/5\n', ...
        F{vi,1}, g, mean(xy), max(mv), fly, max(eR), sp7, sp8);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','E_funnel_together.mat'),'R','F');
fprintf('\n(want: BOTH holds gate 25/25 + maxEngR<1 (funnel held, no overtake) + 7-8x >= COMBINED ref)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; idx=d.idx;
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.final_alt>0.25;
    rbe=abs(d.s_e_log(:,1:idx)./d.P.phi_max(:));
    m.engR=max(max(rbe./max(d.p_r_log(:,1:idx),eps),[],1));
end
