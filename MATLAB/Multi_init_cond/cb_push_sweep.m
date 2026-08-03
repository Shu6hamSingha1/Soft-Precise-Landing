%% CB_PUSH_SWEEP  Push the rejection margin past 7x: lower E_xy / leakage from
%   COMBINED and try to recover 8x WITHOUT breaking the 25/25 gate.
%   COMBINED base: kappa0 .05, N .10, Pleak [.5;.5;1.5], E [.5;.5;.5] (7x=5/5, 8x=1/5).
%   Variants push E_xy and/or lateral leakage lower (more switching authority / higher
%   sustained kappa). Watch the gate (chatter/wind-up shows as fly-aways or maxV creep).
%
% Run:  cd MATLAB/Multi_init_cond; cb_push_sweep
% Saves: ../Datasets/MultiInit/push_sweep.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

base = @(E,Pxy) struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
        'N',diag([0.10,0.10,0.10]),'Pleak',diag([Pxy,Pxy,1.5]),'E',diag([E,E,0.5]));
V = { 'COMBINED  E.5_P.5',   base(0.50,0.5);
      'V1 E.35_P.5',         base(0.35,0.5);
      'V2 E.25_P.5',         base(0.25,0.5);
      'V3 E.35_P.3',         base(0.35,0.3) };

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

R = struct('name',{},'gate',{},'mxy',{},'maxv',{},'fly',{},'sp7',{},'sp8',{},'k7',{});
for vi = 1:size(V,1)
    cfg = V{vi,2};
    % gate (25 realistic)
    g=0; xy=[]; mv=[]; fly=0;
    for t=1:numel(trajList)
        for ic=1:5
            m = onerun(cfg, [], p0(ic,:), trajList(t));
            g=g+m.sp; xy(end+1)=m.xy; mv(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
        end
    end
    % stress 7x, 8x (5 cells)
    sp7=0; sp8=0; k7=[];
    for c=1:size(scells,1)
        m7 = onerun(cfg, 7, p0(scells{c,2},:), scells{c,1}); sp7=sp7+m7.sp; k7(end+1)=mean(m7.krms(1:2)); %#ok<AGROW>
        m8 = onerun(cfg, 8, p0(scells{c,2},:), scells{c,1}); sp8=sp8+m8.sp;
    end
    R(vi)=struct('name',V{vi,1},'gate',g,'mxy',mean(xy),'maxv',max(mv),'fly',fly, ...
                 'sp7',sp7,'sp8',sp8,'k7',mean(k7)); %#ok<AGROW>
    fprintf('  %-18s | gate %2d/25 mXY %.3f maxV %.3f fly %d | 7x %d/5 (kRMSxy %.2f) | 8x %d/5\n', ...
        V{vi,1}, g, mean(xy), max(mv), fly, sp7, mean(k7), sp8);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','push_sweep.mat'),'R','V');
fprintf('\n(target: hold gate 25/25 AND recover 8x>1/5 without fly-aways)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; ka=d.kappa_log(:,1:d.idx);
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly = ~r.success || r.final_alt>0.25; m.krms=sqrt(mean(ka.^2,2));
end
