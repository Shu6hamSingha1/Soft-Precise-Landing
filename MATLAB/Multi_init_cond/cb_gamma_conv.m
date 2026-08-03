%% CB_GAMMA_CONV  Retune for FASTER h_e convergence so a TIGHT funnel becomes feasible.
%   Thesis (user): the tight-funnel failure is NOT "Xi too big" -- it's h_e converging
%   too slowly. The convergence rate is TUNABLE via the reaching gain Gamma (the
%   un-saturated proportional term that speeds h_e reaching; PX4 GAMMA_xy=2.0 vs MATLAB
%   0.4375). So take the TIGHT funnel that FAILED (X1: Xi_h .4, Xi_r .3 -> 7x 0/5) and
%   raise Gamma_xy: does faster convergence make the tight funnel feasible (gate + 7x)?
%
%   Base = COMBINED + tight funnel (Xi_h .4 / Xi_r .3, baked floors).
%
% Run:  cd MATLAB/Multi_init_cond; cb_gamma_conv
% Saves: ../Datasets/MultiInit/gamma_conv.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

base = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
              'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]), ...
              'p_hinf',[1.0;1.0;1.5],'p_rinf',[1.0;1.0], ...
              'Xi_h',diag([0.4,0.4,0.4]),'Xi_r',diag([0.3,0.3]));   % the TIGHT funnel that failed
gammas = [0.4375, 1.0, 2.0, 3.0];   % reaching gain (xy); baked 0.4375

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

R = struct('gxy',{},'gate',{},'mxy',{},'maxv',{},'fly',{},'he_term',{},'sp7',{});
for gxy = gammas
    cfg = base; cfg.Gamma = diag([gxy,gxy,0.75]);
    g=0; xy=[]; mv=[]; fly=0; het=[];
    for t=1:numel(trajList)
        for ic=1:5
            m=onerun(cfg,[],p0(ic,:),trajList(t));
            g=g+m.sp; xy(end+1)=m.xy; mv(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
            het(end+1)=m.he_term;                                            %#ok<AGROW>
        end
    end
    sp7=0;
    for c=1:size(scells,1)
        m7=onerun(cfg,7,p0(scells{c,2},:),scells{c,1}); sp7=sp7+m7.sp;
    end
    R(end+1)=struct('gxy',gxy,'gate',g,'mxy',mean(xy),'maxv',max(mv),'fly',fly, ...
                    'he_term',median(het),'sp7',sp7); %#ok<AGROW>
    fprintf('  Gamma_xy=%.3f | gate %2d/25 mXY %.3f maxV %.3f fly %d | |h_e|_term(med) %.3f | 7x %d/5\n', ...
        gxy, g, mean(xy), max(mv), fly, median(het), sp7);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','gamma_conv.mat'),'R','base');
fprintf('\n(thesis: higher Gamma -> faster h_e convergence (lower |h_e|_term) -> tight funnel FEASIBLE: gate 25/25 + 7x recovers)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; idx=d.idx;
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.final_alt>0.25;
    % terminal h_e convergence proxy: median |h_e| over the last 1.5 m of descent
    z = 1./max(d.beta_log(1:idx),1e-6); term = z<1.5;
    he = vecnorm(d.V_h_e(1:2,1:idx));
    m.he_term = median(he(term));
end
