%% CB_IMGFUNNEL_TIGHTEN  Tighten the IMAGE-FEATURE (position) funnel p_r on the
%   COMBINED base. p_r constrains r_bar_e = s_e/phi_max and enters the combined
%   surface sigma = zeta_h + chi_r*zeta_r, so tightening raises the POSITION barrier
%   gain G_r (distinct from the optic-flow funnel p_h tested in cb_funnel_tighten).
%   Tighten via Xi_r UP (faster contraction) + p_rinf DOWN (lower floor). p_r0 is
%   FoV-derived -> NOT changed. p_rinf<1 dips below Standing Cond 1 -> WATCH FoV breach.
%
%   COMBINED base: kappa0 .05, N .10, Pleak [.5;.5;1.5], E [.5;.5;.5];
%                  image funnel baked Xi_r=[.1;.1], p_rinf=[1;1].
%
% Run:  cd MATLAB/Multi_init_cond; cb_imgfunnel_tighten
% Saves: ../Datasets/MultiInit/imgfunnel_tighten.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

C = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
           'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
mk = @(xir,prinf) setfield(setfield(C,'Xi_r',diag([xir xir])),'p_rinf',[prinf;prinf]); %#ok<SFLD>
F = { 'R0 COMBINED (baked p_r)',  C;
      'R1 Xir.2 prinf1.0 (rate)', mk(0.2,1.0);
      'R2 Xir.2 prinf.85',        mk(0.2,0.85);
      'R3 Xir.3 prinf.70',        mk(0.3,0.70) };

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
    fprintf('  %-26s | gate %2d/25 mXY %.3f maxV %.3f fly %d fov %d | kRMS[%.2f %.2f %.2f] | 7x %d/5 (k%.2f) 8x %d/5\n', ...
        F{vi,1}, g, mean(xy), max(mv), fly, fovf, median(kg,2), sp7, mean(k7), sp8);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','imgfunnel_tighten.mat'),'R','F');
fprintf('\n(want: gate 25/25 + 0 fly/fov + higher kappa + 7x held / 8x recovered. fov>0 = funnel below FoV)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; ka=d.kappa_log(:,1:d.idx);
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.final_alt>0.25; m.fov=r.fov_fail; m.krms=sqrt(mean(ka.^2,2));
end
