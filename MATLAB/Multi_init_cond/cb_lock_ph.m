%% CB_LOCK_PH  Lock the pr-tight+E.5 precision config, then TIGHTEN p_h (optic-flow funnel)
%   to arrest v_lat before the deck (engage zeta_h velocity damping = the PX4 lever).
%   The 7x/8x failure was a terminal SOFT-fail: h_e=v_lat/z blows up at the deck. p_h
%   constrains h_e -> tightening it should arrest v_lat under stress -> fix the soft-fail
%   -> extend margin to 8x. RISK: the PPC overtake (p_h contracts faster than h_e
%   converges) -> watch maxEngH (>1 = overtake) + fly-aways.
%
%   LOCKED base: kappa0 .05, N .10, Pleak [.5;.5;1.5], E [.5;.5;.5], Xi_r .3, p_rinf .85.
%   Tighten p_h via Xi_h (rate) + p_hinf (lateral floor). p_h0 FoV-fixed (not changed).
%
% Run:  cd MATLAB/Multi_init_cond; cb_lock_ph
% Saves: ../Datasets/MultiInit/lock_ph.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

LK = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
            'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]),'Xi_r',diag([0.3,0.3]),'p_rinf',[0.85;0.85]);
mk = @(xih,phl) setfield(setfield(LK,'Xi_h',diag([xih xih xih])),'p_hinf',[phl;phl;1.5]); %#ok<*SFLD>
F = { 'L0 baked p_h (Xih.2 phinf1)', mk(0.2, 1.0);
      'L1 Xih.35 (faster flow)',     mk(0.35,1.0);
      'L2 p_hinf_xy.6 (lower floor)', mk(0.2, 0.6);
      'L3 Xih.35 + p_hinf_xy.6',      mk(0.35,0.6) };

trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

R = struct('name',{},'gate',{},'mxy',{},'maxv',{},'fly',{},'engH',{},'vterm7',{},'sp7',{},'sp8',{});
for vi=1:size(F,1)
    cfg=F{vi,2};
    g=0; xy=[]; mv=[]; fly=0;
    for t=1:numel(trajList)
        for ic=1:5
            m=onerun(cfg,[],p0(ic,:),trajList(t));
            g=g+m.sp; xy(end+1)=m.xy; mv(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
        end
    end
    sp7=0; sp8=0; eH=[]; vt=[];
    for c=1:size(scells,1)
        m7=onerun(cfg,7,p0(scells{c,2},:),scells{c,1}); sp7=sp7+m7.sp; eH(end+1)=m7.engH; vt(end+1)=m7.vterm; %#ok<AGROW>
        sp8=sp8+onerun(cfg,8,p0(scells{c,2},:),scells{c,1}).sp;
    end
    R(vi)=struct('name',F{vi,1},'gate',g,'mxy',mean(xy),'maxv',max(mv),'fly',fly, ...
                 'engH',max(eH),'vterm7',median(vt),'sp7',sp7,'sp8',sp8); %#ok<AGROW>
    fprintf('  %-30s | gate %2d/25 mXY %.3f fly %d | 7x maxEngH %.2f vterm %.3f | 7x %d/5  8x %d/5\n', ...
        F{vi,1}, g, mean(xy), fly, max(eH), median(vt), sp7, sp8);
end

clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','lock_ph.mat'),'R','F');
fprintf('\n(want: tighter p_h -> 7x vterm DOWN (v_lat arrested) + 8x recovers, WITHOUT engH>1 overtake / fly)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; idx=d.idx;
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.final_alt>0.25;
    z=1./max(d.beta_log(1:idx),1e-6);
    m.engH=max(max(abs(d.V_h_e(1:2,1:idx))./max(d.p_h_log(1:2,1:idx),eps),[],1));
    vlat=vecnorm(d.X_DS(8:9,1:idx)-d.dx_t(1:2,1:idx)); m.vterm=median(vlat(z<0.5));
end
