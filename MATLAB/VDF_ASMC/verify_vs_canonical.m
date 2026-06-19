function verify_vs_canonical()
%VERIFY_VS_CANONICAL  Clean package vs canonical visualControl_IBVS_adaptive (5x5).
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here,'..','Common'), fullfile(here,'..','Multi_init_cond'), here);
    trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    fprintf('  cell          | canon xy/vel/L | clean xy/vel/L | dxy   dvel\n');
    spc=0; spn=0; worst=0;
    for ti=1:5
      for ii=1:5
        c = run_canon(ICs{ii},trajs{ti});
        n = run_clean(ICs{ii},trajs{ti});
        spc=spc+(c.l&&c.xy<=0.08&&c.vel<=0.20); spn=spn+(n.l&&n.xy<=0.08&&n.vel<=0.20);
        dxy=abs(c.xy-n.xy); dvel=abs(c.vel-n.vel); worst=max(worst,max(dxy,dvel));
        mm=''; if dxy>2e-3||dvel>2e-3||c.l~=n.l; mm=' <-- MISMATCH'; end
        fprintf('  %-10s %s | %.3f/%.3f/%d | %.3f/%.3f/%d | %.4f %.4f%s\n',...
                trajs{ti},icn{ii},c.xy,c.vel,c.l,n.xy,n.vel,n.l,dxy,dvel,mm);
      end
    end
    fprintf('  >> canon SP=%d/25  clean SP=%d/25  worst|delta|=%.4f\n',spc,spn,worst);
end
function c=run_canon(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    try
        visualControl_IBVS_adaptive;
        c=struct('xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(I_v_c-dx_t(1:3,idx)),'l',double(landed));
    catch ME; fprintf(' canon ERR %s\n',ME.message); c=struct('xy',9,'vel',9,'l',0); end
end
function n=run_clean(ic,traj)
    x0=[ic(:); 1;0;0;0; zeros(6,1)];
    try
        R=simulate_landing(x0, traj, struct('noise',false));
        n=struct('xy',R.final_xy,'vel',R.final_rel_vel,'l',double(R.landed));
    catch ME; fprintf(' clean ERR %s\n',ME.message); n=struct('xy',9,'vel',9,'l',0); end
end
