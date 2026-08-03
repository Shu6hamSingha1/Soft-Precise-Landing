function cb_port_verify()
% Verify the combined-barrier port: run_simulation.m (ported) must reproduce the
% canonical visualControl_IBVS_adaptive.m (COMBINED_BARRIER=1,C_SIMPLE=1) per-cell,
% noiseless, across the 5x5 manuscript set.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    fprintf('  cell          | canon xy / vel / L | rsim xy / vel / L | dxy   dvel\n');
    spc=0; spr=0; worst=0;
    for ti=1:5
      for ii=1:5
        c=run_canon(ICs{ii},trajs{ti});
        r=run_rsim(ICs{ii},trajs{ti});
        csp=c.l&&c.xy<=0.08&&c.vel<=0.20; rsp=r.l&&r.xy<=0.08&&r.vel<=0.20; spc=spc+csp; spr=spr+rsp;
        dxy=abs(c.xy-r.xy); dvel=abs(c.vel-r.vel); worst=max(worst,max(dxy,dvel));
        mm=''; if dxy>1e-3||dvel>1e-3||c.l~=r.l; mm=' <-- MISMATCH'; end
        fprintf('  %-10s %s | %.3f/%.3f/%d | %.3f/%.3f/%d | %.4f %.4f%s\n',...
                trajs{ti},icn{ii},c.xy,c.vel,c.l,r.xy,r.vel,r.l,dxy,dvel,mm);
      end
    end
    fprintf('  >> canon SP=%d/25  rsim SP=%d/25  worst|delta|=%.4f\n',spc,spr,worst);
end
function c=run_canon(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    try
        visualControl_IBVS_adaptive;
        c=struct('xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(I_v_c-dx_t(1:3,idx)),'l',double(landed));
    catch ME; c=struct('xy',9,'vel',9,'l',0); end
end
function r=run_rsim(ic,traj)
    x0=[ic(:); 1;0;0;0; zeros(6,1)];
    cfg=struct('NOISE',0);   % default GE/delay (match canonical noiseless)
    try
        out=run_simulation(x0, traj, [], 1.0, cfg, 1000);
        r=struct('xy',out.final_xy,'vel',out.final_rel_vel,'l',double(out.success));
    catch ME; fprintf('  rsim ERR %s\n',ME.message); r=struct('xy',9,'vel',9,'l',0); end
end
