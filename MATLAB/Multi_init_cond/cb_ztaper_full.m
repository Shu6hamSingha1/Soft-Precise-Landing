function cb_ztaper_full()
% Full 5x5 at low tau=0.5 + z-taper zref=2.0 (responsive s_ddot, terminal roll-off, NO lag).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    SP=0; LD=0; worst=0; fails={};
    for ti=1:5
      line=sprintf('%-11s',trajs{ti});
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
        tag='..'; if ~d.land; tag='XX'; elseif sp; tag='SP'; elseif d.xy<=0.08; tag='So'; else; tag='Pr'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s',icn{ii},tag)];
        if ~sp; fails{end+1}=sprintf('%s %s: xy=%.3f vel=%.3f [vx%+.2f vy%+.2f vz%+.2f]',trajs{ti},icn{ii},d.xy,d.vel,d.vx,d.vy,d.vz); end
      end
      fprintf('%s\n',line);
    end
    fprintf('  >> tau0.5+zref2.0  SP=%d/25 land=%d/25 worstResid=%.3f\n',SP,LD,worst);
    for f=1:numel(fails); fprintf('   FAIL %s\n',fails{f}); end
end
function d=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU CB_SDDOT_ZTAPER P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.5; CB_SDDOT_ZTAPER=2.0; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN,'vx',NaN,'vy',NaN,'vz',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr)); d.vx=vrel(1); d.vy=vrel(2); d.vz=vrel(3);
    catch ME; d.land=0; end
end
