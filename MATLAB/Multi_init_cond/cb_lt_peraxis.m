function cb_lt_peraxis()
% VALID scale-free low-tau (tau=0.5, NO z-taper). Full per-axis POSITION + VELOCITY +
% per-axis funnel residency for every non-SP cell. Categorize the binding axis/mode.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    SP=0;
    fprintf('cell        | px     py   | vx     vy     vz   | rresid_x rresid_y | mode\n');
    for ti=1:5
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; if sp; continue; end
        mode='soft'; if d.xy>0.08&&d.vel>0.20; mode='both'; elseif d.xy>0.08; mode='precise'; end
        latv=hypot(d.vx,d.vy);
        fprintf('%-4s %-7s| %+.3f %+.3f| %+.3f %+.3f %+.3f| %.2f    %.2f   | %s (lat_v=%.2f vz=%.2f)\n',...
          trajs{ti},icn{ii},d.px,d.py,d.vx,d.vy,d.vz,d.rrx,d.rry,mode,latv,abs(d.vz));
      end
    end
    fprintf('  >> low-tau(0.5) scale-free SP=%d/25\n',SP);
end
function d=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.5; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('xy',NaN,'vel',NaN,'px',NaN,'py',NaN,'vx',NaN,'vy',NaN,'vz',NaN,'rrx',NaN,'rry',NaN,'land',0);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); prel=I_p_c-x_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx); res=abs(rb)./prr;
        d.land=double(landed); d.xy=norm(prel(1:2)); d.vel=norm(vrel); d.px=prel(1); d.py=prel(2);
        d.vx=vrel(1); d.vy=vrel(2); d.vz=vrel(3); d.rrx=max(res(1,:)); d.rry=max(res(2,:));
    catch ME; end
end
