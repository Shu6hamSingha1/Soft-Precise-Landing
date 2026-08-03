function cb_seed4_z()
% Identify the 8 failures of the 92/100 config (all on seed 4): chi_r=0.65,
% kR=[4;4;0.5], p2inf_z=1.0, keep s_ddot. Per-axis rel-vel + resid; classify cause.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    nf=0;
    fprintf('=== 92/100 config, seed 4 -- the 8 failures ===\n');
    for ti=1:5
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20;
        if sp; continue; end
        nf=nf+1;
        % classify dominant cause
        cause='';
        if ~d.land; cause='NO-LAND';
        elseif abs(d.vz)>=0.20 && abs(d.vz)>hypot(d.vx,d.vy); cause='DESCENT vz';
        elseif hypot(d.vx,d.vy)>=0.20; if abs(d.vx)>abs(d.vy); cause='LATERAL vx'; else; cause='LATERAL vy'; end
        elseif d.xy>0.08; cause='PRECISION xy';
        else; cause='marginal'; end
        if d.resid>=1; cause=[cause ' +BREACH']; end
        fprintf('  %-10s %s | xy=%.3f |v|=%.3f (vx%+.3f vy%+.3f vz%+.3f) resid=%.2f | %s\n',...
                trajs{ti},icn{ii},d.xy,d.vel,d.vx,d.vy,d.vz,d.resid,cause);
      end
    end
    fprintf('  >> total failures on seed 4 = %d (expect 8 -> 25-8=17/25)\n',nf);
end
function d=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),...
                 'vx',vrel(1),'vy',vrel(2),'vz',vrel(3),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'vx',NaN,'vy',NaN,'vz',NaN,'resid',9.99);
    end
end
