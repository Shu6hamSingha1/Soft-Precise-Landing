function cb_p2z()
% Per-axis Z tune: the noise wall (seed 4) is descent-velocity dominated (vz~0.20 on
% ~8 cells). Tighten the descent flow-funnel floor p_2inf(3) (default 1.5) for vz
% margin, on the best lateral combo chi_r=0.65 kR_xy=4 (keep s_ddot). Noiseless 5x5
% + noisy 4 seeds. Watch for non-landing (too-slow descent) / breach (too tight).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    p2zs=[1.5 1.0 0.7];
    for p2z=p2zs
      SP=0; LD=0; worst=0;
      for ti=1:5; for ii=1:5
          d=run_one(ICs{ii},trajs{ti},p2z,0,0);
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
      end; end
      fprintf('=== p2inf_z=%.1f === NOISELESS SP=%d/25 land=%d/25 worstResid=%.3f\n',p2z,SP,LD,worst);
      if LD<24; fprintf('   (skip noisy: land<24)\n\n'); continue; end
      tot=0; totLD=0; nworst=0;
      for sd=1:4
        sSP=0; sLD=0;
        for ti=1:5; for ii=1:5
            d=run_one(ICs{ii},trajs{ti},p2z,1,sd);
            sp=d.land&&d.xy<=0.08&&d.vel<=0.20; sSP=sSP+sp; sLD=sLD+d.land; nworst=max(nworst,d.resid);
        end; end
        fprintf('   seed %d: SP=%d/25 land=%d/25\n',sd,sSP,sLD); tot=tot+sSP; totLD=totLD+sLD;
      end
      fprintf('   >> p2inf_z=%.1f  NOISY SP=%d/100 land=%d/100 worstResid=%.3f\n\n',p2z,tot,totLD,nworst);
    end
end
function d=run_one(ic,traj,p2z,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=p2z; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
