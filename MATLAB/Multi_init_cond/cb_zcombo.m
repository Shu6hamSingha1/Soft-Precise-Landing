function cb_zcombo()
% COMBINATION tuning for the descent (Z) cycle: pair the damping lever with the cure
% for its runaway. kappa0_z=0.5 gives best noiseless damping (-0.24) but kappa_z runs
% away on breach -> cap it (KAPPA_MAX_Z) + raise leakage P_z. Base = 92/100 config
% (chi_r=0.65, kR=[4;4;0.5], p2inf_z=1.0, keep s_ddot). Noiseless 5x5 + noisy 4 seeds.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    % {label, kappa0_z, KAPPA_MAX_Z, P_z, Gamma_z}  ([]=default 0.25 / off / 5.0 / 0.75)
    cfgs={{'base',         [],   [],  [], []}, ...
          {'k0z.5+cap1',   0.5,  1.0, [], []}, ...
          {'k0z.5+cap.6+P15',0.5, 0.6, 15, []}, ...
          {'k0z.5+cap.6+P15+Gz1.5',0.5,0.6,15,1.5}, ...
          {'cap.4+P20',    [],   0.4, 20, []} };
    for c=cfgs
      cc=c{1}; lab=cc{1}; k0=cc{2}; cap=cc{3}; pz=cc{4}; gz=cc{5};
      nlSP=0; nlLD=0; nlw=0;
      for ti=1:5; for ii=1:5
          d=run_one(ICs{ii},trajs{ti},0,0,k0,cap,pz,gz);
          nlSP=nlSP+(d.land&&d.xy<=0.08&&d.vel<=0.20); nlLD=nlLD+d.land; nlw=max(nlw,d.resid);
      end; end
      fprintf('=== %s === NOISELESS SP=%d/25 land=%d/25 resid=%.3f\n',lab,nlSP,nlLD,nlw);
      if nlLD<24; fprintf('   (skip noisy)\n\n'); continue; end
      tot=0; totLD=0; nworst=0;
      for sd=1:4
        sSP=0; sLD=0;
        for ti=1:5; for ii=1:5
            d=run_one(ICs{ii},trajs{ti},1,sd,k0,cap,pz,gz);
            sSP=sSP+(d.land&&d.xy<=0.08&&d.vel<=0.20); sLD=sLD+d.land; nworst=max(nworst,d.resid);
        end; end
        fprintf('   seed %d: SP=%d/25 land=%d/25\n',sd,sSP,sLD); tot=tot+sSP; totLD=totLD+sLD;
      end
      fprintf('   >> %s  NOISY SP=%d/100 land=%d/100 worstResid=%.3f\n\n',lab,tot,totLD,nworst);
    end
end
function d=run_one(ic,traj,noise,seed,k0,cap,pz,gz)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE ...
           KAPPA0_Z_OVERRIDE KAPPA_MAX_Z P_Z_OVERRIDE GAMMA_Z_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    KAPPA0_Z_OVERRIDE=k0; KAPPA_MAX_Z=cap; P_Z_OVERRIDE=pz; GAMMA_Z_OVERRIDE=gz;
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
