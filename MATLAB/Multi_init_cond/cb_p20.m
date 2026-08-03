function cb_p20()
% Push the residual via the EARLY flow funnel: the Liss IC3 seed-4 cycle ignites at
% altitude (|v|=1.8 @4m) without breaching because p_20_xy=25 is huge. Tighten p_20_xy
% to constrain the flow-error envelope early and catch the oscillation before it grows.
% Risk: too tight breaches on the initial IC offset. Best config: chi_r=0.65, kR=[4;4;0.5],
% p2inf_z=1.0, keep s_ddot. Noiseless guard + noisy 4 seeds; track Liss IC3 + breach.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    p20s=[25 14 10 7];
    for p20=p20s
      nlSP=0; nlLD=0; nlw=0;
      for ti=1:5; for ii=1:5
          d=run_one(ICs{ii},trajs{ti},p20,0,0);
          nlSP=nlSP+(d.land&&d.xy<=0.08&&d.vel<=0.20); nlLD=nlLD+d.land; nlw=max(nlw,d.resid);
      end; end
      fprintf('=== p_20_xy=%d === NOISELESS SP=%d/25 land=%d/25 resid=%.3f\n',p20,nlSP,nlLD,nlw);
      if nlLD<24; fprintf('   (skip noisy: land<24)\n\n'); continue; end
      tot=0; totLD=0; nworst=0;
      for sd=1:4
        sSP=0; sLD=0; l3=NaN;
        for ti=1:5; for ii=1:5
            d=run_one(ICs{ii},trajs{ti},p20,1,sd);
            if ti==4&&ii==3; l3=d.vel; end
            sSP=sSP+(d.land&&d.xy<=0.08&&d.vel<=0.20); sLD=sLD+d.land; nworst=max(nworst,d.resid);
        end; end
        fprintf('   seed %d: SP=%d/25 land=%d/25  LissIC3=%.3f\n',sd,sSP,sLD,l3); tot=tot+sSP; totLD=totLD+sLD;
      end
      fprintf('   >> p_20_xy=%d  NOISY SP=%d/100 land=%d/100 worstResid=%.3f\n\n',p20,tot,totLD,nworst);
    end
end
function d=run_one(ic,traj,p20,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE P20_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; P20_XY_OVERRIDE=[p20;p20]; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
