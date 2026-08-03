function cb_pinvtol2()
% Find the pinv tolerance that RETAINS the loom mode for the near-ground high-featRad
% geometry (fixes Static IC5 seed4) WITHOUT admitting noise elsewhere. tol=4 -> suite
% 92 but S5s4 blows up; tol<=1 -> S5s4 lands but suite 44. Sweep the threshold (1.5-3.5).
% keep s_ddot 92-config. Full 5x5 noiseless + noisy 4 seeds + Static IC5 seed4 outcome.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    for tol=[4 3.5 3 2.5 2 1.5]
      nl=0; nlw=0;
      for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},0,0,tol); nl=nl+d.sp; nlw=max(nlw,d.rs); end; end
      ns=0; nsw=0; s5=0;
      for sd=1:4; for ti=1:5; for ii=1:5
          d=one(ICs{ii},trajs{ti},1,sd,tol); ns=ns+d.sp; nsw=max(nsw,d.rs);
          if ti==1&&ii==5&&sd==4; s5=d.land; end
      end; end; end
      fprintf('  tol=%-4g : noiseless SP=%d/25 (resid %.2f) | noisy SP=%d/100 (resid %.2f) | S5s4 land=%d\n',...
              tol,nl,nlw,ns,nsw,s5);
    end
end
function d=one(ic,traj,noise,seed,tol)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE PINV_TOL
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1]; PINV_TOL=tol;
    try
        visualControl_IBVS_adaptive; vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('sp',double(landed&&norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08&&norm(vrel)<=0.20),'land',double(landed),'rs',max(max(abs(rb)./prr)));
    catch ME; d=struct('sp',0,'land',0,'rs',9.99); end
end
