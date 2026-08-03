function cb_loomgain()
% Apply the scale-free lag-compensation gain to the moment loom (slope~0.82 -> aim 1.0)
% and sweep it. USE_MOMENT_LOOM=1, keep s_ddot 92-config. Full 5x5 noiseless + noisy 4sd.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    for klg=[1.0 1.1 1.2 1.3]
      nl=0; nlw=0;
      for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},0,0,klg); nl=nl+d.sp; nlw=max(nlw,d.rs); end; end
      ns=0; nsw=0;
      for sd=1:4; for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},1,sd,klg); ns=ns+d.sp; nsw=max(nsw,d.rs); end; end; end
      fprintf('  loomGain=%.1f : noiseless SP=%d/25 (r%.2f) | noisy SP=%d/100 (r%.2f)\n',klg,nl,nlw,ns,nsw);
    end
end
function d=one(ic,traj,noise,seed,klg)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE USE_MOMENT_LOOM MOMENT_LOOM_GAIN USE_REDLAT USE_MOMENT_FLOW
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    USE_MOMENT_LOOM=1; MOMENT_LOOM_GAIN=klg; USE_REDLAT=0; USE_MOMENT_FLOW=0;
    try
        visualControl_IBVS_adaptive; vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('sp',double(landed&&norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08&&norm(vrel)<=0.20),'rs',max(max(abs(rb)./prr)));
    catch ME; d=struct('sp',0,'rs',9.99); end
end
