function cb_hrd_up()
% STEEPER descent guidance (user: "if gentler hurt, try increasing h_rd"). h_rd more
% negative = faster descent = LESS flight time = less noise exposure (but bigger descent
% cycle / harder vz). Moment loom, keep-s_ddot 92-config. Per h_rd: NOISY SP/land/breach
% (4 seeds) + NOISELESS per-axis terminal + descent-cycle pp(vz).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    fprintf('  h_rd | noisy SP land res | nl term|vz| |vxy| xy  cyc-pp(vz)\n');
    for hrd=[-0.42 -0.46 -0.50 -0.55]
      ns=0; nsLD=0; nsw=0;
      for sd=1:4; for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},1,sd,hrd); ns=ns+d.sp; nsLD=nsLD+d.land; nsw=max(nsw,d.rs); end; end; end
      VZ=[];VXY=[];XY=[];PPZ=[];
      for ti=1:5; for ii=1:5; d=one(ICs{ii},trajs{ti},0,0,hrd); VZ=[VZ abs(d.vz)]; VXY=[VXY d.vxy]; XY=[XY d.xy]; PPZ=[PPZ d.ppz]; end; end %#ok<AGROW>
      fprintf('  %.2f | %3d/100 %3d %5.2f | %.3f  %.3f %.3f  %.3f\n',...
              hrd, ns, nsLD, nsw, mean(VZ), mean(VXY), mean(XY), mean(PPZ));
    end
end
function d=one(ic,traj,noise,seed,hrd)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE USE_MOMENT_LOOM H_RD_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    USE_MOMENT_LOOM=1; H_RD_OVERRIDE=hrd;
    try
        visualControl_IBVS_adaptive; n=idx; vrel=I_v_c-dx_t(1:3,idx); w=min(300,n-5); k=(n-w):n;
        vz=X_DS(10,k)-dx_t(3,k); rb=r_bar_e(:,1:n); prr=p_r(:,1:n);
        d=struct('sp',double(landed&&norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08&&norm(vrel)<=0.20),'land',double(landed),...
                 'vz',vrel(3),'vxy',norm(vrel(1:2)),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'ppz',max(vz)-min(vz),'rs',max(max(abs(rb)./prr)));
    catch ME; d=struct('sp',0,'land',0,'vz',9,'vxy',9,'xy',9,'ppz',9,'rs',9.99); end
end
