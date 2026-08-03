function cb_hrd_axis()
% Per-axis impact of h_rd (moment loom, keep-s_ddot 92-config), NOISELESS 5x5 pooled.
% Per h_rd report: terminal |vz| (descent vel), |v_xy| (lateral vel), xy (lateral pos);
% and LIMIT-CYCLE amplitude = peak-to-peak over last 3s of vz and of |v_xy|; + worst resid.
% Shows the Z-soften vs lateral-cycle-grow tradeoff.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    fprintf('  h_rd | term|vz| term|vxy|  xy  | cyc pp(vz) pp(vxy) | worstResid\n');
    for hrd=[-0.42 -0.38 -0.35 -0.30]
      VZ=[];VXY=[];XY=[];PPZ=[];PPX=[];RS=0;
      for ti=1:5; for ii=1:5
        r=one(ICs{ii},trajs{ti},hrd);
        VZ=[VZ abs(r.vz)]; VXY=[VXY r.vxy]; XY=[XY r.xy]; PPZ=[PPZ r.ppz]; PPX=[PPX r.ppx]; RS=max(RS,r.rs); %#ok<AGROW>
      end; end
      fprintf('  %.2f |  %.3f    %.3f   %.3f |  %.3f    %.3f   |   %.2f\n',...
              hrd, mean(VZ), mean(VXY), mean(XY), mean(PPZ), mean(PPX), RS);
    end
end
function r=one(ic,traj,hrd)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE USE_MOMENT_LOOM H_RD_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; RNG_SEED_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    USE_MOMENT_LOOM=1; H_RD_OVERRIDE=hrd;
    try
        visualControl_IBVS_adaptive; n=idx; vrel=I_v_c-dx_t(1:3,idx);
        w=min(300,n-5); k=(n-w):n;
        vz=X_DS(10,k)-dx_t(3,k); vxk=X_DS(8,k)-dx_t(1,k); vyk=X_DS(9,k)-dx_t(2,k); vxy=hypot(vxk,vyk);
        rb=r_bar_e(:,1:n); prr=p_r(:,1:n);
        r=struct('vz',vrel(3),'vxy',norm(vrel(1:2)),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),...
                 'ppz',max(vz)-min(vz),'ppx',max(vxy)-min(vxy),'rs',max(max(abs(rb)./prr)));
    catch ME; r=struct('vz',9,'vxy',9,'xy',9,'ppz',9,'ppx',9,'rs',9.99); end
end
