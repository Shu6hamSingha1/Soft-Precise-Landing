function cb_push_liss()
% Push the residual seed-4 Liss IC3 lateral divergence (breach 1.7). kR_x>4 over-
% stiffens (CB37), so use the OTHER damping lever per-axis: chi_r_x LOWER (X-ring
% damping) while keeping chi_r_y=0.65 (Circ IC4 precision). Base: kR=[4;4;0.5],
% p2inf_z=1.0, keep s_ddot. Noisy 4 seeds x 5x5; track Liss IC3 vel + breach per cfg.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfgs={[0.65;0.65],[0.55;0.65],[0.50;0.65],[0.60;0.60]};
    for ci=1:numel(cfgs)
      chi=cfgs{ci};
      % noiseless guard
      nlSP=0; nlLD=0; nlw=0;
      for ti=1:5; for ii=1:5
          d=run_one(ICs{ii},trajs{ti},chi,0,0);
          nlSP=nlSP+(d.land&&d.xy<=0.08&&d.vel<=0.20); nlLD=nlLD+d.land; nlw=max(nlw,d.resid);
      end; end
      fprintf('=== chi_r=[%.2f;%.2f] === NOISELESS SP=%d/25 land=%d/25 resid=%.3f\n',chi(1),chi(2),nlSP,nlLD,nlw);
      tot=0; totLD=0; nworst=0;
      for sd=1:4
        sSP=0; sLD=0; l3=NaN;
        for ti=1:5; for ii=1:5
            d=run_one(ICs{ii},trajs{ti},chi,1,sd);
            if ti==4&&ii==3; l3=d.vel; end
            sSP=sSP+(d.land&&d.xy<=0.08&&d.vel<=0.20); sLD=sLD+d.land; nworst=max(nworst,d.resid);
        end; end
        fprintf('   seed %d: SP=%d/25 land=%d/25  LissIC3=%.3f\n',sd,sSP,sLD,l3); tot=tot+sSP; totLD=totLD+sLD;
      end
      fprintf('   >> chi_r=[%.2f;%.2f]  NOISY SP=%d/100 land=%d/100 worstResid=%.3f\n\n',chi(1),chi(2),tot,totLD,nworst);
    end
end
function d=run_one(ic,traj,chi,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=chi; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
