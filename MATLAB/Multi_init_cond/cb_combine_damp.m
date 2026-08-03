function cb_combine_damp()
% Stacked-damping keep-s_ddot: combine chi_r-LOWER + kR_xy-RAISE (the only two net-
% damping levers, CB37) to beat the ~83-85/100 noise wall. tau=0.7, p2inf=0.5,
% prinf=1.0, kOmega=[0.6;0.6;0.1]. Noiseless 5x5 first; if SP>=24 & no breach -> noisy
% 4 seeds x 5x5. SP = land(zf0.20) & xy<=0.08 & |v|<=0.20.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    % {chi_r, kR_xy}
    combos={{0.65,4.0},{0.65,3.5},{0.50,4.0},{0.70,3.5}};
    for cb=combos
      chi=cb{1}{1}; kx=cb{1}{2}; kr=[kx;kx;0.5];
      % --- noiseless 5x5 ---
      SP=0; LD=0; worst=0; grid='';
      for ti=1:5
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},chi,kr,0,0);
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
        end
      end
      fprintf('=== chi_r=%.2f kR_xy=%.1f === NOISELESS SP=%d/25 land=%d/25 worstResid=%.3f\n',chi,kx,SP,LD,worst);
      if SP<24 || worst>=1
        fprintf('   (skip noisy: SP<24 or breach)\n\n'); continue;
      end
      % --- noisy 4 seeds ---
      tot=0; totLD=0; nworst=0;
      for sd=1:4
        sSP=0; sLD=0;
        for ti=1:5
          for ii=1:5
            d=run_one(ICs{ii},trajs{ti},chi,kr,1,sd);
            sp=d.land&&d.xy<=0.08&&d.vel<=0.20; sSP=sSP+sp; sLD=sLD+d.land; nworst=max(nworst,d.resid);
          end
        end
        fprintf('   seed %d: SP=%d/25 land=%d/25\n',sd,sSP,sLD); tot=tot+sSP; totLD=totLD+sLD;
      end
      fprintf('   >> chi_r=%.2f kR_xy=%.1f  NOISY SP=%d/100 land=%d/100 worstResid=%.3f\n\n',chi,kx,tot,totLD,nworst);
    end
end
function d=run_one(ic,traj,chi,kr,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[chi;chi]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=kr; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
