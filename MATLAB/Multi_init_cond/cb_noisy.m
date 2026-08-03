function cb_noisy()
% Noisy-seed validation of the 25/25 combined-barrier config (chi_r=0.65, p_2inf=0.5,
% s_ddot-drop, corrected). Full 5x5 x 3 seeds with IBVS pixel noise. SP per seed + total.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    seeds=[1 2 3]; tot=0; totLD=0; worst=0;
    for sd=seeds
      SP=0; LD=0;
      for ti=1:5
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},sd);
          sp=d.land && d.xy<=0.08 && d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
        end
      end
      fprintf('  seed %d: SP=%d/25 land=%d/25\n',sd,SP,LD); tot=tot+SP; totLD=totLD+LD;
    end
    fprintf('  >> NOISY TOTAL SP=%d/75 land=%d/75 worstResid=%.3f\n',tot,totLD,worst);
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.65;0.65];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('  ERR %s\n',ME.message); end
end
