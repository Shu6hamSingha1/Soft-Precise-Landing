function cb_drop4()
% FAIR COMPARISON: the baked DROP config (75/75) was only validated on seeds 1-3
% (CB16). Run baked drop over the SAME 4 seeds as the keep-s_ddot 92/100 campaign,
% incl. the hard seed 4. If drop also ~92/100 over 4 seeds, the "drop strictly better"
% claim was a seed artifact. Baked drop = COMBINED_BARRIER+C_SIMPLE, no overrides
% (chi_r=0.85, kR default, p2inf default, s_ddot-drop default-on).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    tot=0; totLD=0; worst=0;
    for sd=1:4
      SP=0; LD=0; s5=NaN;
      for ti=1:5; for ii=1:5
          d=run_one(ICs{ii},trajs{ti},sd);
          if ti==1&&ii==5; s5=d.vel; end
          SP=SP+(d.land&&d.xy<=0.08&&d.vel<=0.20); LD=LD+d.land; worst=max(worst,d.resid);
      end; end
      fprintf('  seed %d: SP=%d/25 land=%d/25  StaticIC5 vel=%.3f\n',sd,SP,LD,s5); tot=tot+SP; totLD=totLD+LD;
    end
    fprintf('  >> BAKED DROP, 4 seeds: SP=%d/100 land=%d/100 worstResid=%.3f\n',tot,totLD,worst);
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE ...
           CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    % baked defaults: clear all tuning overrides
    CB_DROP_SDDOT=[]; CB_SDDOT_TAU=[]; P2INF_XY_OVERRIDE=[]; P2INF_Z_OVERRIDE=[]; CHI_R_OVERRIDE=[]; PRINF_OVERRIDE=[]; KR_OVERRIDE=[]; KOMEGA_OVERRIDE=[];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
