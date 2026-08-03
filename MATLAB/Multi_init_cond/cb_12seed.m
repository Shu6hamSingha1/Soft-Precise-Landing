function cb_12seed()
% Comprehensive validation: FINAL combined-barrier config (corrected c_tilde_h,
% s_ddot-drop, p_2inf_xy=0.5, p_r_inf=1.0 [Standing Cond 1], chi_r=0.85) across
% ALL 5 traj x 5 IC x 12 noisy seeds = 300 runs. SP per cell + grand total + breach.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    NS=12; GSP=0; GLD=0; worst=0;
    for ti=1:5
      line=sprintf('%-11s',trajs{ti});
      for ii=1:5
        sp=0; ld=0;
        for sd=1:NS
          d=run_one(ICs{ii},trajs{ti},sd);
          sp=sp + (d.land && d.xy<=0.08 && d.vel<=0.20); ld=ld+d.land; worst=max(worst,d.resid);
        end
        GSP=GSP+sp; GLD=GLD+ld; line=[line sprintf(' %s:%d/%d',icn{ii},sp,NS)];
      end
      fprintf('%s\n',line);
    end
    fprintf('  >> 12-SEED GRAND TOTAL: SP=%d/300 land=%d/300 worstResid=%.3f\n',GSP,GLD,worst);
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
