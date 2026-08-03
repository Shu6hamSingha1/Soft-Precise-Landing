function cb_ztaper_noisy()
% NOISY payoff test: responsive low-tau + z-taper (tau=0.5, zref=2.0) vs heavy-tau (74/75)
% and drop (75/75). 3 seeds x 5x5. Does the un-lagged FF fix the chase cells under noise?
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    tot=0; totLD=0; worst=0; miss={};
    for sd=1:3
      SP=0; LD=0;
      for ti=1:5, for ii=1:5
        d=run_one(ICs{ii},trajs{ti},sd); ok=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+ok; LD=LD+d.land; worst=max(worst,d.resid);
        if ~ok; miss{end+1}=sprintf('s%d %s %s xy=%.2f vel=%.2f',sd,trajs{ti},icn{ii},d.xy,d.vel); end
      end, end
      fprintf('  seed %d: SP=%d/25 land=%d/25\n',sd,SP,LD); tot=tot+SP; totLD=totLD+LD;
    end
    fprintf('  >> tau0.5+zref2.0 NOISY SP=%d/75 land=%d/75 worstResid=%.3f  (drop 75/75, heavy-tau 74/75)\n',tot,totLD,worst);
    for m=1:numel(miss); fprintf('   miss %s\n',miss{m}); end
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU CB_SDDOT_ZTAPER P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.5; CB_SDDOT_ZTAPER=2.0; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
