function cb_skept_noisy()
% Noisy validation of the KEEP-s_ddot 25/25 config (tau=1.5, chi_r=0.85). 3 seeds x 5x5.
% Compare to the drop config's 75/75 (does keeping the target-accel FF help under noise?).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    tot=0; totLD=0; worst=0;
    for sd=1:3
      SP=0; LD=0;
      for ti=1:5, for ii=1:5
        d=run_one(ICs{ii},trajs{ti},sd); SP=SP+(d.land&&d.xy<=0.08&&d.vel<=0.20); LD=LD+d.land; worst=max(worst,d.resid);
      end, end
      fprintf('  seed %d: SP=%d/25 land=%d/25\n',sd,SP,LD); tot=tot+SP; totLD=totLD+LD;
    end
    fprintf('  >> KEEP-s_ddot(tau1.5) NOISY SP=%d/75 land=%d/75 worstResid=%.3f  (drop was 75/75)\n',tot,totLD,worst);
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=1.5; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
