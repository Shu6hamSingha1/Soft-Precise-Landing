function cb_noisy_resp()
% Noisy validation of the RESPONSIVE-s_ddot 25/25 config (kR=[3;3;0.5], tau=1.2,
% chi_r=[0.85;0.85], p_2inf=0.5, prinf=1.0). Decides effect-vs-cycle: if tau only
% masks the terminal phase, noisy seeds re-break Liss IC3 (and other lateral rings).
% Full 5x5 x 4 seeds + per-seed Liss IC3 vel printed.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    seeds=[1 2 3 4]; tot=0; totLD=0; worst=0;
    for sd=seeds
      SP=0; LD=0; l3=NaN;
      for ti=1:5
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},sd);
          if ti==4&&ii==3; l3=d.vel; end
          sp=d.land && d.xy<=0.08 && d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
        end
      end
      fprintf('  seed %d: SP=%d/25 land=%d/25  LissIC3 vel=%.3f\n',sd,SP,LD,l3); tot=tot+SP; totLD=totLD+LD;
    end
    fprintf('  >> RESPONSIVE NOISY TOTAL SP=%d/100 land=%d/100 worstResid=%.3f\n',tot,totLD,worst);
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=1.2; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;3;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('  ERR %s\n',ME.message); end
end
