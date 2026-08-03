function cb_suite_proof()
% Proof-guided config validation: p_r_inf=1.0 (Standing Cond 1) + higher chi_r (per
% manifold |zeta_r|~|zeta_h|/chi_r). Full 5x5 for chi_r in {0.85,1.0}, p_2inf=0.5,
% s_ddot-drop. Report SP + the BINDING margin (max vel among landed) + worst residency.
    for chi=[0.85 1.0]
      trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
      ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
      SP=0; LD=0; worst=0; maxv=0; binder='';
      for ti=1:5
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},chi);
          sp=d.land && d.xy<=0.08 && d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
          if d.land && d.vel>maxv; maxv=d.vel; binder=sprintf('%s IC%d',trajs{ti},ii); end
        end
      end
      fprintf('  chi_r=%.2f: SP=%d/25 land=%d/25 | binding margin: %s vel=%.3f | worstResid=%.3f\n',chi,SP,LD,binder,maxv,worst);
    end
end
function d=run_one(ic,traj,chi)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[chi;chi]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('  ERR %s\n',ME.message); end
end
