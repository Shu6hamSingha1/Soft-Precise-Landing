function cb_seed4_diag()
% Best symmetric stacked-damping config (chi_r=0.65, kR_xy=4, keep s_ddot) = 89/100;
% seed 4 is the catastrophe (14/25, breach 1.70). Find WHICH cells/axes seed 4 fails
% on -> targets the per-axis velocity-LC tuning. Report per-axis rel-vel + resid, seed 4.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    fprintf('=== chi_r=0.65 kR=[4;4;0.5] seed 4, per-axis ===\n');
    for ti=1:5
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti},4);
        flag=''; if ~(d.land&&d.xy<=0.08&&d.vel<=0.20); flag=' <== FAIL'; end
        if d.resid>=1; flag=[flag ' BREACH']; end
        fprintf('  %-11s %s xy=%.3f |v|=%.3f (vx%+.3f vy%+.3f vz%+.3f) resid=%.2f%s\n',...
                trajs{ti},icn{ii},d.xy,d.vel,d.vx,d.vy,d.vz,d.resid,flag);
      end
    end
end
function d=run_one(ic,traj,kx)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[kx;kx;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),...
                 'vx',vrel(1),'vy',vrel(2),'vz',vrel(3),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'vx',NaN,'vy',NaN,'vz',NaN,'resid',9.99);
    end
end
