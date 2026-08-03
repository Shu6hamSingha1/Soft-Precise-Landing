function cb_yaw_perf()
% Yaw control performance check on the best keep-s_ddot config (chi_r=0.65, kR=[4;4;0.5],
% p2inf_z=1.0). Per cell: terminal |e_a| (orientation error, deg), peak |e_a|, peak yaw
% rate |u_a| (rad/s), final kappa_a. Noiseless 5x5 + the diverging Liss IC3 seed 4 (does
% yaw diverge with the lateral cycle, or stay bounded?).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    fprintf('=== NOISELESS yaw perf ===\n');
    fprintf('  cell          | term|e_a|deg | peak|e_a|deg | peak|u_a| | kappa_a_f\n');
    for ti=[1 3 4 5]   % Static, Sinusoidal, Lissajous, Circular (skip Linear for brevity)
      for ii=[3 4 5]
        y=run_one(ICs{ii},trajs{ti},0,0);
        fprintf('  %-10s %s | %8.2f    | %8.2f    | %7.3f  | %6.2f\n',trajs{ti},icn{ii},y.tea,y.pea,y.pua,y.ka);
      end
    end
    fprintf('\n=== Liss IC3 SEED 4 (the lateral divergence) ===\n');
    y=run_one(ICs{3},"Lissajous",1,4);
    fprintf('  term|e_a|=%.2f deg  peak|e_a|=%.2f deg  peak|u_a|=%.3f rad/s  kappa_a_f=%.2f  landed=%d\n',...
            y.tea,y.pea,y.pua,y.ka,y.land);
end
function y=run_one(ic,traj,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        n=idx; ea=e_a(1:n); ua=u_a_log(1:n);
        y=struct('tea',abs(ea(n))*180/pi,'pea',max(abs(ea))*180/pi,'pua',max(abs(ua)),'ka',kappa_a(n),'land',double(landed));
    catch ME
        y=struct('tea',NaN,'pea',NaN,'pua',NaN,'ka',NaN,'land',0);
    end
end
