function cb_cbf_audit()
% CBF (target-visibility cbf2) performance audit. Per run: max tilt theta_current(deg),
% the cone theta_cone(deg), MIN margin (cone-current; <0 => visibility CONE VIOLATED),
% %steps CBF active (cbf_ok=false => clamping/at-edge), and did it ever lose visibility.
% 92-config (keep s_ddot). Noiseless 5x5 + the diverging noisy cells (Static IC5 s4,
% Liss IC3 s4) to see if CBF held visibility even during the descent/lateral blow-ups.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    fprintf('=== NOISELESS 5x5 CBF audit ===\n');
    fprintf('  cell          | maxTilt | minMargin(deg) | %%active | visLOST\n');
    worstm=99;
    for ti=1:5
      for ii=1:5
        c=run_one(ICs{ii},trajs{ti},0,0); worstm=min(worstm,c.minmarg);
        fprintf('  %-10s %s |  %5.1f  |    %+6.2f     |  %4.1f%%  |  %d\n',...
                trajs{ti},icn{ii},c.maxtilt,c.minmarg,c.pact,c.vlost);
      end
    end
    fprintf('  >> noiseless worst cone margin = %.2f deg (>=0 => visibility held all 25)\n\n',worstm);
    fprintf('=== NOISY seed4 diverging cells ===\n');
    c=run_one([2;2;-3],"Static",1,4);
    fprintf('  Static IC5 s4 (descent blow-up): maxTilt=%.1f minMargin=%+.2f %%active=%.1f visLOST=%d\n',c.maxtilt,c.minmarg,c.pact,c.vlost);
    c=run_one([2;-2;-5],"Lissajous",1,4);
    fprintf('  Liss   IC3 s4 (lateral blow-up): maxTilt=%.1f minMargin=%+.2f %%active=%.1f visLOST=%d\n',c.maxtilt,c.minmarg,c.pact,c.vlost);
end
function c=run_one(ic,traj,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        n=idx; tc=theta_cur_log(1:n)*180/pi; co=theta_cone_log(1:n)*180/pi; ok=cbf_ok_log(1:n);
        marg=co-tc;
        c=struct('maxtilt',max(tc),'minmarg',min(marg),'pact',100*mean(~ok),'vlost',double(any(marg< -0.01)));
    catch ME
        c=struct('maxtilt',NaN,'minmarg',NaN,'pact',NaN,'vlost',-1);
    end
end
