function cb_funnel_cause()
% WHY can't p_20_xy be tightened to catch the Liss IC3 cycle? Hypothesis: the flow
% funnel p_2 is sized for the LEGITIMATE initial-acquisition flow transient (large,
% from the IC offset). That headroom is exactly what lets the noise-ignited cycle grow
% without breaching. Measure, noiseless, per cell: peak lateral flow error |V_h_e_xy|,
% WHEN it peaks (early acquisition vs late), and the flow-funnel occupancy S2=|V_h_e|/p_2
% at p_20=25 and what it would be if p_20=14. Then Liss IC3 seed4 cycle's flow excursion.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    fprintf('=== NOISELESS: legit flow transient vs funnel (p_20_xy=25) ===\n');
    fprintf('  cell          | pk|Vhe_xy| | t_pk(s) | gap@pk | maxOcc@p20=25 | est maxOcc@p20=14\n');
    for ti=1:5
      for ii=[3 4 5]
        r=run_one(ICs{ii},trajs{ti},0,0);
        fprintf('  %-10s %s |  %7.2f   | %6.2f  | %5.2f  |   %5.2f       |   %5.2f\n',...
                trajs{ti},icn{ii},r.pkv,r.tpk,r.gpk,r.occ25,r.occ14);
      end
    end
    fprintf('\n=== Liss IC3 SEED 4: cycle flow excursion vs funnel ===\n');
    r=run_one(ICs{3},"Lissajous",1,4);
    fprintf('  peak|Vhe_xy|=%.2f  maxOcc@p20=25=%.2f  (occ<1 => cycle hides INSIDE the funnel)\n',r.pkv,r.occ25);
end
function r=run_one(ic,traj,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE P20_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; P20_XY_OVERRIDE=[25;25]; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        n=idx; vhe=abs(V_h_e(1:2,1:n)); vmag=max(vhe,[],1);
        [pkv,ip]=max(vmag);
        gap=abs(X_DS(3,1:n)-x_t(3,1:n));
        % flow-funnel occupancy at the realized p_2 (p_20=25): occ = |Vhe|/p_2
        occ25=max(max(vhe./p_2(1:2,1:n)));
        % estimate occ if p_20 were 14: p_2 scales ~ (p20-p2inf)*exp(-g2 t)+p2inf; ratio
        p2_14 = (14-0.5)*( (p_2(1:2,1:n)-0.5)/(25-0.5) ) + 0.5;   % rescale envelope to p_20=14
        occ14 = max(max(vhe./p2_14));
        r=struct('pkv',pkv,'tpk',(ip-1)*0.01,'gpk',gap(ip),'occ25',occ25,'occ14',occ14);
    catch ME
        r=struct('pkv',NaN,'tpk',NaN,'gpk',NaN,'occ25',NaN,'occ14',NaN);
    end
end
