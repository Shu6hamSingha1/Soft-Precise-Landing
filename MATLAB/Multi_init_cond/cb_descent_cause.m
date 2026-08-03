function cb_descent_cause()
% Static IC5 seed 4 descent blow-up persists even with DROP -> NOT the s_ddot pump.
% Hypothesis: the aggressive LATERAL gains (kR=4, chi_r=0.65, p2inf_z=1.0) added to
% fight the lateral cycle are destabilizing the descent (geometric coupling: high kR
% -> large tilt -> robs vertical thrust). Isolate: from the 92-config, vary ONE gain
% back toward the mild baked values; report Static IC5 seed4 peak|vz| + land. keep s_ddot.
    cfgs={ {'92-config (kR4,chi.65,p2z1)', 4,0.65,1.0}, ...
           {'kR=3',  3,0.65,1.0}, {'kR=2', 2,0.65,1.0}, {'kR=1.5',1.5,0.65,1.0}, ...
           {'chi_r=0.85', 4,0.85,1.0}, ...
           {'p2inf_z=1.5', 4,0.65,1.5}, {'p2inf_z=2.5',4,0.65,2.5}, ...
           {'mild: kR1.5+chi.85+p2z1.5', 1.5,0.85,1.5} };
    fprintf('  config                         | s4 peak|vz| | s4 term vz | land\n');
    for c=cfgs
      cc=c{1}; lab=cc{1};
      r=run_one(cc{2},cc{3},cc{4},4);
      fprintf('  %-30s |   %6.3f    |  %+.3f    |  %d\n',lab,r.pk,r.tv,r.land);
    end
end
function r=run_one(kx,chi,p2z,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=p2z; CHI_R_OVERRIDE=[chi;chi];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[kx;kx;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        n=idx; vz=X_DS(10,1:n)-dx_t(3,1:n); vrel=I_v_c-dx_t(1:3,idx);
        r=struct('pk',max(abs(vz)),'tv',vrel(3),'land',double(landed));
    catch ME
        r=struct('pk',99,'tv',NaN,'land',0);
    end
end
