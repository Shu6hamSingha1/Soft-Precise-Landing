function cb_zdamp()
% WHICH gain reduces the Z (descent) limit cycle? On Static IC5 seed 4 (the descent
% DIVERGENCE, vz->2.19), vary ONE descent-axis gain at a time, metric = netDamp
% corr(az_actual, vz) (more NEGATIVE=more damped) + terminal vz + peak vz.
% Baseline (92/100 config): chi_r=0.65, kR=[4;4;0.5], p2inf_z=1.0, keep s_ddot.
% Z gains: chi_z(=Omega_z=0.025), Gamma_z=0.75, E_z=1.0, gamma2_z=0.2, kappa0_z=0.25,
%          p2inf_z(G2 gain), p_20_z=4.
    rows={ {'baseline',     '',0}, ...
           {'chi_z=0.05',   'CZ',0.05}, {'chi_z=0.10','CZ',0.10}, {'chi_z=0.012','CZ',0.012}, ...
           {'Gamma_z=1.5',  'GZ',1.5},  {'Gamma_z=3.0','GZ',3.0}, ...
           {'E_z=0.5',      'EZ',0.5},  {'E_z=0.3','EZ',0.3}, ...
           {'gamma2_z=0.4', 'G2',0.4},  {'gamma2_z=0.1','G2',0.1}, ...
           {'kappa0_z=0.5', 'K0',0.5}, ...
           {'p2inf_z=0.7',  'PZ',0.7},  {'p2inf_z=1.5','PZ',1.5}, ...
           {'p20_z=2',      'P0',2} };
    fprintf('  param        | netDamp(az,vz) | peak vz | term vz | land\n');
    for r=rows
      rr=r{1}; fprintf('ROW %s\n',rr{1});
      try; probe(rr{2},rr{3}); catch; fprintf('DATA DIVERGED\n'); end
    end
end
function probe(pname,pval)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE P20_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE ...
           GAMMA_Z_OVERRIDE E_Z_OVERRIDE CHIZ_OVERRIDE GAMMA2_Z_OVERRIDE KAPPA0_Z_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    GAMMA_Z_OVERRIDE=[]; E_Z_OVERRIDE=[]; CHIZ_OVERRIDE=[]; GAMMA2_Z_OVERRIDE=[]; KAPPA0_Z_OVERRIDE=[]; P20_Z_OVERRIDE=[];
    switch pname
      case 'CZ'; CHIZ_OVERRIDE=pval;     case 'GZ'; GAMMA_Z_OVERRIDE=pval;
      case 'EZ'; E_Z_OVERRIDE=pval;      case 'G2'; GAMMA2_Z_OVERRIDE=pval;
      case 'K0'; KAPPA0_Z_OVERRIDE=pval; case 'PZ'; P2INF_Z_OVERRIDE=pval;
      case 'P0'; P20_Z_OVERRIDE=pval;
    end
    visualControl_IBVS_adaptive;
    n=idx; w=min(400,n-5); k=(n-w):n;
    vz=(X_DS(10,k)-dx_t(3,k))'; az=gradient(vz,0.01);
    a=az-mean(az); b=vz-mean(vz); dd=sqrt(sum(a.^2)*sum(b.^2)); cc=0; if dd>0; cc=sum(a.*b)/dd; end
    vrel=I_v_c-dx_t(1:3,idx);
    fprintf('DATA netDamp=%+.2f peakVz=%.3f termVz=%.3f land=%d\n', cc, max(abs(vz)), vrel(3), double(landed));
end
