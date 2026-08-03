function cb_zclass()
% WHY each Z gain fails: classify by mechanism on a BOUNDED descent cycle (noiseless
% Static IC5). Metrics: netDamp corr(az,vz); authority = std(I_a_cd(3)) (vertical accel
% command effort); freq = vz zero-crossing rate (stiffness signature, Hz). Class A
% (authority/gain knob) moves authority; Class B (stiffness/lag) moves freq.
    rows={ {'baseline',    '',0}, ...
           {'Gamma_z=3.0', 'GZ',3.0}, ...
           {'chi_z=0.10',  'CZ',0.10}, {'chi_z=0.012','CZ',0.012}, ...
           {'E_z=0.3',     'EZ',0.3}, ...
           {'gamma2_z=0.1','G2',0.1}, ...
           {'kappa0_z=0.5','K0',0.5}, ...
           {'p20_z=2',     'P0',2} };
    fprintf('  param        | netDamp | authority(std a_z) | vz freq(Hz)\n');
    for r=rows
      rr=r{1}; fprintf('ROW %s\n',rr{1});
      try; probe(rr{2},rr{3}); catch; fprintf('DATA DIVERGED\n'); end
    end
end
function probe(pname,pval)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE P20_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE ...
           GAMMA_Z_OVERRIDE E_Z_OVERRIDE CHIZ_OVERRIDE GAMMA2_Z_OVERRIDE KAPPA0_Z_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=0; RNG_SEED_OVERRIDE=0; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    GAMMA_Z_OVERRIDE=[]; E_Z_OVERRIDE=[]; CHIZ_OVERRIDE=[]; GAMMA2_Z_OVERRIDE=[]; KAPPA0_Z_OVERRIDE=[]; P20_Z_OVERRIDE=[];
    switch pname
      case 'GZ'; GAMMA_Z_OVERRIDE=pval;  case 'CZ'; CHIZ_OVERRIDE=pval;
      case 'EZ'; E_Z_OVERRIDE=pval;      case 'G2'; GAMMA2_Z_OVERRIDE=pval;
      case 'K0'; KAPPA0_Z_OVERRIDE=pval; case 'P0'; P20_Z_OVERRIDE=pval;
    end
    visualControl_IBVS_adaptive;
    n=idx; w=min(500,n-5); k=(n-w):n;
    vz=(X_DS(10,k)-dx_t(3,k))'; az=gradient(vz,0.01);
    a=az-mean(az); b=vz-mean(vz); dd=sqrt(sum(a.^2)*sum(b.^2)); cc=0; if dd>0; cc=sum(a.*b)/dd; end
    auth=std(I_a_cd(3,k));
    sd=vz-mean(vz); zc=sum(sd(1:end-1).*sd(2:end)<0); freq=zc/(2*w*0.01);
    fprintf('DATA netDamp=%+.2f authority=%.3f freq=%.2f\n', cc, auth, freq);
end
