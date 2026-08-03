function cb_smooth()
% SMOOTH DESCENT goal: which params remove/bound the Z cycle? Static IC5 (the descent
% diverger), all 4 noisy seeds. Metric: peak|vz| during descent (blow-up if >>0.2),
% terminal vz, std(vz) terminal (oscillation), landed. Compare:
%   base    = keep s_ddot 92-config (has the descent cycle/blow-up)
%   cap+P   = bound: KAPPA_MAX_Z=0.4 + P_z=20 (anti-runaway)
%   drop    = remove: CB_DROP_SDDOT=1 (removes the pump)
    cfgs={'base','cap+P','drop'};
    for ci=1:3
      cf=cfgs{ci};
      fprintf('=== %-6s (Static IC5) ===\n',cf);
      pkmax=0;
      for sd=1:4
        r=run_one(cf,sd);
        pkmax=max(pkmax,r.pk);
        fprintf('  seed %d: peak|vz|=%6.3f  term vz=%+.3f  std(vz)=%.3f  land=%d\n',sd,r.pk,r.tv,r.sv,r.land);
      end
      fprintf('  >> %s worst peak|vz| over 4 seeds = %.3f\n\n',cf,pkmax);
    end
end
function r=run_one(cf,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE ...
           KAPPA_MAX_Z P_Z_OVERRIDE KAPPA0_Z_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    CB_DROP_SDDOT=0; KAPPA_MAX_Z=[]; P_Z_OVERRIDE=[]; KAPPA0_Z_OVERRIDE=[];
    switch cf
      case 'cap+P'; KAPPA_MAX_Z=0.4; P_Z_OVERRIDE=20;
      case 'drop';  CB_DROP_SDDOT=1;
    end
    try
        visualControl_IBVS_adaptive;
        n=idx; w=min(400,n-5); k=(n-w):n;
        vz=X_DS(10,1:n)-dx_t(3,1:n);
        vrel=I_v_c-dx_t(1:3,idx);
        r=struct('pk',max(abs(vz)),'tv',vrel(3),'sv',std(vz(k)),'land',double(landed));
    catch ME
        r=struct('pk',99,'tv',NaN,'sv',NaN,'land',0);
    end
end
