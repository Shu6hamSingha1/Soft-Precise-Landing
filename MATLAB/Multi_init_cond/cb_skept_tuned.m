function cb_skept_tuned()
% Apply the FULL tuning (p_2inf=0.5, chi_r=0.85, p_r_inf=1.0) to the s_ddot-KEPT
% variants and find the right s_ddot-filter param. Drop baseline vs tau-LPF vs SG-clean.
% Problematic cells (IC5 diverged w/ cap; IC3 regressed w/ early tau). Noiseless.
    % {label, drop, tau, sgW}
    cfgs={{'drop(base)',1,0,0},{'tau0.5',0,0.5,0},{'tau0.3',0,0.3,0},{'tau0.8',0,0.8,0},{'SGclean9',0,0,9},{'SGclean13',0,0,13}};
    cells={{[2;2;-3],'Static','S-IC5'},{[2;2;-3],'Circular','C-IC5'},{[2;-2;-5],'Lissajous','L-IC3'},...
           {[0;0;-5],'Static','S-IC3? no'},{[2;-2;-5],'Static','St-IC3'},{[2;-2;-5],'Linear','Ln-IC3'}};
    cells{4}={[2;-2;-5],'Sinusoidal','Sn-IC3'};
    for ci=1:numel(cfgs)
      sp=0;
      line=sprintf('%-10s',cfgs{ci}{1});
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},cfgs{ci}{2},cfgs{ci}{3},cfgs{ci}{4});
        ok=d.land&&d.xy<=0.08&&d.vel<=0.20; sp=sp+ok;
        tag='..'; if ~d.land; tag='XX'; elseif ok; tag='SP'; elseif d.xy<=0.08; tag='Pr'; elseif d.vel<=0.20; tag='So'; end
        line=[line sprintf(' %s:%s',cells{k}{3},tag)];
      end
      fprintf('%s  [%d/6]\n',line,sp);
    end
end
function d=run_one(ic,traj,drop,tau,sgW)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU CB_SDOT_FILT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=drop; if tau>0; CB_SDDOT_TAU=tau; else; CB_SDDOT_TAU=[]; end
    if sgW>0; CB_SDOT_FILT=sgW; else; CB_SDOT_FILT=[]; end
    P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel);
    catch ME; d.land=0; end
end
