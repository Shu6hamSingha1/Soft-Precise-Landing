function cb_vh3()
% DEFINITIVE: is the loom flow V_h(3) actually consumed by the descent loop?
% Override ONLY V_h(3): normal (pinv) vs analytical-clean vs ZERO, leaving lateral
% V_h(1:2)+V_w untouched. If Static IC5 seed4 (and others) are INVARIANT to V_h(3),
% the descent does NOT use it (the PINV_TOL effect came via lateral/rotation).
% keep s_ddot 92-config.
    cells={{"Static",[2;2;-3],'Sta-IC5'},{"Lissajous",[2;2;-7],'Lis-IC4'},{"Circular",[2;2;-3],'Cir-IC5'}};
    modes={[],1,2}; mn={'normal(pinv)','analytic h_z','ZERO h_z'};
    fprintf('  cell    |   mode        | term|v| | termVz | land | SP\n');
    for c=cells
      cc=c{1};
      for mi=1:3
        r=run_one(cc{2},cc{1},modes{mi},4);  % seed 4
        fprintf('  %-7s | %-13s | %6.3f | %+.3f |  %d   | %d\n',cc{3},mn{mi},r.tv,r.vz,r.land,r.sp);
      end
    end
end
function r=run_one(ic,traj,vh3,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE VH3_MODE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1]; VH3_MODE=vh3;
    try
        visualControl_IBVS_adaptive; vrel=I_v_c-dx_t(1:3,idx);
        r=struct('tv',norm(vrel),'vz',vrel(3),'land',double(landed),'sp',double(landed&&norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08&&norm(vrel)<=0.20));
    catch ME; r=struct('tv',99,'vz',99,'land',0,'sp',0); end
end
