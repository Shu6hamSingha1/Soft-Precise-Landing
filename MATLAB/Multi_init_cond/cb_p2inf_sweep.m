function cb_p2inf_sweep()
% Analyze the flow-funnel terminal floor p_2inf_xy as the chase-lag (terminal lateral
% velocity) lever. Lower p_2inf -> tighter terminal flow error bound -> less residual
% lateral velocity. s_ddot-drop + corrected c-h + chi_r=1. Watch vlat; controls = SP cells.
    p2infs=[2.5, 1.5, 1.0, 0.5];
    cases={{[2;2;-3],'Static','S-IC5'},{[2;2;-3],'Circular','C-IC5'},{[2;-2;-5],'Sinusoidal','Sin-IC3'},...
           {[2;-2;-5],'Lissajous','Liss-IC3'},{[2;2;-3],'Linear','Lin-IC5*'},{[0;0;-5],'Circular','C-IC2*'}};
    for ci=1:numel(cases)
      fprintf('-- %-8s --\n',cases{ci}{3});
      for pp=p2infs
        r=run_one(cases{ci}{1},cases{ci}{2},pp);
        sp=r.land&&r.xy<=0.08&&r.vel<=0.20;
        fprintf('   p2inf=%.1f -> land=%d xy=%.3f vel=%.3f vlat=%.3f %s\n',pp,r.land,r.xy,r.vel,r.vlat,ternary(sp,'<<SP',''));
      end
    end
end
function s=ternary(c,a,b); if c; s=a; else; s=b; end; end
function r=run_one(ic,traj,p2inf)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[p2inf;p2inf];
    xy=NaN;vel=NaN;ld=0;vlat=NaN;
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(vrel); vlat=norm(vrel(1:2)); ld=double(landed);
    catch ME; fprintf('   ERR %s\n',ME.message); end
    r=struct('land',ld,'xy',xy,'vel',vel,'vlat',vlat);
end
