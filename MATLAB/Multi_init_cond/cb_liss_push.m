function cb_liss_push()
% Push the last holdout Liss IC3 (vel 0.228 at p_2inf=0.5) with lower p_2inf_xy,
% verifying SP control cells don't regress. s_ddot-drop, corrected, chi_r=1.
    p2infs=[0.5, 0.35, 0.25];
    cases={{[2;-2;-5],'Lissajous','Liss-IC3'},{[2;-2;-5],'Sinusoidal','Sin-IC3*'},...
           {[0;0;-5],'Circular','C-IC2*'},{[2;2;-5],'Lissajous','Liss-IC2*'}};
    for ci=1:numel(cases)
      fprintf('-- %-9s --\n',cases{ci}{3});
      for pp=p2infs
        r=run_one(cases{ci}{1},cases{ci}{2},pp);
        sp=r.land&&r.xy<=0.08&&r.vel<=0.20;
        fprintf('   p2inf=%.2f -> land=%d xy=%.3f vel=%.3f vlat=%.3f %s\n',pp,r.land,r.xy,r.vel,r.vlat,tern(sp));
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
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
