function cb_2d_sweep()
% Combined-barrier: 2D co-tune chi_r x p_h to find a stabilizing combo (or prove
% none exists -> structural). Circular IC2 noiseless, old c-h.
    chis=[0.1,0.3]; phs=[2.0,4.0];
    for ci=1:numel(chis)
      for pj=1:numel(phs)
        r=run_one(chis(ci),phs(pj));
        fprintf('  chi_r=%.2f p_h=%.1f -> land=%d fov=%d t=%.2f xy=%.3f vel=%.3f\n',chis(ci),phs(pj),r.land,r.fov,r.t,r.xy,r.vel);
      end
    end
end
function r=run_one(chi,ph)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CHI_R_OVERRIDE P20_XY_OVERRIDE
    IC_OVERRIDE=[2;2;-5]; NOISE_OVERRIDE=0; TRAJ_OVERRIDE='Circular'; COMBINED_BARRIER=1; C_SIMPLE=0; CHI_R_OVERRIDE=[chi;chi]; P20_XY_OVERRIDE=[ph;ph];
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;tt=NaN;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx)); ld=double(landed); fov=double(fov_fail); tt=tRange(idx);
    catch ME; fprintf('  ERR %s\n',ME.message); end
    r=struct('land',ld,'fov',fov,'t',tt,'xy',xy,'vel',vel);
end
