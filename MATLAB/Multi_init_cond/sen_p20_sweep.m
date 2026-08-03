function sen_p20_sweep()
% Test p_20 (velocity-funnel initial width, NOW unlocked) as the IC5 recovery
% lever. Tighter p_20 -> more aggressive middle-loop flow tracking; wider -> gentler.
% p_s_0 stays FoV-LOCKED. C_SIMPLE+K_rd2.5, Static+Circular IC5.
    p20=[10, 25, 40]; trajs={'Static','Circular'}; NS=5;
    for pi=1:numel(p20)
        for ti=1:2
            sp=0; fv=0;
            for sd=1:NS
                r=run_one(p20(pi),trajs{ti},sd); sp=sp+r.sp; fv=fv+r.fov;
            end
            fprintf('  >> p20=%g %-9s : SP=%d/%d fov=%d\n',p20(pi),trajs{ti},sp,NS,fv);
        end
    end
end
function r=run_one(p20,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE P20_XY_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE=traj;
    P20_XY_OVERRIDE=[p20;p20];
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
