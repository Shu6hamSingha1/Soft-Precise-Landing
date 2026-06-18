function recover_ic5()
% Try to recover the corrected-controller IC5 deficit with a PRINCIPLED lever
% (middle-loop reaching gain Gamma_xy = how aggressively the SMC closes the
% lateral error). C_SIMPLE+K_rd2.5 on Static IC5 (regression cell) + Circular
% IC5 (no-regression check), seeds 1-5. Gamma_xy in {0.5 base, 1.5, 3.0}.
    gxy={[],[1.5;1.5],[3.0;3.0]}; gnm={'Gxy0.5(base)','Gxy1.5','Gxy3.0'};
    trajs={'Static','Circular'}; NS=5;
    for gi=1:3
        for ti=1:2
            sp=0;
            for sd=1:NS
                r=run_one(gxy{gi},trajs{ti},sd); sp=sp+r.sp;
                fprintf('%-12s %-9s sd%d -> L%d fov%d SP%d xy%.2f v%.2f\n',gnm{gi},trajs{ti},sd,r.land,r.fov,r.sp,r.xy,r.vel);
            end
            fprintf('  >> %-12s %-9s : SP=%d/%d\n',gnm{gi},trajs{ti},sp,NS);
        end
    end
end
function r=run_one(gxy,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE GAMMA_XY_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE=traj; GAMMA_XY_OVERRIDE=gxy;
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
