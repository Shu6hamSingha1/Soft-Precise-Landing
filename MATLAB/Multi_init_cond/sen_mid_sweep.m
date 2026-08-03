function sen_mid_sweep()
% Final retune lever set for the C_SIMPLE IC5 deficit: middle-loop DELIVERY
% authority (boundary layer E_xy smaller=stiffer; initial adaptive gain kappa0_xy
% higher=more authority). p_20 LOCKED (untouched). C_SIMPLE+K_rd2.5, Static+Circular IC5.
    % cfg: {E_xy, kappa0_xy}
    cfgs={{[],[]},{[0.5;0.5],[]},{[],[0.5;0.5]},{[0.5;0.5],[0.5;0.5]}};
    cnm={'base','E0.5','k0=0.5','E0.5+k0.5'}; trajs={'Static','Circular'}; NS=5;
    for ci=1:numel(cfgs)
        for ti=1:2
            sp=0; fv=0;
            for sd=1:NS
                r=run_one(cfgs{ci},trajs{ti},sd); sp=sp+r.sp; fv=fv+r.fov;
            end
            fprintf('  >> %-10s %-9s : SP=%d/%d fov=%d\n',cnm{ci},trajs{ti},sp,NS,fv);
        end
    end
end
function r=run_one(cfg,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE E_XY_OVERRIDE KAPPA0_XY_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE=traj;
    E_XY_OVERRIDE=cfg{1}; KAPPA0_XY_OVERRIDE=cfg{2};
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
