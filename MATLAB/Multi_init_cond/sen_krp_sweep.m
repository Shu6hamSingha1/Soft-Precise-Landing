function sen_krp_sweep()
% Last outer-loop lever for the C_SIMPLE IC5 deficit: K_rp (SEN proportional gain
% = closing-demand strength). Higher K_rp -> faster EARLY closure before s_e_n hits
% the starving clamp (breaks the slow-closure->starvation positive feedback). Stay
% under the K_rp>=13.5 instability dead-end. C_SIMPLE+K_rd2.5, Static+Circular IC5.
    rps=[9, 11, 13]; trajs={'Static','Circular'}; NS=5;
    for pi=1:numel(rps)
        for ti=1:2
            sp=0; fv=0;
            for sd=1:NS
                r=run_one(rps(pi),trajs{ti},sd); sp=sp+r.sp; fv=fv+r.fov;
            end
            fprintf('  >> K_rp=%g %-9s : SP=%d/%d fov=%d\n',rps(pi),trajs{ti},sp,NS,fv);
        end
    end
end
function r=run_one(rp,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE SEN_RP_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE=traj;
    SEN_RP_OVERRIDE=[rp;rp];
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
