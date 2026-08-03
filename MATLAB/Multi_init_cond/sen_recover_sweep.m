function sen_recover_sweep()
% Test the §9 SEN hard-containment recovery on the corrected controller's IC5
% deficit. C_SIMPLE + K_rd=2.5, sweep SEN_RECOVER_ST (strong past-breach recovery:
% smaller ST = stronger -rp restoring) on the regression cell (Static IC5) + a
% no-regression check (Circular IC5). seeds 1-5, matched.
    sts = [0, 0.3, 0.5, 0.7]; trajs={'Static','Circular'}; NS=5;
    for si=1:numel(sts)
        for ti=1:2
            sp=0; fv=0;
            for sd=1:NS
                r=run_one(sts(si),trajs{ti},sd); sp=sp+r.sp; fv=fv+r.fov;
                fprintf('ST=%.1f %-9s sd%d -> L%d fov%d SP%d xy%.2f v%.2f\n',sts(si),trajs{ti},sd,r.land,r.fov,r.sp,r.xy,r.vel);
            end
            fprintf('  >> ST=%.1f %-9s : SP=%d/%d fov=%d\n',sts(si),trajs{ti},sp,NS,fv);
        end
    end
end
function r=run_one(st,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE SEN_RECOVER_ST
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE=traj;
    if st>0; SEN_RECOVER_ST=st; else; SEN_RECOVER_ST=[]; end
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
