function sen_psw_sweep()
% Retune the SEN funnel WIDTH to cure the C_SIMPLE IC5 demand-starvation (s_e_n
% pins at the 0.95 clamp pre-breach -> G_s^-1 -> 0 -> weak demand -> stall). Wider
% p_s_0 keeps S_s below the clamp so the corrected c-term's clean tracking follows a
% strong demand. C_SIMPLE+K_rd2.5, Static IC5 (regression) + Circular IC5 (no-reg).
    ps0=[1.2, 2.0, 3.0]; trajs={'Static','Circular'}; NS=5;
    for pi=1:numel(ps0)
        for ti=1:2
            sp=0; fv=0;
            for sd=1:NS
                r=run_one(ps0(pi),trajs{ti},sd); sp=sp+r.sp; fv=fv+r.fov;
                fprintf('p_s0=%.1f %-9s sd%d -> L%d fov%d SP%d xy%.2f v%.2f\n',ps0(pi),trajs{ti},sd,r.land,r.fov,r.sp,r.xy,r.vel);
            end
            fprintf('  >> p_s0=%.1f %-9s : SP=%d/%d fov=%d\n',ps0(pi),trajs{ti},sp,NS,fv);
        end
    end
end
function r=run_one(ps0,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE SEN_PS0_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE=traj;
    SEN_PS0_OVERRIDE=[ps0;ps0];
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
