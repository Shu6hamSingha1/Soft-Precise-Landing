function test_static_ic5()
% Confirm the Static IC5 regression is robust (not seed-luck): old-c vs
% C_SIMPLE (at K_rd 1.4375 AND 2.5) on Static [2,2,-3] noisy, seeds 1-5.
    cfgs={{0,[]},{1,[]},{1,[2.5;2.5]}}; cnm={'old-c','C_SIMPLE Krd1.44','C_SIMPLE Krd2.5'};
    NS=5; R=cell(3,NS);
    for ci=1:3, for sd=1:NS
        r=run_one(cfgs{ci},sd); R{ci,sd}=r;
        fprintf('%-16s seed%d -> L%d fov%d SP%d xy%.3f v%.3f\n',cnm{ci},sd,r.land,r.fov,r.sp,r.xy,r.vel);
    end, end
    fprintf('\n===== Static IC5 [2,2,-3] noisy, seeds 1-5 =====\n');
    for ci=1:3
        rr=R(ci,:); sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr)); fv=sum(cellfun(@(x)x.fov,rr));
        fprintf('  %-16s: SP=%d/%d land=%d fov=%d\n',cnm{ci},sp,NS,ld,fv);
    end
end
function r=run_one(cfg,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=cfg{1}; SEN_RD_OVERRIDE=cfg{2}; TRAJ_OVERRIDE='Static';
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME
        sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message);
    end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
