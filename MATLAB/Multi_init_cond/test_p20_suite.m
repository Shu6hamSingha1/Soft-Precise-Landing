function test_p20_suite()
% Validate the p_20=10 IC5 recovery across the FULL suite (5 traj x 5 IC, noisy
% n=1), C_SIMPLE+K_rd2.5+p20=10 vs old-c. Compare to MT7 (old-c 24/25, C_SIMPLE
% +K_rd2.5 only 23/25). Does tightening p_20 recover IC5 WITHOUT breaking others?
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icnm={'IC1','IC2','IC3','IC4','IC5'};
    cfgs={{0,[],[]},{1,[2.5;2.5],[10;10]}}; cnm={'old-c','C_SIMPLE+Krd2.5+p20=10'};
    seed=1; R=cell(2,5,5);
    for ci=1:2, for ti=1:5, for ii=1:5
        r=run_one(cfgs{ci},trajs{ti},ICs{ii},seed); R{ci,ti,ii}=r;
    end, end, end
    fprintf('\n===== p20=10 SUITE: old-c vs C_SIMPLE+Krd2.5+p20=10 (noisy n=1) =====\n');
    for ti=1:5
        line=sprintf('%-11s',trajs{ti});
        for ci=1:2
            rr=squeeze(R(ci,ti,:)); sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
            line=[line sprintf(' | %s SP%d/L%d',cnm{ci},sp,ld)];
        end
        fprintf('%s\n',line);
    end
    for ci=1:2
        sp=sum(cellfun(@(x)x.sp,R(ci,:,:)),'all'); ld=sum(cellfun(@(x)x.land,R(ci,:,:)),'all');
        fprintf('  >> TOTAL %-24s SP=%d/25 land=%d/25\n',cnm{ci},sp,ld);
    end
end
function r=run_one(cfg,traj,ic,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE P20_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=cfg{1}; SEN_RD_OVERRIDE=cfg{2}; TRAJ_OVERRIDE=traj; P20_XY_OVERRIDE=cfg{3};
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME; sp=0;vel=NaN;xy=NaN;ld=0;fov=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
