function csimple_final()
% Full 12-seed IC5 [2,2,-3] noisy: old-c base (K_rd=1.4375) vs the option-(b)
% corrected controller C_SIMPLE + K_rd=2.5. Confirms the broader IC5 picture
% (incl. the seed-8 terminal/CBF wall) for the results table.
    here=fileparts(mfilename('fullpath'));
    cfgs={{0,[]},{1,[2.5;2.5]}}; cnm={'old-c base','C_SIMPLE+Krd2.5'};
    NS=12; R=cell(2,NS);
    for ci=1:2
        for sd=1:NS
            r=run_one(cfgs{ci},sd); R{ci,sd}=r;
            fprintf('%-15s seed%2d -> L%d fov%d SP%d xy%.3f v%.3f t%.1f\n', ...
                cnm{ci},sd,r.land,r.fov,r.sp,r.xy,r.vel,r.t);
        end
    end
    fprintf('\n===== IC5 noisy 12-seed: old-c base vs C_SIMPLE+Krd2.5 =====\n');
    for ci=1:2
        rr=R(ci,:); sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
        fv=sum(cellfun(@(x)x.fov,rr)); v=cellfun(@(x)x.vel,rr);
        bad=find(cellfun(@(x)~x.sp,rr));
        fprintf('  %-15s: SP=%2d/%d land=%2d fov=%d vel(mean=%.2f max=%.2f) | non-SP seeds: %s\n', ...
            cnm{ci},sp,NS,ld,fv,mean(v,'omitnan'),max(v),mat2str(bad));
    end
    save(fullfile(here,'csimple_final.mat'),'R');
end
function r=run_one(cfg,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=cfg{1}; SEN_RD_OVERRIDE=cfg{2};
    sp=0;vel=NaN;xy=NaN;ld=0;fov=0;tt=NaN;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed); fov=double(fov_fail); tt=tRange(idx);
    catch ME
        sp=0;vel=NaN;xy=NaN;ld=0;fov=0;tt=NaN; fprintf('  ERR %s\n',ME.message);
    end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov,'t',tt);
end
