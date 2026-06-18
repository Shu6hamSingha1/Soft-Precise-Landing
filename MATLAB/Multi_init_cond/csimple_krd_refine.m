function csimple_krd_refine()
% C_SIMPLE: refine K_rd in {2.8, 3.0} across all 12 IC5 [2,2,-3] noisy seeds,
% to recover the marginal misses (seeds 7,8 at v0.21/0.26 under K_rd=2.5=9/12)
% without over-damping (K_rd=4.0 broke easy seeds). Matched seeds.
    here=fileparts(mfilename('fullpath'));
    rds=[2.8, 3.0]; NS=12; R=cell(numel(rds),NS);
    for ri=1:numel(rds)
        for sd=1:NS
            r=run_one(rds(ri),sd); R{ri,sd}=r;
            fprintf('K_rd=%.1f seed%2d -> L%d fov%d SP%d xy%.3f v%.3f\n', ...
                rds(ri),sd,r.land,r.fov,r.sp,r.xy,r.vel);
        end
    end
    fprintf('\n===== C_SIMPLE K_rd refine (IC5 12-seed) [ref: K_rd=2.5 -> SP9/12 fail 6,7,8] =====\n');
    for ri=1:numel(rds)
        rr=R(ri,:); sp=sum(cellfun(@(x)x.sp,rr)); v=cellfun(@(x)x.vel,rr);
        bad=find(cellfun(@(x)~x.sp,rr));
        fprintf('  K_rd=%.1f: SP=%2d/%d vel(mean=%.2f max=%.2f) non-SP: %s\n', ...
            rds(ri),sp,NS,mean(v,'omitnan'),max(v),mat2str(bad));
    end
    save(fullfile(here,'csimple_krd_refine.mat'),'R','rds');
end
function r=run_one(rd,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[rd;rd];
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
