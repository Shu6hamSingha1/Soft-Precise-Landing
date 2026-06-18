function csimple_retune()
% C_SIMPLE terminal retune: sweep SEN-funnel lateral damping K_rd to kill the
% terminal lateral-velocity overshoot on the marginal IC5 seeds (10/11/12),
% with easy seeds (1,5) as a no-regression check. IC5 [2,2,-3] noisy, matched seeds.
    here  = fileparts(mfilename('fullpath'));
    rds   = [1.4375, 2.5, 4.0, 6.0];           % K_rd values (base = 1.4375)
    seeds = [1 5 10 11 12];                     % 2 easy (no-reg) + 3 marginal
    R = nan(numel(rds), numel(seeds), 3);       % [SP, vel, t]
    for ri = 1:numel(rds)
        for si = 1:numel(seeds)
            r = run_one(rds(ri), seeds(si));
            R(ri,si,:) = [r.sp, r.vel, r.t];
            fprintf('K_rd=%.3f seed=%2d -> SP=%d vel=%.3f xy=%.3f t=%.1f\n', ...
                    rds(ri), seeds(si), r.sp, r.vel, r.xy, r.t);
        end
    end
    fprintf('\n===== C_SIMPLE K_rd sweep (IC5 noisy, seeds %s) =====\n', mat2str(seeds));
    for ri = 1:numel(rds)
        sp = sum(R(ri,:,1));
        vs = sprintf('%.2f ', R(ri,:,2));
        fprintf('  K_rd=%.4f : SP=%d/%d  vel=[%s]\n', rds(ri), sp, numel(seeds), vs);
    end
    save(fullfile(here,'csimple_retune_R.mat'),'R','rds','seeds');
end

function r = run_one(rd, seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1;
    SEN_RD_OVERRIDE=[rd;rd];
    sp=0; vel=NaN; tt=NaN; xy=NaN;
    try
        visualControl_IBVS_adaptive;           % runs in this ws; clears it; leaves results
        xy = norm(I_p_c(1:2)-x_t(1:2,idx)); vel = norm(I_v_c-dx_t(1:3,idx));
        sp = double(landed && xy<=0.08 && vel<=0.2); tt = tRange(idx);
    catch ME
        sp=0; vel=NaN; tt=NaN; xy=NaN;
        fprintf('  ERR %s\n', ME.message);
    end
    r = struct('sp',sp,'vel',vel,'t',tt,'xy',xy);
end
