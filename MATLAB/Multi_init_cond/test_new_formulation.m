function test_new_formulation()
% Test the NEW control formulation (corrected manuscript c-term C_SIMPLE + the
% K_rd=2.5 terminal damping) vs the OLD full-w c-term across the FULL multi-init
% suite: 5 trajectories x 5 ICs, noisy (matched seed=1, as the paper's protocol).
% Confirms no-regression beyond the Circular/IC5 case already validated.
    here  = fileparts(mfilename('fullpath'));
    trajs = {"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs   = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icnm={'IC1','IC2','IC3','IC4','IC5'};
    cfgs  = {{0,[]}, {1,[2.5;2.5]}}; cnm={'old-c', 'C_SIMPLE+Krd2.5'};
    seed  = 1;
    R = cell(2,5,5);
    for ci=1:2, for ti=1:5, for ii=1:5
        r=run_one(cfgs{ci}, trajs{ti}, ICs{ii}, seed); R{ci,ti,ii}=r;
        fprintf('%-15s %-11s %s -> L%d fov%d SP%d xy%.3f v%.3f\n', ...
            cnm{ci}, trajs{ti}, icnm{ii}, r.land, r.fov, r.sp, r.xy, r.vel);
    end, end, end
    fprintf('\n===== NEW FORMULATION test: old-c vs C_SIMPLE+Krd2.5 (noisy n=1, 5traj x 5IC) =====\n');
    for ti=1:5
        line=sprintf('%-11s', trajs{ti});
        for ci=1:2
            rr=squeeze(R(ci,ti,:)); sp=sum(cellfun(@(x)x.sp,rr)); ld=sum(cellfun(@(x)x.land,rr));
            line=[line sprintf(' | %-15s SP%d/L%d', cnm{ci}, sp, ld)];
        end
        fprintf('%s\n', line);
    end
    fprintf('-----------------------------------------------------------------\n');
    for ci=1:2
        sp=sum(cellfun(@(x)x.sp,R(ci,:,:)),'all'); ld=sum(cellfun(@(x)x.land,R(ci,:,:)),'all');
        fprintf('TOTAL %-15s: SP=%2d/25 land=%2d/25\n', cnm{ci}, sp, ld);
    end
    save(fullfile(here,'test_new_formulation.mat'),'R');
end
function r=run_one(cfg, traj, ic, seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed;
    C_SIMPLE=cfg{1}; SEN_RD_OVERRIDE=cfg{2}; TRAJ_OVERRIDE=traj;
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
