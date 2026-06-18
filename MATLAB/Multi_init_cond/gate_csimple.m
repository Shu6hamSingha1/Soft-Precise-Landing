function gate_csimple()
% IC1-5 no-regression gate: current baked controller (old c-term, K_rd=1.4375)
% vs the option-(b) corrected controller (C_SIMPLE manuscript c-term + K_rd=2.5).
% Everything else baked (GAMMA_COG=0.005 etc). noiseless n=1 + noisy n=5 per IC.
    here = fileparts(mfilename('fullpath'));
    ICs = {[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    nm  = {'IC1','IC2','IC3','IC4','IC5'};
    cfgs = {{0,[]}, {1,[2.5;2.5]}};                 % {C_SIMPLE, SEN_RD_OVERRIDE}
    cnm  = {'old-c base', 'C_SIMPLE+Krd2.5'};
    jobs = {};
    for ic=1:5, for ci=1:2
        jobs{end+1}=[ic ci 0 1];                    % noiseless n=1
        for sd=1:5; jobs{end+1}=[ic ci 1 sd]; end   % noisy n=5
    end, end
    R = cell(1,numel(jobs));
    for j=1:numel(jobs)
        jb=jobs{j}; r=run_one(ICs{jb(1)}, cfgs{jb(2)}, jb(3), jb(4));
        r.ic=jb(1); r.ci=jb(2); r.noise=jb(3); R{j}=r;
        fprintf('%s %-15s noise=%d sd=%d -> L%d fov%d SP%d xy%.3f v%.3f\n', ...
            nm{jb(1)}, cnm{jb(2)}, jb(3), jb(4), r.land, r.fov, r.sp, r.xy, r.vel);
    end
    fprintf('\n===== IC1-5 GATE: old-c base vs C_SIMPLE+Krd2.5 (nl n=1 + ny n=5) =====\n');
    for ic=1:5
        line=sprintf('%-4s', nm{ic});
        for ci=1:2
            rr=R(cellfun(@(x)x.ic==ic&&x.ci==ci,R));
            n0=rr(cellfun(@(x)x.noise==0,rr)); n1=rr(cellfun(@(x)x.noise==1,rr));
            line=[line sprintf(' | %-15s nlSP%d/L%d nySP%d/L%d', cnm{ci}, ...
                sum(cellfun(@(x)x.sp,n0)),sum(cellfun(@(x)x.land,n0)), ...
                sum(cellfun(@(x)x.sp,n1)),sum(cellfun(@(x)x.land,n1)))];
        end
        fprintf('%s\n', line);
    end
    fprintf('-----------------------------------------------------------------\n');
    for ci=1:2
        sp=sum(cellfun(@(x)x.sp&&x.ci==ci,R)); ld=sum(cellfun(@(x)x.land&&x.ci==ci,R));
        fprintf('TOTAL %-15s: SP=%2d land=%2d (of 30)\n', cnm{ci}, sp, ld);
    end
    % per-IC no-regression check (C_SIMPLE vs base)
    fprintf('-----------------------------------------------------------------\n');
    ok=true;
    for ic=1:5
        b=R(cellfun(@(x)x.ic==ic&&x.ci==1,R)); c=R(cellfun(@(x)x.ic==ic&&x.ci==2,R));
        if sum(cellfun(@(x)x.sp,c))<sum(cellfun(@(x)x.sp,b)) || sum(cellfun(@(x)x.land,c))<sum(cellfun(@(x)x.land,b))
            ok=false; fprintf('  REGRESSION at %s\n', nm{ic});
        end
    end
    fprintf('  C_SIMPLE+Krd2.5 no-regression on all 5 ICs: %s\n', string(ok));
    save(fullfile(here,'gate_csimple.mat'),'R');
end

function r = run_one(ic, cfg, noise, seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed;
    C_SIMPLE=cfg{1}; SEN_RD_OVERRIDE=cfg{2};
    sp=0; vel=NaN; xy=NaN; ld=0; fov=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed && xy<=0.08 && vel<=0.2); ld=double(landed); fov=double(fov_fail);
    catch ME
        sp=0; vel=NaN; xy=NaN; ld=0; fov=0; fprintf('  ERR %s\n', ME.message);
    end
    r=struct('sp',sp,'vel',vel,'xy',xy,'land',ld,'fov',fov);
end
