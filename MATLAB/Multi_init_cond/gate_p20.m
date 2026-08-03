function gate_p20()
% Confirm the IC1-5 Circular gate still passes with the recovered config
% C_SIMPLE+K_rd2.5+p20=10 (p_20 is a gain change). noiseless n=1 + noisy n=5.
% Compare to known: old-c 29/30, C_SIMPLE+K_rd2.5 (no p20) 30/30 (MT4).
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; nm={'IC1','IC2','IC3','IC4','IC5'};
    jobs={}; for ic=1:5; jobs{end+1}=[ic 0 1]; for sd=1:5; jobs{end+1}=[ic 1 sd]; end; end
    R=cell(1,numel(jobs));
    for j=1:numel(jobs); jb=jobs{j}; r=run_one(ICs{jb(1)},jb(2),jb(3)); r.ic=jb(1); r.noise=jb(2); R{j}=r; end
    fprintf('\n===== IC1-5 Circular gate: C_SIMPLE+K_rd2.5+p20=10 (nl1+ny5) =====\n');
    tsp=0; tld=0;
    for ic=1:5
        rr=R(cellfun(@(x)x.ic==ic,R)); n0=rr(cellfun(@(x)x.noise==0,rr)); n1=rr(cellfun(@(x)x.noise==1,rr));
        sp0=sum(cellfun(@(x)x.sp,n0)); sp1=sum(cellfun(@(x)x.sp,n1)); ld1=sum(cellfun(@(x)x.land,n1));
        fprintf('  %s: nl SP%d/1  ny SP%d/L%d/5\n',nm{ic},sp0,sp1,ld1);
        tsp=tsp+sp0+sp1; tld=tld+sum(cellfun(@(x)x.land,rr));
    end
    fprintf('  >> TOTAL SP=%d/30 land=%d/30  (cf old-c 29, C_SIMPLE+Krd2.5 30)\n',tsp,tld);
end
function r=run_one(ic,noise,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE C_SIMPLE SEN_RD_OVERRIDE TRAJ_OVERRIDE P20_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noise; RNG_SEED_OVERRIDE=seed; C_SIMPLE=1; SEN_RD_OVERRIDE=[2.5;2.5]; TRAJ_OVERRIDE='Circular'; P20_XY_OVERRIDE=[10;10];
    sp=0;ld=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx));
        sp=double(landed&&xy<=0.08&&vel<=0.2); ld=double(landed);
    catch ME; sp=0;ld=0; fprintf('  ERR %s\n',ME.message); end
    r=struct('sp',sp,'land',ld);
end
