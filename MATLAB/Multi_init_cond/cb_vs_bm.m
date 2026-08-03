function cb_vs_bm()
% Quantify the noise tradeoff: BACK-MAPPED form (default, no combined-barrier; the
% originally 50/50-validated PLASMC) vs the combined-barrier (16/24), same 4 weak
% cells x 6 noisy seeds. Is the combined-barrier a genuine noise regression?
    cells={{[2;-2;-5],'Sinusoidal','Sin-IC3'},{[2;-2;-5],'Lissajous','Liss-IC3'},...
           {[2;2;-3],'Static','S-IC5'},{[2;2;-3],'Circular','C-IC5'}};
    tsp=0; tn=0; wr=0;
    for k=1:numel(cells)
      sp=0; ld=0;
      for sd=1:6
        d=run_one(cells{k}{1},cells{k}{2},sd);
        sp=sp+(d.land&&d.xy<=0.08&&d.vel<=0.20); ld=ld+d.land; wr=max(wr,d.resid);
      end
      fprintf('   BACK-MAPPED %-8s SP=%d/6 land=%d/6\n',cells{k}{3},sp,ld); tsp=tsp+sp; tn=tn+6;
    end
    fprintf('   >> BACK-MAPPED TOTAL SP=%d/%d worstResidSEN=%.2f  (combined-barrier was 16/24)\n',tsp,tn,wr);
end
function d=run_one(ic,traj,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=[];  % back-mapped default
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel);
        sen=abs(V_s_e_n(:,1:idx))./max(p_s(:,1:idx),1e-6); d.resid=max(sen(:));
    catch ME; d.land=0; d.resid=NaN; end
end
