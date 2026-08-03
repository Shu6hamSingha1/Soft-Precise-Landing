function cb_filt_noisy()
% Does the Savitzky-Golay s_dot filter (CB_SDOT_FILT=W) improve NOISE robustness?
% Proof config + s_ddot-drop. Weakest noisy cells x 6 seeds, filter OFF vs W=9 vs W=13.
    Ws=[0 9 13];
    cells={{[2;-2;-5],'Sinusoidal','Sin-IC3'},{[2;-2;-5],'Lissajous','Liss-IC3'},...
           {[2;2;-3],'Static','S-IC5'},{[2;2;-3],'Circular','C-IC5'}};
    for wi=1:numel(Ws)
      fprintf('== CB_SDOT_FILT=%d ==\n',Ws(wi));
      tsp=0; tn=0; wr=0;
      for k=1:numel(cells)
        sp=0; ld=0;
        for sd=1:6
          d=run_one(cells{k}{1},cells{k}{2},sd,Ws(wi));
          sp=sp+(d.land&&d.xy<=0.08&&d.vel<=0.20); ld=ld+d.land; wr=max(wr,d.resid);
        end
        fprintf('   %-8s SP=%d/6 land=%d/6\n',cells{k}{3},sp,ld); tsp=tsp+sp; tn=tn+6;
      end
      fprintf('   >> W=%d TOTAL SP=%d/%d worstResid=%.2f\n',Ws(wi),tsp,tn,wr);
    end
end
function d=run_one(ic,traj,seed,W)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE CB_SDOT_FILT
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    if W>0; CB_SDOT_FILT=W; else; CB_SDOT_FILT=[]; end
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
