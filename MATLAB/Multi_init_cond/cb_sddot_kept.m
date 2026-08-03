function cb_sddot_kept()
% KEEP s_ddot (CB_DROP_SDDOT=0) and tame the 1/z-inflated terminal h_d_dot spike via
% the existing cap DH_D_CAP (default 20 = too loose) and/or LPF DH_D_TAU. Proof config
% (chi_r=0.85, p_2inf=0.5, p_r_inf=1.0). Noiseless; IC5 cells (diverged with s_ddot) + ctrls.
    % {cap, tau, label}
    cfgs={{20,0,'cap20(def)'},{10,0,'cap10'},{5,0,'cap5'},{3,0,'cap3'},{2,0,'cap2'},{5,0.05,'cap5+tau.05'}};
    cells={{[2;2;-3],'Static','S-IC5'},{[2;2;-3],'Circular','C-IC5'},...
           {[2;-2;-5],'Lissajous','L-IC3'},{[0;0;-5],'Circular','C-IC2*'}};
    for ci=1:numel(cfgs)
      fprintf('== s_ddot KEPT, %s ==\n',cfgs{ci}{3});
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},cfgs{ci}{1},cfgs{ci}{2});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BR'; end
        fprintf('   %-7s land=%d xy=%.3f vel=%.3f resid=%.2f %s%s\n',cells{k}{3},d.land,d.xy,d.vel,d.resid,tern(sp),br);
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,cap,tau)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE DH_D_CAP DH_D_TAU
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    DH_D_CAP=cap; DH_D_TAU=tau;
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
