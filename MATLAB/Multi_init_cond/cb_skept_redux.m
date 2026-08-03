function cb_skept_redux()
% KEEP s_ddot (user directive), REDUCE chi_r_x. Test if a gentler position barrier
% stops the full-s_ddot over-drive that diverged at chi_r>=0.85. Full s_ddot (no filter)
% vs tau0.8, at reduced symmetric chi_r. p_2inf=0.5, p_r_inf=1.0. IC5/IC3 cells + ctrl.
    % {label, drop, tau}
    modes={{'full',0,0},{'tau0.8',0,0.8}};
    chis=[0.8 0.6 0.4];
    cells={{[2;2;-3],'Static','S-IC5'},{[2;2;-3],'Circular','C-IC5'},{[2;-2;-5],'Lissajous','L-IC3'},{[0;0;-5],'Circular','C-IC2*'}};
    for mi=1:numel(modes)
      for chi=chis
        fprintf('== %s, chi_r=%.1f ==\n',modes{mi}{1},chi);
        for k=1:numel(cells)
          d=run_one(cells{k}{1},cells{k}{2},modes{mi}{2},modes{mi}{3},chi);
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BR'; end
          fprintf('   %-7s land=%d xy=%.3f vel=%.3f resid=%.2f %s%s\n',cells{k}{3},d.land,d.xy,d.vel,d.resid,tern(sp),br);
        end
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,drop,tau,chi)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=drop;
    if tau>0; CB_SDDOT_TAU=tau; else; CB_SDDOT_TAU=[]; end
    P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[chi;chi]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
