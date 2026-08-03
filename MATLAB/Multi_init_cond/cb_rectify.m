function cb_rectify()
% RECTIFY chi_r=[1.1,0.65] divergence: the high chi_r_x=1.1 overshoots without enough
% X-damping. Add it by tightening p_2inf_x (X flow funnel). Sweep p_2inf_x; watch per-axis
% residency (overshoot) + outcome. Must FIX Static IC5/Liss IC3 WITHOUT breaking Circ IC4/IC5.
% s_ddot-KEPT (tau=0.8), chi_r=[1.1,0.65], p_r_inf=1.0. p_2inf=[p2x, 0.5].
    p2xs=[0.5 0.3 0.2 0.15];
    cells={{[2;2;-3],'Static','S-IC5'},{[2;-2;-5],'Lissajous','L-IC3'},...
           {[2;2;-7],'Circular','C-IC4'},{[2;2;-3],'Circular','C-IC5'}};
    for pi=1:numel(p2xs)
      fprintf('== p_2inf_x=%.2f (chi_r=[1.1,0.65]) ==\n',p2xs(pi));
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},p2xs(pi));
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BR'; end
        fprintf('   %-7s land=%d xy=%.3f vel=%.3f resid[x%.2f y%.2f] %s%s\n',cells{k}{3},d.land,d.xy,d.vel,d.rrx,d.rry,tern(sp),br);
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,p2x)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.8; CHI_R_OVERRIDE=[1.1;0.65]; P2INF_XY_OVERRIDE=[p2x;0.5]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN,'rrx',NaN,'rry',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx); res=abs(rb)./prr;
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(res(:)); d.rrx=max(res(1,:)); d.rry=max(res(2,:));
    catch ME; d.land=0; end
end
