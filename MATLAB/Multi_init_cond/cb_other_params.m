function cb_other_params()
% Liss IC3 vy floors at ~0.19 under p_2inf. Try OTHER levers (tracking-bandwidth):
% reaching gain Gamma_xy, funnel contraction gamma_2, surface gain chi_r, boundary E.
% Base: combined-barrier + corrected + s_ddot-drop + p_2inf=0.5 + chi_r=1. Watch vy + resid(no breach).
    swp={ {'Gamma',[0.4375 1.0 2.0]}, {'gamma2',[0.2 0.5 1.0]}, {'chi_r',[0.5 1.0 2.0]}, {'E',[1.0 0.5 0.25]} };
    for k=1:numel(swp)
      nm=swp{k}{1}; vals=swp{k}{2};
      fprintf('== %s (Liss IC3 / Circ IC2 ctrl) ==\n',nm);
      for v=vals
        d=run_one([2;-2;-5],'Lissajous',nm,v); dc=run_one([0;0;-5],'Circular',nm,v);
        spc=dc.land&&dc.xy<=0.08&&dc.vel<=0.20;
        fprintf('   %s=%-6.3g L3: land=%d vel=%.3f vy=%+.3f resid=%.2f | C2: vel=%.3f resid=%.2f %s\n',...
            nm,v,d.land,d.vel,d.vy,d.resid,dc.vel,dc.resid,tern(spc));
      end
    end
end
function s=tern(c); if c; s='ctrlSP'; else; s='ctrl!!'; end; end
function d=run_one(ic,traj,nm,v)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE
    global GAMMA_XY_OVERRIDE GAMMA2_XY_OVERRIDE CHI_R_OVERRIDE E_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5];
    GAMMA_XY_OVERRIDE=[]; GAMMA2_XY_OVERRIDE=[]; CHI_R_OVERRIDE=[]; E_XY_OVERRIDE=[];
    switch nm
        case 'Gamma';  GAMMA_XY_OVERRIDE=[v;v];
        case 'gamma2'; GAMMA2_XY_OVERRIDE=[v;v];
        case 'chi_r';  CHI_R_OVERRIDE=[v;v];
        case 'E';      E_XY_OVERRIDE=[v;v];
    end
    d=struct('land',0,'vel',NaN,'vy',NaN,'resid',NaN,'xy',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.vel=norm(vrel); d.vy=vrel(2); d.resid=max(max(abs(rb)./prr)); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
