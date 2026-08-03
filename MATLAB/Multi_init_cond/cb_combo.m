function cb_combo()
% Combo search: chi_r (velocity, helps Liss IC3) x p_r_inf (position precision, helps
% Circ IC4) -> find the pair where BOTH opposed cells are SP, controls hold, no breach.
% Base: corrected + s_ddot-drop + p_2inf=0.5.
    chis=[0.5 0.7]; prinfs=[0.40 0.25];
    cells={{[2;-2;-5],'Lissajous','L-IC3(vel)'},{[2;2;-7],'Circular','C-IC4(prec)'},...
           {[0;0;-5],'Circular','C-IC2*'},{[2;2;-5],'Lissajous','L-IC2*'}};
    for ic=1:numel(chis)
      for ip=1:numel(prinfs)
        fprintf('== chi_r=%.1f p_r_inf=%.2f ==\n',chis(ic),prinfs(ip));
        for k=1:numel(cells)
          d=run_one(cells{k}{1},cells{k}{2},chis(ic),prinfs(ip));
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BREACH'; end
          fprintf('   %-11s land=%d xy=%.3f vel=%.3f resid=%.2f %s%s\n',cells{k}{3},d.land,d.xy,d.vel,d.resid,tern(sp),br);
        end
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,chi,prinf)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1;
    P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[chi;chi]; PRINF_OVERRIDE=[prinf;prinf];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
