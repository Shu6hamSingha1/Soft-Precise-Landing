function cb_proof_guided()
% PROOF-GUIDED (COMBINED_SURFACE_PROOF_ADDENDUM manifold |zeta_r|~|zeta_h|/chi_r):
% tighten p_h (p_2inf) -> reduces zeta_h -> improves BOTH soft (zeta_h) AND precise
% (zeta_r=zeta_h/chi_r). Test p_2inf x chi_r grid; want MORE margin than chi_r0.65/p2inf0.5.
% s_ddot-drop, p_r_inf=1.0 (Standing Cond 1). margin cells + precision control.
    p2s=[0.5 0.3 0.2]; chis=[0.65 0.85];
    cells={{[2;-2;-5],'Lissajous','L-IC3'},{[2;2;-7],'Circular','C-IC4'},...
           {[2;2;-7],'Lissajous','L-IC4*'},{[2;-2;-5],'Circular','C-IC3*'}};
    for ci=1:numel(chis)
      for pj=1:numel(p2s)
        fprintf('== chi_r=%.2f p_2inf=%.1f ==\n',chis(ci),p2s(pj));
        for k=1:numel(cells)
          d=run_one(cells{k}{1},cells{k}{2},chis(ci),p2s(pj));
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BR'; end
          fprintf('   %-7s xy=%.3f vel=%.3f resid=%.2f %s%s\n',cells{k}{3},d.xy,d.vel,d.resid,tern(sp),br);
        end
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,chi,p2)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1;
    P2INF_XY_OVERRIDE=[p2;p2]; CHI_R_OVERRIDE=[chi;chi]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
