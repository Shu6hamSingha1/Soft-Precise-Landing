function cb_chi_fine()
% Thread the monotonic chi_r trade: find chi_r where BOTH L-IC3 (vel) & C-IC4 (prec)
% are SP. Plus a combo row (chi_r=0.6 + p_2inf=0.3 velocity nudge). p_2inf=0.5 base.
    rows={ {0.55,0.5},{0.60,0.5},{0.65,0.5},{0.60,0.3} };
    cells={{[2;-2;-5],'Lissajous','L-IC3(vel)'},{[2;2;-7],'Circular','C-IC4(prec)'},...
           {[0;0;-5],'Circular','C-IC2*'},{[2;2;-5],'Sinusoidal','S-IC2*'}};
    for r=1:numel(rows)
      fprintf('== chi_r=%.2f p_2inf=%.1f ==\n',rows{r}{1},rows{r}{2});
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},rows{r}{1},rows{r}{2});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br='BREACH'; end
        fprintf('   %-11s xy=%.3f vel=%.3f resid=%.2f %s%s\n',cells{k}{3},d.xy,d.vel,d.resid,tern(sp),br);
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,chi,p2)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1;
    P2INF_XY_OVERRIDE=[p2;p2]; CHI_R_OVERRIDE=[chi;chi]; PRINF_OVERRIDE=[];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
