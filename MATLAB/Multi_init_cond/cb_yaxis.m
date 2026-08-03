function cb_yaxis()
% Liss IC3 residual vel is vy-dominant -> tighten p_2inf_y ONLY (x stays 0.5).
% Verify vy drops, cell -> SP, AND r_bar_e stays inside p_r (no breach). Controls included.
    pys=[0.5, 0.25, 0.15, 0.10];
    cases={{[2;-2;-5],'Lissajous','Liss-IC3'},{[2;2;-5],'Lissajous','Liss-IC2*'},...
           {[2;-2;-5],'Sinusoidal','Sin-IC3*'},{[2;-2;-5],'Circular','Circ-IC3*'}};
    for ci=1:numel(cases)
      fprintf('-- %-9s --\n',cases{ci}{3});
      for py=pys
        d=run_one(cases{ci}{1},cases{ci}{2},py);
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20; br=''; if d.resid>=1; br=' <<BREACH'; end
        fprintf('   p2inf_y=%.2f -> land=%d xy=%.3f vel=%.3f vy=%+.3f resid=%.2f %s%s\n',...
            py,d.land,d.xy,d.vel,d.vy,d.resid,tern(sp),br);
      end
    end
end
function s=tern(c); if c; s='<<SP'; else; s=''; end; end
function d=run_one(ic,traj,py)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;py];
    d=struct('land',0,'xy',NaN,'vel',NaN,'vy',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.vy=vrel(2);
        d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('   ERR %s\n',ME.message); end
end
