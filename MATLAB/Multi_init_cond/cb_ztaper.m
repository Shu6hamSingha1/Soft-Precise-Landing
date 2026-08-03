function cb_ztaper()
% Low tau=0.5 (responsive) + ALTITUDE taper on s_ddot (z_ref roll-off). Does it cut the
% lateral terminal over-drive WITHOUT lag? Sweep z_ref on the 8 tau=0.5 fails + 2 SP controls.
    zrefs=[0 2.0 1.5 1.0 0.6];
    cells={{[2;-2;-5],'Static','S-IC3'},{[2;-2;-5],'Sinusoidal','Si-IC3'},{[2;-2;-5],'Lissajous','L-IC3'},...
           {[2;2;-7],'Lissajous','L-IC4'},{[2;2;-5],'Circular','C-IC2'},{[2;-2;-5],'Circular','C-IC3'},...
           {[2;2;-7],'Circular','C-IC4'},{[2;2;-3],'Circular','C-IC5'},{[0;0;-5],'Static','S-IC1*'},{[2;2;-3],'Linear','L5*'}};
    for zi=1:numel(zrefs)
      sp=0; line=sprintf('zref=%.1f',zrefs(zi));
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},zrefs(zi));
        ok=d.land&&d.xy<=0.08&&d.vel<=0.20; sp=sp+ok;
        tag='..'; if ~d.land; tag='XX'; elseif ok; tag='SP'; elseif d.xy<=0.08; tag='So'; else; tag='Pr'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s',cells{k}{3},tag)];
      end
      fprintf('%s  [%d/10]\n',line,sp);
    end
end
function d=run_one(ic,traj,zref)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU CB_SDDOT_ZTAPER P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.5;
    if zref>0; CB_SDDOT_ZTAPER=zref; else; CB_SDDOT_ZTAPER=[]; end
    P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
