function cb_other_gains()
% Y-axis is what breaches under chi_r=[1.1,0.65]. Check OTHER gains on Y to rectify:
% Y-velocity damping p_2inf_y, Y boundary E_y, and raising chi_r_y. s_ddot-KEPT(tau0.8).
% Must FIX Static IC5/Liss IC3 (Y-breach) WITHOUT losing Circ IC4(SP)/Circ IC5.
    % {label, p2y, Ey, chiY}
    cfgs={{'base',0.5,0.8,0.65},{'p2y0.3',0.3,0.8,0.65},{'p2y0.2',0.2,0.8,0.65},...
          {'Ey0.4',0.5,0.4,0.65},{'chiY0.85',0.5,0.8,0.85},{'p2y0.3+chiY0.8',0.3,0.8,0.8}};
    cells={{[2;2;-3],'Static','S-IC5'},{[2;-2;-5],'Lissajous','L-IC3'},{[2;2;-7],'Circular','C-IC4'},{[2;2;-3],'Circular','C-IC5'}};
    for ci=1:numel(cfgs)
      sp=0; line=sprintf('%-14s',cfgs{ci}{1});
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},cfgs{ci}{2},cfgs{ci}{3},cfgs{ci}{4});
        ok=d.land&&d.xy<=0.08&&d.vel<=0.20; sp=sp+ok;
        tag='..'; if ~d.land; tag='XX'; elseif ok; tag='SP'; elseif d.xy<=0.08; tag='Pr'; elseif d.vel<=0.20; tag='So'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s',cells{k}{3},tag)];
      end
      fprintf('%s  [%d/4]\n',line,sp);
    end
end
function d=run_one(ic,traj,p2y,Ey,chiY)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE E_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.8;
    CHI_R_OVERRIDE=[1.1;chiY]; P2INF_XY_OVERRIDE=[0.5;p2y]; PRINF_OVERRIDE=[1.0;1.0]; E_XY_OVERRIDE=[0.8;Ey];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
