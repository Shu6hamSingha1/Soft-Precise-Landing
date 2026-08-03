function cb_asym_kr()
% Liss IC3 rings harder on X (faster Lissajous w1). Asymmetric kR: raise roll kR_x, hold kR_y=x2
% (protect Circ IC5 which rings on Y). tau0.7. [sx sy] per-axis kR scale.
    cfgs={{'2/2',2,2},{'3/2',3,2},{'4/2',4,2},{'3/2.5',3,2.5},{'4/2.5',4,2.5},{'2.5/2.5',2.5,2.5}};
    cells={{[2;-2;-5],'Lissajous','L3'},{[2;2;-3],'Circular','C5'},{[2;2;-7],'Circular','C4'},...
           {[2;-2;-5],'Static','S3*'},{[2;-2;-5],'Sinusoidal','Si3*'},{[2;2;-7],'Lissajous','L4*'}};
    for ci=1:numel(cfgs)
      sp=0; line=sprintf('kR %-6s',cfgs{ci}{1});
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},cfgs{ci}{2},cfgs{ci}{3});
        ok=d.land&&d.xy<=0.08&&d.vel<=0.20; sp=sp+ok;
        tag='..'; if ~d.land; tag='XX'; elseif ok; tag='SP'; elseif d.xy<=0.08; tag='So'; else; tag='Pr'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s(%.2f)',cells{k}{3},tag,d.vel)];
      end
      fprintf('%s [%d/6]\n',line,sp);
    end
end
function d=run_one(ic,traj,sx,sy)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    KR_OVERRIDE=[1.5*sx;1.5*sy;0.5]; KOMEGA_OVERRIDE=[0.3*sx;0.3*sy;0.1];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
