function cb_flowfilt()
% OPTICAL FLOW control: filter the measured flow-rate (CB_SDOT_FILT savgol) to break the
% positive-feedback limit cycle BEFORE it enters h_d. Sweep window at low tau=0.5. Does it
% damp the velocity cycle? (cycle is slow ~0.5Hz, so need wide W -> watch lag tradeoff.)
    Ws=[0 25 51 101 151];
    cells={{[2;-2;-5],'Sinusoidal','Si3'},{[2;2;-7],'Lissajous','L4'},{[2;2;-7],'Circular','C4'},...
           {[2;2;-3],'Circular','C5'},{[2;-2;-5],'Lissajous','L3'},{[0;0;-5],'Static','S1*'}};
    for wi=1:numel(Ws)
      sp=0; line=sprintf('W=%-3d',Ws(wi));
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},Ws(wi));
        ok=d.land&&d.xy<=0.08&&d.vel<=0.20; sp=sp+ok;
        tag='..'; if ~d.land; tag='XX'; elseif ok; tag='SP'; elseif d.xy<=0.08; tag='So'; else; tag='Pr'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s(%.2f/%.2f)',cells{k}{3},tag,d.xy,d.vel)];
      end
      fprintf('%s [%d/6]\n',line,sp);
    end
end
function d=run_one(ic,traj,W)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU CB_SDOT_FILT P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.5;
    if W>=3; CB_SDOT_FILT=W; else; CB_SDOT_FILT=[]; end
    P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
