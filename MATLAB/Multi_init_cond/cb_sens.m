function cb_sens()
% One-at-a-time sensitivity of the ACTIVE untested lateral levers (E, gamma_2, N, kappa_0, P)
% on the low-tau (0.5) soft-fails. Which moves the lateral terminal velocity? base = others nominal.
    % {label, E, g2, N, k0, P}  ([] = nominal)
    cfgs={{'base',[],[],[],[],[]},...
          {'E0.5',0.5,[],[],[],[]},{'E2.0',2.0,[],[],[],[]},...
          {'g2=0.5',[],0.5,[],[],[]},{'g2=1.0',[],1.0,[],[],[]},...
          {'N0.1',[],[],0.1,[],[]},{'N0.005',[],[],0.005,[],[]},...
          {'k0=0.4',[],[],[],0.4,[]},{'k0=0.04',[],[],[],0.04,[]},...
          {'P0.5',[],[],[],[],0.5},{'P4',[],[],[],[],4.0}};
    cells={{[2;-2;-5],'Sinusoidal','Si3'},{[2;2;-7],'Circular','C4'},{[2;2;-3],'Circular','C5'},{[2;2;-5],'Circular','C2'},{[0;0;-5],'Static','S1*'}};
    for ci=1:numel(cfgs)
      sp=0; line=sprintf('%-7s',cfgs{ci}{1}); vs='';
      for k=1:numel(cells)
        d=run_one(cells{k}{1},cells{k}{2},cfgs{ci}{2},cfgs{ci}{3},cfgs{ci}{4},cfgs{ci}{5},cfgs{ci}{6});
        ok=d.land&&d.xy<=0.08&&d.vel<=0.20; sp=sp+ok;
        tag='..'; if ~d.land; tag='XX'; elseif ok; tag='SP'; elseif d.xy<=0.08; tag='So'; else; tag='Pr'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s(%.2f)',cells{k}{3},tag,d.vel)];
      end
      fprintf('%s  [%d/5]\n',line,sp);
    end
end
function d=run_one(ic,traj,E,g2,N,k0,P)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE E_XY_OVERRIDE GAMMA2_XY_OVERRIDE N_XY_OVERRIDE KAPPA0_XY_OVERRIDE P_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.5; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    if ~isempty(E); E_XY_OVERRIDE=[E;E]; else; E_XY_OVERRIDE=[]; end
    if ~isempty(g2); GAMMA2_XY_OVERRIDE=[g2;g2]; else; GAMMA2_XY_OVERRIDE=[]; end
    if ~isempty(N); N_XY_OVERRIDE=[N;N]; else; N_XY_OVERRIDE=[]; end
    if ~isempty(k0); KAPPA0_XY_OVERRIDE=[k0;k0]; else; KAPPA0_XY_OVERRIDE=[]; end
    if ~isempty(P); P_XY_OVERRIDE=[P;P]; else; P_XY_OVERRIDE=[]; end
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
