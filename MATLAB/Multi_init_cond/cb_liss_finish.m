function cb_liss_finish()
% kR=[3;3;0.5] base (24/25, only Liss IC3 fails vel=0.27, X-ring). Asymmetric kR is
% a dead end (degrades Circ). Try localized chi_r_x / tau to damp Liss IC3 X-ring
% without disturbing Circ IC4/IC5. Each cfg = full 5x5.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    % cfg = {chi_r, tau}
    cfgs={{[0.95;0.85],0.7},{[1.0;0.85],0.7},{[0.85;0.85],0.8},{[0.95;0.85],0.8},{[1.0;0.85],0.75}};
    for ci=1:numel(cfgs)
      chi=cfgs{ci}{1}; tau=cfgs{ci}{2}; SP=0; LD=0; worst=0;
      fprintf('=== chi_r=[%.2f %.2f] tau=%.2f ===\n',chi(1),chi(2),tau);
      for ti=1:5
        line=sprintf('%-11s',trajs{ti});
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},chi,tau);
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
          tag='..'; if ~d.land; tag='XX'; elseif sp; tag='SP'; elseif d.xy<=0.08; tag='Pr'; elseif d.vel<=0.20; tag='So'; end
          if d.resid>=1; tag=[tag '!']; end
          line=[line sprintf(' %s:%s(%.2f/%.2f)',icn{ii},tag,d.xy,d.vel)];
        end
        fprintf('%s\n',line);
      end
      fprintf('  >> chi=[%.2f %.2f] tau=%.2f  SP=%d/25 land=%d/25 worstResid=%.3f\n\n',chi(1),chi(2),tau,SP,LD,worst);
    end
end
function d=run_one(ic,traj,chi,tau)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=tau; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=chi; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;3;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
