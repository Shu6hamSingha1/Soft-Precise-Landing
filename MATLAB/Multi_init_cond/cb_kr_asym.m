function cb_kr_asym()
% Responsive s_ddot (tau=0.7) + asymmetric kR sweep to reclaim 25/25.
% Base: COMBINED_BARRIER, C_SIMPLE, CB_DROP_SDDOT=0, CB_SDDOT_TAU=0.7,
% CHI_R=[0.85;0.85], P2INF_XY=[0.5;0.5], PRINF=[1.0;1.0], KOMEGA=[0.6;0.6;0.1].
% Sweep kR=[kx;ky;0.5]. kR=[3;3] gives 24/25 (Liss IC3 vel=0.273).
% kR=[3;2] -> Liss IC3 SP but Circ IC5 + Sin IC3 slip to 0.21. Reclaim the 0.01.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    krs={[3;3;0.5],[2.75;2.25;0.5],[3;2.25;0.5],[2.75;2.5;0.5]};
    for ki=1:numel(krs)
      kr=krs{ki}; SP=0; LD=0; worst=0;
      fprintf('=== kR=[%.2f %.2f %.2f] ===\n',kr(1),kr(2),kr(3));
      for ti=1:5
        line=sprintf('%-11s',trajs{ti});
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},kr);
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
          tag='..'; if ~d.land; tag='XX'; elseif sp; tag='SP'; elseif d.xy<=0.08; tag='Pr'; elseif d.vel<=0.20; tag='So'; end
          if d.resid>=1; tag=[tag '!']; end
          line=[line sprintf(' %s:%s(%.2f/%.2f)',icn{ii},tag,d.xy,d.vel)];
        end
        fprintf('%s\n',line);
      end
      fprintf('  >> kR=[%.2f %.2f]  SP=%d/25 land=%d/25 worstResid=%.3f\n\n',kr(1),kr(2),SP,LD,worst);
    end
end
function d=run_one(ic,traj,kr)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=kr; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; d.land=0; end
end
