function cb_asym_p2y()
% CANDIDATE (keep s_ddot): asymmetric kR=[3;2;0.5] (kR_x=3 kills Liss IC3 X-ring)
% + p2inf_y=0.35 (Y-axis flow-funnel floor, the principled CB7 velocity lever,
% per-axis) to recover the Static/Circ Y-cells broken by low kR_y. tau=0.7 (responsive).
% Full 5x5 noiseless; report SP + worst residency (breach check). Then noisy if clean.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    p2ys=[0.35 0.40 0.30];
    for p2y=p2ys
      SP=0; LD=0; worst=0;
      fprintf('=== kR=[3;2;0.5] p2inf=[0.5;%.2f] tau=0.7 ===\n',p2y);
      for ti=1:5
        line=sprintf('%-11s',trajs{ti});
        for ii=1:5
          d=run_one(ICs{ii},trajs{ti},p2y);
          sp=d.land&&d.xy<=0.08&&d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
          tag='..'; if ~d.land; tag='XX'; elseif sp; tag='SP'; elseif d.xy<=0.08; tag='Pr'; elseif d.vel<=0.20; tag='So'; end
          if d.resid>=1; tag=[tag '!']; end
          line=[line sprintf(' %s:%s(%.2f/%.2f)',icn{ii},tag,d.xy,d.vel)];
        end
        fprintf('%s\n',line);
      end
      fprintf('  >> p2inf_y=%.2f  SP=%d/25 land=%d/25 worstResid=%.3f\n\n',p2y,SP,LD,worst);
    end
end
function d=run_one(ic,traj,p2y)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;p2y]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;2;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('land',double(landed),'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'resid',max(max(abs(rb)./prr)));
    catch ME
        d=struct('land',0,'xy',NaN,'vel',9.99,'resid',9.99);
    end
end
