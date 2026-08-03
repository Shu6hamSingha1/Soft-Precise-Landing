function cb_suite_final()
% Full 5x5: combined-barrier + corrected + s_ddot-drop + p_2inf=0.5 + CHI_R=0.5.
% Target clean 25/25 SP with NO r_bar_e/p_r breach (worst residency < 1).
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    SP=0; LD=0; worst=0;
    for ti=1:5
      line=sprintf('%-11s',trajs{ti});
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti});
        sp=d.land && d.xy<=0.08 && d.vel<=0.20; SP=SP+sp; LD=LD+d.land; worst=max(worst,d.resid);
        tag='..'; if ~d.land; tag='XX'; elseif sp; tag='SP'; elseif d.xy<=0.08; tag='Pr'; elseif d.vel<=0.20; tag='So'; end
        if d.resid>=1; tag=[tag '!']; end
        line=[line sprintf(' %s:%s',icn{ii},tag)];
      end
      fprintf('%s\n',line);
    end
    fprintf('  >> chi_r=0.5,p2inf=0.5  SP=%d/25 land=%d/25  worstResid=%.3f (no breach if<1)\n',SP,LD,worst);
end
function d=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE CHI_R_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.5;0.5];
    d=struct('land',0,'xy',NaN,'vel',NaN,'resid',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.land=double(landed); d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.vel=norm(vrel); d.resid=max(max(abs(rb)./prr));
    catch ME; fprintf('  ERR %s\n',ME.message); end
end
