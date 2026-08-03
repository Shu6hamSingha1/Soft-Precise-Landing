function cb_suite()
% Full-suite validation of the combined-barrier + s_ddot-drop (corrected c-h, chi_r=1):
% 5 traj x 5 IC noiseless. Does the recovered formulation cover the canonical set?
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    SP=0; LD=0;
    for ti=1:5
      line=sprintf('%-11s',trajs{ti});
      for ii=1:5
        r=run_one(ICs{ii},trajs{ti});
        sp=r.land && r.xy<=0.08 && r.vel<=0.20; SP=SP+sp; LD=LD+r.land;
        tag='..'; if ~r.land; tag='XX'; elseif sp; tag='SP'; elseif r.xy<=0.08; tag='Pr'; elseif r.vel<=0.20; tag='So'; end
        line=[line sprintf(' %s:%s',icn{ii},tag)];
      end
      fprintf('%s\n',line);
    end
    fprintf('  >> TOTAL  SP(soft+precise)=%d/25  landed=%d/25\n',SP,LD);
end
function r=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1;
    xy=NaN;vel=NaN;ld=0;
    try
        visualControl_IBVS_adaptive;
        xy=norm(I_p_c(1:2)-x_t(1:2,idx)); vel=norm(I_v_c-dx_t(1:3,idx)); ld=double(landed);
    catch ME; fprintf('  ERR %s\n',ME.message); end
    r=struct('land',ld,'xy',xy,'vel',vel);
end
