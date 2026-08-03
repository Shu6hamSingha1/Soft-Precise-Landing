function cb_skept_suite_peraxis()
% Full 5x5 with s_ddot-KEPT (tau=0.8) + full tuning (p_2inf=0.5, chi_r=0.85, p_r_inf=1.0).
% Per cell: land, xy, vel, per-axis terminal pos[px,py] + vel[vx,vy,vz], classify
% PRECISE-fail (xy>0.08) vs SOFT-fail (vel>0.20), and the BINDING axis for each.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    for ti=1:5
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti});
        sp=d.land&&d.xy<=0.08&&d.vel<=0.20;
        tag='SP'; if ~d.land; tag='XX'; elseif d.xy>0.08&&d.vel>0.20; tag='..'; elseif d.xy>0.08; tag='Pr-FAIL'; elseif d.vel>0.20; tag='So-FAIL'; end
        if sp; continue; end   % only print non-SP
        fprintf('%-11s %s [%s] xy=%.3f vel=%.3f | pos[px%+.3f py%+.3f] vel[vx%+.3f vy%+.3f vz%+.3f]\n',...
          trajs{ti},icn{ii},tag,d.xy,d.vel,d.px,d.py,d.vx,d.vy,d.vz);
      end
    end
    fprintf('(cells not listed = SP)\n');
end
function d=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.8; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0];
    d=struct('land',0,'xy',NaN,'vel',NaN,'px',NaN,'py',NaN,'vx',NaN,'vy',NaN,'vz',NaN);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx); prel=I_p_c-x_t(1:3,idx);
        d.land=double(landed); d.xy=norm(prel(1:2)); d.vel=norm(vrel);
        d.px=prel(1); d.py=prel(2); d.vx=vrel(1); d.vy=vrel(2); d.vz=vrel(3);
    catch ME; d.land=0; end
end
