function cb_momdiag()
% Failure analysis: list & classify the failures of (A) moment-loom-only (95/100) and
% (B) moment-FLOW no-pinv (86/100). keep s_ddot 92-config. Per failing cell: seed, xy,
% per-axis vel, resid, dominant cause.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    modes={[1 0],[0 1]}; names={'MOMENT LOOM (95/100)','MOMENT FLOW no-pinv (86/100)'};
    for mi=1:2
      ml=modes{mi}(1); mf=modes{mi}(2); nf=0;
      fprintf('=== %s ===\n',names{mi});
      for sd=1:4
        for ti=1:5; for ii=1:5
          d=one(ICs{ii},trajs{ti},sd,ml,mf);
          if d.sp; continue; end
          nf=nf+1;
          cause='';
          if ~d.land; cause='NO-LAND';
          elseif abs(d.vz)>=0.20 && abs(d.vz)>hypot(d.vx,d.vy); cause='DESCENT vz';
          elseif hypot(d.vx,d.vy)>=0.20; if abs(d.vx)>abs(d.vy); cause='LAT vx'; else; cause='LAT vy'; end
          elseif d.xy>0.08; cause='PRECISION xy'; else; cause='marginal'; end
          if d.resid>=1; cause=[cause ' +BREACH']; end
          fprintf('  s%d %-10s %s | xy=%.3f |v|=%.3f (vx%+.2f vy%+.2f vz%+.2f) res=%.2f | %s\n',...
                  sd,trajs{ti},icn{ii},d.xy,d.vel,d.vx,d.vy,d.vz,d.resid,cause);
        end; end
      end
      fprintf('  >> %d failures\n\n',nf);
    end
end
function d=one(ic,traj,seed,ml,mf)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU ...
           P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE USE_MOMENT_LOOM USE_MOMENT_FLOW
    IC_OVERRIDE=ic; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7;
    P2INF_XY_OVERRIDE=[0.5;0.5]; P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    USE_MOMENT_LOOM=ml; USE_MOMENT_FLOW=mf;
    try
        visualControl_IBVS_adaptive; vrel=I_v_c-dx_t(1:3,idx); rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d=struct('sp',double(landed&&norm(I_p_c(1:2)-x_t(1:2,idx))<=0.08&&norm(vrel)<=0.20),'land',double(landed),...
                 'xy',norm(I_p_c(1:2)-x_t(1:2,idx)),'vel',norm(vrel),'vx',vrel(1),'vy',vrel(2),'vz',vrel(3),'resid',max(max(abs(rb)./prr)));
    catch ME; d=struct('sp',0,'land',0,'xy',9,'vel',9,'vx',9,'vy',9,'vz',9,'resid',9.99); end
end
