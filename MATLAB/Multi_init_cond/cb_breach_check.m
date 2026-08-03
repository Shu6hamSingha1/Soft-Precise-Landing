function cb_breach_check()
% Verify the prescribed-performance guarantee holds for the tuned combined-barrier:
% r_bar_e (= s_e_n analog) must NOT breach p_r (= p_s analog), i.e. max|r_bar_e/p_r|<1,
% across the full suite with p_2inf=0.5 + s_ddot-drop. Also split vel into vx/vy/vz.
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]}; icn={'IC1','IC2','IC3','IC4','IC5'};
    worst=0;
    for ti=1:5
      for ii=1:5
        d=run_one(ICs{ii},trajs{ti});
        br=''; if d.resid>=1; br=' <<BREACH'; end
        worst=max(worst,d.resid);
        flag=''; if ~(d.land&&d.xy<=0.08&&d.vel<=0.20); flag='  [not SP]'; end
        fprintf('%-11s %s: resid=%.2f | vel=%.3f (vx%+.2f vy%+.2f vz%+.2f) xy=%.3f%s%s\n',...
            trajs{ti},icn{ii},d.resid,d.vel,d.vx,d.vy,d.vz,d.xy,br,flag);
      end
    end
    fprintf('  >> WORST residency max|r_bar_e/p_r| = %.3f  (guarantee holds iff < 1)\n',worst);
end
function d=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT P2INF_XY_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; P2INF_XY_OVERRIDE=[0.5;0.5];
    d=struct('resid',NaN,'vel',NaN,'vx',NaN,'vy',NaN,'vz',NaN,'xy',NaN,'land',0);
    try
        visualControl_IBVS_adaptive;
        vrel=I_v_c-dx_t(1:3,idx);
        rb=r_bar_e(:,1:idx); prr=p_r(:,1:idx);
        d.resid=max(max(abs(rb)./prr)); d.vel=norm(vrel); d.vx=vrel(1); d.vy=vrel(2); d.vz=vrel(3);
        d.xy=norm(I_p_c(1:2)-x_t(1:2,idx)); d.land=double(landed);
    catch ME; fprintf('  ERR %s\n',ME.message); end
end
