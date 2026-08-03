function cb_lag_probe()
% Why is the s_ddot FF anti-damping? Hypothesis: measurement/filter LAG rotates the
% acceleration-FF ~90 deg so it aligns with +velocity. If so, NO tau setting removes
% the anti-damping (more tau = more lag). Sweep tau; report corr(-FF_x,vx) [anti-damp
% if >0], net corr(a_actual,vx) [<0 = stable], terminal vx. Compare vs DROP (no FF).
    taus=[0.5 0.7 1.0 1.2 1.5];
    fprintf('  cfg        | corr(-FF,vx) | net corr(a,vx) | pp(vx) | term vx\n');
    for t=taus
      try; row(t,0,sprintf('keep tau=%.1f',t)); catch; fprintf('  keep tau=%.1f| DIVERGED\n',t); end
    end
    try; row([],1,'DROP        '); catch; fprintf('  DROP       | DIVERGED\n'); end
end
function row(tau,drop,name)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=[2;-2;-5]; NOISE_OVERRIDE=0; TRAJ_OVERRIDE="Lissajous"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=drop; CB_SDDOT_TAU=tau; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;3;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    visualControl_IBVS_adaptive;
    n=idx; w=min(400,n-5); k=(n-w):n;
    vx=(X_DS(8,k)-dx_t(1,k))'; ffx=-raw_dh_d(1,k+3)'; ax=gradient(vx,0.01);
    global CB_DROP_SDDOT CB_SDDOT_TAU
    if CB_DROP_SDDOT; lbl='DROP       '; else; lbl=sprintf('keep tau=%.1f',CB_SDDOT_TAU); end
    fprintf('  %-11s| %+8.2f     | %+8.2f       | %5.3f  | %+.3f\n',...
            lbl, cl(ffx,vx), cl(ax,vx), max(vx)-min(vx), vx(end));
end
function c=cl(a,b); a=a-mean(a); b=b-mean(b); d=sqrt(sum(a.^2)*sum(b.^2)); if d==0; c=0; else; c=sum(a.*b)/d; end; end
