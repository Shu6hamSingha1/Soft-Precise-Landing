function cb_phase_probe()
% Decisive phase test for the limit-cycle pump. The s_ddot FF enters the accel
% command as a_x ~ -V_dh_d_x. If -FF_x correlates POSITIVELY with lateral velocity
% vx over the cycle, it is anti-damping (injects energy in phase with motion) ->
% pumps the under-damped loop. Compare KEEP vs DROP over the terminal 4 s.
% Also report damping sign of the realized command I_a_cd_x vs vx.
    fprintf('--- KEEP s_ddot (tau=0.7) ---\n'); probe(0.7,0);
    fprintf('--- DROP s_ddot           ---\n'); probe([],1);
end
function probe(tau,drop)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=[2;-2;-5]; NOISE_OVERRIDE=0; TRAJ_OVERRIDE="Lissajous"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=drop; CB_SDDOT_TAU=tau; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;3;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    visualControl_IBVS_adaptive;
    n=idx; w=min(400,n-5); k=(n-w):n;
    vx  = (X_DS(8,k)-dx_t(1,k))';
    ffx = -raw_dh_d(1,k+3)';                 % FF contribution to a_x (sign as enters command)
    iax = (I_a_cd(1,k))';                     % realized lateral accel command (inertial X)
    ax  = gradient(vx, 0.01);                 % actual lateral acceleration
    % anti-damping test: correlation of a-contribution with +velocity
    c_ff  = corrloc(ffx, vx);
    c_iax = corrloc(iax, vx);
    c_ax  = corrloc(ax,  vx);
    % energy injected by FF over the window: sum(ffx .* vx) (power ~ force*vel); >0 => pumps
    P_ff = mean(ffx.*vx);
    fprintf('  corr(-FF_x, vx)=%+.2f   corr(Iacd_x, vx)=%+.2f   corr(ax_actual, vx)=%+.2f\n', c_ff, c_iax, c_ax);
    fprintf('  mean(FF_x*vx)=%+.4f  (>0 => anti-damping pump)   pp(vx)=%.3f  term vx=%.3f\n\n', P_ff, max(vx)-min(vx), vx(end));
end
function c=corrloc(a,b)
    a=a-mean(a); b=b-mean(b); d=sqrt(sum(a.^2)*sum(b.^2));
    if d==0; c=0; else; c=sum(a.*b)/d; end
end
