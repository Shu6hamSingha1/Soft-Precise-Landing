function cb_liss_trace()
% TRACE the residual limit cycle: Liss IC3, seed 4, best keep-s_ddot config
% (chi_r=0.65, kR=[4;4;0.5], p2inf_z=1.0). Dump the terminal time history to see HOW
% it diverges: growing oscillation (unstable limit cycle) vs monotone runaway. Report
% per-time: altGap, vx, vy (lateral rel-vel), max funnel residency |r_bar_e|/p_r.
% Print every 25th step over the last ~6 s + flag the breach onset.
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=[2;-2;-5]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE="Lissajous";
    COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=0.7; P2INF_XY_OVERRIDE=[0.5;0.5];
    P2INF_Z_OVERRIDE=1.0; CHI_R_OVERRIDE=[0.65;0.65]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[4;4;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    visualControl_IBVS_adaptive;
    n=idx;
    gap=abs(X_DS(3,1:n)-x_t(3,1:n));
    vx=X_DS(8,1:n)-dx_t(1,1:n); vy=X_DS(9,1:n)-dx_t(2,1:n);
    res=max(abs(r_bar_e(:,1:n))./p_r(:,1:n),[],1);
    b=find(res>=1,1);
    if isempty(b); fprintf('  no breach (resid<1 throughout); n=%d\n',n);
    else; fprintf('  FIRST BREACH at idx=%d  t=%.2fs  altGap=%.2f\n',b,(b-1)*0.01,gap(b)); end
    s0=max(2,n-600);
    fprintf('   t(s) | altGap |   vx    |   vy    | |v_xy| | maxResid\n');
    for k=s0:25:n
      fprintf('  %5.2f | %5.2f  | %+.3f | %+.3f | %5.3f  |  %5.2f\n',...
              (k-1)*0.01, gap(k), vx(k), vy(k), hypot(vx(k),vy(k)), res(k));
    end
    fprintf('  terminal: vx=%.3f vy=%.3f |v_xy|=%.3f  peak|v_xy|(last6s)=%.3f\n',...
            vx(n),vy(n),hypot(vx(n),vy(n)), max(hypot(vx(s0:n),vy(s0:n))));
end
