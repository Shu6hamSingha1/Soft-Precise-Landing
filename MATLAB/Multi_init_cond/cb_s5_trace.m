function cb_s5_trace()
% Trace Static IC5 seed4 descent divergence (DROP config, isolate from s_ddot).
% Dump: altGap, vz, measured h_z (=V_h_e(3)+V_h_d(3)), desired h_d_z (V_h_d(3)),
% flow error V_h_e(3), descent funnel occ |V_h_e(3)|/p_2(3), thrust T, kappa_z.
% Find WHEN vz runs away and WHICH signal leads it (perception h_z? thrust sat? funnel?).
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE ...
           CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE P2INF_Z_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=1; CB_SDDOT_TAU=[]; P2INF_XY_OVERRIDE=[]; P2INF_Z_OVERRIDE=[]; CHI_R_OVERRIDE=[]; PRINF_OVERRIDE=[]; KR_OVERRIDE=[]; KOMEGA_OVERRIDE=[];
    visualControl_IBVS_adaptive;
    n=idx;
    gap = abs(X_DS(3,1:n)-x_t(3,1:n));
    vz  = X_DS(10,1:n)-dx_t(3,1:n);
    hdz = D_DS(3,1:n);                 % desired flow z (V_h_d(3))
    vhe = V_h_e(3,1:n);                % flow error z (measured - desired)
    hz  = vhe + hdz;                   % measured h_z
    occ = abs(vhe)./p_2(3,1:n);        % descent flow-funnel occupancy
    T   = D_DS(13,1:n);                % thrust
    kz  = kappa(3,1:n);
    b=find(abs(vz)>0.5,1);
    if isempty(b); fprintf('  vz never exceeds 0.5; n=%d\n',n);
    else; fprintf('  vz>0.5 FIRST at idx=%d t=%.2fs altGap=%.2f\n',b,(b-1)*0.01,gap(b)); end
    fprintf('   t(s) |altGap|   vz   | meas h_z | des h_z | Vhe_z | funOcc |  T   | kap_z\n');
    s0=max(2, (isempty(b))*2 + (~isempty(b))*(b-40));
    for k=s0:8:min(n,s0+260)
      fprintf('  %5.2f | %4.2f |%+7.3f | %+7.3f | %+7.3f |%+6.2f| %5.2f |%5.2f |%5.2f\n',...
              (k-1)*0.01,gap(k),vz(k),hz(k),hdz(k),vhe(k),occ(k),T(k),kz(k));
    end
    fprintf('  terminal: vz=%.3f altGap=%.3f T=%.2f kappa_z=%.2f\n',vz(n),gap(n),T(n),kz(n));
end
