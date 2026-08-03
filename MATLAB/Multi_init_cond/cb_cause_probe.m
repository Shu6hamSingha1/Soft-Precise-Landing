function cb_cause_probe()
% ROOT-CAUSE probe for the lateral limit cycle (Liss IC3, the X-ring case).
% Hypothesis: s_ddot FF makes the lateral loop gain grow ~1/z near ground ->
% Hopf onset below a threshold altitude -> bounded limit cycle. Drop removes the
% FF -> stable (decaying). Bin terminal data by altitude-gap; report per bin:
%   pp-amp of vx_rel (cycle amplitude), mean |s_ddot FF| (raw_dh_d lateral),
%   max |sigma_lat|/E (boundary-layer occupancy).
    fprintf('--- KEEP s_ddot (tau=0.7) ---\n'); probe(0.7,0);
    fprintf('--- DROP s_ddot           ---\n'); probe([], 1);
end
function probe(tau,drop)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=[2;-2;-5]; NOISE_OVERRIDE=0; TRAJ_OVERRIDE="Lissajous"; COMBINED_BARRIER=1; C_SIMPLE=1;
    CB_DROP_SDDOT=drop; CB_SDDOT_TAU=tau; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85];
    PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;3;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    visualControl_IBVS_adaptive;
    n=idx; k=2:n;
    gap = abs(X_DS(3,k)-x_t(3,k));                 % altitude above target
    vx  = X_DS(8,k)-dx_t(1,k);                      % lateral X rel-vel
    ff  = sqrt(raw_dh_d(1,k+3).^2 + raw_dh_d(2,k+3).^2);  % lateral s_ddot FF mag
    Ed  = max(diag(K_ctrl.E));                      % boundary-layer half-width (scalar)
    slr = max(abs(sigma(1:2,k)),[],1)/Ed;           % |sigma_lat|/E
    edges=[3 2 1.5 1.0 0.6 0.3 0.0];
    fprintf('  altGap bin |  pp(vx)  | mean|sddotFF| | max|sig|/E |  n\n');
    for b=1:numel(edges)-1
      hi=edges(b); lo=edges(b+1); m=(gap<hi)&(gap>=lo);
      if ~any(m); continue; end
      fprintf('  [%4.1f,%4.1f) | %6.3f  |   %7.3f    |   %6.3f   | %4d\n',...
              lo,hi, max(vx(m))-min(vx(m)), mean(ff(m)), max(slr(m)), sum(m));
    end
    fprintf('  terminal vx=%.3f  gap_end=%.3f  Ediag=%.2f\n\n', vx(end), gap(end), Ed);
end
