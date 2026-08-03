function cb_areadiv()
% Reconcile: the descent flow is the divergence vz/z (apparent-area rate). Question:
% does the AREA-RATE divergence avoid the sign-flip that the pinv(L_s) extraction shows?
% Static IC5 seed4 (DROP). featrad = mean feature radius (area ~ featrad^2), so the
% area-expansion-rate divergence = 2*d(log featrad)/dt_img. Compare its sign/track to
% the analytical h_z (truth) and the pinv h_z (used). If area-div tracks truth w/o
% sign-flip, it confirms the area-rate measurement is well-conditioned.
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT ZOH_PROBE
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1;
    visualControl_IBVS_adaptive;
    n=idx; R=find(condLs_log(1:n)>0);              % refresh steps
    fr=featrad_log(R); hi=Vhi3_log(R); ha=Vha3_log(R);
    t=(R-1)*0.01; dti=mean(diff(t));
    % area-rate divergence: area ~ fr^2 -> div = d/dt(log area) = 2*d(log fr)/dt
    logfr=log(max(fr,1e-6)); dlog=gradient(logfr,dti); areadiv=2*dlog;
    % terminal window
    m=numel(R); s0=max(1,m-40);
    fprintf('  t(s) | featRad | pinv h_z | analyt h_z | areaDiv(2 dlnfr/dt) | pinvFLIP | areaFLIP\n');
    for k=s0:m
      pf=''; if sign(hi(k))~=sign(ha(k))&&abs(ha(k))>0.05; pf='YES'; end
      af=''; if sign(areadiv(k))~=sign(ha(k))&&abs(ha(k))>0.05; af='YES'; end
      fprintf('  %4.2f | %6.3f |%+8.3f |%+8.3f   |   %+8.3f         | %3s | %3s\n',...
              t(k),fr(k),hi(k),ha(k),areadiv(k),pf,af);
    end
    fprintf('  >> pinv sign-flips: %d/%d steps | areaDiv sign-flips: %d/%d steps (vs analytic truth)\n',...
            sum(sign(hi)~=sign(ha)&abs(ha)>0.05), m, sum(sign(areadiv)~=sign(ha)&abs(ha)>0.05), m);
end
