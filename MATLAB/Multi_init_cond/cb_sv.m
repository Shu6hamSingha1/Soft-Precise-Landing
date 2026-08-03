function cb_sv()
% WHAT happens at pinv tol=0.01? Show it at the singular-value level. L_s SV spectrum:
% if sigma_min sits in the truncation zone, tol=4 ZEROS that mode (loses loom) while
% tol=0.01 RETAINS it (1/sigma_min amplifies error). Log svmin/svmax of L_s on
% Static IC5 seed4 (DROP) + report pinv h_z error and outcome at tol=4 vs 0.01.
    fprintf('=== Static IC5 seed4: L_s singular-value spectrum ===\n');
    [smin,smax,~,~,~]=run_one(4);
    fprintf('  sigma_min(L_s): min=%.3f  median=%.3f  max=%.3f   (tol=4 zeros all <=4; tol=0.01 keeps them)\n',...
            min(smin), median(smin), max(smin));
    fprintf('  sigma_max(L_s): median=%.1f  -> cond = %.0f\n', median(smax), median(smax)/median(smin));
    fprintf('  fraction of refresh steps with sigma_min in (0.01, 4] = %.0f%%\n\n', 100*mean(smin>0.01 & smin<=4));
    fprintf('=== outcome: tol=4 vs tol=0.01 ===\n');
    for t=[4 0.01]
        [~,~,le,pk,land]=run_one(t);
        fprintf('  tol=%-5g: loomErr=%.3f  peak|vz|=%.3f  land=%d\n',t,le,pk,land);
    end
end
function [smin,smax,le,pk,land]=run_one(tol)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT PINV_TOL
    IC_OVERRIDE=[2;2;-3]; NOISE_OVERRIDE=1; RNG_SEED_OVERRIDE=4; TRAJ_OVERRIDE="Static"; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; PINV_TOL=tol;
    visualControl_IBVS_adaptive; n=idx; R=find(svmin_log(1:n)>0);
    smin=svmin_log(R); smax=svmax_log(R);
    le=mean(abs(Vhi3_log(R)-Vha3_log(R))); vz=X_DS(10,1:n)-dx_t(3,1:n); pk=max(abs(vz)); land=double(landed);
end
