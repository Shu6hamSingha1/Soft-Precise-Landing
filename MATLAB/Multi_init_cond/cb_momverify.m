function cb_momverify()
% Verify the SCALE-FREE moment loom (Vh3_mom = -1/2 dlnM/dt) against the analytical
% truth V_h_a(3), WITHOUT applying it (USE_MOMENT_LOOM=0). Check: sign, factor (~1),
% and that the slope/agreement is CONSISTENT across cells & altitudes (=> scale-free,
% no per-scenario calibration). Report over the descent (refresh steps, mid-flight to
% near-ground), correlation + best-fit slope (should be ~1 everywhere if factor right).
    cells={{"Static",[2;2;-3],'Sta-IC5'},{"Lissajous",[2;2;-7],'Lis-IC4'},...
           {"Circular",[2;-2;-5],'Cir-IC3'},{"Sinusoidal",[2;2;-5],'Sin-IC2'}};
    fprintf('  cell    | n | corr(mom,truth) | slope(truth~mom) | mom@hi-alt | truth@hi-alt\n');
    for c=cells
      cc=c{1}; r=run_one(cc{2},cc{1});
      fprintf('  %-7s |%3d|     %+.2f       |     %+.3f       |  %+.3f    |  %+.3f\n',...
              cc{3},r.n,r.cr,r.sl,r.mhi,r.thi);
    end
    fprintf('  (slope ~1 and consistent across cells => fixed scale-free factor is correct)\n');
end
function r=run_one(ic,traj)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT USE_MOMENT_LOOM
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; RNG_SEED_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; USE_MOMENT_LOOM=0;
    visualControl_IBVS_adaptive; n=idx; R=find(Vh3mom_log(1:n)~=0 & Vha3_log(1:n)~=0);
    R=R(R>R(1)+3);  % drop the initial buffer-fill transient
    mom=Vh3mom_log(R)'; tru=Vha3_log(R)';
    a=mom-mean(mom); b=tru-mean(tru); cr=sum(a.*b)/sqrt(sum(a.^2)*sum(b.^2));
    sl=mom\tru;  % least-squares slope truth ~ slope*mom
    r=struct('n',numel(R),'cr',cr,'sl',sl,'mhi',mom(1),'thi',tru(1));
end
