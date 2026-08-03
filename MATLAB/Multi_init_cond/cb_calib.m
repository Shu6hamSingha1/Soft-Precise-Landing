function cb_calib()
% Calibration of every flow estimator vs the analytical truth. Per estimator: slope
% (gain, 1=ideal), offset (bias, 0=ideal), corr, RMS error. Pooled over cells (slope
% consistency across cells => scale-free). NOISELESS first, then NOISY (seed-averaged).
% Estimators: loom pinv (Vhi3) vs moment (Vh3mom) vs truth Vha3; lateral pinv (Vhi12)
% vs centroid (Vh12raw) vs reduced (Vh12red) vs truth Vha12.
    cells={{"Static",[2;2;-3]},{"Lissajous",[2;-2;-5]},{"Circular",[2;2;-7]},...
           {"Sinusoidal",[2;2;-5]},{"Linear",[2;2;-5]}};
    for noisy=[0 1]
      fprintf('\n===== %s =====\n', ternary(noisy,'NOISY (seeds 1-3 pooled)','NOISELESS'));
      Lt=[];Lp=[];Lm=[]; Xt=[];Xp=[];Xc=[];Xr=[];
      seeds = ternary(noisy,1:3,0);
      for c=cells; for sd=seeds
        cc=c{1}; D=run_one(cc{2},cc{1},noisy,sd);
        Lt=[Lt;D.lt]; Lp=[Lp;D.lp]; Lm=[Lm;D.lm];
        Xt=[Xt;D.xt]; Xp=[Xp;D.xp]; Xc=[Xc;D.xc]; Xr=[Xr;D.xr]; %#ok<AGROW>
      end; end
      fprintf('  LOOM   estimator | slope  offset   corr    RMS\n');
      rep('pinv  ', Lp, Lt); rep('moment', Lm, Lt);
      fprintf('  LATERAL estimator| slope  offset   corr    RMS\n');
      rep('pinv  ', Xp, Xt); rep('centroid', Xc, Xt); rep('reduced ', Xr, Xt);
    end
end
function rep(name, est, tru)
    m=~isnan(est)&~isnan(tru); est=est(m); tru=tru(m);
    A=[est ones(size(est))]; co=A\tru; sl=co(1); off=co(2);
    a=est-mean(est); b=tru-mean(tru); cr=sum(a.*b)/sqrt(sum(a.^2)*sum(b.^2));
    rms=sqrt(mean((tru-A*co).^2));
    fprintf('  %-16s | %+.3f %+.3f  %+.3f  %.4f\n',name,sl,off,cr,rms);
end
function D=run_one(ic,traj,noisy,seed)
    global IC_OVERRIDE NOISE_OVERRIDE RNG_SEED_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT USE_MOMENT_LOOM USE_REDLAT USE_MOMENT_FLOW
    IC_OVERRIDE=ic; NOISE_OVERRIDE=noisy; RNG_SEED_OVERRIDE=seed; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=1; USE_MOMENT_LOOM=0; USE_REDLAT=0; USE_MOMENT_FLOW=0;
    visualControl_IBVS_adaptive; n=idx;
    R=find(Vha3_log(1:n)~=0); R=R(R>R(1)+4 & R<R(end)-2);
    D.lt=Vha3_log(R)'; D.lp=Vhi3_log(R)'; D.lm=Vh3mom_log(R)';
    D.xt=[Vha12_log(1,R)';Vha12_log(2,R)']; D.xp=[Vhi12_log(1,R)';Vhi12_log(2,R)'];
    D.xc=[Vh12raw_log(1,R)';Vh12raw_log(2,R)']; D.xr=[Vh12red_log(1,R)';Vh12red_log(2,R)'];
end
function r=ternary(c,a,b); if c; r=a; else; r=b; end; end
