function cb_cycle_probe()
% Liss IC3 (the X-ring case). For tau in {0.7(fail),1.0,1.2(pass),1.5}, dump the
% terminal X relative-velocity waveform to decide: is tau shrinking the limit-cycle
% AMPLITUDE (cycle itself) or just landing at a low-velocity phase (masking effect)?
% Report peak-to-peak amplitude + dominant period over the last 4 s, plus terminal vx.
    taus=[0.7 1.0 1.2 1.5];
    for ki=1:numel(taus)
      tau=taus(ki);
      [vx,vy,nidx]=run_one([2;-2;-5],"Lissajous",tau);
      % last 4 s window (400 steps @ dt=0.01), clipped to available
      w=min(400,nidx-1); seg=vx(nidx-w:nidx);
      pp=max(seg)-min(seg);
      % zero-crossing period estimate on detrended seg
      sd=seg-mean(seg); zc=find(sd(1:end-1).*sd(2:end)<0);
      per=NaN; if numel(zc)>=2; per=2*mean(diff(zc))*0.01; end
      fprintf('tau=%.2f | term vx=%.3f vy=%.3f | last4s X: pp-amp=%.3f mean=%.3f period=%.2fs ncross=%d\n',...
              tau, vx(nidx), vy(nidx), pp, mean(seg), per, numel(zc));
    end
end
function [vx,vy,nidx]=run_one(ic,traj,tau)
    global IC_OVERRIDE NOISE_OVERRIDE TRAJ_OVERRIDE COMBINED_BARRIER C_SIMPLE CB_DROP_SDDOT CB_SDDOT_TAU P2INF_XY_OVERRIDE CHI_R_OVERRIDE PRINF_OVERRIDE KR_OVERRIDE KOMEGA_OVERRIDE
    IC_OVERRIDE=ic; NOISE_OVERRIDE=0; TRAJ_OVERRIDE=traj; COMBINED_BARRIER=1; C_SIMPLE=1; CB_DROP_SDDOT=0; CB_SDDOT_TAU=tau; P2INF_XY_OVERRIDE=[0.5;0.5]; CHI_R_OVERRIDE=[0.85;0.85]; PRINF_OVERRIDE=[1.0;1.0]; KR_OVERRIDE=[3;3;0.5]; KOMEGA_OVERRIDE=[0.6;0.6;0.1];
    visualControl_IBVS_adaptive;
    nidx=idx;
    vx=(X_DS(8,1:idx)-dx_t(1,1:idx))'; vy=(X_DS(9,1:idx)-dx_t(2,1:idx))';
end
