function cb_robusteval()
% RESOLVE robustness: multi-seed SP-rate for candidate configs (h_rd=-0.42 now). Tests
% whether the cycle-reduction gains (esp the SHARP kR=2.5) are robust or seed-fragile.
% Metric: SP-rate + #fly-aways(vel>0.5) over seeds 1000-1014 x 5 trajs x {IC2,IC3,IC5}.
    global VDF_OVERRIDE
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    hardIC={[2;2;-5],[2;-2;-5],[2;2;-3]};   % IC2,IC3,IC5
    cfg=struct('NOISE',1,'GE',1,'delay',1);
    C={ {'ORIGINAL(old)',struct('chi_r',[.85;.85],'kR',diag([1.5 1.5 .5]),'kOmega',diag([.3 .3 .1]))}, ...
        {'CURRENT(c1.15,kR2.5,kO.2)',[]}, ...
        {'kR2.0',struct('kR',diag([2.0 1.5 .5]))}, ...
        {'kR1.5',struct('kR',diag([1.5 1.5 .5]))}, ...
        {'kR3.0',struct('kR',diag([3.0 1.5 .5]))}, ...
        {'kOmz0.1',struct('kOmega',diag([.3 .3 .1]))}, ...
        {'chi0.85',struct('chi_r',[.85;.85])} };
    N=15*5*3;
    fprintf('  config                     | SP-rate (of %d) | flyaways\n',N);
    for i=1:numel(C)
        VDF_OVERRIDE=C{i}{2}; sp=0; fa=0;
        for sd=1000:1014
          for ti=1:5
            for ic=1:3
              o=run_simulation([hardIC{ic}(:);1;0;0;0;zeros(6,1)],trajs{ti},[],1.0,cfg,sd);
              sp=sp+(o.success&&o.precise&&o.soft); fa=fa+(o.final_rel_vel>0.5);
            end
          end
        end
        fprintf('  %-26s | %3d/%d (%.0f%%)   | %d\n',C{i}{1},sp,N,100*sp/N,fa);
    end
    VDF_OVERRIDE=[];
end
