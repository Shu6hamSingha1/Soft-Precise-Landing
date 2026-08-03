function cb_retune()
% FULL RETUNE with kR=2.5 baked. (A) envelope extension: how far does +-X% go now?
% (B) can chi_r go higher (cycle now damped)? (C) re-screen key levers for more margin.
    global VDF_OVERRIDE
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfgN=struct('NOISE',0,'GE',1,'delay',1); cfgR=struct('NOISE',1,'GE',1,'delay',1);
    z=[0;0;-5;1;0;0;0;zeros(6,1)];

    fprintf('=== (A) SPEED ENVELOPE at baked (chi_r=1.15, kR=2.5) ===\n');
    fprintf('  traj        | 1.0 1.2 1.4 1.5 1.6 1.8\n');
    for tr=["Linear","Sinusoidal","Lissajous","Circular"]
        row='';for m=[1.0 1.2 1.4 1.5 1.6 1.8],row=[row sprintf(' %d  ',pass(run_simulation(z,tr,[],m,cfgR,1)))];end
        fprintf('  %-11s |%s\n',tr,row);
    end

    fprintf('=== (B) chi_r magnitude with kR=2.5 (cycle now damped) ===\n');
    fprintf('  chi_r | NL25 real25 | env@1.4 Sin(123) | Liss3 swing\n');
    for chi=[1.15 1.3 1.5]
        VDF_OVERRIDE=struct('chi_r',[chi;chi]);
        nl=set25(trajs,ICs,cfgN,1); r1=set25(trajs,ICs,cfgR,1);
        e=0;for tr=["Linear","Sinusoidal","Lissajous","Circular"],e=e+pass(run_simulation(z,tr,[],1.4,cfgR,1));end
        sc='';for sd=1:3,sc=[sc sprintf('%d',pass(run_simulation(z,"Sinusoidal",[],1.4,cfgR,sd)))];end
        fprintf('  %.2f  | %2d   %2d/25  | %d       %s     | %.3f\n',chi,nl,r1,e,sc,lissw());
    end
    VDF_OVERRIDE=[];

    fprintf('=== (C) re-screen key levers at new baseline (margin search) ===\n');
    fprintf('  lever          | NL25 real25 | env Sin(123) | Liss3 swing | adv\n');
    L={ {'BASE',[]}, {'p_rinf[1;1.2]',s('p_rinf',[1;1.2])}, {'Gam x.6',s('Gamma',diag([.6 .5 .75]))}, ...
        {'N x.04',s('N',diag([.04 .02 .02]))}, {'kap0 x.25',s('kappa0',[.25;.125;.25])}, ...
        {'Xi_h y.3',s('Xi_h',diag([.2 .3 .2]))}, {'kR pitch2',s('kR',diag([2.5 2 .5]))}, ...
        {'kR roll3',s('kR',diag([3 1.5 .5]))} };
    for i=1:numel(L)
        VDF_OVERRIDE=L{i}{2};
        nl=set25(trajs,ICs,cfgN,1); r1=set25(trajs,ICs,cfgR,1);
        e=0;for tr=["Linear","Sinusoidal","Lissajous","Circular"],e=e+pass(run_simulation(z,tr,[],1.4,cfgR,1));end
        sc='';for sd=1:3,sc=[sc sprintf('%d',pass(run_simulation(z,"Sinusoidal",[],1.4,cfgR,sd)))];end
        adv=''; if nl==25&&r1==25&&e==4&&sum(sc=='1')>=2, adv='OK'; end
        fprintf('  %-14s | %2d   %2d/25  | %d   %s     | %.3f       | %s\n',L{i}{1},nl,r1,e,sc,lissw(),adv);
    end
    VDF_OVERRIDE=[];
end
function sp=set25(trajs,ICs,cfg,sd)
    sp=0;for ti=1:5;for ii=1:5;sp=sp+pass(run_simulation([ICs{ii}(:);1;0;0;0;zeros(6,1)],trajs{ti},[],1.0,cfg,sd));end;end
end
function sw=lissw()
    o=run_simulation([2;-2;-5;1;0;0;0;zeros(6,1)],"Lissajous",[],1.0,struct('NOISE',0,'GE',1,'delay',1),1);
    d=o.data;n=d.idx;X=d.X_DS;dxt=d.dx_t;k0=max(1,n-250);
    vx=X(8,k0:n)-dxt(1,k0:n);vy=X(9,k0:n)-dxt(2,k0:n);sw=max(sqrt(vx.^2+vy.^2))-min(sqrt(vx.^2+vy.^2));
end
function b=pass(o), b=double(o.success&&o.precise&&o.soft); end
function st=s(f,v), st=struct(f,v); end
