function cb_cycmin()
% COMPREHENSIVE cycle-reduction re-test at baked (chi_r=1.15, kR=2.5). Objective: MIN
% Liss-IC3 swing-pp while holding NL25 + real25(s1) + full +-40% + Sin robust. Re-test
% all key levers + other params + damping combos. baseline swing = 0.562.
    global VDF_OVERRIDE GAMMA_V
    trajs={"Static","Linear","Sinusoidal","Lissajous","Circular"};
    ICs={[0;0;-5],[2;2;-5],[2;-2;-5],[2;2;-7],[2;2;-3]};
    cfgN=struct('NOISE',0,'GE',1,'delay',1); cfgR=struct('NOISE',1,'GE',1,'delay',1);
    z=[0;0;-5;1;0;0;0;zeros(6,1)];
    L={ {'BASE',[],[]}, ...
        {'kR roll2.75',s('kR',diag([2.75 1.5 .5])),[]}, {'kR roll3',s('kR',diag([3 1.5 .5])),[]}, ...
        {'kOm roll.4',s('kOmega',diag([.4 .3 .1])),[]}, {'kOm roll.5',s('kOmega',diag([.5 .3 .1])),[]}, ...
        {'gv1.1',[],1.1}, {'gv1.2',[],1.2}, {'gv1.3',[],1.3}, ...
        {'p_rinf[1;1.2]',s('p_rinf',[1;1.2]),[]}, {'p_rinf[1;1.3]',s('p_rinf',[1;1.3]),[]}, ...
        {'Xi_h y.3',s('Xi_h',diag([.2 .3 .2])),[]}, {'Xi_r y.05',s('Xi_r',diag([.1 .05])),[]}, ...
        {'Gam y.35',s('Gamma',diag([.4375 .35 .75])),[]}, {'N y.04',s('N',diag([.02 .04 .02])),[]}, ...
        {'kap0 y.25',s('kappa0',[.125;.25;.25]),[]}, {'chi_z.04',s('chi_z',.04),[]}, ...
        {'kR2.5+kOm.4',s('kOmega',diag([.4 .3 .1])),[]}, {'kR3+prinf',sm({'kR',diag([3 1.5 .5]),'p_rinf',[1;1.2]}),[]}, ...
        {'kR2.5+gv1.2',[],1.2}, {'kR2.75+gv1.1',s('kR',diag([2.75 1.5 .5])),1.1} };
    fprintf('  lever          | swing | NL25 real25 env Sin | BEAT?\n');
    for i=1:numel(L)
        VDF_OVERRIDE=L{i}{2}; GAMMA_V=L{i}{3};
        sw=lissw();
        nl=set25(trajs,ICs,cfgN,1); r1=set25(trajs,ICs,cfgR,1);
        e=0;for tr=["Linear","Sinusoidal","Lissajous","Circular"],e=e+pass(run_simulation(z,tr,[],1.4,cfgR,1));end
        sc=0;for sd=1:3,sc=sc+pass(run_simulation(z,"Sinusoidal",[],1.4,cfgR,sd));end
        ok=(nl==25 && r1==25 && e==4 && sc>=2);
        tag=''; if ok && sw<0.562, tag=sprintf('*** %.0f%%',100*(1-sw/0.562)); elseif ok, tag='gate-ok'; end
        fprintf('  %-14s | %.3f | %2d   %2d/25  %d  %d  | %s\n',L{i}{1},sw,nl,r1,e,sc,tag);
    end
    VDF_OVERRIDE=[];GAMMA_V=[];
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
function st=sm(c), st=struct(); for i=1:2:numel(c), st.(c{i})=c{i+1}; end; end
