%% CB_E25_VALIDATE  E_xy=0.25 candidate (fuller kappa delivery) -- gate + stress vs COMBINED.
%   E25 = COMBINED + E_xy 0.5->0.25: kappa0 .05, N .10, Pleak [.5;.5;1.5], E [.25;.25;.5].
%   Tighter E -> sigma escapes boundary layer -> full kappa delivered against sigma (100%
%   engaged under stress). Does the fuller delivery hold the 25/25 gate AND extend the
%   rejection margin past COMBINED's 7x (5/5) / 8x (1/5)?
%
% Run:  cd MATLAB/Multi_init_cond; cb_E25_validate
% Saves: ../Datasets/MultiInit/E25_validate.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

E25  = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
              'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.25,0.25,0.5]));
COMB = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]), ...
              'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
scells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };

% ---- Part 1: E25 gate (25 realistic) ----
fprintf('\n===== PART 1: E25 gate (25 realistic) =====\n');
sp=0; n=0; xy=[]; v=[]; fly=0;
for t=1:numel(trajList)
    for ic=1:5
        m=onerun(E25,[],p0(ic,:),trajList(t));
        sp=sp+m.sp; n=n+1; xy(end+1)=m.xy; v(end+1)=m.v; if m.fly,fly=fly+1;end %#ok<AGROW>
    end
end
fprintf('  E25 gate %d/25 SP | mXY %.3f mV %.3f maxV %.3f | flyaways %d\n', ...
    sp, mean(xy), mean(v), max(v), fly);

% ---- Part 2: stress E25 vs COMBINED at 6/7/8x ----
fprintf('\n===== PART 2: stress E25 vs COMBINED(E.5) =====\n');
fprintf('  scale | E25 SP  mXY  | COMB SP  mXY\n');
S=struct('scale',{},'e25_sp',{},'comb_sp',{});
for s=[6,7,8]
    e25=0; ex=[]; cm=0; cx=[];
    for c=1:size(scells,1)
        me=onerun(E25, s,p0(scells{c,2},:),scells{c,1}); e25=e25+me.sp; ex(end+1)=me.xy; %#ok<AGROW>
        mc=onerun(COMB,s,p0(scells{c,2},:),scells{c,1}); cm=cm+mc.sp; cx(end+1)=mc.xy; %#ok<AGROW>
    end
    fprintf('   %dx  |  %d/5  %.3f |  %d/5  %.3f\n', s, e25, mean(ex), cm, mean(cx));
    S(end+1)=struct('scale',s,'e25_sp',e25,'comb_sp',cm); %#ok<AGROW>
end
clear global VDF_OVERRIDE STRESS_SCALE
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','E25_validate.mat'),'S');
fprintf('\n(want: gate 25/25 + E25 SP >= COMBINED at 7-8x = fuller kappa delivery buys margin)\nDONE\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.v=r.final_rel_vel;
    m.fly=~r.success || r.data.idx<2 || r.final_alt>0.25;
end
