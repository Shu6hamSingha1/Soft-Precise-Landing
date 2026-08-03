%% CB_STRESS_BRACKET  Find the stress band where COMBINED holds SP but BAKED fails.
%   COMBINED (D+L3+E_xy=0.5) and BAKED both hold at 6x, both break at 10x. Bracket
%   {7,8,9}x on the engaging cells to see if kappa's adaptation EXTENDS the margin
%   (COMBINED SP > BAKED SP at the same stress) = "kappa buys rejection margin".
%
% Run:  cd MATLAB/Multi_init_cond; cb_stress_bracket
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE STRESS_SCALE %#ok<GVMIS>

COMB  = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05], ...
               'N',diag([0.10,0.10,0.10]),'Pleak',diag([0.5,0.5,1.5]),'E',diag([0.5,0.5,0.5]));
BAKED = struct('theta_per_axis',true);
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];
cells = { 'Sinusoidal',2; 'Sinusoidal',4; 'Sinusoidal',5; 'Circular',3; 'Circular',5 };
scales = [7, 8, 9];

fprintf('\n========== STRESS BRACKET (margin extension) ==========\n');
fprintf('  scale | COMBINED SP  mXY   kRMSxy | BAKED SP  mXY\n');
for s = scales
    cs=0; cx=[]; ck=[]; bk=0; bx=[];
    for c=1:size(cells,1)
        mc = onerun(COMB,  s, p0(cells{c,2},:), cells{c,1});
        mb = onerun(BAKED, s, p0(cells{c,2},:), cells{c,1});
        cs=cs+mc.sp; cx(end+1)=mc.xy; ck(end+1)=mean(mc.krms(1:2));      %#ok<AGROW>
        bk=bk+mb.sp; bx(end+1)=mb.xy;                                    %#ok<AGROW>
    end
    fprintf('   %2dx  |    %d/5     %.3f  %.3f  |   %d/5    %.3f  %s\n', ...
        s, cs, mean(cx), mean(ck), bk, mean(bx), tf(cs>bk));
end

clear global VDF_OVERRIDE STRESS_SCALE
fprintf('\n(YES = COMBINED outlasts BAKED at that stress => kappa buys margin)\n');

function m = onerun(cfgOv, stress, p0row, traj)
    global VDF_OVERRIDE STRESS_SCALE        %#ok<GVMIS>
    VDF_OVERRIDE = cfgOv; STRESS_SCALE = stress;
    x0 = [p0row(:); 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, traj, [], 1.0, struct('NOISE',1,'GE',1,'delay',1), 1);
    d=r.data; ka=d.kappa_log(:,1:d.idx);
    m.sp=r.precise&&r.soft; m.xy=r.final_xy; m.krms=sqrt(mean(ka.^2,2));
end
function s=tf(b), if b, s='<-- YES'; else, s=''; end, end
