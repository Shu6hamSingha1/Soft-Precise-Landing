%% CB_EXY_TERMINAL  Tightened lateral boundary layer E_xy: SP gate + disturbance
%   rejection + terminal a_u (balloon) check.
%
%   Two opposing hypotheses about tightening E_xy (delivers MORE lateral switching
%   authority -- kappa*sat(sigma/E) gets closer to +-kappa):
%     (+) more authority -> damps lateral error / disturbance better
%     (-) PX4 warns lateral E kept WIDE because switching there is NOISE-PUMPED;
%         and harder switching at the deck could SPIKE a_u -> the terminal balloon.
%
%   Part 1  SP GATE     : 25 realistic cells, E_xy in {1.0,0.5,0.25}. SP/precision/
%                         max-v/fly-aways + terminal a_u spike (max|I_a_cd_xy| last 0.5s).
%   Part 2  KNOWN-DIST  : Static + 5N lateral step (KNOWN_DIST). Does tighter E_xy
%                         lower the error excursion? Does it spike a_u (balloon)?
%
% Run:  cd MATLAB/Multi_init_cond; cb_Exy_terminal
% Saves: ../Datasets/MultiInit/Exy_terminal.mat
clc; clear;
mfile_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(mfile_dir, '..', 'Common'));
clear mfile_dir;
global VDF_OVERRIDE KNOWN_DIST %#ok<GVMIS>

Dbase = struct('theta_per_axis',true,'kappa0',[0.05;0.05;0.05],'N',diag([0.10,0.10,0.10]));
Exys  = [1.0, 0.5, 0.25];
trajList = ["Static","Linear","Sinusoidal","Lissajous","Circular"];
p0 = [ 0,0,-5; 2.0,2.0,-5; 2.0,-2.0,-5; 2.0,2.0,-7; 2.0,2.0,-3 ];

% ---------------- Part 1: SP GATE across 25 realistic cells ----------------
fprintf('\n========== PART 1: E_xy SP GATE (25 realistic) ==========\n');
P1 = struct('Exy',{},'sp',{},'mxy',{},'mv',{},'maxv',{},'flyaways',{},'au_term',{});
co = struct('NOISE',1,'GE',1,'delay',1);
for Exy = Exys
    cfg = Dbase; cfg.E = diag([Exy,Exy,0.5]);
    sp=0; n=0; xy=[]; v=[]; fly=0; au=[];
    for t=1:numel(trajList)
        for ic=1:5
            VDF_OVERRIDE = cfg; KNOWN_DIST = [];
            x0 = [p0(ic,:)'; 1;0;0;0; zeros(3,1); zeros(3,1)];
            r = run_simulation(x0, trajList(t), [], 1.0, co, 1);
            d=r.data; idx=d.idx; sp=sp+(r.precise&&r.soft); n=n+1;
            xy(end+1)=r.final_xy; v(end+1)=r.final_rel_vel;                 %#ok<AGROW>
            if ~r.success || r.final_alt>0.25, fly=fly+1; end
            w=max(1,idx-50):idx; au(end+1)=max(vecnorm(d.I_a_cd(1:2,w)));   %#ok<AGROW>
        end
    end
    P1(end+1)=struct('Exy',Exy,'sp',sp,'mxy',mean(xy),'mv',mean(v),'maxv',max(v), ...
                     'flyaways',fly,'au_term',median(au)); %#ok<AGROW>
    fprintf('  E_xy=%.2f | SP %2d/25 | mXY %.3f mV %.3f maxV %.3f | flyaways %d | au_term(med) %.1f\n', ...
        Exy, sp, mean(xy), mean(v), max(v), fly, median(au));
end

% ---------------- Part 2: KNOWN-DIST rejection + terminal a_u ----------------
fprintf('\n========== PART 2: known 5N lateral step, E_xy effect ==========\n');
fprintf('  E_xy | max|s|/E | k_peak | max|h_e_x| (excursion) | max|au_xy| | landed\n');
P2 = struct('Exy',{},'soe',{},'kpk',{},'hemax',{},'aumax',{},'landed',{});
for Exy = Exys
    cfg = Dbase; cfg.E = diag([Exy,Exy,0.5]);
    VDF_OVERRIDE = cfg;
    KNOWN_DIST = struct('force',[5;0;0],'t_on',2.0,'t_off',inf);
    x0 = [0;0;-5; 1;0;0;0; zeros(3,1); zeros(3,1)];
    r = run_simulation(x0, "Static", [], 1.0, co, 1);
    d=r.data; idx=d.idx; P=d.P; on=d.F_known_log(1,1:idx)'~=0;
    soe = max(abs(d.sigma(1,find(on))')/P.E(1,1));
    kpk = max(d.kappa_log(1,find(on)));
    hemax = max(abs(d.V_h_e(1,find(on))));
    aumax = max(abs(d.I_a_cd(1,find(on))));
    P2(end+1)=struct('Exy',Exy,'soe',soe,'kpk',kpk,'hemax',hemax,'aumax',aumax,'landed',r.success); %#ok<AGROW>
    fprintf('  %4.2f |  %6.2f  | %.3f  |       %8.4f        |   %6.2f   |   %d\n', ...
        Exy, soe, kpk, hemax, aumax, r.success);
end

clear global VDF_OVERRIDE KNOWN_DIST
save(fullfile(fileparts(mfilename('fullpath')),'..','Datasets','MultiInit','Exy_terminal.mat'),'P1','P2');
fprintf('\nSaved -> Datasets/MultiInit/Exy_terminal.mat\nDONE\n');
